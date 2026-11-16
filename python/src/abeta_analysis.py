"""Stream assembly, stimulus-artifact pulse detection and A-beta area under the curve.

The board streams the SAADC continuously and carries no stimulus markers, so pulse onsets are
recovered from the stimulation artifact itself: the steepest feature in the recording by far.
"""
import threading
from dataclasses import dataclass

import numpy as np

FS_HZ = 25000.0
SAMPLES_PER_PACKET = 120
SAMPLE_PACKET_LEN = 2 + 2 * SAMPLES_PER_PACKET

# SAADC: 0.6 V internal reference, gain 1/4, 12-bit (matches signal_mv.c).
ADC_UV_PER_COUNT = 600000.0 * 4.0 / 4096.0

_SEQ_MOD = 1 << 16
# A forward jump this large cannot be dropped packets inside one session (~157 s at 25 kHz);
# it means the firmware restarted its ADC session and reset the counter.
_SEQ_RESTART_THRESHOLD = _SEQ_MOD // 2


def parse_sample_packet(data):
    """Return (seq, int16 samples) for a sample notification, or None for anything else."""
    if len(data) != SAMPLE_PACKET_LEN:
        return None
    seq = int.from_bytes(data[0:2], "little")
    samples = np.frombuffer(bytes(data[2:]), dtype="<i2")
    return seq, samples


def counts_to_uv(counts, front_end_gain=1.0):
    """Input-referred microvolts for raw SAADC counts."""
    return np.asarray(counts, dtype=np.float64) * ADC_UV_PER_COUNT / front_end_gain


class SampleStream:
    """Thread-safe contiguous sample buffer that fills dropped packets and marks them invalid.

    push() is called from the BLE thread; snapshot() and clear() from the GUI thread.
    """

    def __init__(self, samples_per_packet=SAMPLES_PER_PACKET):
        self._spp = samples_per_packet
        self._lock = threading.Lock()
        self._chunks = []
        self._valid = []
        self._last_seq = None
        self._last_value = 0
        self.dropped_packets = 0
        self.restarts = 0

    def push(self, seq, samples):
        samples = np.asarray(samples, dtype=np.int16)
        with self._lock:
            if self._last_seq is not None:
                missing = (seq - self._last_seq - 1) % _SEQ_MOD
                if missing >= _SEQ_RESTART_THRESHOLD:
                    # Unknown-length discontinuity: one invalid sample keeps epochs from spanning it.
                    self._append(np.array([self._last_value], dtype=np.int16), False)
                    self.restarts += 1
                elif missing:
                    fill = np.full(missing * self._spp, self._last_value, dtype=np.int16)
                    self._append(fill, False)
                    self.dropped_packets += missing
            self._append(samples, True)
            self._last_seq = seq
            if samples.size:
                self._last_value = int(samples[-1])

    def _append(self, samples, valid):
        self._chunks.append(samples)
        self._valid.append(np.full(samples.size, valid, dtype=bool))

    def snapshot(self):
        """Return (samples int16, valid bool) copies of everything since the last clear()."""
        with self._lock:
            if not self._chunks:
                return np.zeros(0, dtype=np.int16), np.zeros(0, dtype=bool)
            return np.concatenate(self._chunks), np.concatenate(self._valid)

    def clear(self):
        """Drop buffered samples but keep the sequence state so the next gap is still detected."""
        with self._lock:
            self._chunks = []
            self._valid = []
            self.dropped_packets = 0
            self.restarts = 0


def detect_pulses(x, valid, fs_hz, stim_hz, k_noise=10.0, frac_of_max=0.25):
    """Sample indices of stimulus onsets, found as the first steep edge in each stim period.

    The threshold is the larger of k_noise robust standard deviations of the sample-to-sample
    difference and frac_of_max of the largest difference, so it tracks artifact size across
    amplitudes but never drops into the noise when no pulse is present. A refractory period of
    80% of the stim period keeps the A-beta and artifact tail from re-triggering.
    """
    x = np.asarray(x, dtype=np.float64)
    if x.size < 2:
        return np.zeros(0, dtype=int)

    d = np.abs(np.diff(x))
    # An edge touching a dropped-packet fill is not a real transition.
    d[~(valid[1:] & valid[:-1])] = 0.0
    if not d.any():
        return np.zeros(0, dtype=int)

    med = np.median(d)
    mad = 1.4826 * np.median(np.abs(d - med))
    # At least one ADC count, so quantised quiet signals don't trigger on every LSB step.
    noise = max(mad, 1.0)
    thr = max(med + k_noise * noise, frac_of_max * d.max())

    refractory = max(int(0.8 * fs_hz / stim_hz), 1)
    onsets = []
    last = -refractory
    for i in np.flatnonzero(d > thr):
        if i - last >= refractory:
            onsets.append(i + 1)
            last = i
    return np.asarray(onsets, dtype=int)


@dataclass
class EpochConfig:
    fs_hz: float = FS_HZ
    pre_ms: float = 2.0
    post_ms: float = 20.0
    # Onset detection can land a sample or two late; keep those out of the baseline.
    baseline_guard_ms: float = 0.2
    win_start_ms: float = 1.5
    win_end_ms: float = 4.0
    front_end_gain: float = 1.0

    @property
    def pre_samples(self):
        return int(round(self.pre_ms * self.fs_hz / 1000.0))

    @property
    def post_samples(self):
        return int(round(self.post_ms * self.fs_hz / 1000.0))

    @property
    def baseline_samples(self):
        guard = int(round(self.baseline_guard_ms * self.fs_hz / 1000.0))
        return self.pre_samples - guard

    def validate(self):
        if self.front_end_gain <= 0:
            raise ValueError("Front-end gain must be positive")
        if self.baseline_samples < max(int(0.5e-3 * self.fs_hz), 2):
            raise ValueError("Pre-stimulus window must leave at least 0.5 ms of baseline")
        if not 0.0 <= self.win_start_ms < self.win_end_ms <= self.post_ms:
            raise ValueError("A-beta window must satisfy 0 <= start < end <= post-stimulus length")


def extract_epochs(x, valid, onsets, cfg):
    """Cut [-pre, +post) windows around each onset, skipping any that touch a gap or the edges.

    Returns (epochs int16 [n, pre+post], used onsets).
    """
    pre, post = cfg.pre_samples, cfg.post_samples
    epochs, used = [], []
    for o in onsets:
        a, b = o - pre, o + post
        if a < 0 or b > len(x) or not valid[a:b].all():
            continue
        epochs.append(x[a:b])
        used.append(o)
    if not epochs:
        return np.zeros((0, pre + post), dtype=np.int16), np.zeros(0, dtype=int)
    return np.vstack(epochs).astype(np.int16), np.asarray(used, dtype=int)


@dataclass
class BlockResult:
    t_ms: np.ndarray
    epochs_uv: np.ndarray
    mean_uv: np.ndarray
    auc: float
    auc_raw: float
    noise_auc: float
    auc_sem: float
    per_rep_auc: np.ndarray
    n_reps: int


def analyze_block(epochs_counts, cfg):
    """Baseline-correct, average and measure the rectified A-beta area (uV*ms).

    Rectified area is used because the filtered A-beta is multiphasic and a signed area cancels.
    Rectifying turns noise into a positive bias, so the mean rectified baseline level over the
    same window length is subtracted; otherwise a zero response would read as a small one.
    """
    cfg.validate()
    epochs = counts_to_uv(epochs_counts, cfg.front_end_gain)
    n = epochs.shape[0]
    if n == 0:
        raise ValueError("No epochs to analyse")

    base = slice(0, cfg.baseline_samples)
    epochs = epochs - epochs[:, base].mean(axis=1, keepdims=True)

    dt_ms = 1000.0 / cfg.fs_hz
    t_ms = (np.arange(epochs.shape[1]) - cfg.pre_samples) * dt_ms
    win = (t_ms >= cfg.win_start_ms) & (t_ms < cfg.win_end_ms)
    win_len_ms = win.sum() * dt_ms

    mean_uv = epochs.mean(axis=0)
    auc_raw = float(np.abs(mean_uv[win]).sum() * dt_ms)
    noise_auc = float(np.abs(mean_uv[base]).mean() * win_len_ms)
    auc = max(auc_raw - noise_auc, 0.0)

    per_rep = (np.abs(epochs[:, win]).sum(axis=1) * dt_ms
               - np.abs(epochs[:, base]).mean(axis=1) * win_len_ms)
    auc_sem = float(per_rep.std(ddof=1) / np.sqrt(n)) if n > 1 else float("nan")

    return BlockResult(t_ms=t_ms, epochs_uv=epochs, mean_uv=mean_uv, auc=auc, auc_raw=auc_raw,
                       noise_auc=noise_auc, auc_sem=auc_sem, per_rep_auc=per_rep, n_reps=n)
