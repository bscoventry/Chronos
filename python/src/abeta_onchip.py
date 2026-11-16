"""Wire formats for on-chip A-beta closed loop (mirrors src/closed_loop.h)."""
import struct
from dataclasses import dataclass
from enum import IntEnum

CL_PKT_PULSE = 0xC1
CL_PKT_BLOCK = 0xC2
CL_PKT_STATE = 0xC3
CL_PKT_CONFIG = 0xC4

CL_CTRL_CONFIG = 0x06
CL_CTRL_RUN_BLOCK = 0x07
CL_CTRL_ARM = 0x04
CL_CTRL_DISARM = 0x05

CONFIG_WIRE_FMT = "<f7H2h4H4B"
CONFIG_WIRE_LEN = struct.calcsize(CONFIG_WIRE_FMT)
CONFIG_ACK_FMT = "<BB" + CONFIG_WIRE_FMT[1:]
CONFIG_ACK_LEN = struct.calcsize(CONFIG_ACK_FMT)

PULSE_FMT = "<BIIHIB"
PULSE_LEN = struct.calcsize(PULSE_FMT)

BLOCK_FMT = "<BIHH5ffBBB"
BLOCK_LEN = struct.calcsize(BLOCK_FMT)

STATE_FMT = "<BBBH"
STATE_LEN = struct.calcsize(STATE_FMT)


class ClDecision(IntEnum):
    MEASURED = 0
    HOLD = 1
    CONVERGED = 2
    STEP_UP = 3
    STEP_DOWN = 4
    HOLD_AT_LIMIT = 5
    UNREACHABLE_HIGH = 6
    UNREACHABLE_LOW = 7


class ClPulseStatus(IntEnum):
    USED = 0
    CLIPPED = 1
    DISCARD_SETTLE = 2
    DISCARD_AMP = 3
    GAP = 4
    EXTRA = 5


@dataclass
class ClConfigWire:
    target_auc: float
    tol_permille: int
    rearm_tol_permille: int
    pre_samples: int
    post_samples: int
    baseline_samples: int
    win_start: int
    win_end: int
    clip_lo: int
    clip_hi: int
    amp_min: int
    amp_max: int
    step_init: int
    step_min: int
    n_reps: int
    discard: int
    in_band_required: int
    limit_blocks_required: int


def pack_config_wire(cfg: ClConfigWire) -> bytes:
    return struct.pack(
        CONFIG_WIRE_FMT,
        cfg.target_auc,
        cfg.tol_permille,
        cfg.rearm_tol_permille,
        cfg.pre_samples,
        cfg.post_samples,
        cfg.baseline_samples,
        cfg.win_start,
        cfg.win_end,
        cfg.clip_lo,
        cfg.clip_hi,
        cfg.amp_min,
        cfg.amp_max,
        cfg.step_init,
        cfg.step_min,
        cfg.n_reps,
        cfg.discard,
        cfg.in_band_required,
        cfg.limit_blocks_required,
    )


def parse_cl_packet(data: bytes):
    """Return a parsed on-chip packet dict, or None if not a CL binary packet."""
    if not data or data[0] < 0x80:
        return None
    kind = data[0]
    if kind == CL_PKT_CONFIG and len(data) >= CONFIG_ACK_LEN:
        accepted, = struct.unpack_from("<B", data, 1)
        fields = struct.unpack_from(CONFIG_WIRE_FMT, data, 2)
        return {"type": kind, "accepted": bool(accepted), "cfg": _cfg_from_fields(fields)}
    if kind == CL_PKT_PULSE and len(data) >= PULSE_LEN:
        t, pulse_no, onset, amp, block_no, status = struct.unpack_from(PULSE_FMT, data, 0)
        return {
            "type": kind,
            "pulse_no": pulse_no,
            "onset_sample": onset,
            "amp_code": amp,
            "block_no": block_no,
            "status": ClPulseStatus(status),
        }
    if kind == CL_PKT_BLOCK and len(data) >= BLOCK_LEN:
        (t, block_no, amp, next_amp, auc, auc_raw, noise, sem, err, step, decision, n_reps,
         converged) = struct.unpack_from(BLOCK_FMT, data, 0)
        return {
            "type": kind,
            "block_no": block_no,
            "amp_code": amp,
            "next_amp_code": next_amp,
            "auc": auc,
            "auc_raw": auc_raw,
            "noise_auc": noise,
            "auc_sem": sem,
            "error_frac": err,
            "step": step,
            "decision": ClDecision(decision),
            "n_reps": n_reps,
            "converged": bool(converged),
        }
    if kind == CL_PKT_STATE and len(data) >= STATE_LEN:
        t, state, event, amp = struct.unpack_from(STATE_FMT, data, 0)
        return {"type": kind, "state": state, "event": event, "amp_code": amp}
    return None


def chip_auc_to_uv_ms(auc_chip: float, front_end_gain: float = 1.0) -> float:
    """Convert on-chip AUC (sum of |counts| over the window) to uV*ms."""
    from abeta_analysis import ADC_UV_PER_COUNT, FS_HZ

    dt_ms = 1000.0 / FS_HZ
    scale = ADC_UV_PER_COUNT / front_end_gain
    return float(auc_chip) * scale * dt_ms


def uv_ms_target_to_chip(target_uv_ms: float, front_end_gain: float = 1.0) -> float:
    from abeta_analysis import ADC_UV_PER_COUNT, FS_HZ

    dt_ms = 1000.0 / FS_HZ
    scale = ADC_UV_PER_COUNT / front_end_gain
    return float(target_uv_ms) / (scale * dt_ms)


def _cfg_from_fields(fields):
    keys = [
        "target_auc", "tol_permille", "rearm_tol_permille", "pre_samples", "post_samples",
        "baseline_samples", "win_start", "win_end", "clip_lo", "clip_hi", "amp_min",
        "amp_max", "step_init", "step_min", "n_reps", "discard", "in_band_required",
        "limit_blocks_required",
    ]
    return ClConfigWire(**dict(zip(keys, fields)))
