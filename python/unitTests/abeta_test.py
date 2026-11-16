import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
import abeta_analysis as aa
from abeta_controller import Decision, StaircaseConfig, StaircaseController

FS = aa.FS_HZ
STIM_HZ = 25


def synthetic_block(n_pulses, abeta_peak_counts, rng, noise_counts=2.0, artifact_counts=1500,
                    latency_ms=2.5, width_ms=0.4, lead_ms=10.0):
    """Stream with a sharp artifact and a Gaussian 'A-beta' after each pulse; returns (x, onsets)."""
    period = int(FS / STIM_HZ)
    lead = int(lead_ms * FS / 1000)
    n = lead + n_pulses * period
    t = np.arange(n)
    x = rng.normal(0.0, noise_counts, n)
    onsets = lead + np.arange(n_pulses) * period
    for o in onsets:
        art = np.zeros(n)
        art[o:o + 5] = artifact_counts
        tail = np.arange(n - o - 5)
        art[o + 5:] = -0.3 * artifact_counts * np.exp(-tail / (0.15e-3 * FS))
        x += art
        mu = o + latency_ms * FS / 1000
        sigma = width_ms * FS / 1000
        x += abeta_peak_counts * np.exp(-0.5 * ((t - mu) / sigma) ** 2)
    return np.round(x).astype(np.int16), onsets


class TestStream(unittest.TestCase):
    def test_parse_rejects_text(self):
        self.assertIsNone(aa.parse_sample_packet(b"CL_STIM_ON\n"))

    def test_parse_sample_packet(self):
        samples = np.arange(aa.SAMPLES_PER_PACKET, dtype="<i2")
        seq, out = aa.parse_sample_packet((513).to_bytes(2, "little") + samples.tobytes())
        self.assertEqual(seq, 513)
        np.testing.assert_array_equal(out, samples)

    def test_gap_is_filled_and_invalid(self):
        s = aa.SampleStream(samples_per_packet=4)
        s.push(10, [1, 2, 3, 4])
        s.push(12, [5, 6, 7, 8])
        x, valid = s.snapshot()
        self.assertEqual(len(x), 12)
        self.assertEqual(s.dropped_packets, 1)
        np.testing.assert_array_equal(valid, [True] * 4 + [False] * 4 + [True] * 4)

    def test_sequence_wraparound_is_not_a_gap(self):
        s = aa.SampleStream(samples_per_packet=2)
        s.push(65535, [1, 2])
        s.push(0, [3, 4])
        _, valid = s.snapshot()
        self.assertTrue(valid.all())
        self.assertEqual(s.dropped_packets, 0)

    def test_session_restart_breaks_continuity(self):
        s = aa.SampleStream(samples_per_packet=2)
        s.push(500, [1, 2])
        s.push(0, [3, 4])
        _, valid = s.snapshot()
        self.assertEqual(s.restarts, 1)
        self.assertEqual(int((~valid).sum()), 1)

    def test_clear_keeps_sequence_state(self):
        s = aa.SampleStream(samples_per_packet=2)
        s.push(1, [1, 2])
        s.clear()
        s.push(3, [3, 4])
        _, valid = s.snapshot()
        self.assertEqual(s.dropped_packets, 1)
        self.assertEqual(len(valid), 4)


class TestDetection(unittest.TestCase):
    def test_detects_each_pulse_once(self):
        rng = np.random.default_rng(0)
        x, onsets = synthetic_block(12, abeta_peak_counts=40, rng=rng)
        found = aa.detect_pulses(x, np.ones(len(x), bool), FS, STIM_HZ)
        self.assertEqual(len(found), len(onsets))
        self.assertTrue(np.all(np.abs(found - onsets) <= 1))

    def test_no_pulses_in_noise(self):
        rng = np.random.default_rng(1)
        x = np.round(rng.normal(0, 2.0, int(FS))).astype(np.int16)
        self.assertEqual(len(aa.detect_pulses(x, np.ones(len(x), bool), FS, STIM_HZ)), 0)

    def test_epochs_skip_gaps(self):
        rng = np.random.default_rng(2)
        x, onsets = synthetic_block(5, abeta_peak_counts=40, rng=rng)
        valid = np.ones(len(x), bool)
        valid[onsets[2] + 50] = False
        cfg = aa.EpochConfig()
        epochs, used = aa.extract_epochs(x, valid, onsets, cfg)
        self.assertEqual(len(epochs), 4)
        self.assertNotIn(onsets[2], used)


class TestAnalysis(unittest.TestCase):
    def setUp(self):
        self.cfg = aa.EpochConfig(win_start_ms=1.5, win_end_ms=4.0)

    def _auc(self, peak, seed=3, noise=2.0):
        rng = np.random.default_rng(seed)
        x, onsets = synthetic_block(11, abeta_peak_counts=peak, rng=rng, noise_counts=noise)
        epochs, _ = aa.extract_epochs(x, np.ones(len(x), bool), onsets[1:], self.cfg)
        return aa.analyze_block(epochs, self.cfg)

    def test_auc_matches_gaussian_area(self):
        peak = 40
        result = self._auc(peak, noise=0.0)
        expected = peak * aa.ADC_UV_PER_COUNT * 0.4 * np.sqrt(2 * np.pi)
        self.assertAlmostEqual(result.auc, expected, delta=0.05 * expected)
        self.assertEqual(result.n_reps, 10)

    def test_zero_response_reads_near_zero(self):
        small = self._auc(0).auc
        real = self._auc(40).auc
        self.assertLess(small, 0.1 * real)

    def test_auc_monotonic_in_response(self):
        aucs = [self._auc(p).auc for p in (10, 20, 40, 80)]
        self.assertTrue(all(a < b for a, b in zip(aucs, aucs[1:])))

    def test_front_end_gain_scales_auc(self):
        cfg2 = aa.EpochConfig(win_start_ms=1.5, win_end_ms=4.0, front_end_gain=2.0)
        rng = np.random.default_rng(4)
        x, onsets = synthetic_block(11, abeta_peak_counts=40, rng=rng, noise_counts=0.0)
        epochs, _ = aa.extract_epochs(x, np.ones(len(x), bool), onsets[1:], cfg2)
        r1 = aa.analyze_block(epochs, self.cfg)
        r2 = aa.analyze_block(epochs, cfg2)
        self.assertAlmostEqual(r2.auc, r1.auc / 2.0, places=6)

    def test_invalid_window_rejected(self):
        with self.assertRaises(ValueError):
            aa.EpochConfig(win_start_ms=5.0, win_end_ms=4.0).validate()


def sigmoid_plant(amp_ua, threshold=300.0, slope=60.0, auc_max=50.0):
    return auc_max / (1.0 + np.exp(-(amp_ua - threshold) / slope))


class TestController(unittest.TestCase):
    def _run(self, cfg, plant):
        ctl = StaircaseController(cfg)
        while not ctl.done:
            ctl.update(plant(ctl.amp_ua))
        return ctl

    def test_converges_from_below(self):
        cfg = StaircaseConfig(target_auc=25.0, amp_start_ua=100, amp_max_ua=1000,
                              tolerance_frac=0.10, step_init_ua=100, step_min_ua=5)
        ctl = self._run(cfg, sigmoid_plant)
        last = ctl.history[-1]
        self.assertEqual(last.decision, Decision.CONVERGED)
        self.assertLessEqual(abs(sigmoid_plant(last.amp_ua) - 25.0) / 25.0, 0.10)

    def test_converges_from_above(self):
        cfg = StaircaseConfig(target_auc=25.0, amp_start_ua=900, amp_max_ua=1000,
                              tolerance_frac=0.10, step_init_ua=100, step_min_ua=5)
        ctl = self._run(cfg, sigmoid_plant)
        self.assertEqual(ctl.history[-1].decision, Decision.CONVERGED)

    def test_step_halves_on_reversal(self):
        cfg = StaircaseConfig(target_auc=25.0, amp_start_ua=250, amp_max_ua=1000,
                              step_init_ua=200, step_min_ua=5)
        ctl = StaircaseController(cfg)
        ctl.update(sigmoid_plant(250))  # below target: step up to 450
        self.assertEqual(ctl.amp_ua, 450)
        ctl.update(sigmoid_plant(450))  # above target: reverse, step halves to 100
        self.assertEqual(ctl.step_ua, 100)
        self.assertEqual(ctl.amp_ua, 350)

    def test_unreachable_high(self):
        cfg = StaircaseConfig(target_auc=80.0, amp_start_ua=500, amp_max_ua=600,
                              step_init_ua=100, step_min_ua=5)
        ctl = self._run(cfg, sigmoid_plant)
        self.assertEqual(ctl.history[-1].decision, Decision.UNREACHABLE_HIGH)
        self.assertLessEqual(ctl.amp_ua, 600)

    def test_unreachable_low(self):
        cfg = StaircaseConfig(target_auc=25.0, amp_start_ua=100, amp_min_ua=50, amp_max_ua=1000,
                              step_init_ua=50, step_min_ua=5)
        ctl = self._run(cfg, lambda a: 100.0)
        self.assertEqual(ctl.history[-1].decision, Decision.UNREACHABLE_LOW)
        self.assertGreaterEqual(ctl.amp_ua, 50)

    def test_never_exceeds_limits(self):
        cfg = StaircaseConfig(target_auc=25.0, amp_start_ua=100, amp_max_ua=400,
                              step_init_ua=500, step_min_ua=5, max_iterations=50)
        ctl = self._run(cfg, sigmoid_plant)
        self.assertTrue(all(0 <= h.next_amp_ua <= 400 for h in ctl.history))

    def test_max_iterations(self):
        cfg = StaircaseConfig(target_auc=25.0, amp_start_ua=100, amp_max_ua=1000,
                              step_init_ua=5, step_min_ua=5, max_iterations=3)
        ctl = self._run(cfg, sigmoid_plant)
        self.assertEqual(len(ctl.history), 3)
        self.assertEqual(ctl.history[-1].decision, Decision.MAX_ITERATIONS)

    def test_rejects_bad_config(self):
        with self.assertRaises(ValueError):
            StaircaseController(StaircaseConfig(target_auc=10, amp_start_ua=2000,
                                                amp_max_ua=1000))


if __name__ == '__main__':
    unittest.main()
