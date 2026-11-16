"""Amplitude titration toward a target A-beta area: a staircase that halves its step on reversal.

The amplitude-to-recruitment curve is sigmoidal (flat below threshold, steep, then saturating),
so a fixed-gain controller either crawls or overshoots. Halving the step on every direction
change only assumes more current gives more response, and converges like a bisection.
"""
from dataclasses import dataclass, field
from enum import Enum


class Decision(str, Enum):
    STEP_UP = "STEP_UP"
    STEP_DOWN = "STEP_DOWN"
    HOLD = "HOLD"
    HOLD_AT_LIMIT = "HOLD_AT_LIMIT"
    CONVERGED = "CONVERGED"
    UNREACHABLE_HIGH = "UNREACHABLE_HIGH"
    UNREACHABLE_LOW = "UNREACHABLE_LOW"
    MAX_ITERATIONS = "MAX_ITERATIONS"


TERMINAL_DECISIONS = {
    Decision.CONVERGED,
    Decision.UNREACHABLE_HIGH,
    Decision.UNREACHABLE_LOW,
    Decision.MAX_ITERATIONS,
}


@dataclass
class StaircaseConfig:
    target_auc: float
    amp_start_ua: float
    amp_max_ua: float
    amp_min_ua: float = 0.0
    tolerance_frac: float = 0.10
    step_init_ua: float = 50.0
    step_min_ua: float = 5.0
    in_band_required: int = 2
    stall_at_limit_required: int = 2
    max_iterations: int = 20

    def validate(self):
        if self.target_auc <= 0:
            raise ValueError("Target AUC must be positive")
        if not 0 < self.tolerance_frac < 1:
            raise ValueError("Tolerance must be between 0 and 100 %")
        if not 0 <= self.amp_min_ua < self.amp_max_ua:
            raise ValueError("Amplitude limits must satisfy 0 <= min < max")
        if not self.amp_min_ua <= self.amp_start_ua <= self.amp_max_ua:
            raise ValueError("Start amplitude must lie within the amplitude limits")
        if not 0 < self.step_min_ua <= self.step_init_ua:
            raise ValueError("Steps must satisfy 0 < min step <= initial step")
        if self.in_band_required < 1 or self.stall_at_limit_required < 1:
            raise ValueError("Required block counts must be at least 1")
        if self.max_iterations < 1:
            raise ValueError("Max iterations must be at least 1")


@dataclass
class StepRecord:
    iteration: int
    amp_ua: float
    auc: float
    error_frac: float
    decision: Decision
    next_amp_ua: float
    step_ua: float


@dataclass
class StaircaseController:
    cfg: StaircaseConfig
    amp_ua: float = field(init=False)
    step_ua: float = field(init=False)
    history: list = field(init=False, default_factory=list)

    def __post_init__(self):
        self.cfg.validate()
        self.amp_ua = float(self.cfg.amp_start_ua)
        self.step_ua = float(self.cfg.step_init_ua)
        self._last_dir = 0
        self._in_band = 0
        self._at_limit = 0
        self._done = False

    @property
    def done(self):
        return self._done

    def _clamp(self, amp):
        return min(max(amp, self.cfg.amp_min_ua), self.cfg.amp_max_ua)

    def update(self, auc):
        """Consume one block's AUC (measured at self.amp_ua) and return a StepRecord."""
        if self._done:
            raise RuntimeError("Titration already finished")

        measured_amp = self.amp_ua
        err = (auc - self.cfg.target_auc) / self.cfg.target_auc

        if abs(err) <= self.cfg.tolerance_frac:
            self._in_band += 1
            self._at_limit = 0
            decision = (Decision.CONVERGED if self._in_band >= self.cfg.in_band_required
                        else Decision.HOLD)
        else:
            self._in_band = 0
            direction = 1 if err < 0 else -1
            if self._last_dir and direction != self._last_dir:
                self.step_ua = max(self.step_ua / 2.0, self.cfg.step_min_ua)
            self._last_dir = direction

            proposed = self._clamp(self.amp_ua + direction * self.step_ua)
            if proposed == self.amp_ua:
                self._at_limit += 1
                if self._at_limit >= self.cfg.stall_at_limit_required:
                    decision = (Decision.UNREACHABLE_HIGH if direction > 0
                                else Decision.UNREACHABLE_LOW)
                else:
                    decision = Decision.HOLD_AT_LIMIT
            else:
                self._at_limit = 0
                self.amp_ua = proposed
                decision = Decision.STEP_UP if direction > 0 else Decision.STEP_DOWN

        iteration = len(self.history) + 1
        if decision not in TERMINAL_DECISIONS and iteration >= self.cfg.max_iterations:
            decision = Decision.MAX_ITERATIONS
            self.amp_ua = measured_amp

        if decision in TERMINAL_DECISIONS:
            self._done = True

        record = StepRecord(iteration=iteration, amp_ua=measured_amp, auc=float(auc),
                            error_frac=err, decision=decision, next_amp_ua=self.amp_ua,
                            step_ua=self.step_ua)
        self.history.append(record)
        return record
