"""A-beta titration: calibration blocks and on-chip closed-loop control with live ADC streaming."""
import csv
import datetime
import time
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import h5py
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import D2B
from abeta_analysis import (ADC_UV_PER_COUNT, FS_HZ, EpochConfig, SampleStream, analyze_block,
                            extract_epochs)
from abeta_onchip import (CL_PKT_BLOCK, CL_PKT_CONFIG, CL_PKT_STATE, ClDecision, ClPulseStatus,
                          ClConfigWire, chip_auc_to_uv_ms, pack_config_wire, uv_ms_target_to_chip)

POLL_MS = 150
CONFIG_TIMEOUT_S = 3.0
BLOCK_TIMEOUT_S = 30.0

FIELDS = [
    ("target_auc", "Target A-beta AUC (uV*ms)", "10.0"),
    ("tolerance_pct", "Tolerance (%)", "10"),
    ("rearm_tol_pct", "Re-arm tolerance (%)", "15"),
    ("amp_start", "Start amplitude (uA, + = cathode leading)", "200"),
    ("amp_min", "Min amplitude |uA|", "0"),
    ("amp_max", "Max amplitude |uA|", "1000"),
    ("step_init", "Initial step (uA)", "50"),
    ("step_min", "Min step (uA)", "5"),
    ("pulse_width", "Pulse width (us)", "200"),
    ("freq", "Stim frequency (Hz)", "25"),
    ("n_reps", "Reps per block", "10"),
    ("discard", "Discard pulses after amp change", "1"),
    ("in_band", "In-band blocks to converge", "2"),
    ("limit_blocks", "Blocks at limit before fault", "2"),
    ("pre_ms", "Epoch pre-stimulus (ms)", "2.0"),
    ("post_ms", "Epoch post-stimulus (ms)", "20.0"),
    ("win_start", "A-beta window start (ms)", "1.5"),
    ("win_end", "A-beta window end (ms)", "4.0"),
    ("gain", "Front-end gain (V/V)", "1.0"),
]

FAULT_EVENTS = {6, 7, 8, 9, 10}

BLOCK_CSV_FIELDS = (
    "block_no",
    "amp_ua",
    "next_amp_ua",
    "auc_uv_ms",
    "auc_chip",
    "auc_raw",
    "noise_auc",
    "auc_sem",
    "error_frac",
    "step",
    "n_reps",
    "decision",
    "converged",
    "target_auc_uv_ms",
)


class AbetaTitrationWindow(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app.root)
        self.app = app
        self.title("A-beta Titration (on-chip control)")
        self.geometry("1100x760")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.stream = SampleStream()
        self.state = "idle"
        self.single_block = False
        self.epoch_cfg = None
        self.params = None
        self.h5 = None
        self.session_basename = None
        self.block_idx = 0
        self.pulses = []
        self.block_history = []
        self.started_adc_stream = False
        self._poll_id = None
        self._config_event = threading.Event()
        self._config_ok = False
        self._deadline = 0.0

        self._build_ui()

    def _build_ui(self):
        left = ttk.Frame(self, padding=8)
        left.grid(row=0, column=0, sticky="ns")
        self.inputs_frame = left
        right = ttk.Frame(self, padding=8)
        right.grid(row=0, column=1, sticky="nsew")
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        self.vars = {}
        for row, (key, label, default) in enumerate(FIELDS):
            ttk.Label(left, text=label).grid(row=row, column=0, sticky=tk.W, pady=1)
            var = tk.StringVar(value=default)
            ttk.Entry(left, textvariable=var, width=10).grid(row=row, column=1, padx=(6, 0))
            self.vars[key] = var
        self.vars["pulse_width"].set(self.app.pulse_var.get())
        self.vars["freq"].set("25")

        row = len(FIELDS)
        buttons = ttk.Frame(left)
        buttons.grid(row=row, column=0, columnspan=2, pady=(10, 4))
        self.run_one_button = ttk.Button(buttons, text="Run one block",
                                         command=lambda: self.start(single_block=True))
        self.run_one_button.grid(row=0, column=0, padx=2)
        self.start_button = ttk.Button(buttons, text="Arm on-chip control",
                                       command=lambda: self.start(single_block=False))
        self.start_button.grid(row=0, column=1, padx=2)
        self.abort_button = ttk.Button(buttons, text="Disarm / abort",
                                       command=self.abort, state="disabled")
        self.abort_button.grid(row=0, column=2, padx=2)

        self.status_label = ttk.Label(left, text="Status: idle", foreground="gray",
                                      wraplength=320, justify=tk.LEFT)
        self.status_label.grid(row=row + 1, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))
        self.result_label = ttk.Label(left, text="", wraplength=320, justify=tk.LEFT)
        self.result_label.grid(row=row + 2, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))

        self.fig = Figure(figsize=(7.5, 7), dpi=100)
        self.ax_epochs = self.fig.add_subplot(2, 1, 1)
        self.ax_trend = self.fig.add_subplot(2, 1, 2)
        self.ax_amp = self.ax_trend.twinx()
        self.fig.tight_layout(pad=2.5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._draw_trend()
        self.canvas.draw_idle()

    def on_cl_packet(self, pkt):
        if pkt["type"] == CL_PKT_CONFIG:
            self._config_ok = pkt["accepted"]
            self._config_event.set()
            if not pkt["accepted"]:
                self.after(0, lambda: self._finish("device rejected CL config", "red"))
            return
        if pkt["type"] == CL_PKT_STATE:
            if pkt["event"] in FAULT_EVENTS:
                self.after(0, lambda e=pkt["event"]: self._finish(f"device fault event {e}", "red"))
            elif pkt["event"] == 4 and self.state != "idle":
                self.after(0, lambda: self._finish("disarmed", "orange"))
            return
        if pkt["type"] == CL_PKT_BLOCK:
            self.after(0, lambda p=pkt: self._on_block_packet(p))

    def _on_block_packet(self, pkt):
        p = self.params
        gain = p["gain"]
        auc_uv = chip_auc_to_uv_ms(pkt["auc"], gain)
        amp_ua = D2B.ua_from_dac_code(pkt["amp_code"])
        next_ua = D2B.ua_from_dac_code(pkt["next_amp_code"])
        self.block_history.append({
            "block_no": pkt["block_no"],
            "amp_ua": amp_ua,
            "next_ua": next_ua,
            "auc_uv": auc_uv,
            "auc_chip": pkt["auc"],
            "auc_raw": pkt["auc_raw"],
            "noise_auc": pkt["noise_auc"],
            "auc_sem": pkt["auc_sem"],
            "error_frac": pkt["error_frac"],
            "step": pkt["step"],
            "n_reps": pkt["n_reps"],
            "decision": pkt["decision"],
            "converged": pkt["converged"],
        })
        self._plot_block_from_stream(amp_ua, auc_uv)
        self._draw_trend()
        self.canvas.draw_idle()
        self._save_block_packet(pkt, auc_uv, amp_ua)

        text = (f"Block {pkt['block_no']} at {amp_ua:.0f} uA: AUC {auc_uv:.2f} uV*ms "
                f"(chip raw {pkt['auc']:.1f}), {pkt['decision'].name}, "
                f"next {next_ua:.0f} uA")
        self.result_label.config(text=text)
        self.app.log_message(text)
        self.block_idx = pkt["block_no"] + 1
        if not self.single_block:
            self._deadline = time.monotonic() + BLOCK_TIMEOUT_S

        if self.single_block:
            self._finish("single block complete", "green")
            return
        if pkt["decision"] in (ClDecision.UNREACHABLE_HIGH, ClDecision.UNREACHABLE_LOW):
            self._finish(f"fault: {pkt['decision'].name}", "red")

    def _set_status(self, text, color="black"):
        self.status_label.config(text=f"Status: {text}", foreground=color)

    def _set_running(self, running):
        self.run_one_button.config(state="disabled" if running else "normal")
        self.start_button.config(state="disabled" if running else "normal")
        self.abort_button.config(state="normal" if running else "disabled")
        for child in self.inputs_frame.winfo_children():
            if isinstance(child, ttk.Entry):
                child.config(state="disabled" if running else "normal")

    def _read_params(self):
        p = {}
        for key, label, _ in FIELDS:
            try:
                p[key] = float(self.vars[key].get())
            except ValueError:
                raise ValueError(f"{label}: not a number") from None
        for key in ("pulse_width", "freq", "n_reps", "discard", "in_band", "limit_blocks"):
            if p[key] != int(p[key]):
                raise ValueError(f"{key} must be an integer")
            p[key] = int(p[key])

        if not 1 <= p["pulse_width"] <= 65535:
            raise ValueError("Pulse width must be 1-65535 us")
        if not 1 <= p["freq"] <= 100:
            raise ValueError("Stim frequency must be 1-100 Hz")
        if p["n_reps"] < 1 or p["discard"] < 0:
            raise ValueError("Reps must be >= 1 and discard >= 0")
        if p["amp_max"] > D2B.MAX_CURRENT:
            raise ValueError(f"Max amplitude exceeds DAC range ({D2B.MAX_CURRENT} uA)")

        period_ms = 1000.0 / p["freq"]
        if p["pre_ms"] + p["post_ms"] > period_ms:
            raise ValueError(f"Epoch longer than stim period ({period_ms:.1f} ms)")

        p["polarity"] = 1.0 if p["amp_start"] >= 0 else -1.0
        p["amp_start"] = abs(p["amp_start"])

        epoch_cfg = EpochConfig(fs_hz=FS_HZ, pre_ms=p["pre_ms"], post_ms=p["post_ms"],
                                win_start_ms=p["win_start"], win_end_ms=p["win_end"],
                                front_end_gain=p["gain"])
        epoch_cfg.validate()
        return p, epoch_cfg

    def _build_cl_config(self, p, epoch_cfg):
        tol = p["tolerance_pct"] / 100.0
        rearm = p["rearm_tol_pct"] / 100.0
        target_chip = uv_ms_target_to_chip(p["target_auc"], p["gain"])
        pre = epoch_cfg.pre_samples
        post = epoch_cfg.post_samples
        win_start = pre + int(round(p["win_start"] * FS_HZ / 1000.0))
        win_end = pre + int(round(p["win_end"] * FS_HZ / 1000.0))
        return ClConfigWire(
            target_auc=target_chip,
            tol_permille=int(round(tol * 1000)),
            rearm_tol_permille=int(round(rearm * 1000)),
            pre_samples=pre,
            post_samples=post,
            baseline_samples=epoch_cfg.baseline_samples,
            win_start=win_start,
            win_end=win_end,
            clip_lo=-8000,
            clip_hi=8000,
            amp_min=D2B.mag_code_from_ua(p["amp_min"] * p["polarity"]),
            amp_max=D2B.mag_code_from_ua(p["amp_max"] * p["polarity"]),
            step_init=D2B.mag_code_from_ua(p["step_init"]),
            step_min=D2B.mag_code_from_ua(p["step_min"]),
            n_reps=p["n_reps"],
            discard=p["discard"],
            in_band_required=p["in_band"],
            limit_blocks_required=p["limit_blocks"],
        )

    def start(self, single_block):
        if self.state != "idle":
            return
        if not self.app.connected:
            messagebox.showerror("Titration", "Connect to the device first.", parent=self)
            return
        if self.app.start_var.get():
            messagebox.showerror("Titration", "Stop manual stimulation first.", parent=self)
            return
        try:
            self.params, self.epoch_cfg = self._read_params()
        except ValueError as e:
            messagebox.showerror("Input Error", str(e), parent=self)
            return

        self.single_block = single_block
        self.block_idx = 0
        self.pulses = []
        self.block_history = []
        self._open_h5()
        self.stream.clear()

        self.app.titration_active = True
        self.started_adc_stream = self.app.begin_titration_stream(on_chip=True)
        self._set_running(True)
        self._draw_trend()
        self.state = "configuring"
        self._set_status("sending configuration", "blue")
        self.app.log_message("A-beta on-chip " + ("single block" if single_block else "control")
                             + " starting")

        amp_ua = self.params["polarity"] * self.params["amp_start"]
        self.app.send_stim_pending(amp_ua, self.params["pulse_width"], self.params["freq"])
        wire = pack_config_wire(self._build_cl_config(self.params, self.epoch_cfg))
        self._config_event.clear()
        self._config_ok = False
        self.app.send_cl_config(wire)

        self._deadline = time.monotonic() + CONFIG_TIMEOUT_S
        self.app._open_hdf5_file("abeta_onchip_stream")
        self._schedule_poll()

    def abort(self):
        if self.state == "idle":
            return
        self.app.send_cl_disarm()
        self._finish("aborted by user", "red")

    def _finish(self, text, color):
        self.state = "idle"
        if self._poll_id is not None:
            self.after_cancel(self._poll_id)
            self._poll_id = None
        self.app.send_cl_disarm()
        self.app.end_titration_stream(self.started_adc_stream, on_chip=True)
        self.started_adc_stream = False
        self.app.titration_active = False
        self.app._close_hdf5_file()
        self._export_block_csv()
        self._close_h5()
        self._set_running(False)
        self._set_status(text, color)
        self.app.log_message(f"A-beta titration finished: {text}")

    def _schedule_poll(self):
        self._poll_id = self.after(POLL_MS, self._poll)

    def _poll(self):
        self._poll_id = None
        now = time.monotonic()
        if self.state == "configuring":
            if self._config_event.is_set():
                if self._config_ok:
                    self.state = "running"
                    self._deadline = now + BLOCK_TIMEOUT_S
                    if self.single_block:
                        self.app.send_cl_run_block()
                        self._set_status("calibration block running", "blue")
                    else:
                        self.app.send_cl_arm()
                        self._set_status("on-chip control armed", "green")
                else:
                    self._finish("config rejected", "red")
                    return
            elif now > self._deadline:
                self._finish("config timeout", "red")
                return
        elif self.state == "running" and self.single_block and now > self._deadline:
            self._finish("block timeout", "red")
            return

        if self.state != "idle":
            self._schedule_poll()

    def _plot_block_from_stream(self, amp_ua, auc_uv):
        x, valid = self.stream.snapshot()
        used_onsets = [pl["onset"] for pl in self.pulses
                       if pl.get("status") == ClPulseStatus.USED]
        if not used_onsets:
            used_onsets = [pl["onset"] for pl in self.pulses
                           if pl.get("status") in (ClPulseStatus.USED, ClPulseStatus.DISCARD_SETTLE)]
        epochs, _ = extract_epochs(x, valid, np.asarray(used_onsets[-self.params["n_reps"]:],
                                                        dtype=int), self.epoch_cfg)
        if epochs.size == 0:
            return
        try:
            result = analyze_block(epochs, self.epoch_cfg)
        except ValueError:
            return
        self._draw_epochs(result, amp_ua, auc_uv)

    def _draw_epochs(self, result, amp, auc_uv):
        ax = self.ax_epochs
        ax.clear()
        for ep in result.epochs_uv:
            ax.plot(result.t_ms, ep, color="0.75", linewidth=0.6)
        ax.plot(result.t_ms, result.mean_uv, color="k", linewidth=1.6,
                label=f"mean of {result.n_reps}")
        ax.axvspan(self.epoch_cfg.win_start_ms, self.epoch_cfg.win_end_ms, color="tab:green",
                   alpha=0.15, label="A-beta window")
        ax.axvline(0.0, color="tab:red", linewidth=0.8, linestyle="--")
        ax.set_xlabel("Time from stimulus onset (ms)")
        ax.set_ylabel("uV (input-referred)")
        ax.set_title(f"Block at {amp:.0f} uA: chip AUC {auc_uv:.2f} uV*ms")
        ax.legend(loc="upper right", fontsize=8)

    def _draw_trend(self):
        ax, ax_amp = self.ax_trend, self.ax_amp
        ax.clear()
        ax_amp.clear()
        ax.set_xlabel("Block")
        ax.set_ylabel("A-beta AUC (uV*ms)")
        ax_amp.set_ylabel("Amplitude (uA)", color="tab:blue")
        if self.params is None:
            return
        target = self.params["target_auc"]
        tol = self.params["tolerance_pct"] / 100.0
        ax.axhspan(target * (1 - tol), target * (1 + tol), color="tab:green", alpha=0.15)
        ax.axhline(target, color="tab:green", linewidth=1.0)
        if self.block_history:
            xs = [h["block_no"] for h in self.block_history]
            ax.plot(xs, [h["auc_uv"] for h in self.block_history], "ko-", label="AUC (chip)")
            ax_amp.step(xs, [abs(h["amp_ua"]) for h in self.block_history], where="mid",
                        color="tab:blue")

    def _open_h5(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = "abeta_block" if self.single_block else "abeta_onchip"
        self.session_basename = f"{prefix}_{timestamp}"
        self.h5 = h5py.File(f"{self.session_basename}.h5", "w")
        self.h5.attrs["fs_hz"] = FS_HZ
        self.h5.attrs["adc_uv_per_count"] = ADC_UV_PER_COUNT
        self.h5.attrs["on_chip"] = True
        self.app.log_message(f"Titration HDF5: {self.session_basename}.h5")

    def _export_block_csv(self):
        if not self.block_history or not self.session_basename:
            return
        path = f"{self.session_basename}.csv"
        target = self.params["target_auc"] if self.params else float("nan")
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=BLOCK_CSV_FIELDS)
                writer.writeheader()
                for h in self.block_history:
                    writer.writerow({
                        "block_no": h["block_no"],
                        "amp_ua": h["amp_ua"],
                        "next_amp_ua": h["next_ua"],
                        "auc_uv_ms": h["auc_uv"],
                        "auc_chip": h["auc_chip"],
                        "auc_raw": h["auc_raw"],
                        "noise_auc": h["noise_auc"],
                        "auc_sem": h["auc_sem"],
                        "error_frac": h["error_frac"],
                        "step": h["step"],
                        "n_reps": h["n_reps"],
                        "decision": h["decision"].name,
                        "converged": int(h["converged"]),
                        "target_auc_uv_ms": target,
                    })
            self.app.log_message(f"Titration CSV: {path}")
        except OSError as e:
            self.app.log_message(f"Titration CSV export failed: {e}")

    def _save_block_packet(self, pkt, auc_uv, amp_ua):
        if self.h5 is None:
            return
        g = self.h5.create_group(f"block_{pkt['block_no']:03d}")
        g.attrs["amp_ua"] = amp_ua
        g.attrs["auc_uv_ms"] = auc_uv
        g.attrs["auc_chip"] = pkt["auc"]
        g.attrs["decision"] = pkt["decision"].name
        g.attrs["next_amp_ua"] = D2B.ua_from_dac_code(pkt["next_amp_code"])
        x, valid = self.stream.snapshot()
        g.create_dataset("raw_counts", data=x, compression="gzip")
        g.create_dataset("raw_valid", data=valid, compression="gzip")
        self.h5.flush()

    def _close_h5(self):
        if self.h5 is not None:
            self.h5.close()
            self.h5 = None

    def _on_close(self):
        if self.state != "idle":
            self.abort()
        self.app.titration_window = None
        self.destroy()

    def note_pulse(self, pkt):
        self.pulses.append(pkt)
