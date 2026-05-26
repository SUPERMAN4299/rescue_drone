"""
PID Simulator
Dark-themed tkinter GUI — no external dependencies
Run: python pid_simulator.py
"""

import tkinter as tk
import time


# ─────────────────────────────────────────────
#  PID Logic  (pure functions)
# ─────────────────────────────────────────────

def pid_step(target, current, prev_error, integral, kp, ki, kd):
    """One PID iteration. Returns (new_current, new_prev_error, new_integral, p, i, d)."""
    error      = target - current
    p          = kp * error
    integral  += error
    i          = ki * integral
    derivative = error - prev_error
    d          = kd * derivative
    output     = p + i + d
    new_current = current + output * 0.5
    return new_current, error, integral, p, i, d


def run_simulation(target, init_angle, kp, ki, kd, steps=60):
    """Run full simulation and return list of (step, angle, p, i, d) tuples."""
    current    = init_angle
    prev_error = 0
    integral   = 0
    history    = [(0, current, 0.0, 0.0, 0.0)]
    for s in range(1, steps + 1):
        current, prev_error, integral, p, i, d = pid_step(
            target, current, prev_error, integral, kp, ki, kd
        )
        history.append((s, current, p, i, d))
    return history


# ─────────────────────────────────────────────
#  Dark Palette
# ─────────────────────────────────────────────

BG      = "#111318"
SURFACE = "#1c1f26"
BORDER  = "#2e3240"
TEXT    = "#e8eaf0"
MUTED   = "#6b7280"
ACCENT  = "#3b82f6"
GREEN   = "#22c55e"
AMBER   = "#f59e0b"
RED     = "#ef4444"
BLUE    = "#3b82f6"
TARGET_CLR = "#22c55e"


def clamp(v, lo=0, hi=100):
    return max(lo, min(hi, v))


# ─────────────────────────────────────────────
#  Reusable Widgets
# ─────────────────────────────────────────────

class Card(tk.Frame):
    def __init__(self, parent, **kw):
        super().__init__(parent, bg=SURFACE,
                         highlightbackground=BORDER,
                         highlightthickness=1, **kw)


class SectionLabel(tk.Label):
    def __init__(self, parent, text):
        super().__init__(parent, text=text.upper(),
                         bg=BG, fg=MUTED,
                         font=("Helvetica", 8, "bold"), anchor="w")


class SliderRow(tk.Frame):
    """Label — Scale — Value label in one row."""
    def __init__(self, parent, label, var, from_, to, resolution, fmt="{:.2f}", **kw):
        super().__init__(parent, bg=SURFACE, **kw)
        self._fmt = fmt
        tk.Label(self, text=label, bg=SURFACE, fg=MUTED,
                 font=("Helvetica", 9), width=18, anchor="w").pack(side="left")
        self._val_lbl = tk.Label(self, text=fmt.format(var.get()),
                                  bg=SURFACE, fg=TEXT,
                                  font=("Courier New", 9), width=7, anchor="e")
        self._val_lbl.pack(side="right")
        scale = tk.Scale(self, variable=var, from_=from_, to=to,
                         resolution=resolution, orient="horizontal",
                         showvalue=False, bg=SURFACE, fg=TEXT,
                         troughcolor=BORDER, activebackground=ACCENT,
                         highlightthickness=0, bd=0, sliderrelief="flat",
                         sliderlength=14, width=6,
                         command=lambda v: self._val_lbl.configure(
                             text=fmt.format(float(v))))
        scale.pack(side="left", fill="x", expand=True, padx=(6, 6))


class MetricCard(tk.Frame):
    def __init__(self, parent, label, unit=""):
        super().__init__(parent, bg=SURFACE,
                         highlightbackground=BORDER,
                         highlightthickness=1,
                         width=120, height=78)
        self.pack_propagate(False)
        tk.Label(self, text=label.upper(), bg=SURFACE, fg=MUTED,
                 font=("Helvetica", 7, "bold")).pack(anchor="w", padx=10, pady=(9, 0))
        self._val = tk.Label(self, text="—", bg=SURFACE, fg=TEXT,
                             font=("Helvetica", 16, "bold"))
        self._val.pack(anchor="w", padx=10)
        if unit:
            tk.Label(self, text=unit, bg=SURFACE, fg=MUTED,
                     font=("Helvetica", 8)).pack(anchor="w", padx=10)

    def set(self, value, colour=TEXT):
        self._val.configure(text=value, fg=colour)


class TermCard(tk.Frame):
    def __init__(self, parent, label, colour):
        super().__init__(parent, bg=SURFACE,
                         highlightbackground=colour,
                         highlightthickness=1,
                         width=120, height=62)
        self.pack_propagate(False)
        tk.Label(self, text=label, bg=SURFACE, fg=colour,
                 font=("Helvetica", 8, "bold")).pack(anchor="w", padx=10, pady=(7, 0))
        self._val = tk.Label(self, text="—", bg=SURFACE, fg=TEXT,
                             font=("Courier New", 13, "bold"))
        self._val.pack(anchor="w", padx=10)

    def set(self, value):
        self._val.configure(text=value)


class MiniChart(tk.Canvas):
    """Pure-tkinter line chart — no matplotlib needed."""

    PAD_L, PAD_R, PAD_T, PAD_B = 46, 14, 14, 28

    def __init__(self, parent, **kw):
        super().__init__(parent, bg=SURFACE,
                         highlightthickness=0, **kw)
        self._data:  list[float] = []
        self._target: float      = 0.0
        self.bind("<Configure>", lambda e: self._redraw())

    def update_data(self, angles: list[float], target: float):
        self._data   = angles
        self._target = target
        self._redraw()

    def _redraw(self):
        self.delete("all")
        W = self.winfo_width()
        H = self.winfo_height()
        if W < 10 or H < 10 or not self._data:
            return

        pl = self.PAD_L
        pr = self.PAD_R
        pt = self.PAD_T
        pb = self.PAD_B
        iw = W - pl - pr   # inner width
        ih = H - pt - pb   # inner height

        mn = min(min(self._data), self._target) - 3
        mx = max(max(self._data), self._target) + 3
        rng = mx - mn if mx != mn else 1

        def sx(i):   return pl + i / max(len(self._data) - 1, 1) * iw
        def sy(v):   return pt + (1 - (v - mn) / rng) * ih

        # Grid lines + y-axis labels
        for tick in range(5):
            v   = mn + tick / 4 * rng
            y   = sy(v)
            self.create_line(pl, y, W - pr, y,
                             fill=BORDER, width=1)
            self.create_text(pl - 4, y, text=f"{v:.0f}°",
                             anchor="e", fill=MUTED,
                             font=("Helvetica", 7))

        # X-axis label
        self.create_text(pl + iw // 2, H - 6,
                         text="step",
                         fill=MUTED, font=("Helvetica", 7))

        # Target dashed line
        ty = sy(self._target)
        dash_gap = 6
        x = pl
        while x < W - pr:
            self.create_line(x, ty, min(x + dash_gap, W - pr), ty,
                             fill=TARGET_CLR, width=1)
            x += dash_gap * 2

        # Angle line
        if len(self._data) >= 2:
            pts = [coord
                   for i, v in enumerate(self._data)
                   for coord in (sx(i), sy(v))]
            self.create_line(*pts, fill=BLUE, width=2,
                             smooth=True, joinstyle="round")

        # Current-position dot
        if self._data:
            cx = sx(len(self._data) - 1)
            cy = sy(self._data[-1])
            self.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                             fill=BLUE, outline=SURFACE, width=2)

        # Legend
        lx = W - pr - 4
        self.create_line(lx - 28, H - 8, lx - 16, H - 8,
                         fill=BLUE, width=2)
        self.create_text(lx, H - 8, text="angle",
                         anchor="e", fill=MUTED, font=("Helvetica", 7))

        self.create_line(pl + 4, H - 8, pl + 16, H - 8,
                         fill=TARGET_CLR, width=1, dash=(3, 3))
        self.create_text(pl + 56, H - 8, text="target",
                         anchor="e", fill=MUTED, font=("Helvetica", 7))


class Divider(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=BORDER, height=1)


# ─────────────────────────────────────────────
#  Main Application
# ─────────────────────────────────────────────

class PIDApp(tk.Tk):

    STEPS       = 60
    INTERVAL_MS = 60   # ms between animation frames

    def __init__(self):
        super().__init__()
        self.title("PID Simulator")
        self.configure(bg=BG)
        self.resizable(False, False)

        # State
        self._kp      = tk.DoubleVar(value=1.5)
        self._ki      = tk.DoubleVar(value=0.02)
        self._kd      = tk.DoubleVar(value=0.8)
        self._init    = tk.DoubleVar(value=20.0)
        self._target  = tk.DoubleVar(value=0.0)

        self._running    = False
        self._step       = 0
        self._current    = 20.0
        self._prev_error = 0.0
        self._integral   = 0.0
        self._angles: list[float] = []
        self._after_id   = None

        # Trace sliders → auto-reset
        for var in (self._kp, self._ki, self._kd, self._init, self._target):
            var.trace_add("write", lambda *_: self._on_param_change())

        self._build()
        self._reset_state()

    # ── Build UI ─────────────────────────────────────────────────────

    def _build(self):
        root = tk.Frame(self, bg=BG, padx=24, pady=20)
        root.pack()

        # Header
        tk.Label(root, text="PID Simulator", bg=BG, fg=TEXT,
                 font=("Helvetica", 18, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w")
        tk.Label(root, text="u(t) = Kp·e(t) + Ki·∫e dt + Kd·de/dt",
                 bg=BG, fg=MUTED, font=("Courier New", 9)).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(2, 18))

        left  = tk.Frame(root, bg=BG)
        right = tk.Frame(root, bg=BG)
        left.grid (row=2, column=0, sticky="n", padx=(0, 18))
        right.grid(row=2, column=1, sticky="n")

        self._build_controls(left)
        self._build_output(right)

    def _build_controls(self, parent):
        # PID gains
        SectionLabel(parent, "PID Gains").pack(anchor="w", pady=(0, 4))
        gains_card = Card(parent)
        gains_card.pack(fill="x")
        for label, var, lo, hi, res, fmt in [
            ("Kp  proportional", self._kp,   0.0, 5.0,  0.05, "{:.2f}"),
            ("Ki  integral",     self._ki,   0.0, 0.5,  0.005,"{:.3f}"),
            ("Kd  derivative",   self._kd,   0.0, 5.0,  0.05, "{:.2f}"),
        ]:
            r = SliderRow(gains_card, label, var, lo, hi, res, fmt)
            r.pack(fill="x", padx=12, pady=5)

        # Initial conditions
        SectionLabel(parent, "Initial Conditions").pack(anchor="w", pady=(16, 4))
        cond_card = Card(parent)
        cond_card.pack(fill="x")
        for label, var, lo, hi in [
            ("Initial angle  (°)", self._init,   -60, 60),
            ("Target angle   (°)", self._target, -60, 60),
        ]:
            r = SliderRow(cond_card, label, var, lo, hi, 1, "{:.0f}°")
            r.pack(fill="x", padx=12, pady=5)

        # Buttons
        btns = tk.Frame(parent, bg=BG)
        btns.pack(fill="x", pady=(16, 0))
        self._run_btn = tk.Button(
            btns, text="  Run  ",
            command=self._on_run,
            bg=ACCENT, fg="white",
            font=("Helvetica", 10, "bold"),
            relief="flat", bd=0, cursor="hand2",
            padx=10, pady=8,
            activebackground="#2563eb",
            activeforeground="white")
        self._run_btn.pack(side="left", padx=(0, 8))

        tk.Button(
            btns, text="  Reset  ",
            command=self._on_reset,
            bg=SURFACE, fg=MUTED,
            font=("Helvetica", 10),
            relief="flat", bd=0, cursor="hand2",
            padx=10, pady=8,
            highlightthickness=1,
            highlightbackground=BORDER,
            activebackground=BORDER).pack(side="left")

        # Step counter
        self._step_lbl = tk.Label(parent, text="Step 0 / 60",
                                   bg=BG, fg=MUTED,
                                   font=("Helvetica", 8))
        self._step_lbl.pack(anchor="w", pady=(10, 0))

    def _build_output(self, parent):
        # Metric cards
        SectionLabel(parent, "State").pack(anchor="w", pady=(0, 4))
        metrics_card = Card(parent)
        metrics_card.pack(fill="x")
        grid = tk.Frame(metrics_card, bg=SURFACE)
        grid.pack(padx=10, pady=10)

        self._m_angle  = MetricCard(grid, "Angle",   "°")
        self._m_target = MetricCard(grid, "Target",  "°")
        self._m_error  = MetricCard(grid, "Error",   "°")
        self._m_output = MetricCard(grid, "Output",  "")

        self._m_angle .grid(row=0, column=0, padx=4, pady=4)
        self._m_target.grid(row=0, column=1, padx=4, pady=4)
        self._m_error .grid(row=0, column=2, padx=4, pady=4)
        self._m_output.grid(row=0, column=3, padx=4, pady=4)

        # PID term cards
        SectionLabel(parent, "PID Terms").pack(anchor="w", pady=(14, 4))
        terms_card = Card(parent)
        terms_card.pack(fill="x")
        tgrid = tk.Frame(terms_card, bg=SURFACE)
        tgrid.pack(padx=10, pady=10)

        self._t_p = TermCard(tgrid, "P  proportional", ACCENT)
        self._t_i = TermCard(tgrid, "I  integral",     AMBER)
        self._t_d = TermCard(tgrid, "D  derivative",   GREEN)
        self._t_p.grid(row=0, column=0, padx=4, pady=0)
        self._t_i.grid(row=0, column=1, padx=4, pady=0)
        self._t_d.grid(row=0, column=2, padx=4, pady=0)

        # Chart
        SectionLabel(parent, "Angle over Time").pack(anchor="w", pady=(14, 4))
        self._chart = MiniChart(parent, width=500, height=210)
        self._chart.pack(fill="x")

    # ── State management ─────────────────────────────────────────────

    def _reset_state(self):
        self._step       = 0
        self._current    = self._init.get()
        self._prev_error = 0.0
        self._integral   = 0.0
        self._angles     = [self._current]
        self._refresh_ui(p=0, i=0, d=0, output=0)
        self._chart.update_data(self._angles, self._target.get())
        self._step_lbl.configure(text=f"Step 0 / {self.STEPS}")

    def _refresh_ui(self, p, i, d, output):
        err = self._target.get() - self._current
        err_colour = GREEN if abs(err) < 0.5 else AMBER if abs(err) < 5 else RED
        self._m_angle .set(f"{self._current:.2f}°")
        self._m_target.set(f"{self._target.get():.1f}°")
        self._m_error .set(f"{err:.2f}°", err_colour)
        self._m_output.set(f"{output:.3f}")
        self._t_p.set(f"{p:+.3f}")
        self._t_i.set(f"{i:+.3f}")
        self._t_d.set(f"{d:+.3f}")

    # ── Handlers ─────────────────────────────────────────────────────

    def _on_param_change(self):
        if self._running:
            self._stop()
        self._reset_state()

    def _on_run(self):
        if self._running:
            self._stop()
        else:
            self._start()

    def _on_reset(self):
        self._stop()
        self._reset_state()

    def _start(self):
        if self._step >= self.STEPS:
            self._reset_state()
        self._running = True
        self._run_btn.configure(text="  Pause  ", bg="#374151")
        self._tick()

    def _stop(self):
        self._running = False
        self._run_btn.configure(text="  Run  ", bg=ACCENT)
        if self._after_id:
            self.after_cancel(self._after_id)
            self._after_id = None

    def _tick(self):
        if not self._running or self._step >= self.STEPS:
            self._stop()
            return

        self._current, self._prev_error, self._integral, p, i, d = pid_step(
            self._target.get(),
            self._current,
            self._prev_error,
            self._integral,
            self._kp.get(),
            self._ki.get(),
            self._kd.get(),
        )
        output = p + i + d
        self._step += 1
        self._angles.append(self._current)

        self._refresh_ui(p, i, d, output)
        self._chart.update_data(self._angles, self._target.get())
        self._step_lbl.configure(text=f"Step {self._step} / {self.STEPS}")

        self._after_id = self.after(self.INTERVAL_MS, self._tick)


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────

if __name__ == "__main__":
    PIDApp().mainloop()