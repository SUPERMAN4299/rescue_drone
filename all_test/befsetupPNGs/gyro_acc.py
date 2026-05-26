"""
MPU6050 Sensor Fusion Simulator
Dark-themed tkinter GUI — no external dependencies
Run: python mpu6050_simulator.py
"""

import tkinter as tk
import random
import math


# ─────────────────────────────────────────────
#  Sensor Fusion Logic  (pure functions)
# ─────────────────────────────────────────────

def gyro_step(angle: float, gyro_rate: float, dt: float) -> float:
    """Integrate gyroscope rate to get angle."""
    return angle + gyro_rate * dt


def complementary_filter(gyro_angle: float, accel_angle: float,
                          alpha: float = 0.98) -> float:
    """Fuse gyro + accelerometer with complementary filter."""
    return alpha * gyro_angle + (1 - alpha) * accel_angle


def simulate_sensors(gyro_range: float = 2.0,
                     accel_noise: float = 5.0) -> tuple[float, float]:
    """Return (gyro_rate, accel_angle) simulated readings."""
    gyro_rate   = random.uniform(-gyro_range, gyro_range)
    accel_angle = random.uniform(-accel_noise, accel_noise)
    return gyro_rate, accel_angle


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
PURPLE  = "#a78bfa"
TEAL    = "#2dd4bf"


def clamp(v, lo, hi):
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


class MetricCard(tk.Frame):
    def __init__(self, parent, label, unit="", accent=TEXT):
        super().__init__(parent, bg=SURFACE,
                         highlightbackground=accent,
                         highlightthickness=1,
                         width=128, height=74)
        self.pack_propagate(False)
        tk.Label(self, text=label.upper(), bg=SURFACE, fg=accent,
                 font=("Helvetica", 7, "bold")).pack(anchor="w", padx=10, pady=(8, 0))
        self._val = tk.Label(self, text="—", bg=SURFACE, fg=TEXT,
                             font=("Courier New", 16, "bold"))
        self._val.pack(anchor="w", padx=10)
        if unit:
            tk.Label(self, text=unit, bg=SURFACE, fg=MUTED,
                     font=("Helvetica", 8)).pack(anchor="w", padx=10)

    def set(self, value, colour=TEXT):
        self._val.configure(text=value, fg=colour)


class SliderRow(tk.Frame):
    def __init__(self, parent, label, var, from_, to, resolution, fmt="{:.2f}"):
        super().__init__(parent, bg=SURFACE)
        tk.Label(self, text=label, bg=SURFACE, fg=MUTED,
                 font=("Helvetica", 9), width=20, anchor="w").pack(side="left")
        self._lbl = tk.Label(self, text=fmt.format(var.get()),
                              bg=SURFACE, fg=TEXT,
                              font=("Courier New", 9), width=6, anchor="e")
        self._lbl.pack(side="right")
        tk.Scale(self, variable=var, from_=from_, to=to,
                 resolution=resolution, orient="horizontal",
                 showvalue=False, bg=SURFACE, fg=TEXT,
                 troughcolor=BORDER, activebackground=ACCENT,
                 highlightthickness=0, bd=0, sliderrelief="flat",
                 sliderlength=14, width=6,
                 command=lambda v: self._lbl.configure(
                     text=fmt.format(float(v)))).pack(
            side="left", fill="x", expand=True, padx=(6, 6))


class AngleDial(tk.Canvas):
    """A circular gauge showing the current fused angle."""

    def __init__(self, parent, size=160, **kw):
        super().__init__(parent, width=size, height=size,
                         bg=SURFACE, highlightthickness=0, **kw)
        self._size  = size
        self._angle = 0.0
        self.bind("<Configure>", lambda _: self._draw())

    def set_angle(self, angle: float):
        self._angle = angle
        self._draw()

    def _draw(self):
        self.delete("all")
        s   = self._size
        cx  = s // 2
        cy  = s // 2
        r   = s // 2 - 14
        ri  = r - 10

        # Outer ring
        self.create_oval(cx - r, cy - r, cx + r, cy + r,
                         outline=BORDER, width=2)
        # Inner ring
        self.create_oval(cx - ri, cy - ri, cx + ri, cy + ri,
                         outline=BORDER, width=1)

        # Tick marks
        for deg in range(0, 360, 30):
            rad      = math.radians(deg - 90)
            x1 = cx + (r - 2) * math.cos(rad)
            y1 = cy + (r - 2) * math.sin(rad)
            x2 = cx + (r - 8) * math.cos(rad)
            y2 = cy + (r - 8) * math.sin(rad)
            self.create_line(x1, y1, x2, y2, fill=BORDER, width=1)

        # Degree labels at cardinal points
        for deg, lbl in [(0, "0°"), (90, "90°"), (180, "180°"), (270, "-90°")]:
            rad = math.radians(deg - 90)
            lx  = cx + (r - 18) * math.cos(rad)
            ly  = cy + (r - 18) * math.sin(rad)
            self.create_text(lx, ly, text=lbl, fill=MUTED,
                             font=("Helvetica", 6))

        # Needle (fused angle)
        a_rad   = math.radians(self._angle - 90)
        nx      = cx + (ri - 6) * math.cos(a_rad)
        ny      = cy + (ri - 6) * math.sin(a_rad)
        # Shadow needle
        self.create_line(cx, cy, nx, ny, fill=BORDER, width=4,
                         capstyle="round")
        # Main needle
        self.create_line(cx, cy, nx, ny, fill=ACCENT, width=2,
                         capstyle="round")
        # Centre dot
        self.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                         fill=ACCENT, outline=SURFACE, width=2)
        # Angle text
        self.create_text(cx, cy + ri - 6,
                         text=f"{self._angle:.1f}°",
                         fill=TEXT, font=("Courier New", 9, "bold"))


class MiniChart(tk.Canvas):
    """Pure-tkinter multi-line chart."""

    PAD_L, PAD_R, PAD_T, PAD_B = 44, 12, 12, 24
    MAX_POINTS = 60

    def __init__(self, parent, **kw):
        super().__init__(parent, bg=SURFACE,
                         highlightthickness=0, **kw)
        self._series: dict[str, list] = {
            "gyro":   [],
            "accel":  [],
            "fused":  [],
        }
        self._colours = {
            "gyro":  PURPLE,
            "accel": AMBER,
            "fused": ACCENT,
        }
        self.bind("<Configure>", lambda _: self._redraw())

    def push(self, gyro_angle, accel_angle, fused_angle):
        for key, val in [("gyro", gyro_angle),
                         ("accel", accel_angle),
                         ("fused", fused_angle)]:
            self._series[key].append(val)
            if len(self._series[key]) > self.MAX_POINTS:
                self._series[key].pop(0)
        self._redraw()

    def _redraw(self):
        self.delete("all")
        W = self.winfo_width()
        H = self.winfo_height()
        if W < 10 or H < 10:
            return

        pl, pr, pt, pb = self.PAD_L, self.PAD_R, self.PAD_T, self.PAD_B
        iw = W - pl - pr
        ih = H - pt - pb

        all_vals = [v for s in self._series.values() for v in s]
        if not all_vals:
            return

        mn  = min(all_vals) - 2
        mx  = max(all_vals) + 2
        rng = mx - mn if mx != mn else 1

        n = max(len(s) for s in self._series.values())

        def sx(i): return pl + (i / max(n - 1, 1)) * iw
        def sy(v): return pt + (1 - (v - mn) / rng) * ih

        # Grid
        for tick in range(5):
            v = mn + tick / 4 * rng
            y = sy(v)
            self.create_line(pl, y, W - pr, y, fill=BORDER, width=1)
            self.create_text(pl - 4, y, text=f"{v:.0f}°",
                             anchor="e", fill=MUTED, font=("Helvetica", 7))

        # Zero line
        if mn < 0 < mx:
            self.create_line(pl, sy(0), W - pr, sy(0),
                             fill=BORDER, width=1, dash=(4, 4))

        # Series
        for key, data in self._series.items():
            if len(data) < 2:
                continue
            colour = self._colours[key]
            pts = [c for i, v in enumerate(data) for c in (sx(i), sy(v))]
            self.create_line(*pts, fill=colour, width=2,
                             smooth=True, joinstyle="round")

        # Legend
        lx, ly = pl + 4, H - 6
        for key, label in [("fused", "fused"), ("gyro", "gyro"), ("accel", "accel")]:
            c = self._colours[key]
            self.create_line(lx, ly, lx + 14, ly, fill=c, width=2)
            lx += 16
            self.create_text(lx, ly, text=label, anchor="w",
                             fill=MUTED, font=("Helvetica", 7))
            lx += 38


class Divider(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=BORDER, height=1)


# ─────────────────────────────────────────────
#  Main Application
# ─────────────────────────────────────────────

class MPUApp(tk.Tk):

    INTERVAL_MS = 120

    def __init__(self):
        super().__init__()
        self.title("MPU6050 Sensor Fusion")
        self.configure(bg=BG)
        self.resizable(False, False)

        # Params
        self._alpha      = tk.DoubleVar(value=0.98)
        self._gyro_range = tk.DoubleVar(value=2.0)
        self._accel_noise = tk.DoubleVar(value=5.0)
        self._dt         = tk.DoubleVar(value=0.1)

        # State
        self._angle      = 0.0
        self._step       = 0
        self._running    = False
        self._after_id   = None

        self._build()

    # ── Build UI ─────────────────────────────────────────────────────

    def _build(self):
        root = tk.Frame(self, bg=BG, padx=24, pady=20)
        root.pack()

        # Header
        tk.Label(root, text="MPU6050 Sensor Fusion",
                 bg=BG, fg=TEXT,
                 font=("Helvetica", 18, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w")
        tk.Label(root,
                 text="Complementary filter:  angle = α·(angle + ω·dt) + (1−α)·accel",
                 bg=BG, fg=MUTED,
                 font=("Courier New", 9)).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(2, 18))

        left  = tk.Frame(root, bg=BG)
        right = tk.Frame(root, bg=BG)
        left.grid (row=2, column=0, sticky="n", padx=(0, 18))
        right.grid(row=2, column=1, sticky="n")

        self._build_left(left)
        self._build_right(right)

    def _build_left(self, parent):
        # Dial
        SectionLabel(parent, "Fused Angle").pack(anchor="w", pady=(0, 4))
        dial_card = Card(parent)
        dial_card.pack(fill="x")
        dial_wrap = tk.Frame(dial_card, bg=SURFACE)
        dial_wrap.pack(pady=14)
        self._dial = AngleDial(dial_wrap, size=170)
        self._dial.pack()

        # Metrics
        SectionLabel(parent, "Readings").pack(anchor="w", pady=(16, 4))
        m_card = Card(parent)
        m_card.pack(fill="x")
        mg = tk.Frame(m_card, bg=SURFACE)
        mg.pack(padx=10, pady=10)

        self._m_gyro_rate  = MetricCard(mg, "Gyro rate",  "°/s",  PURPLE)
        self._m_gyro_angle = MetricCard(mg, "Gyro angle", "°",    PURPLE)
        self._m_accel      = MetricCard(mg, "Accel angle","°",    AMBER)
        self._m_fused      = MetricCard(mg, "Fused angle","°",    ACCENT)

        self._m_gyro_rate .grid(row=0, column=0, padx=4, pady=4)
        self._m_gyro_angle.grid(row=0, column=1, padx=4, pady=4)
        self._m_accel     .grid(row=1, column=0, padx=4, pady=4)
        self._m_fused     .grid(row=1, column=1, padx=4, pady=4)

        # Step label
        self._step_lbl = tk.Label(parent, text="Step 0",
                                   bg=BG, fg=MUTED,
                                   font=("Helvetica", 8))
        self._step_lbl.pack(anchor="w", pady=(10, 0))

    def _build_right(self, parent):
        # Chart
        SectionLabel(parent, "Angle History").pack(anchor="w", pady=(0, 4))
        self._chart = MiniChart(parent, width=440, height=200)
        self._chart.pack(fill="x")

        # Parameters
        SectionLabel(parent, "Parameters").pack(anchor="w", pady=(16, 4))
        p_card = Card(parent)
        p_card.pack(fill="x")
        for label, var, lo, hi, res, fmt in [
            ("Alpha  (gyro trust)",   self._alpha,       0.80, 0.99, 0.01, "{:.2f}"),
            ("Gyro range  (°/s)",     self._gyro_range,  0.5,  10.0, 0.5,  "{:.1f}"),
            ("Accel noise  (°)",      self._accel_noise, 0.5,  20.0, 0.5,  "{:.1f}"),
            ("dt  (time step)",       self._dt,          0.01, 0.5,  0.01, "{:.2f}"),
        ]:
            r = SliderRow(p_card, label, var, lo, hi, res, fmt)
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

    # ── Simulation loop ───────────────────────────────────────────────

    def _tick(self):
        if not self._running:
            return

        gyro_rate, accel_angle = simulate_sensors(
            self._gyro_range.get(), self._accel_noise.get()
        )
        gyro_angle  = gyro_step(self._angle, gyro_rate, self._dt.get())
        fused       = complementary_filter(gyro_angle, accel_angle,
                                           self._alpha.get())
        self._angle = fused
        self._step += 1

        # Update widgets
        self._dial.set_angle(fused % 360)
        self._m_gyro_rate .set(f"{gyro_rate:.2f}")
        self._m_gyro_angle.set(f"{gyro_angle:.2f}")
        self._m_accel     .set(f"{accel_angle:.2f}")
        self._m_fused     .set(f"{fused:.2f}")
        self._chart.push(gyro_angle, accel_angle, fused)
        self._step_lbl.configure(text=f"Step {self._step}")

        self._after_id = self.after(self.INTERVAL_MS, self._tick)

    # ── Handlers ─────────────────────────────────────────────────────

    def _on_run(self):
        if self._running:
            self._stop()
        else:
            self._start()

    def _on_reset(self):
        self._stop()
        self._angle = 0.0
        self._step  = 0
        self._step_lbl.configure(text="Step 0")
        self._dial.set_angle(0)
        self._chart._series = {"gyro": [], "accel": [], "fused": []}
        self._chart._redraw()
        for m in (self._m_gyro_rate, self._m_gyro_angle,
                  self._m_accel, self._m_fused):
            m.set("—")

    def _start(self):
        self._running = True
        self._run_btn.configure(text="  Pause  ", bg="#374151")
        self._tick()

    def _stop(self):
        self._running = False
        self._run_btn.configure(text="  Run  ", bg=ACCENT)
        if self._after_id:
            self.after_cancel(self._after_id)
            self._after_id = None


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────

if __name__ == "__main__":
    MPUApp().mainloop()