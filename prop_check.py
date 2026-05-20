"""
Drone Design Tool
Clean, minimal GUI — tkinter only (no external dependencies)
Run: python drone_design_tool.py
"""

import tkinter as tk


# ─────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────

GRAVITY          = 9.81
PRACTICAL_FACTOR = 0.80
TWR_GOOD         = 2.0
TWR_MARGINAL     = 1.5


# ─────────────────────────────────────────────
#  Calculations  (pure functions)
# ─────────────────────────────────────────────

def calc_total_voltage(cell_v, cells):        return cell_v * cells
def calc_rpm(kv, voltage):                    return kv * voltage
def calc_power(voltage, current):             return voltage * current
def calc_total_current(motor_a, n, elec_a):  return motor_a * n + elec_a
def calc_weight(mass):                        return mass * GRAVITY
def calc_max_thrust(thrust_motor, n):         return thrust_motor * n
def calc_twr(max_thrust, weight):             return max_thrust / weight if weight else 0
def calc_hover_pct(weight, max_thrust):       return (weight / max_thrust * 100) if max_thrust else 0
def calc_per_motor(weight, n):                return weight / n if n else 0
def calc_flight_theo(mah, total_a):           return (mah / 1000 / total_a * 60) if total_a else 0
def calc_flight_prac(theo):                   return theo * PRACTICAL_FACTOR

def compute(inp):
    voltage     = calc_total_voltage(inp["cell_v"], inp["cells"])
    weight      = calc_weight(inp["mass"])
    total_a     = calc_total_current(inp["motor_a"], inp["motors"], inp["elec_a"])
    max_thrust  = calc_max_thrust(inp["thrust_motor"], inp["motors"])
    flight_theo = calc_flight_theo(inp["mah"], total_a)
    return {
        "voltage":     voltage,
        "rpm":         calc_rpm(inp["kv"], voltage),
        "power":       calc_power(voltage, inp["pack_a"]),
        "total_a":     total_a,
        "flight_theo": flight_theo,
        "flight_prac": calc_flight_prac(flight_theo),
        "weight":      weight,
        "max_thrust":  max_thrust,
        "twr":         calc_twr(max_thrust, weight),
        "hover_pct":   calc_hover_pct(weight, max_thrust),
        "per_motor":   calc_per_motor(weight, inp["motors"]),
    }


# ─────────────────────────────────────────────
#  Palette
# ─────────────────────────────────────────────

BG     = "#111318"
WHITE  = "#1c1f26"
BORDER = "#2e3240"
TEXT   = "#e8eaf0"
MUTED  = "#6b7280"
ACCENT = "#3b82f6"
GREEN  = "#22c55e"
AMBER  = "#f59e0b"
RED    = "#ef4444"


def clamp(v, lo=0, hi=100):
    return max(lo, min(hi, v))


# ─────────────────────────────────────────────
#  Reusable widgets
# ─────────────────────────────────────────────

class Card(tk.Frame):
    def __init__(self, parent, **kw):
        super().__init__(parent, bg=WHITE,
                         highlightbackground=BORDER,
                         highlightthickness=1, **kw)


class SectionLabel(tk.Label):
    def __init__(self, parent, text):
        super().__init__(parent, text=text.upper(),
                         bg=BG, fg=MUTED,
                         font=("Helvetica", 8, "bold"), anchor="w")


class FieldRow(tk.Frame):
    def __init__(self, parent, label, var, unit=""):
        super().__init__(parent, bg=WHITE)
        tk.Label(self, text=label, bg=WHITE, fg=TEXT,
                 font=("Helvetica", 10),
                 width=22, anchor="w").pack(side="left")
        self.entry = tk.Entry(self, textvariable=var, width=10,
                              relief="flat", bd=0,
                              bg="#0d0f14", fg=TEXT,
                              font=("Helvetica", 10),
                              insertbackground=ACCENT,
                              highlightthickness=1,
                              highlightbackground=BORDER,
                              highlightcolor=ACCENT)
        self.entry.pack(side="left", ipady=5, padx=(0, 6))
        if unit:
            tk.Label(self, text=unit, bg=WHITE, fg=MUTED,
                     font=("Helvetica", 9)).pack(side="left")

    def flash_error(self):
        self.entry.configure(highlightbackground=RED, highlightcolor=RED)

    def clear_error(self):
        self.entry.configure(highlightbackground=BORDER, highlightcolor=ACCENT)


class MetricCard(tk.Frame):
    def __init__(self, parent, label, unit):
        super().__init__(parent, bg=WHITE,
                         highlightbackground=BORDER,
                         highlightthickness=1,
                         width=130, height=90)
        self.pack_propagate(False)
        tk.Label(self, text=label.upper(), bg=WHITE, fg=MUTED,
                 font=("Helvetica", 7, "bold")).pack(anchor="w", padx=10, pady=(10, 0))
        self._val = tk.Label(self, text="—", bg=WHITE, fg=TEXT,
                             font=("Helvetica", 17, "bold"))
        self._val.pack(anchor="w", padx=10)
        tk.Label(self, text=unit, bg=WHITE, fg=MUTED,
                 font=("Helvetica", 8)).pack(anchor="w", padx=10)

    def set(self, value, colour=TEXT):
        self._val.configure(text=value, fg=colour)


class BarRow(tk.Frame):
    def __init__(self, parent, label):
        super().__init__(parent, bg=WHITE)
        top = tk.Frame(self, bg=WHITE)
        top.pack(fill="x", padx=12, pady=(10, 3))
        tk.Label(top, text=label, bg=WHITE, fg=TEXT,
                 font=("Helvetica", 9)).pack(side="left")
        self._lbl = tk.Label(top, text="—", bg=WHITE, fg=MUTED,
                              font=("Helvetica", 9))
        self._lbl.pack(side="right")
        track = tk.Frame(self, bg="#2e3240", height=5)
        track.pack(fill="x", padx=12, pady=(0, 12))
        track.pack_propagate(False)
        self._fill = tk.Frame(track, bg=ACCENT, height=5)
        self._fill.place(relx=0, rely=0, relwidth=0, relheight=1)

    def set(self, pct, colour=ACCENT):
        self._lbl.configure(text=f"{pct:.1f}%")
        self._fill.configure(bg=colour)
        self._fill.place(relwidth=clamp(pct) / 100)


class StatusRow(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=WHITE)
        self._dot = tk.Label(self, text="●", bg=WHITE, fg=MUTED,
                              font=("Helvetica", 10))
        self._dot.pack(side="left", padx=(12, 6), pady=7)
        self._msg = tk.Label(self, text="—", bg=WHITE, fg=TEXT,
                              font=("Helvetica", 9), anchor="w")
        self._msg.pack(side="left", fill="x", expand=True)
        self._val = tk.Label(self, text="", bg=WHITE, fg=MUTED,
                              font=("Helvetica", 9, "bold"))
        self._val.pack(side="right", padx=12)

    def set(self, msg, val, colour):
        self._dot.configure(fg=colour)
        self._msg.configure(text=msg)
        self._val.configure(text=val)


class Divider(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=BORDER, height=1)


# ─────────────────────────────────────────────
#  Application
# ─────────────────────────────────────────────

FIELD_DEFS = [
    ("Airframe & Motors", [
        ("Mass",               "mass",         "kg"),
        ("Number of motors",   "motors",       ""),
        ("Motor KV rating",    "kv",           "KV"),
        ("Max thrust / motor", "thrust_motor", "N"),
    ]),
    ("Battery Pack", [
        ("Cell count (S)",     "cells",   ""),
        ("Cell voltage",       "cell_v",  "V"),
        ("Capacity",           "mah",     "mAh"),
        ("Pack current draw",  "pack_a",  "A"),
    ]),
    ("Current Draw", [
        ("Current per motor",  "motor_a", "A"),
        ("Electronics",        "elec_a",  "A"),
    ]),
]

METRIC_DEFS = [
    ("Total voltage", "V"),
    ("Motor RPM",     "rpm"),
    ("Power",         "W"),
    ("Flight (theo)", "min"),
    ("Flight (80%)",  "min"),
    ("TWR",           "ratio"),
]

STATUS_LABELS = [
    "Thrust-to-weight ratio",
    "Hover throttle",
    "Practical flight time",
    "Current vs pack rating",
]


class DroneApp(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Drone Design Tool")
        self.configure(bg=BG)
        self.resizable(False, False)
        self._vars: dict[str, tk.StringVar] = {}
        self._rows: dict[str, FieldRow]     = {}
        self._build()

    # ── Layout ───────────────────────────────────────────────────────

    def _build(self):
        root = tk.Frame(self, bg=BG, padx=28, pady=24)
        root.pack()

        # Header
        tk.Label(root, text="Drone Design Tool", bg=BG, fg=TEXT,
                 font=("Helvetica", 18, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w")
        tk.Label(root, text="Performance & flight analysis",
                 bg=BG, fg=MUTED, font=("Helvetica", 10)).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(2, 20))

        # Columns
        left  = tk.Frame(root, bg=BG)
        right = tk.Frame(root, bg=BG)
        left.grid (row=2, column=0, sticky="n", padx=(0, 20))
        right.grid(row=2, column=1, sticky="n")

        self._build_inputs(left)
        self._build_results(right)

    def _build_inputs(self, parent):
        for section, fields in FIELD_DEFS:
            SectionLabel(parent, section).pack(anchor="w", pady=(14, 4))
            card = Card(parent)
            card.pack(fill="x")
            for i, (label, key, unit) in enumerate(fields):
                var = tk.StringVar()
                self._vars[key] = var
                row = FieldRow(card, label, var, unit)
                pad_t = 10 if i == 0 else 4
                pad_b = 10 if i == len(fields) - 1 else 4
                row.pack(fill="x", padx=12, pady=(pad_t, pad_b))
                self._rows[key] = row

        # Buttons
        btns = tk.Frame(parent, bg=BG)
        btns.pack(fill="x", pady=(18, 0))
        tk.Button(btns, text="  Calculate  ",
                  command=self._on_calculate,
                  bg=ACCENT, fg="white",
                  font=("Helvetica", 10, "bold"),
                  relief="flat", bd=0, cursor="hand2",
                  padx=10, pady=9,
                  activebackground="#1d4ed8",
                  activeforeground="white").pack(side="left", padx=(0, 8))
        tk.Button(btns, text="  Reset  ",
                  command=self._on_reset,
                  bg=WHITE, fg=MUTED,
                  font=("Helvetica", 10),
                  relief="flat", bd=0, cursor="hand2",
                  padx=10, pady=9,
                  highlightthickness=1,
                  highlightbackground=BORDER,
                  activebackground="#2e3240").pack(side="left")

    def _build_results(self, parent):
        # Metric cards
        SectionLabel(parent, "Results").pack(anchor="w", pady=(14, 4))
        metrics_card = Card(parent)
        metrics_card.pack(fill="x")
        grid = tk.Frame(metrics_card, bg=WHITE)
        grid.pack(padx=12, pady=12)
        self._metrics: list[MetricCard] = []
        for i, (label, unit) in enumerate(METRIC_DEFS):
            m = MetricCard(grid, label, unit)
            m.grid(row=i // 3, column=i % 3, padx=4, pady=4)
            self._metrics.append(m)

        # Bars
        SectionLabel(parent, "Load").pack(anchor="w", pady=(14, 4))
        bar_card = Card(parent)
        bar_card.pack(fill="x")
        self._hover_bar = BarRow(bar_card, "Hover throttle")
        self._hover_bar.pack(fill="x")
        Divider(bar_card).pack(fill="x", padx=12)
        self._motor_bar = BarRow(bar_card, "Per-motor load vs max")
        self._motor_bar.pack(fill="x")

        # Status
        SectionLabel(parent, "Status checks").pack(anchor="w", pady=(14, 4))
        status_card = Card(parent)
        status_card.pack(fill="x")
        self._status: list[StatusRow] = []
        for i, lbl in enumerate(STATUS_LABELS):
            s = StatusRow(status_card)
            s.pack(fill="x")
            s.set(lbl, "", MUTED)
            if i < len(STATUS_LABELS) - 1:
                Divider(status_card).pack(fill="x", padx=12)
            self._status.append(s)

    # ── Handlers ─────────────────────────────────────────────────────

    def _on_calculate(self):
        inp = self._validate()
        if inp:
            self._refresh(compute(inp), inp)

    def _on_reset(self):
        for var in self._vars.values():
            var.set("")
        for row in self._rows.values():
            row.clear_error()
        for m in self._metrics:
            m.set("—")
        self._hover_bar.set(0)
        self._motor_bar.set(0)
        for s, lbl in zip(self._status, STATUS_LABELS):
            s.set(lbl, "", MUTED)

    def _validate(self):
        INT_KEYS = {"cells", "motors"}
        parsed, ok = {}, True
        for key, var in self._vars.items():
            try:
                val = int(var.get()) if key in INT_KEYS else float(var.get())
                if val <= 0:
                    raise ValueError
                parsed[key] = val
                self._rows[key].clear_error()
            except ValueError:
                self._rows[key].flash_error()
                ok = False
        return parsed if ok else None

    def _refresh(self, r, inp):
        # Metric values
        metric_vals = [
            (f"{r['voltage']:.1f}",    TEXT),
            (f"{r['rpm']:,.0f}",        TEXT),
            (f"{r['power']:.0f}",       TEXT),
            (f"{r['flight_theo']:.1f}", TEXT),
            (f"{r['flight_prac']:.1f}",
             GREEN if r["flight_prac"] > 10 else AMBER if r["flight_prac"] > 5 else RED),
            (f"{r['twr']:.2f}",
             GREEN if r["twr"] >= TWR_GOOD else AMBER if r["twr"] >= TWR_MARGINAL else RED),
        ]
        for m, (val, clr) in zip(self._metrics, metric_vals):
            m.set(val, clr)

        # Bars
        h = r["hover_pct"]
        self._hover_bar.set(h, RED if h >= 70 else AMBER if h >= 50 else GREEN)

        ml = clamp((r["per_motor"] / inp["thrust_motor"]) * 100)
        self._motor_bar.set(ml, RED if ml >= 80 else AMBER if ml >= 60 else ACCENT)

        # Status rows
        fp, ta, pa = r["flight_prac"], r["total_a"], inp["pack_a"]
        checks = [
            (
                "TWR — Excellent (≥ 2:1)" if r["twr"] >= TWR_GOOD
                else "TWR — Marginal (≥ 1.5:1)" if r["twr"] >= TWR_MARGINAL
                else "TWR — Underpowered",
                f"{r['twr']:.2f}",
                GREEN if r["twr"] >= TWR_GOOD else AMBER if r["twr"] >= TWR_MARGINAL else RED,
            ),
            (
                "Hover — Efficient" if h < 50 else "Hover — High load" if h < 70 else "Hover — Very high!",
                f"{h:.1f}%",
                GREEN if h < 50 else AMBER if h < 70 else RED,
            ),
            (
                "Flight — Good endurance" if fp > 10 else "Flight — Moderate" if fp > 5 else "Flight — Short",
                f"{fp:.1f} min",
                GREEN if fp > 10 else AMBER if fp > 5 else RED,
            ),
            (
                "Current — Within limits" if ta < pa * 0.9 else "Current — Near limit" if ta < pa else "Current — Exceeds pack!",
                f"{ta:.1f} A",
                GREEN if ta < pa * 0.9 else AMBER if ta < pa else RED,
            ),
        ]
        for s, (msg, val, clr) in zip(self._status, checks):
            s.set(msg, val, clr)


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

if __name__ == "__main__":
    DroneApp().mainloop()