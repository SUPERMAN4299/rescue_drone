"""
drone_launcher.py — GUI Launcher for the Drone Navigation Stack
================================================================
Configure and launch analysing_cap.py without touching the source file.

Features:
  • Runtime Mode selector (Safe Test / Simulation / Real Flight / Debug / Low Power)
  • Serial port input (COM5 / /dev/ttyUSB0 / etc.)
  • Camera stream URL input
  • Serial baud rate selector
  • One-click Launch / Stop
  • Live console output inside the window
  • Config save/load (drone_config.json)
  • Built-in Serial Monitor with quick-command buttons (ARM, DISARM, STATUS…)
  • Config Preview tab showing exactly what lines get patched

Usage:
    python drone_launcher.py
"""

import json
import os
import platform
import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

# ── Constants ──────────────────────────────────────────────────────────────────

CONFIG_FILE  = "drone_config.json"
SCRIPT_NAME  = "analysing_cap.py"
TITLE        = "Drone Navigation Launcher"

BG       = "#0f1117"
PANEL    = "#1a1d27"
ACCENT   = "#4f8ef7"
ACCENT2  = "#34c98d"
DANGER   = "#e05252"
WARNING  = "#f7a94f"
TEXT     = "#e8eaf0"
TEXT_DIM = "#7a7f94"
BORDER   = "#2a2d3e"
MONO     = ("Consolas", 10) if platform.system() == "Windows" else ("Menlo", 10)

RUNTIME_MODES = [
    "SAFE_TEST_MODE",
    "SIMULATION_MODE",
    "REAL_FLIGHT_MODE",
    "DEBUG_MODE",
    "LOW_POWER_MODE",
]
MODE_HINTS = {
    "SAFE_TEST_MODE"  : "Reduced PWM, no fast forward — safe for bench tests",
    "SIMULATION_MODE" : "No serial, virtual sensors, full HUD only",
    "REAL_FLIGHT_MODE": "⚠ Live serial + full power — production flight",
    "DEBUG_MODE"      : "Verbose logs, extra HUD overlay fields",
    "LOW_POWER_MODE"  : "Frame-skip, lower inference resolution",
}
BAUD_RATES = ["9600", "19200", "57600", "115200", "230400"]

DEFAULT_CFG = {
    "script_path": SCRIPT_NAME,
    "mode"       : "SAFE_TEST_MODE",
    "serial_port": "COM5",
    "baud"       : "115200",
    "stream_url" : "http://192.168.1.2:8080/video",
    "model"      : "yolov8n.pt",
    "conf"       : "0.30",
    "imgsz"      : "416",
}

# ── Config helpers ─────────────────────────────────────────────────────────────

def load_config() -> dict:
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE) as f:
                saved = json.load(f)
            cfg = dict(DEFAULT_CFG)
            cfg.update(saved)
            return cfg
        except Exception:
            pass
    return dict(DEFAULT_CFG)


def save_config(cfg: dict) -> None:
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        print(f"[Config] Save failed: {e}")


def patch_and_write(src_path: str, cfg: dict) -> str:
    """
    Read the original script, patch the seven configurable lines in-memory,
    write to a temp file, and return its path.
    The original file is NEVER modified.
    """
    if not os.path.exists(src_path):
        raise FileNotFoundError(f"Script not found: {src_path}")

    with open(src_path, "r", encoding="utf-8") as f:
        code = f.read()

    patches = [
        # (pattern, replacement)
        (r'^(ACTIVE_MODE\s*:\s*RuntimeMode\s*=\s*)RuntimeMode\.\w+',
         rf'\g<1>RuntimeMode.{cfg["mode"]}'),

        (r'(serial_port\s*:\s*str\s*=\s*)[\'"].*?[\'"]',
         rf'\g<1>"{cfg["serial_port"]}"'),

        (r'(serial_baud\s*:\s*int\s*=\s*)\d+',
         rf'\g<1>{cfg["baud"]}'),

        (r'(stream_url\s*=\s*)[f]?["\'].*?["\']',
         rf'\g<1>"{cfg["stream_url"]}"'),

        (r'(model\s*:\s*str\s*=\s*)["\'].*?["\']',
         rf'\g<1>"{cfg["model"]}"'),

        (r'(conf\s*:\s*float\s*=\s*)\d+\.\d+',
         rf'\g<1>{cfg["conf"]}'),

        (r'(imgsz\s*:\s*int\s*=\s*)\d+',
         rf'\g<1>{cfg["imgsz"]}'),
    ]

    flags_map = [re.MULTILINE, 0, 0, 0, 0, 0, 0]
    counts    = [0, 1, 1, 1, 1, 1, 1]

    for (pat, repl), flags, cnt in zip(patches, flags_map, counts):
        code = re.sub(pat, repl, code, count=cnt, flags=flags)

    tmp = "_drone_launcher_run.py"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(code)
    return tmp


# ── Main Application ───────────────────────────────────────────────────────────

class DroneApp(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title(TITLE)
        self.configure(bg=BG)
        self.minsize(860, 660)

        self.cfg    = load_config()
        self._proc  = None
        self._q     = queue.Queue()
        self._alive = False

        self._ser_conn  = None
        self._ser_alive = threading.Event()

        self._build_ui()
        self._poll_output()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ─── UI skeleton ──────────────────────────────────────────────────────────

    def _build_ui(self):
        # Header bar
        hdr = tk.Frame(self, bg=ACCENT, height=52)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        tk.Label(hdr, text="🚁  Drone Navigation Launcher",
                 bg=ACCENT, fg="white",
                 font=("Helvetica", 14, "bold")).pack(side="left", padx=16)
        self._status_lbl = tk.Label(hdr, text="● STOPPED",
                                    bg=ACCENT, fg="white",
                                    font=("Helvetica", 11, "bold"))
        self._status_lbl.pack(side="right", padx=16)

        # Body: left panel + right notebook
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=10, pady=10)

        left = tk.Frame(body, bg=BG, width=320)
        left.pack(side="left", fill="y", padx=(0, 8))
        left.pack_propagate(False)

        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True)

        self._build_config_panel(left)
        self._build_right_panel(right)

    # ─── Config panel (left) ─────────────────────────────────────────────────

    def _card(self, parent, title):
        """Titled card widget; returns inner frame."""
        outer = tk.Frame(parent, bg=PANEL,
                         highlightbackground=BORDER, highlightthickness=1)
        outer.pack(fill="x", pady=(0, 8))
        tk.Label(outer, text=title, bg=PANEL, fg=ACCENT,
                 font=("Helvetica", 9, "bold")).pack(anchor="w", padx=10, pady=(8, 2))
        tk.Frame(outer, bg=BORDER, height=1).pack(fill="x", padx=10)
        inner = tk.Frame(outer, bg=PANEL)
        inner.pack(fill="x", padx=10, pady=8)
        return inner

    def _field(self, parent, label, var, values=None):
        """Label + Entry (or Combobox) row."""
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", pady=3)
        tk.Label(row, text=label, bg=PANEL, fg=TEXT_DIM,
                 font=("Helvetica", 9), width=13, anchor="w").pack(side="left")
        if values:
            w = ttk.Combobox(row, textvariable=var, values=values,
                             state="readonly", font=MONO, width=18)
            self._style_combo(w)
        else:
            w = tk.Entry(row, textvariable=var,
                         bg="#22263a", fg=TEXT, relief="flat",
                         font=MONO, insertbackground=TEXT,
                         highlightbackground=BORDER, highlightthickness=1)
        w.pack(side="left", fill="x", expand=True)
        return w

    @staticmethod
    def _style_combo(cb):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("D.TCombobox",
                    fieldbackground="#22263a", background="#22263a",
                    foreground=TEXT, arrowcolor=ACCENT,
                    bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER)
        cb.configure(style="D.TCombobox")

    def _build_config_panel(self, parent):
        # Script path
        c = self._card(parent, "📄  Script File")
        self._v_script = tk.StringVar(value=self.cfg["script_path"])
        self._field(c, "Path", self._v_script)

        # Runtime mode
        c = self._card(parent, "⚙️  Runtime Mode")
        self._v_mode = tk.StringVar(value=self.cfg["mode"])
        self._field(c, "Mode", self._v_mode, values=RUNTIME_MODES)
        self._hint_lbl = tk.Label(c, text=MODE_HINTS[self.cfg["mode"]],
                                  bg=PANEL, fg=WARNING,
                                  font=("Helvetica", 8), wraplength=270, justify="left")
        self._hint_lbl.pack(anchor="w", pady=(4, 0))
        self._v_mode.trace_add("write", self._update_hint)

        # Serial
        c = self._card(parent, "🔌  Serial / Arduino")
        self._v_port = tk.StringVar(value=self.cfg["serial_port"])
        self._v_baud = tk.StringVar(value=self.cfg["baud"])
        self._field(c, "Port", self._v_port)
        self._field(c, "Baud rate", self._v_baud, values=BAUD_RATES)
        tk.Label(c, text="Linux: /dev/ttyUSB0   Mac: /dev/cu.*   Win: COM#",
                 bg=PANEL, fg=TEXT_DIM, font=("Helvetica", 7)).pack(anchor="w")

        # Camera
        c = self._card(parent, "📷  ESP32-CAM Stream")
        self._v_url = tk.StringVar(value=self.cfg["stream_url"])
        self._field(c, "Stream URL", self._v_url)

        # YOLO
        c = self._card(parent, "🤖  YOLO Inference")
        self._v_model = tk.StringVar(value=self.cfg["model"])
        self._v_conf  = tk.StringVar(value=self.cfg["conf"])
        self._v_imgsz = tk.StringVar(value=self.cfg["imgsz"])
        self._field(c, "Model", self._v_model)
        self._field(c, "Confidence", self._v_conf)
        self._field(c, "Image size", self._v_imgsz)

        # Launch / Stop
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=(4, 0))
        self._btn_launch = tk.Button(
            row, text="▶  LAUNCH", command=self._launch,
            bg=ACCENT2, fg="white", activebackground="#28a878",
            font=("Helvetica", 11, "bold"), relief="flat",
            padx=10, pady=8, cursor="hand2")
        self._btn_launch.pack(side="left", fill="x", expand=True, padx=(0, 4))

        self._btn_stop = tk.Button(
            row, text="■  STOP", command=self._stop,
            bg=DANGER, fg="white", activebackground="#b83e3e",
            font=("Helvetica", 11, "bold"), relief="flat",
            padx=10, pady=8, cursor="hand2", state="disabled")
        self._btn_stop.pack(side="left", fill="x", expand=True)

        tk.Button(parent, text="💾  Save Config", command=self._save,
                  bg=PANEL, fg=ACCENT, activebackground=BORDER,
                  font=("Helvetica", 9), relief="flat",
                  padx=8, pady=5, cursor="hand2").pack(fill="x", pady=(6, 0))

    def _update_hint(self, *_):
        m = self._v_mode.get()
        col = DANGER if m == "REAL_FLIGHT_MODE" else WARNING
        self._hint_lbl.config(text=MODE_HINTS.get(m, ""), fg=col)

    # ─── Right panel: notebook ────────────────────────────────────────────────

    def _build_right_panel(self, parent):
        s = ttk.Style()
        s.configure("D.TNotebook", background=BG, borderwidth=0)
        s.configure("D.TNotebook.Tab",
                    background=PANEL, foreground=TEXT_DIM,
                    padding=[12, 5], font=("Helvetica", 9))
        s.map("D.TNotebook.Tab",
              background=[("selected", ACCENT)],
              foreground=[("selected", "white")])

        nb = ttk.Notebook(parent, style="D.TNotebook")
        nb.pack(fill="both", expand=True)

        # Console
        cf = tk.Frame(nb, bg=BG)
        nb.add(cf, text="  Console  ")
        self._build_console(cf)

        # Serial monitor
        sf = tk.Frame(nb, bg=BG)
        nb.add(sf, text="  Serial Monitor  ")
        self._build_serial_monitor(sf)

        # Config preview
        pf = tk.Frame(nb, bg=BG)
        nb.add(pf, text="  Config Preview  ")
        self._build_preview(pf)

    # ─── Console ─────────────────────────────────────────────────────────────

    def _build_console(self, parent):
        bar = tk.Frame(parent, bg=BG)
        bar.pack(fill="x", pady=(0, 4))
        tk.Label(bar, text="Script output", bg=BG, fg=TEXT_DIM,
                 font=("Helvetica", 9)).pack(side="left")
        tk.Button(bar, text="Clear", command=self._clear_console,
                  bg=PANEL, fg=TEXT_DIM, relief="flat",
                  font=("Helvetica", 8), padx=6, cursor="hand2").pack(side="right")

        self._console = scrolledtext.ScrolledText(
            parent, bg="#0a0c14", fg=TEXT, font=MONO,
            relief="flat", insertbackground=TEXT,
            state="disabled", wrap="word")
        self._console.pack(fill="both", expand=True)

        self._console.tag_config("err",  foreground=DANGER)
        self._console.tag_config("warn", foreground=WARNING)
        self._console.tag_config("ok",   foreground=ACCENT2)
        self._console.tag_config("info", foreground=ACCENT)
        self._console.tag_config("dim",  foreground=TEXT_DIM)

    def _log(self, text: str, tag=None):
        self._console.config(state="normal")
        self._console.insert("end", text, tag or "")
        self._console.see("end")
        self._console.config(state="disabled")

    def _clear_console(self):
        self._console.config(state="normal")
        self._console.delete("1.0", "end")
        self._console.config(state="disabled")

    def _tag_for(self, line: str):
        ll = line.lower()
        if any(w in ll for w in ("error", "fail", "exception", "traceback")):
            return "err"
        if any(w in ll for w in ("warn", "disarm", "emergency", "overrun")):
            return "warn"
        if any(w in ll for w in ("armed", " ok", "success", "ready", "✅")):
            return "ok"
        if any(w in ll for w in ("[cal]", "[filter]", "[boot]", "[model]",
                                   "[config]", "[device]", "[torch]", "[timing]")):
            return "info"
        return None

    # ─── Serial monitor ───────────────────────────────────────────────────────

    def _build_serial_monitor(self, parent):
        top = tk.Frame(parent, bg=BG)
        top.pack(fill="x", pady=(0, 6))

        def lbl(text):
            tk.Label(top, text=text, bg=BG, fg=TEXT_DIM,
                     font=("Helvetica", 9)).pack(side="left", padx=(6, 2))

        lbl("Port:")
        self._ser_port_v = tk.StringVar(value=self.cfg["serial_port"])
        tk.Entry(top, textvariable=self._ser_port_v, bg="#22263a", fg=TEXT,
                 relief="flat", font=MONO, width=14,
                 insertbackground=TEXT).pack(side="left")

        lbl("Baud:")
        self._ser_baud_v = tk.StringVar(value=self.cfg["baud"])
        cb = ttk.Combobox(top, values=BAUD_RATES, textvariable=self._ser_baud_v,
                          width=8, state="readonly", font=MONO)
        self._style_combo(cb)
        cb.pack(side="left", padx=4)

        self._btn_sc = tk.Button(top, text="Connect",
                                  command=self._ser_connect,
                                  bg=ACCENT, fg="white", relief="flat",
                                  font=("Helvetica", 9), padx=8, cursor="hand2")
        self._btn_sc.pack(side="left", padx=2)

        self._btn_sd = tk.Button(top, text="Disconnect",
                                  command=self._ser_disconnect,
                                  bg=PANEL, fg=TEXT_DIM, relief="flat",
                                  font=("Helvetica", 9), padx=8,
                                  cursor="hand2", state="disabled")
        self._btn_sd.pack(side="left")

        # Output area
        self._ser_out = scrolledtext.ScrolledText(
            parent, bg="#0a0c14", fg=ACCENT2, font=MONO,
            relief="flat", insertbackground=TEXT,
            state="disabled", wrap="word", height=14)
        self._ser_out.pack(fill="both", expand=True, pady=(0, 6))
        self._ser_out.tag_config("tx",   foreground=ACCENT)
        self._ser_out.tag_config("err",  foreground=DANGER)
        self._ser_out.tag_config("warn", foreground=WARNING)
        self._ser_out.tag_config("ok",   foreground=ACCENT2)

        # Send row
        send = tk.Frame(parent, bg=BG)
        send.pack(fill="x")

        self._ser_cmd_v = tk.StringVar()
        e = tk.Entry(send, textvariable=self._ser_cmd_v,
                     bg="#22263a", fg=TEXT, relief="flat",
                     font=MONO, insertbackground=TEXT)
        e.pack(side="left", fill="x", expand=True, padx=(0, 6))
        e.bind("<Return>", lambda _: self._ser_send())

        for cmd in ["ARM", "DISARM", "STATUS", "RECAL", "RESETSTATS"]:
            tk.Button(send, text=cmd,
                      command=lambda c=cmd: self._ser_write(c),
                      bg=PANEL, fg=ACCENT, relief="flat",
                      font=("Helvetica", 8), padx=5, cursor="hand2").pack(side="left", padx=1)

        tk.Button(send, text="Send ▶", command=self._ser_send,
                  bg=ACCENT2, fg="white", relief="flat",
                  font=("Helvetica", 9), padx=8, cursor="hand2").pack(side="left", padx=(4, 0))

    def _ser_connect(self):
        try:
            import serial as _serial
        except ImportError:
            messagebox.showerror("Missing", "Install pyserial:\npip install pyserial")
            return
        port = self._ser_port_v.get().strip()
        baud = int(self._ser_baud_v.get().strip())
        try:
            self._ser_conn = _serial.Serial(port, baud, timeout=1)
            self._ser_alive.set()
            self._btn_sc.config(state="disabled")
            self._btn_sd.config(state="normal")
            self._ser_log(f"[Monitor] Connected to {port} @ {baud}\n", "ok")
            threading.Thread(target=self._ser_reader, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Serial error", str(e))

    def _ser_disconnect(self):
        self._ser_alive.clear()
        if self._ser_conn and self._ser_conn.is_open:
            self._ser_conn.close()
        self._btn_sc.config(state="normal")
        self._btn_sd.config(state="disabled")
        self._ser_log("[Monitor] Disconnected.\n", "warn")

    def _ser_reader(self):
        while self._ser_alive.is_set():
            try:
                if self._ser_conn and self._ser_conn.is_open:
                    raw = self._ser_conn.readline()
                    if raw:
                        line = raw.decode(errors="ignore")
                        self.after(0, lambda l=line: self._ser_log(l))
            except Exception:
                break

    def _ser_send(self):
        cmd = self._ser_cmd_v.get().strip()
        if cmd:
            self._ser_write(cmd)
            self._ser_cmd_v.set("")

    def _ser_write(self, cmd: str):
        if not self._ser_conn or not self._ser_conn.is_open:
            self._ser_log("[Monitor] Not connected.\n", "err")
            return
        try:
            self._ser_conn.write(f"{cmd}\n".encode())
            self._ser_log(f">>> {cmd}\n", "tx")
        except Exception as ex:
            self._ser_log(f"[Monitor] Write error: {ex}\n", "err")

    def _ser_log(self, text: str, tag=None):
        self._ser_out.config(state="normal")
        self._ser_out.insert("end", text, tag or "")
        self._ser_out.see("end")
        self._ser_out.config(state="disabled")

    # ─── Config preview ───────────────────────────────────────────────────────

    def _build_preview(self, parent):
        tk.Label(parent,
                 text="Shows the exact lines that will be patched (original file untouched).",
                 bg=BG, fg=TEXT_DIM, font=("Helvetica", 9)).pack(anchor="w", pady=(0, 4))

        self._prev_box = scrolledtext.ScrolledText(
            parent, bg="#0a0c14", fg=ACCENT2, font=MONO,
            relief="flat", insertbackground=TEXT,
            state="disabled", wrap="none")
        self._prev_box.pack(fill="both", expand=True)

        tk.Button(parent, text="🔄  Refresh Preview",
                  command=self._refresh_preview,
                  bg=PANEL, fg=ACCENT, relief="flat",
                  font=("Helvetica", 9), padx=8, pady=5,
                  cursor="hand2").pack(pady=6)

    def _refresh_preview(self):
        c = self._collect_cfg()
        lines = [
            "Lines patched into the temp script (original is never changed):",
            "=" * 60,
            f"Script path  : {c['script_path']}",
            f"Runtime mode : {c['mode']}",
            f"Serial port  : {c['serial_port']}",
            f"Baud rate    : {c['baud']}",
            f"Stream URL   : {c['stream_url']}",
            f"Model        : {c['model']}",
            f"Confidence   : {c['conf']}",
            f"Image size   : {c['imgsz']}",
            "",
            "─── Patched lines ───",
            f"ACTIVE_MODE : RuntimeMode = RuntimeMode.{c['mode']}",
            f"serial_port : str = \"{c['serial_port']}\"",
            f"serial_baud : int = {c['baud']}",
            f"stream_url  = \"{c['stream_url']}\"",
            f"model       : str = \"{c['model']}\"",
            f"conf        : float = {c['conf']}",
            f"imgsz       : int = {c['imgsz']}",
        ]
        self._prev_box.config(state="normal")
        self._prev_box.delete("1.0", "end")
        self._prev_box.insert("end", "\n".join(lines))
        self._prev_box.config(state="disabled")

    # ─── Launch / Stop ────────────────────────────────────────────────────────

    def _collect_cfg(self) -> dict:
        return {
            "script_path": self._v_script.get().strip(),
            "mode"       : self._v_mode.get().strip(),
            "serial_port": self._v_port.get().strip(),
            "baud"       : self._v_baud.get().strip(),
            "stream_url" : self._v_url.get().strip(),
            "model"      : self._v_model.get().strip(),
            "conf"       : self._v_conf.get().strip(),
            "imgsz"      : self._v_imgsz.get().strip(),
        }

    def _launch(self):
        if self._alive:
            self._log("[Launcher] Already running.\n", "warn")
            return

        cfg = self._collect_cfg()
        self.cfg = cfg

        try:
            tmp = patch_and_write(cfg["script_path"], cfg)
        except FileNotFoundError as e:
            messagebox.showerror("Script not found",
                                 f"{e}\n\nFix the path in the Script field.")
            return
        except Exception as e:
            messagebox.showerror("Patch error", str(e))
            return

        self._log(f"[Launcher] Patching → {tmp}\n", "info")
        self._log(f"[Launcher] Mode={cfg['mode']}  Port={cfg['serial_port']}"
                  f"@{cfg['baud']}  URL={cfg['stream_url']}\n", "info")
        self._log("─" * 60 + "\n", "dim")

        try:
            extra = {}
            if platform.system() == "Windows":
                extra["creationflags"] = subprocess.CREATE_NO_WINDOW
            self._proc = subprocess.Popen(
                [sys.executable, tmp],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True, bufsize=1, **extra
            )
        except Exception as e:
            self._log(f"[Launcher] Failed to start: {e}\n", "err")
            return

        self._alive = True
        self._set_state(running=True)
        threading.Thread(target=self._read_proc, daemon=True).start()

    def _read_proc(self):
        for line in self._proc.stdout:
            self._q.put(line)
        self._proc.wait()
        self._q.put(f"\n[Launcher] Process exited (code {self._proc.returncode})\n")
        self._alive = False
        self.after(0, lambda: self._set_state(running=False))

    def _stop(self):
        if self._proc and self._proc.poll() is None:
            self._log("[Launcher] Stopping…\n", "warn")
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._log("[Launcher] Force-killed.\n", "err")
        self._alive = False
        self._set_state(running=False)
        _tmp = "_drone_launcher_run.py"
        if os.path.exists(_tmp):
            try:
                os.remove(_tmp)
            except Exception:
                pass

    def _set_state(self, running: bool):
        if running:
            self._btn_launch.config(state="disabled")
            self._btn_stop.config(state="normal")
            self._status_lbl.config(text="● RUNNING", fg=ACCENT2)
        else:
            self._btn_launch.config(state="normal")
            self._btn_stop.config(state="disabled")
            self._status_lbl.config(text="● STOPPED", fg="white")

    def _poll_output(self):
        try:
            while True:
                line = self._q.get_nowait()
                self._log(line, self._tag_for(line))
        except queue.Empty:
            pass
        self.after(80, self._poll_output)

    # ─── Save ─────────────────────────────────────────────────────────────────

    def _save(self):
        self.cfg = self._collect_cfg()
        save_config(self.cfg)
        self._log("[Launcher] Config saved to drone_config.json\n", "ok")

    # ─── Close ────────────────────────────────────────────────────────────────

    def _on_close(self):
        if self._alive:
            if messagebox.askyesno("Quit", "Script is running. Stop and quit?"):
                self._stop()
            else:
                return
        self._ser_disconnect()
        self._save()
        self.destroy()


# ── Entry ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = DroneApp()
    app.mainloop()
