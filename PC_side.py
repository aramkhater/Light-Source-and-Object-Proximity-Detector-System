import threading
import time
import queue
from dataclasses import dataclass
import re
import math
import os  # needed for script file browsing
from collections import deque, defaultdict  # <-- added

try:
    import serial  # pyserial
except Exception:
    serial = None

import PySimpleGUI as sg

# Matplotlib for plots
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

# -------------------------- PROTOCOL CONSTANTS -------------------------- #
ENTER_STATE1_CMD = b'1'   # object detector (scan & stream angle+distance)
ENTER_STATE2_CMD = b'2'   # telemeter (send 3-digit angle, get distance)
ENTER_STATE3_CMD = b'3'   # light detector (send CAL arrays once, then stream mV + angle)
ENTER_STATE4_CMD = b'4'   # mix detector (distance + mV + angle)
ENTER_STATE5_CMD = b'5'   # calibration menu
ENTER_STATE6_CMD = b'6'   # script mode on MCU
EXIT_STATE_CMD   = b'0'   # back to state0
ANGLE_FMT        = "{angle:03d}"     # 3 ASCII digits, no newline
LINE_TERM_RX     = b"\n"             # MCU terminates lines with '\n'
DEFAULT_PORT     = "COM3"
DEFAULT_BAUD     = 9600
READ_TIMEOUT_S   = 0.1

# --------------------------- RADAR SETTINGS --------------------------- #
R_MAX_CM       = 100
R_MIN_ACCEPT   = 2
R_MAX_ACCEPT   = 100
R_TICKS        = [20, 40, 60, 80, 100]

# Angle mapping for radar plots
THETA_ZERO     = "E"   # 0° to the right
ANGLE_PLOT_DIR = 1     # 1 = CCW; set to -1 if your servo increases CW on screen

# ------------------------ OBJECT CLUSTERING (STATE1/4) -------------------- #
SERVO_STEP_DEG       = 2.0
ANGLE_TOL_DEG        = 1.0
OBJ_DIST_TOL_CM      = 1.0         # base tolerance (will be adaptive)
OBJ_MIN_POINTS       = 3
OBJ_MIN_SPAN_DEG     = 3.0
OBJ_GROUP_MAD_MAX_CM = 0.9
DRAW_INTERVAL        = 0.10        # seconds
MAX_WIDTH_CM         = 200.0
DIVIDE_WIDTH_BY_TWO  = True        # as requested earlier

# ------------------------ LIGHT CLUSTERING (STATE3/4) --------------------- #
# “few continuous angles with almost same distance ±3 cm” => one source
LIGHT_DIST_TOL_CM       = 3.0
LIGHT_MIN_POINTS        = 3
LIGHT_MIN_SPAN_DEG      = 2.0
LIGHT_GROUP_MAD_MAX_CM  = 2.5

# Light acceptance / mapping
ACCEPT_NEAR_MARGIN_CM = 8
MV_DELTA_ABS_MIN      = 50
MV_DELTA_FRAC_MIN     = 0.05
LIGHT_LIVE_EMA_ALPHA  = 0.4  # UI smoothing only

# ---------------------- DEFAULT SCRIPTS DIRECTORY --------------------- #
SCRIPT_DEFAULT_DIR = os.path.expanduser(r"C:\Users\LENOVO\Desktop\DCS\scripts")

# ---------------------- SCRIPT UPLOAD SETTINGS ------------------------ #
ACK_TIMEOUT_S = 8.0
SEND_NEWLINE_AFTER_DOLLAR = False

# ------------------------------ SERIAL WRAPPER ------------------------------ #
@dataclass
class SerialConfig:
    port: str
    baud: int
    simulate: bool = False

class SerialIO:
    def __init__(self, cfg: SerialConfig, rx_queue: "queue.Queue[tuple[str, str]]"):
        self.cfg = cfg
        self.rx_queue = rx_queue
        self._stop = threading.Event()
        self._ser = None
        self._reader_th: threading.Thread | None = None
        self._sim_scan_stop = threading.Event()
        self._pause_reader = threading.Event()

    def open(self):
        if self.cfg.simulate:
            self._ser = None
        else:
            if serial is None:
                raise RuntimeError("pyserial not installed")
            self._ser = serial.Serial(self.cfg.port, self.cfg.baud, timeout=READ_TIMEOUT_S)
            try:
                self._ser.dtr = False
                self._ser.rts = False
            except Exception:
                pass
            time.sleep(1.0)
            try:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
            except Exception:
                pass
        self._reader_th = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_th.start()

    def close(self):
        self._stop.set()
        self._sim_scan_stop.set()
        if self._reader_th and self._reader_th.is_alive():
            self._reader_th.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    def write(self, data: bytes):
        if self.cfg.simulate:
            threading.Thread(target=self._simulate_rx, args=(data,), daemon=True).start()
            return
        if not self._ser:
            raise RuntimeError("Serial not open")
        self._ser.write(data)

    def pause_reader(self):
        self._pause_reader.set()
        time.sleep(0.02)

    def resume_reader(self):
        self._pause_reader.clear()

    def _reader_loop(self):
        if self.cfg.simulate:
            while not self._stop.is_set():
                time.sleep(0.05)
            return

        buf = bytearray()
        while not self._stop.is_set():
            if self._pause_reader.is_set():
                time.sleep(0.01)
                continue
            try:
                chunk = self._ser.read(128)
            except Exception as e:
                self.rx_queue.put(("LOG", f"[ERR] Serial read error: {e}"))
                break
            if not chunk:
                continue
            buf.extend(chunk)
            while LINE_TERM_RX in buf:
                line, _, buf = buf.partition(LINE_TERM_RX)
                try:
                    text = line.decode("ascii", errors="ignore")
                except Exception:
                    text = ""
                self.rx_queue.put(("LINE", text))

    # -------------------------- Simulation -------------------------- #
    def _simulate_rx(self, tx: bytes):
        import random
        if tx == ENTER_STATE2_CMD:
            time.sleep(0.1)
            self.rx_queue.put(("LINE", "READY"))
            return
        if tx == EXIT_STATE_CMD:
            self._sim_scan_stop.set()
            time.sleep(0.05)
            self.rx_queue.put(("LINE", "BACK_TO_STATE0"))
            return
        if tx == ENTER_STATE1_CMD:
            self._sim_scan_stop.clear()
            threading.Thread(target=self._sim_object_stream, daemon=True).start()
            return
        if tx == ENTER_STATE3_CMD:
            self._sim_scan_stop.clear()
            threading.Thread(target=self._sim_light_stream, daemon=True).start()
            return
        if tx == ENTER_STATE5_CMD:
            self._sim_scan_stop.clear()
            threading.Thread(target=self._sim_calib_stream, daemon=True).start()
            return
        if tx == ENTER_STATE6_CMD:
            return
        if tx == ENTER_STATE4_CMD:  # MIX moved to state4
            self._sim_scan_stop.clear()
            threading.Thread(target=self._sim_mix_stream, daemon=True).start()
            return

        try:
            s = tx.decode("ascii", errors="ignore")
        except Exception:
            s = ""
        if len(s) == 3 and s.isdigit():
            ang = int(s)
            dist = int(20 + 50 * abs(math.cos(math.radians(ang))) + 3 * random.random())
            time.sleep(0.12)
            self.rx_queue.put(("LINE", f"{dist:03d}"))
            return

    def _sim_object_stream(self):
        import random
        for angle in range(0, 181, int(round(SERVO_STEP_DEG))):
            if self._sim_scan_stop.is_set():
                break
            base = 42 if 28 <= angle <= 60 else (78 if 112 <= angle <= 140 else 90)
            dist = int(base + random.choice([0, 1, -1]))
            self.rx_queue.put(("LINE", f"{dist:03d}"))
            time.sleep(0.02)
            if self._sim_scan_stop.is_set():
                break
            self.rx_queue.put(("LINE", f"{angle:03d}"))
            time.sleep(0.03)

    def _sim_light_stream(self):
        import random
        # send CAL lines first
        self.rx_queue.put(("LINE", "CAL0:3200,3000,2800,2600,2400,2200,2000,1850,1700,1600"))
        time.sleep(0.1)
        self.rx_queue.put(("LINE", "CAL3:2800,2600,2400,2200,2000,1800,1600,1500,1450,1400"))
        time.sleep(0.2)
        for angle in range(0, 181, int(round(SERVO_STEP_DEG))):
            if self._sim_scan_stop.is_set():
                break
            mV = (3200 + random.randint(-20, 20)) if angle < 90 else (1800 + random.randint(-20, 20))
            mV = max(0, min(5000, mV))
            self.rx_queue.put(("LINE", f"{mV:04d}"))
            time.sleep(0.02)
            if self._sim_scan_stop.is_set():
                break
            self.rx_queue.put(("LINE", f"{angle:03d}"))
            time.sleep(0.03)

    def _sim_mix_stream(self):
        """Simulate state4: distance, mV, angle (plus CAL lines once)."""
        import random
        # send CAL lines first (like state3)
        self.rx_queue.put(("LINE", "CAL0:3200,3000,2800,2600,2400,2200,2000,1850,1700,1600"))
        time.sleep(0.1)
        self.rx_queue.put(("LINE", "CAL3:2800,2600,2400,2200,2000,1800,1600,1500,1450,1400"))
        time.sleep(0.2)
        for angle in range(0, 181, int(round(SERVO_STEP_DEG))):
            if self._sim_scan_stop.is_set():
                break
            # distance like state1 (two clusters)
            base = 42 if 28 <= angle <= 60 else (78 if 112 <= angle <= 140 else 90)
            dist = int(base + random.choice([0, 1, -1]))
            # mV like state3 (A0 bright <90°, A3 dim >=90°)
            mV = (3200 + random.randint(-20, 20)) if angle < 90 else (1800 + random.randint(-20, 20))
            mV = max(0, min(5000, mV))

            # send distance (ddd\n)
            self.rx_queue.put(("LINE", f"{dist:03d}"))
            time.sleep(0.02)
            # send voltage (vvvv\n)
            self.rx_queue.put(("LINE", f"{mV:04d}"))
            time.sleep(0.02)
            # send angle (aaa\n)
            self.rx_queue.put(("LINE", f"{angle:03d}"))
            time.sleep(0.03)

    def _sim_calib_stream(self):
        msgs = ["CAL_START","ZEROING_SERVO","MEASURING_OFFSETS","SAVING","CAL_DONE"]
        for m in msgs:
            if self._sim_scan_stop.is_set():
                break
            self.rx_queue.put(("LINE", m))
            time.sleep(0.4)

# ------------------------------ HELPERS -------------------------------- #
def parse_digits(line: str) -> str | None:
    s = line.replace("\r", "").strip()
    last = None
    for m in re.finditer(r"(\d{1,4})", s):
        last = m.group(1)
    return last

def parse_cal_line(line: str):
    s = line.strip().replace(" ", "")
    m = re.match(r"^CAL([03]):([0-9,]+)$", s)
    if not m:
        return None, None
    which = m.group(1)  # '0' or '3'
    nums = [int(x) for x in m.group(2).split(",") if x != ""]
    nums = [max(0, min(5000, v)) for v in nums]
    if len(nums) < 2:
        return None, None
    return which, nums

def _draw_figure(canvas_elem, figure):
    canvas = canvas_elem.TKCanvas
    fig_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    fig_canvas_agg.draw()
    fig_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    return fig_canvas_agg

def _setup_polar(ax, title):
    ax.set_theta_zero_location(THETA_ZERO)
    ax.set_theta_direction(ANGLE_PLOT_DIR)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_rlim(0, R_MAX_CM)
    ax.set_rticks(R_TICKS)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title(title, va="bottom")

def _make_radar_figure(title="Radar"):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(7.0, 7.0), dpi=100)
    _setup_polar(ax, title)
    return fig, ax

def _format_label(d):
    """Objects: show distance, angle, width. Light: distance + angle only."""
    r  = int(round(d["r_cm"]))
    a  = int(round(d["angle_deg"]))
    if "width_cm" in d and d["width_cm"] is not None:
        return f'd={r}cm, a={a}°, w={int(round(d["width_cm"]))}cm'
    return f'd={r}cm, a={a}°'

# ---------- DRAW HELPERS ----------
def _radar_update_multi(ax, detections, live_sample=None):
    """Draw a single list of detections (used by state1/state3)."""
    ax.clear()
    _setup_polar(ax, "Radar")

    if live_sample is not None:
        a_raw, r_raw = live_sample
        ax.scatter([math.radians(a_raw)], [r_raw], s=18, alpha=0.6)

    if detections:
        thetas = [math.radians(d["angle_deg"]) for d in detections]
        rs     = [d["r_cm"] for d in detections]
        ax.scatter(thetas, rs, s=60)
        for d in detections:
            th_mid = math.radians(d["angle_deg"])
            r      = d["r_cm"]
            ax.text(th_mid, min(r + 4, R_MAX_CM), _format_label(d),
                    fontsize=10, ha="center", va="bottom")

def _radar_update_mix(ax, obj_dets, light_dets, live_light=None):
    """Objects in BLUE, Light sources in ORANGE."""
    ax.clear()
    _setup_polar(ax, "Mix: Objects + Light")

    if live_light is not None:
        a_raw, r_raw = live_light
        ax.scatter([math.radians(a_raw)], [r_raw], s=20, alpha=0.7, color='orange')

    if obj_dets:
        thetas = [math.radians(d["angle_deg"]) for d in obj_dets]
        rs     = [d["r_cm"] for d in obj_dets]
        ax.scatter(thetas, rs, s=70, color='blue')
        for d in obj_dets:
            th_mid = math.radians(d["angle_deg"])
            r      = d["r_cm"]
            ax.text(th_mid, min(r + 4, R_MAX_CM), _format_label(d),
                    fontsize=10, ha="center", va="bottom")

    if light_dets:
        thetasL = [math.radians(d["angle_deg"]) for d in light_dets]
        rsL     = [d["r_cm"] for d in light_dets]
        ax.scatter(thetasL, rsL, s=55, color='orange', alpha=0.9)
        for d in light_dets:
            th_mid = math.radians(d["angle_deg"])
            r      = d["r_cm"]
            ax.text(th_mid, min(r + 4, R_MAX_CM), _format_label(d),
                    fontsize=10, ha="center", va="bottom")

    handles = []
    handles.append(mlines.Line2D([], [], color='blue', marker='o', linestyle='None', markersize=8, label='Objects'))
    handles.append(mlines.Line2D([], [], color='orange', marker='o', linestyle='None', markersize=8, label='Light sources'))
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.02, 1.02))

# ---------------------- ROBUST STATS (noise suppression) -------------------- #
def _median(xs):
    xs_sorted = sorted(xs)
    n = len(xs_sorted)
    if n == 0: return 0.0
    m = n // 2
    return float(xs_sorted[m]) if n % 2 else 0.5 * (xs_sorted[m-1] + xs_sorted[m])

def _mad(xs):
    if not xs:
        return 0.0
    med = _median(xs)
    absdev = [abs(x - med) for x in xs]
    return _median(absdev)

# -------------------- CLUSTER MERGE & TRACKER (new) ------------------------ #
def merge_clusters(dets, angle_gap_deg=4.0, dist_gap_cm=3.0):
    if not dets:
        return dets
    dets = sorted(dets, key=lambda d: d["angle_deg"])
    out = []
    cur = dets[0].copy()
    cur["n_total"] = cur.get("n", 1)
    for d in dets[1:]:
        if (abs(d["angle_deg"] - cur["angle_deg"]) <= angle_gap_deg and
            abs(d["r_cm"]      - cur["r_cm"])      <= dist_gap_cm):
            n1 = cur.get("n_total", cur.get("n", 1))
            n2 = d.get("n", 1)
            tot = n1 + n2
            cur["angle_deg"] = (cur["angle_deg"]*n1 + d["angle_deg"]*n2) / tot
            cur["r_cm"]      = (cur["r_cm"]*n1      + d["r_cm"]*n2)      / tot
            if "width_cm" in cur and "width_cm" in d:
                cur["width_cm"] = max(cur["width_cm"], d["width_cm"])
            cur["n_total"] = tot
        else:
            out.append(cur)
            cur = d.copy()
            cur["n_total"] = cur.get("n", 1)
    out.append(cur)
    for x in out:
        x.pop("n_total", None)
    return out

class SimpleTracker:
    def __init__(self, angle_gate=5.0, dist_gate=5.0, require_hits=2, max_miss=1):
        self.tracks = []
        self.angle_gate = angle_gate
        self.dist_gate  = dist_gate
        self.require_hits = require_hits
        self.max_miss = max_miss
        self.next_id = 1

    def update(self, dets):
        for t in self.tracks:
            t["miss"] += 1
        for d in dets:
            best = None; best_cost = 1e9
            for t in self.tracks:
                da = abs(d["angle_deg"] - t["angle_deg"])
                dr = abs(d["r_cm"]      - t["r_cm"])
                if da <= self.angle_gate and dr <= self.dist_gate:
                    cost = da + 0.5*dr
                    if cost < best_cost:
                        best_cost = cost; best = t
            if best is None:
                self.tracks.append({
                    "id": self.next_id, "angle_deg": d["angle_deg"], "r_cm": d["r_cm"],
                    "hits": 1, "miss": 0, "width_cm": d.get("width_cm")
                })
                self.next_id += 1
            else:
                best["angle_deg"] = 0.7*best["angle_deg"] + 0.3*d["angle_deg"]
                best["r_cm"]      = 0.7*best["r_cm"]      + 0.3*d["r_cm"]
                if "width_cm" in best and "width_cm" in d and d["width_cm"] is not None:
                    best["width_cm"] = max(best["width_cm"] or 0.0, d["width_cm"])
                best["hits"] += 1
                best["miss"]  = 0
        self.tracks = [t for t in self.tracks if t["miss"] <= self.max_miss]
        stable = []
        for t in self.tracks:
            if t["hits"] >= self.require_hits:
                out = {"angle_deg": t["angle_deg"], "r_cm": t["r_cm"]}
                if t.get("width_cm") is not None:
                    out["width_cm"] = t["width_cm"]
                stable.append(out)
        return stable

# ------------------------- GENERIC CLUSTERING ------------------------------ #
def cluster_points_generic(points_deg_cm,
                           step_deg,
                           angle_tol_deg,
                           dist_tol_cm,
                           min_points,
                           min_span_deg,
                           group_mad_max_cm,
                           divide_width=False,
                           include_width=True,
                           dist_rel_frac=0.02):  # <-- adaptive slack (+2% of median distance)
    """
    Make runs of nearly continuous angles (≈ step_deg within angle_tol_deg)
    whose distances stay within a tolerance of the run median (adaptive).
    Keep runs that have >= min_points and span >= min_span_deg and
    with MAD <= group_mad_max_cm.
    """
    objs = []
    if not points_deg_cm:
        return objs

    pts = sorted(points_deg_cm, key=lambda x: x[0])
    run = [pts[0]]  # list of (angle, dist)

    def finalize(group):
        if len(group) < min_points:
            return
        group.sort(key=lambda x: x[0])
        angles = [a for a, _ in group]
        dists  = [r for _, r in group]

        span_deg = max(0.0, angles[-1] - angles[0])
        if span_deg < min_span_deg:
            return

        mad = _mad(dists)
        if mad > group_mad_max_cm:
            return

        r_med = _median(dists)
        angle_mid = 0.5 * (angles[0] + angles[-1])

        item = {
            "angle_deg": angle_mid,
            "r_cm":      r_med,
            "n":         len(group),
            "angle_min": angles[0],
            "angle_max": angles[-1],
            "span_deg":  span_deg
        }
        if include_width:
            dtheta = math.radians(span_deg)
            width_cm = 0.0 if dtheta <= 1e-9 else 2.0 * r_med * math.sin(dtheta / 2.0)
            if divide_width:
                width_cm *= 0.5
            width_cm = max(0.0, min(width_cm, MAX_WIDTH_CM))
            item["width_cm"] = width_cm
        objs.append(item)

    for a, d in pts[1:]:
        pa, pd = run[-1]
        cont_angle = (abs((a - pa) - step_deg) <= angle_tol_deg)

        run_dists = [x[1] for x in run]
        med = _median(run_dists)
        local_tol = dist_tol_cm + dist_rel_frac * max(med, 1.0)
        same_dist = (abs(d - med) <= local_tol)

        if cont_angle and same_dist:
            run.append((a, d))
        else:
            finalize(run)
            run = [(a, d)]
    finalize(run)
    return objs

def cluster_points_obj(points_deg_cm):
    return cluster_points_generic(points_deg_cm,
                                  SERVO_STEP_DEG, ANGLE_TOL_DEG,
                                  OBJ_DIST_TOL_CM, OBJ_MIN_POINTS,
                                  OBJ_MIN_SPAN_DEG, OBJ_GROUP_MAD_MAX_CM,
                                  divide_width=DIVIDE_WIDTH_BY_TWO,
                                  include_width=True)

def cluster_points_light(points_deg_cm):
    # Light sources should NOT have width
    return cluster_points_generic(points_deg_cm,
                                  SERVO_STEP_DEG, ANGLE_TOL_DEG,
                                  LIGHT_DIST_TOL_CM, LIGHT_MIN_POINTS,
                                  LIGHT_MIN_SPAN_DEG, LIGHT_GROUP_MAD_MAX_CM,
                                  divide_width=False,
                                  include_width=False)

# ----------------------------- LIGHT HELPERS ----------------------------- #
def infer_grid_from_len(N: int):
    if N <= 1: return 0, 50
    if N == 11: return 0, 5
    if N == 10: return 5, 5
    step = int(round(50.0 / (N - 1)))
    return (0, step if step > 0 else 1)

def distance_cm_from_mV_int(mV: int, cal_mV: list[int], start_cm: int, step_cm: int) -> int:
    N = len(cal_mV)
    if N < 2: return start_cm
    inc = 1 if cal_mV[-1] > cal_mV[0] else (-1 if cal_mV[-1] < cal_mV[0] else 0)
    if inc >= 0:
        if mV <= cal_mV[0]:  return start_cm
        if mV >= cal_mV[-1]: return start_cm + (N-1)*step_cm
    else:
        if mV >= cal_mV[0]:  return start_cm
        if mV <= cal_mV[-1]: return start_cm + (N-1)*step_cm
    for i in range(N-1):
        a, b = cal_mV[i], cal_mV[i+1]
        if (a <= mV <= b) if inc >= 0 else (b <= mV <= a):
            if a == b: return start_cm + i*step_cm + step_cm//2
            num, den = mV - a, b - a
            if den < 0: num, den = -num, -den
            t_scaled = (num * 1000 + den//2) // den
            add_cm   = (t_scaled * step_cm + 500) // 1000
            return start_cm + i*step_cm + add_cm
    return start_cm

def accept_light_sample(mV: int, cal: list[int] | None, dist_cm: int, far_cm: int) -> bool:
    if not cal or len(cal) < 2:
        return False
    if dist_cm <= max(R_MIN_ACCEPT, far_cm - ACCEPT_NEAR_MARGIN_CM):
        return True
    vmin, vmax = min(cal), max(cal)
    need = max(MV_DELTA_ABS_MIN, int(MV_DELTA_FRAC_MIN * (vmax - vmin)))
    return mV >= (vmin + need)

# --------------------------- SCRIPT MODE (friend-like) -----------------------
_SCRIPT_CMD_MAP = {
    "inc_lcd":  0x01,
    "dec_lcd":  0x02,
    "rra_lcd":  0x03,
    "set_delay":0x04,
    "clear_lcd":0x05,
    "servo_deg":0x06,
    "servo_scan":0x07,
    "sleep":    0x08,
}

def _script_encode_text(txt: str) -> str:
    out = []
    for raw in txt.splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        cmd = parts[0]
        args = parts[1] if len(parts) > 1 else ""
        if cmd not in _SCRIPT_CMD_MAP:
            continue
        op_hex = f"{_SCRIPT_CMD_MAP[cmd]:02X}"
        arg_hex = ""
        if args:
            try:
                arg_hex = "".join(f"{int(a.strip()):02X}" for a in args.split(","))
            except Exception:
                continue
        out.append(op_hex + arg_hex + "\n")
    return "".join(out)

def _ser_read_ack_until_nul(sio, timeout=ACK_TIMEOUT_S) -> str:
    if sio.cfg.simulate or sio._ser is None:
        return ""
    old = sio._ser.timeout
    sio._ser.timeout = timeout
    try:
        buf = bytearray()
        while True:
            b = sio._ser.read(1)
            if not b:
                break
            if b == b"\0":
                break
            buf += b
        return buf.decode("ascii", errors="ignore").strip()
    finally:
        sio._ser.timeout = old

def _ser_read_one_byte(sio, timeout=2.0) -> str:
    if sio.cfg.simulate or sio._ser is None:
        return ""
    old = sio._ser.timeout
    sio._ser.timeout = timeout
    try:
        b = sio._ser.read(1)
    finally:
        sio._ser.timeout = old
    return b.decode("ascii", errors="ignore") if b else ""

def _ser_read_line(sio, timeout=2.0) -> str:
    if sio.cfg.simulate or sio._ser is None:
        return ""
    end = time.time() + timeout
    buf = bytearray()
    while time.time() < end:
        b = sio._ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return buf.decode("ascii", errors="ignore")
        buf += b
    return ""

def _script_upload(sio, slot_letter: str, payload_hex: str) -> str:
    hex_compact = "".join(ch for ch in payload_hex if ch.strip() != "")
    if len(hex_compact) % 2 != 0:
        hex_compact = hex_compact[:-1]

    decoded_len = len(hex_compact) // 2
    if decoded_len > 255:
        raise ValueError(f"Encoded script is {decoded_len} bytes; max is 255. Split or shorten the file.")

    sio.pause_reader()
    try:
        sio.write(ENTER_STATE6_CMD)
        time.sleep(0.03)
        sio.write(slot_letter.encode("ascii"))
        time.sleep(0.02)

        if sio.cfg.simulate or sio._ser is None:
            return ""

        sio._ser.reset_input_buffer()
        sio._ser.reset_output_buffer()

        sio._ser.write(bytes([decoded_len]))
        sio._ser.write(hex_compact.encode("ascii", errors="ignore"))
        sio._ser.write(b"$")
        if SEND_NEWLINE_AFTER_DOLLAR:
            sio._ser.write(b"\n")
        sio._ser.flush()

        old_to = sio._ser.timeout
        sio._ser.timeout = ACK_TIMEOUT_S
        try:
            buf = bytearray()
            while True:
                b = sio._ser.read(1)
                if not b:
                    break
                if b == b"\x00":
                    break
                buf += b
            if buf[:1] in (b"1", b"2", b"3"):
                return buf[:1].decode()
            one = sio._ser.read(1)
            if one in (b"1", b"2", b"3"):
                return one.decode()
            return ""
        finally:
            sio._ser.timeout = old_to
    finally:
        sio.resume_reader()

def _script_play(sio, play_letter: str, out_elem):
    sio.write(ENTER_STATE6_CMD)
    time.sleep(0.05)
    sio.write(play_letter.encode("ascii"))
    time.sleep(0.05)

    names = {'1':"inc_lcd",'2':"dec_lcd",'3':"rra_lcd",'4':"set_delay",
             '5':"clear_lcd",'6':"servo_deg",'7':"servo_scan",'8':"sleep"}

    while True:
        op = _ser_read_one_byte(sio, timeout=2.0)
        if not op:
            out_elem.update("No opcode received (timeout). Stopping.\n", append=True)
            break

        out_elem.Widget.tag_configure("blue_text", foreground="blue")
        out_elem.Widget.insert("end", f"Opcode {op} ({names.get(op,'?')})\n", "blue_text")

        if op == '6':
            dist = _ser_read_line(sio, timeout=2.0)
            ang  = _ser_read_line(sio, timeout=2.0)
            if dist and ang:
                out_elem.update(f"Measured Distance: {dist} [cm] | Angle: {ang} [°]\n", append=True)

        elif op == '7':
            a1 = _ser_read_line(sio, timeout=2.0)
            a2 = _ser_read_line(sio, timeout=2.0)
            if not a1 or not a2:
                continue
            try:
                fa1 = float(a1); fa2 = float(a2)
            except Exception:
                continue
            dists = []
            while True:
                d = _ser_read_line(sio, timeout=3.0)
                if not d:
                    break
                if d.strip() == "999":
                    break
                try:
                    dists.append(int(d))
                except Exception:
                    pass
            if len(dists) >= 2:
                deg = [round(fa1 + i*(fa2-fa1)/(len(dists)-1), 1) for i in range(len(dists))]
                out_elem.update(f"Distances: {dists}\n", append=True)
                out_elem.update(f"Degrees:   {deg}\n", append=True)
            else:
                out_elem.update("Scan returned too few points.\n", append=True)

        elif op == '8':
            break

def script_mode(sio):
    try:
        os.makedirs(SCRIPT_DEFAULT_DIR, exist_ok=True)
    except Exception:
        pass

    layout = [
        [sg.Text("Choose a TXT script to upload:", background_color="#9AF1FF")],
        [sg.InputText(key="-FILE-", size=(60,1)),
         sg.FileBrowse(initial_folder=SCRIPT_DEFAULT_DIR, file_types=[("Text Files","*.txt")])],
        [sg.Button("Upload Script 1", key="-UP1-"), sg.Button("Play Script 1", key="-PL1-")],
        [sg.Button("Upload Script 2", key="-UP2-"), sg.Button("Play Script 2", key="-PL2-")],
        [sg.Button("Upload Script 3", key="-UP3-"), sg.Button("Play Script 3", key="-PL3-")],
        [sg.Button("Back", key="-BACK-")],
        [sg.Text("", key="-STATUS-", background_color="#9AF1FF")],
        [sg.Output(key="-OUT-", size=(90,12))]
    ]
    win = sg.Window("Script Mode (state6)", layout, background_color="#9AF1FF",
                    resizable=True, finalize=True)
    out = win["-OUT-"]

    while True:
        ev, vals = win.read()
        if ev in (sg.WINDOW_CLOSED, "-BACK-"):
            try:
                sio.write(EXIT_STATE_CMD)
            except Exception:
                pass
            break

        elif ev in ("-UP1-","-UP2-","-UP3-"):
            path = vals["-FILE-"]
            if not path or not os.path.isfile(path):
                win["-STATUS-"].update("Pick a valid .txt file first.")
                continue
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                payload = _script_encode_text(f.read())
            slot = {"-UP1-":"A","-UP2-":"B","-UP3-":"C"}[ev]
            win["-STATUS-"].update(f"Uploading to slot {slot}…")
            try:
                ack = _script_upload(sio, slot, payload)
            except ValueError as e:
                win["-STATUS-"].update(str(e)); continue
            if ack in ("1","2","3"):
                win["-STATUS-"].update(f"Script{ack} uploaded successfully.")
            else:
                win["-STATUS-"].update("Upload done, but no ACK received (timeout).")

        elif ev in ("-PL1-","-PL2-","-PL3-"):
            win["-STATUS-"].update("Playing…")
            _script_play(sio, {"-PL1-":"D","-PL2-":"E","-PL3-":"F"}[ev], out)
            win["-STATUS-"].update("Done.")

    win.close()
# ------------------------- end SCRIPT MODE -----------------------------------


# ------------------------------ MAIN PROGRAM ------------------------------- #
def main():
    sg.theme("DarkBlue3")  # More modern theme

    # Updated layout with centered, modern-looking vertical buttons
    layout = [
        [sg.VPush()],  # Vertical push to center content
        [sg.Push(), sg.Button("Telemeter", key="-TELEM-", size=(15, 2), font=("Arial", 12)), sg.Push()],
        [sg.Push(), sg.Button("Object Detector", key="-OBJ-", size=(15, 2), font=("Arial", 12)), sg.Push()],
        [sg.Push(), sg.Button("Light Detector", key="-LIGHT-", size=(15, 2), font=("Arial", 12)), sg.Push()],
        [sg.Push(), sg.Button("Mix Detector", key="-MIX-", size=(15, 2), font=("Arial", 12)), sg.Push()],
        [sg.Push(), sg.Button("Calibration", key="-CALIB-", size=(15, 2), font=("Arial", 12)), sg.Push()],
        [sg.VPush()]  # Vertical push to center content
    ]

    win = sg.Window("MCU Control (COM3@9600)", layout,
                    element_justification='center',
                    finalize=True,
                    size=(300, 400))  # Set a fixed window size for better appearance

    # ... rest of the code remains the same ...

    rx_queue: "queue.Queue[tuple[str, str]]" = queue.Queue()
    try:
        sio = SerialIO(SerialConfig(port=DEFAULT_PORT, baud=DEFAULT_BAUD, simulate=False), rx_queue)
        sio.open()
    except Exception:
        sio = SerialIO(SerialConfig(port=DEFAULT_PORT, baud=DEFAULT_BAUD, simulate=True), rx_queue)
        sio.open()

    telem_win = obj_win = light_win = mix_win = calib_win = None
    telem_last_angle = 90

    # Object detector state
    obj_pending_distance = None
    sweep_points = []
    prev_angle = None
    obj_fig_canvas = None
    obj_ax = None
    last_draw = 0.0

    # Light detector state
    light_pending_mV = None
    ld_points = []
    ld_prev_angle = None
    ld_detections = []
    light_fig_canvas = None
    light_ax = None
    light_last_draw = 0.0
    live_light_sample = None
    live_light_dist_smooth = None

    # MIX (state4) state
    s4_pending_distance = None
    s4_pending_mV = None
    s4_prev_angle = None
    s4_obj_points = []
    s4_light_points = []
    s4_obj_detections = []
    s4_light_detections = []
    s4_live_light = None
    s4_live_light_smooth = None
    mix_fig_canvas = None
    mix_ax = None
    mix_last_draw = 0.0

    # Calibration arrays (used in state3 and state4)
    cal_A0 = None
    cal_A3 = None
    grid_A0 = (0, 5)
    grid_A3 = (0, 5)

    # -------- NEW: smoothing buffers & trackers --------
    obj_smooth = defaultdict(lambda: deque(maxlen=3))
    light_smooth = defaultdict(lambda: deque(maxlen=3))
    mix_dist_smooth = defaultdict(lambda: deque(maxlen=3))
    mix_light_smooth = defaultdict(lambda: deque(maxlen=3))

    obj_tracker = SimpleTracker(require_hits=2)
    light_tracker = SimpleTracker(require_hits=2)
    mix_obj_tracker = SimpleTracker(require_hits=2)
    mix_light_tracker = SimpleTracker(require_hits=2)

    def exit_state2_cleanly():
        try:
            payload = ANGLE_FMT.format(angle=telem_last_angle).encode("ascii")
            sio.write(payload)
            time.sleep(0.05)
            sio.write(EXIT_STATE_CMD)
        except Exception:
            pass

    # -------------------- Pump lines from serial to windows ----------------- #
    while True:
        event, values = win.read(timeout=50)

        # Forward RX lines to sub-windows
        try:
            for _ in range(500):
                tag, payload = rx_queue.get_nowait()
                if tag == "LINE":
                    if telem_win is not None:
                        telem_win.write_event_value("-RX-LINE-", payload)
                    if obj_win is not None:
                        obj_win.write_event_value("-OBJ-RX-LINE-", payload)
                    if light_win is not None:
                        light_win.write_event_value("-LIGHT-RX-LINE-", payload)
                    if mix_win is not None:
                        mix_win.write_event_value("-MIX-RX-LINE-", payload)
                    if calib_win is not None:
                        calib_win.write_event_value("-CALIB-RX-LINE-", payload)
        except queue.Empty:
            pass

        if event is None:
            if telem_win is not None:
                exit_state2_cleanly()
            break

        # ---------------- Telemetry (state2) ----------------
        if event == "-TELEM-":
            try:
                sio.write(ENTER_STATE2_CMD)
            except Exception:
                pass

            telem_layout = [
                [sg.Text("Telemeter (state2)", font=("Segoe UI", 11, "bold"))],
                [sg.Text("Angle:"), sg.Slider(range=(0, 180), default_value=telem_last_angle, orientation="h",
                                              key="-ANGLE-", enable_events=True, size=(44, 20))],
                [sg.Input(str(telem_last_angle), size=(5, 1), key="-ANGLE-IN-"), sg.Button("Send Angle", key="-SEND-")],
                [sg.Text("Last distance [cm]:"), sg.Text("—", key="-DIST-", font=("Segoe UI", 14, "bold"))],
                [sg.Multiline(size=(64, 10), key="-LOG2-", autoscroll=True, disabled=True, write_only=True)],
                [sg.Button("Close", key="-CLOSE-TELEM-")]
            ]
            telem_win = sg.Window("Telemeter", telem_layout, modal=False, finalize=True)

        if telem_win is not None:
            tev, tvals = telem_win.read(timeout=0)
            if tev in (None, "-CLOSE-TELEM-"):
                exit_state2_cleanly()
                telem_win.close(); telem_win = None
            elif tev == "-ANGLE-":
                try:
                    telem_last_angle = int(float(tvals["-ANGLE-"]))
                except Exception:
                    pass
                telem_win["-ANGLE-IN-"].update(str(telem_last_angle))
            elif tev == "-SEND-":
                try:
                    ang = int(float(tvals["-ANGLE-IN-"]))
                except Exception:
                    try:
                        ang = int(float(tvals["-ANGLE-"]))
                    except Exception:
                        ang = telem_last_angle
                ang = max(0, min(180, ang))
                telem_last_angle = ang
                telem_win["-ANGLE-IN-"].update(str(ang))
                payload = ANGLE_FMT.format(angle=ang).encode("ascii")
                try:
                    sio.write(payload)
                    telem_win["-LOG2-"].update(f"TX angle: {payload!r}\n", append=True)
                except Exception as e:
                    telem_win["-LOG2-"].update(f"[ERR] Write failed: {e}\n", append=True)
            elif tev == "-RX-LINE-":
                line = tvals["-RX-LINE-"]
                d = parse_digits(line)
                if d is not None:
                    telem_win["-DIST-"].update(d)
                telem_win["-LOG2-"].update(f"RX: {line}\n", append=True)

        # ---------------- Object Detector (state1) ----------------
        if event == "-OBJ-":
            obj_pending_distance = None
            sweep_points = []
            prev_angle = None
            obj_smooth.clear()
            obj_tracker = SimpleTracker(require_hits=2)  # reset tracker per session
            try:
                sio.write(ENTER_STATE1_CMD)
            except Exception:
                pass

            obj_layout = [
                [sg.Text("Object Detector (state1)", font=("Segoe UI", 11, "bold"))],
                [sg.Text("Rule: continuous angles & distance ±1cm → one object. Labels show d, a, w.", font=("Segoe UI", 9))],
                [sg.Canvas(key="-OBJ-CANVAS-", size=(720, 720))],
                [sg.Button("Clear", key="-OBJ-CLEAR-"), sg.Button("Close", key="-OBJ-CLOSE-")]
            ]
            obj_win = sg.Window("Object Detector", obj_layout, modal=False, finalize=True)

            fig, ax = _make_radar_figure("Radar (state1)")
            fig_canvas = _draw_figure(obj_win["-OBJ-CANVAS-"], fig)
            obj_fig_canvas = fig_canvas; obj_ax = ax

            _radar_update_multi(obj_ax, [])
            obj_fig_canvas.draw_idle()
            last_draw = time.time()

        if obj_win is not None:
            oev, ovals = obj_win.read(timeout=0)
            if oev in (None, "-OBJ-CLOSE-"):
                try: sio.write(EXIT_STATE_CMD)
                except Exception: pass
                if obj_fig_canvas is not None:
                    try: obj_fig_canvas.get_tk_widget().forget()
                    except Exception: pass
                obj_win.close(); obj_win=None
                obj_pending_distance=None; sweep_points=[]; prev_angle=None; obj_ax=None; obj_fig_canvas=None
            elif oev == "-OBJ-CLEAR-":
                sweep_points=[]; prev_angle=None
                obj_smooth.clear()
                obj_tracker = SimpleTracker(require_hits=2)
                if obj_ax is not None and obj_fig_canvas is not None:
                    _radar_update_multi(obj_ax, [])
                    obj_fig_canvas.draw_idle(); last_draw=time.time()
            elif oev == "-OBJ-RX-LINE-":
                line = ovals["-OBJ-RX-LINE-"]
                num = parse_digits(line)
                if num is None:
                    continue
                if obj_pending_distance is None:
                    obj_pending_distance = num
                else:
                    angle_deg = int(num)
                    dist_cm  = int(obj_pending_distance)
                    obj_pending_distance = None
                    if not (R_MIN_ACCEPT <= dist_cm <= R_MAX_ACCEPT):
                        prev_angle = angle_deg; continue

                    new_sweep_started = prev_angle is not None and angle_deg < prev_angle
                    prev_angle = angle_deg

                    # median-of-3 per-angle smoothing (bin on step)
                    bin_a = int(round(angle_deg / SERVO_STEP_DEG) * SERVO_STEP_DEG)
                    obj_smooth[bin_a].append(dist_cm)
                    dm = sorted(obj_smooth[bin_a])[len(obj_smooth[bin_a])//2]
                    sweep_points.append((bin_a, dm))

                    now = time.time()
                    if now - last_draw > DRAW_INTERVAL and obj_ax is not None and obj_fig_canvas is not None:
                        clusters = cluster_points_obj(sweep_points)
                        dets = [o for o in clusters if o["n"] >= OBJ_MIN_POINTS]
                        dets = merge_clusters(dets)
                        dets = obj_tracker.update(dets)
                        _radar_update_multi(obj_ax, dets)
                        obj_fig_canvas.draw_idle()
                        last_draw = now

                    if new_sweep_started:
                        sweep_points = []

        # ---------------- Light Detector (state3) ----------------
        if event == "-LIGHT-":
            light_pending_mV = None
            ld_points = []
            ld_prev_angle = None
            ld_detections = []
            live_light_sample = None
            live_light_dist_smooth = None
            cal_A0 = None; cal_A3 = None
            grid_A0 = (0, 5); grid_A3 = (0, 5)
            light_smooth.clear()
            light_tracker = SimpleTracker(require_hits=2)

            try:
                sio.write(ENTER_STATE3_CMD)
            except Exception:
                pass

            # Window: radar + calibration arrays + Close
            light_layout = [
                [sg.Canvas(key="-LIGHT-CANVAS-", size=(720, 720))],
                [sg.Frame("Calibration",
                          [[sg.Text("A0 (<90°):")],
                           [sg.Multiline(size=(86, 2), key="-CAL-A0-ML-", autoscroll=False, disabled=True, write_only=True)],
                           [sg.Text("A3 (≥90°):")],
                           [sg.Multiline(size=(86, 2), key="-CAL-A3-ML-", autoscroll=False, disabled=True, write_only=True)],
                           [sg.Text("Grids:"), sg.Text("A0 start/step: — / —    |    A3 start/step: — / —", key="-CAL-GRID-")]],
                          relief='sunken', expand_x=True)],
                [sg.Button("Close", key="-LIGHT-CLOSE-")]
            ]
            light_win = sg.Window("Light Source Detector (state3)", light_layout, modal=False, finalize=True)

            lfig, lax = _make_radar_figure("Light Sources (state3)")
            lcanvas = _draw_figure(light_win["-LIGHT-CANVAS-"], lfig)
            light_fig_canvas = lcanvas; light_ax = lax

            _radar_update_multi(light_ax, ld_detections, live_sample=None)
            light_fig_canvas.draw_idle()
            light_last_draw = time.time()

        if light_win is not None:
            lev, lvals = light_win.read(timeout=0)
            if lev in (None, "-LIGHT-CLOSE-"):
                try: sio.write(EXIT_STATE_CMD)
                except Exception: pass
                if light_fig_canvas is not None:
                    try: light_fig_canvas.get_tk_widget().forget()
                    except Exception: pass
                light_win.close(); light_win=None
                light_pending_mV=None; ld_points=[]; light_ax=None; light_fig_canvas=None
                live_light_sample=None; live_light_dist_smooth=None
            elif lev == "-LIGHT-RX-LINE-":
                line = lvals["-LIGHT-RX-LINE-"]

                which, arr = parse_cal_line(line)
                if which is not None:
                    if which == '0':
                        cal_A0 = arr
                        grid_A0 = infer_grid_from_len(len(cal_A0))
                        try:
                            light_win["-CAL-A0-ML-"].update(",".join(str(v) for v in cal_A0) + "\n", append=False)
                        except Exception:
                            pass
                    else:
                        cal_A3 = arr
                        grid_A3 = infer_grid_from_len(len(cal_A3))
                        try:
                            light_win["-CAL-A3-ML-"].update(",".join(str(v) for v in cal_A3) + "\n", append=False)
                        except Exception:
                            pass

                    try:
                        sA0, stA0 = grid_A0
                        sA3, stA3 = grid_A3
                        light_win["-CAL-GRID-"].update(
                            f"A0 start/step: {sA0} / {stA0}    |    A3 start/step: {sA3} / {stA3}"
                        )
                    except Exception:
                        pass
                    continue

                num = parse_digits(line)
                if num is None:
                    continue
                if light_pending_mV is None:
                    light_pending_mV = num
                else:
                    angle_deg = int(num)
                    try:
                        mV = int(light_pending_mV)
                    except ValueError:
                        mV = -1
                    light_pending_mV = None

                    # reject invalid/saturated readings
                    if mV <= 0 or mV >= 5000:
                        ld_prev_angle = angle_deg
                        continue

                    # Map voltage to distance using the proper calibration half
                    if angle_deg < 90:
                        cal = cal_A0; start, step = grid_A0
                    else:
                        cal = cal_A3; start, step = grid_A3
                    if not (cal and len(cal) >= 2):
                        continue

                    far_cm = start + (len(cal)-1) * step
                    dist_cm = distance_cm_from_mV_int(mV, cal, start, step)
                    dist_cm = max(R_MIN_ACCEPT, min(R_MAX_ACCEPT, dist_cm))

                    if not accept_light_sample(mV, cal, dist_cm, far_cm):
                        ld_prev_angle = angle_deg
                        continue

                    # live dot smoothing (UI only)
                    if live_light_dist_smooth is None:
                        live_light_dist_smooth = float(dist_cm)
                    else:
                        live_light_dist_smooth = (
                            LIGHT_LIVE_EMA_ALPHA * float(dist_cm)
                            + (1.0 - LIGHT_LIVE_EMA_ALPHA) * live_light_dist_smooth
                        )
                    live_light_sample = (angle_deg, live_light_dist_smooth)

                    new_sweep = (ld_prev_angle is not None and angle_deg < ld_prev_angle)
                    ld_prev_angle = angle_deg

                    # median-of-3 bin smoothing, NO width
                    bin_a = int(round(angle_deg / SERVO_STEP_DEG) * SERVO_STEP_DEG)
                    light_smooth[bin_a].append(dist_cm)
                    dm = sorted(light_smooth[bin_a])[len(light_smooth[bin_a])//2]
                    ld_points.append((bin_a, dm))

                    # Incremental cluster + merge + track
                    clusters_inc = cluster_points_light(ld_points)
                    ld_detections = [{"angle_deg": o["angle_deg"], "r_cm": o["r_cm"]}
                                     for o in clusters_inc if o["n"] >= LIGHT_MIN_POINTS]
                    ld_detections = merge_clusters(ld_detections, angle_gap_deg=4.0, dist_gap_cm=4.0)
                    ld_detections = light_tracker.update(ld_detections)

                    if new_sweep or angle_deg == 180:
                        clusters = cluster_points_light(ld_points)
                        ld_detections = [{"angle_deg": o["angle_deg"], "r_cm": o["r_cm"]}
                                         for o in clusters if o["n"] >= LIGHT_MIN_POINTS]
                        ld_detections = merge_clusters(ld_detections, angle_gap_deg=4.0, dist_gap_cm=4.0)
                        ld_detections = light_tracker.update(ld_detections)
                        if new_sweep:
                            ld_points = []

                    now = time.time()
                    if now - light_last_draw > DRAW_INTERVAL and light_ax is not None and light_fig_canvas is not None:
                        _radar_update_multi(light_ax, ld_detections, live_sample=live_light_sample)
                        light_fig_canvas.draw_idle()
                        light_last_draw = now

        # ---------------- MIX Detector (state4) ----------------
        if event == "-MIX-":
            # Reset state4 buffers
            s4_pending_distance = None
            s4_pending_mV = None
            s4_prev_angle = None
            s4_obj_points = []
            s4_light_points = []
            s4_obj_detections = []
            s4_light_detections = []
            s4_live_light = None
            s4_live_light_smooth = None
            cal_A0 = None; cal_A3 = None
            grid_A0 = (0, 5); grid_A3 = (0, 5)
            mix_dist_smooth.clear()
            mix_light_smooth.clear()
            mix_obj_tracker = SimpleTracker(require_hits=2)
            mix_light_tracker = SimpleTracker(require_hits=2)

            try:
                sio.write(ENTER_STATE4_CMD)   # send '4' now
            except Exception:
                pass

            mix_layout = [
                [sg.Text("Mix Detector (state4): Objects (blue) + Light (orange)", font=("Segoe UI", 11, "bold"))],
                [sg.Canvas(key="-MIX-CANVAS-", size=(720, 720))],
                [sg.Frame("Calibration", [
                    [sg.Text("A0 (<90°):")],
                    [sg.Multiline(size=(86, 2), key="-MIX-A0-ML-", autoscroll=False, disabled=True, write_only=True)],
                    [sg.Text("A3 (≥90°):")],
                    [sg.Multiline(size=(86, 2), key="-MIX-A3-ML-", autoscroll=False, disabled=True, write_only=True)],
                    [sg.Text("Grids:"), sg.Text("A0 start/step: — / —    |    A3 start/step: — / —", key="-MIX-GRID-")],
                ], relief='sunken', expand_x=True)],
                [sg.Button("Clear", key="-MIX-CLEAR-"), sg.Button("Close", key="-MIX-CLOSE-")]
            ]
            mix_win = sg.Window("Mix Detector (state4)", mix_layout, modal=False, finalize=True)

            mfig, maxx = _make_radar_figure("Mix: Objects + Light")
            mcanvas = _draw_figure(mix_win["-MIX-CANVAS-"], mfig)
            mix_fig_canvas = mcanvas; mix_ax = maxx

            _radar_update_mix(mix_ax, s4_obj_detections, s4_light_detections, live_light=None)
            mix_fig_canvas.draw_idle()
            mix_last_draw = time.time()

        if mix_win is not None:
            mev, mvals = mix_win.read(timeout=0)
            if mev in (None, "-MIX-CLOSE-"):
                try: sio.write(EXIT_STATE_CMD)
                except Exception: pass
                if mix_fig_canvas is not None:
                    try: mix_fig_canvas.get_tk_widget().forget()
                    except Exception: pass
                mix_win.close(); mix_win=None
                s4_pending_distance = None; s4_pending_mV = None
                s4_obj_points = []; s4_light_points = []
                s4_obj_detections = []; s4_light_detections = []
                s4_live_light = None; s4_live_light_smooth = None
            elif mev == "-MIX-CLEAR-":
                s4_obj_points=[]; s4_light_points=[]
                s4_obj_detections=[]; s4_light_detections=[]
                s4_prev_angle=None; s4_live_light=None; s4_live_light_smooth=None
                mix_dist_smooth.clear(); mix_light_smooth.clear()
                mix_obj_tracker = SimpleTracker(require_hits=2)
                mix_light_tracker = SimpleTracker(require_hits=2)
                if mix_ax is not None and mix_fig_canvas is not None:
                    _radar_update_mix(mix_ax, s4_obj_detections, s4_light_detections, live_light=None)
                    mix_fig_canvas.draw_idle(); mix_last_draw=time.time()
            elif mev == "-MIX-RX-LINE-":
                line = mvals["-MIX-RX-LINE-"]

                # 1) Calibration lines (exactly like state3)
                which, arr = parse_cal_line(line)
                if which is not None:
                    if which == '0':
                        cal_A0 = arr
                        grid_A0 = infer_grid_from_len(len(cal_A0))
                        try:
                            mix_win["-MIX-A0-ML-"].update(",".join(str(v) for v in cal_A0) + "\n", append=False)
                        except Exception:
                            pass
                    else:
                        cal_A3 = arr
                        grid_A3 = infer_grid_from_len(len(cal_A3))
                        try:
                            mix_win["-MIX-A3-ML-"].update(",".join(str(v) for v in cal_A3) + "\n", append=False)
                        except Exception:
                            pass

                    try:
                        sA0, stA0 = grid_A0
                        sA3, stA3 = grid_A3
                        mix_win["-MIX-GRID-"].update(
                            f"A0 start/step: {sA0} / {stA0}    |    A3 start/step: {sA3} / {stA3}"
                        )
                    except Exception:
                        pass
                    continue

                # 2) Triplet stream: distance -> mV -> angle
                num = parse_digits(line)
                if num is None:
                    continue

                if s4_pending_distance is None:
                    s4_pending_distance = num    # first in the triplet
                elif s4_pending_mV is None:
                    s4_pending_mV = num          # second in the triplet
                else:
                    try:
                        angle_deg = int(num)
                        dist_cm  = int(s4_pending_distance)
                        mV_val   = int(s4_pending_mV)
                    except Exception:
                        s4_pending_distance = None
                        s4_pending_mV = None
                        continue

                    s4_pending_distance = None
                    s4_pending_mV = None

                    if not (R_MIN_ACCEPT <= dist_cm <= R_MAX_ACCEPT):
                        s4_prev_angle = angle_deg
                        continue

                    new_sweep = (s4_prev_angle is not None and angle_deg < s4_prev_angle)
                    s4_prev_angle = angle_deg

                    # ---- OBJECTS (state1 logic + smoothing/merge/track) ----
                    bin_a = int(round(angle_deg / SERVO_STEP_DEG) * SERVO_STEP_DEG)
                    mix_dist_smooth[bin_a].append(dist_cm)
                    dm = sorted(mix_dist_smooth[bin_a])[len(mix_dist_smooth[bin_a])//2]
                    s4_obj_points.append((bin_a, dm))
                    obj_clusters = cluster_points_obj(s4_obj_points)
                    s4_obj_detections = [o for o in obj_clusters if o["n"] >= OBJ_MIN_POINTS]
                    s4_obj_detections = merge_clusters(s4_obj_detections)
                    s4_obj_detections = mix_obj_tracker.update(s4_obj_detections)

                    # ---- LIGHT (state3 logic, NO WIDTH) ----
                    # ignore invalid/saturated readings
                    if not (mV_val <= 0 or mV_val >= 5000):
                        if angle_deg < 90:
                            cal = cal_A0; start, step = grid_A0
                        else:
                            cal = cal_A3; start, step = grid_A3

                        if cal and len(cal) >= 2:
                            far_cm = start + (len(cal)-1) * step
                            dist_light = distance_cm_from_mV_int(mV_val, cal, start, step)
                            dist_light = max(R_MIN_ACCEPT, min(R_MAX_ACCEPT, dist_light))
                            if accept_light_sample(mV_val, cal, dist_light, far_cm):
                                if s4_live_light_smooth is None:
                                    s4_live_light_smooth = float(dist_light)
                                else:
                                    s4_live_light_smooth = (
                                        LIGHT_LIVE_EMA_ALPHA * float(dist_light)
                                        + (1.0 - LIGHT_LIVE_EMA_ALPHA) * s4_live_light_smooth
                                    )
                                s4_live_light = (angle_deg, s4_live_light_smooth)

                                # median-of-3 smoothing for light distance
                                mix_light_smooth[bin_a].append(dist_light)
                                dlm = sorted(mix_light_smooth[bin_a])[len(mix_light_smooth[bin_a])//2]
                                s4_light_points.append((bin_a, dlm))
                                light_clusters = cluster_points_light(s4_light_points)
                                s4_light_detections = [o for o in light_clusters if o["n"] >= LIGHT_MIN_POINTS]
                                s4_light_detections = merge_clusters(s4_light_detections, angle_gap_deg=4.0, dist_gap_cm=4.0)
                                s4_light_detections = mix_light_tracker.update(s4_light_detections)
                    # else: skip

                    if new_sweep:
                        s4_obj_points = []
                        s4_light_points = []

                    now = time.time()
                    if now - mix_last_draw > DRAW_INTERVAL and mix_ax is not None and mix_fig_canvas is not None:
                        _radar_update_mix(mix_ax, s4_obj_detections, s4_light_detections, live_light=s4_live_light)
                        mix_fig_canvas.draw_idle()
                        mix_last_draw = now

        # ---------------- Calibration (state5) ----------------
        if event == "-CALIB-":
            try:
                sio.write(ENTER_STATE5_CMD)
            except Exception:
                pass
            calib_layout = [
                [sg.Text("Calibration (state5)", font=("Segoe UI", 11, "bold"))],
                [sg.Multiline(size=(72, 12), key="-CALIB-LOG-", autoscroll=True, disabled=True, write_only=True)],
                [sg.Button("Recalibration", key="-CALIB-RECAL-"),
                 sg.Button("Close", key="-CALIB-CLOSE-")]
            ]
            calib_win = sg.Window("Calibration", calib_layout, modal=False, finalize=True)

        if calib_win is not None:
            cev, cvals = calib_win.read(timeout=0)
            if cev == "-CALIB-RECAL-":
                try:
                    sio.write(ENTER_STATE5_CMD)
                    calib_win["-CALIB-LOG-"].update("Recalibration requested...\n", append=True)
                except Exception:
                    calib_win["-CALIB-LOG-"].update("[ERR] Write failed]\n", append=True)
            elif cev in (None, "-CALIB-CLOSE-"):
                try: sio.write(EXIT_STATE_CMD)
                except Exception: pass
                calib_win.close(); calib_win=None
            elif cev == "-CALIB-RX-LINE-":
                line = cvals["-CALIB-RX-LINE-"]
                calib_win["-CALIB-LOG-"].update(line + "\n", append=True)

    # Cleanup
    try:
        if telem_win is not None:
            exit_state2_cleanly()
    except Exception:
        pass
    sio.close()
    win.close()

if __name__ == "__main__":
    main()
