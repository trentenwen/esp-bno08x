# -*- coding: utf-8 -*-
"""
BNO085 实时姿态 + G 力绘图器  (matplotlib 版)
===============================================
数据协议 (来自 firmware src/main.cpp):
     6 列 (新):  roll_deg pitch_deg yaw_deg  ax ay az      (10 Hz, 115200 8N1)
                 ax/ay/az = LinearAccelerometer, m/s², 已扣重力, sensor frame
                            (X=右, Y=前/镜头方向, Z=上)
    或  7 列 (旧): i j k r roll_deg pitch_deg yaw_deg        (兼容, 无加速度)

使用:
    pip install -r requirements.txt
    python app.py                       # 自动选第一个串口
    python app.py --port COM5           # 指定串口 (Windows)
    python app.py --port /dev/ttyACM0   # 指定串口 (Linux)

画图:
    左上:  3D 立方体 (姿态)
    右上:  Roll / Pitch / Yaw 实时时序曲线 (红/绿/蓝)
    左下:  三轴 G 力时序 + |a| 总 G (G = m/s² / 9.80665)
    右下:  X-Y 平面加速度矢量图 (sensor frame, 看横移/前后方向)

按键:
    关闭窗口即停止 (后台读串口线程自动结束)
"""
import argparse, sys, time, threading
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
import serial

G0 = 9.80665  # 标准重力加速度, m/s²

# ============================================================================
# 1. 协议解析
# ============================================================================

FRAME_NCOLS = (6, 7)  # 6 = 新协议 (rpy + ax ay az), 7 = 旧协议 (quat + rpy)


def parse_frame(line):
    """一行 -> dict 或 None.  支持 6 列 (新) / 7 列 (旧)."""
    parts = line.split()
    if len(parts) not in FRAME_NCOLS:
        return None
    try:
        v = [float(x) for x in parts]
    except ValueError:
        return None

    if len(parts) == 6:
        roll, pitch, yaw, ax, ay, az = v
    else:
        # 旧协议: i j k r roll pitch yaw —— 没有加速度字段.
        roll, pitch, yaw, ax, ay, az = v[4], v[5], v[6], 0.0, 0.0, 0.0

    return {
        "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
        "ax": ax, "ay": ay, "az": az,           # m/s², sensor frame, 已扣重力
        "ax_g": ax / G0, "ay_g": ay / G0, "az_g": az / G0,
        "g_mag": float(np.sqrt(ax * ax + ay * ay + az * az) / G0),
    }


# ============================================================================
# 2. 后台串口读取
# ============================================================================

class SerialReader(threading.Thread):
    """daemon 线程, pyserial 非阻塞读, 主线程 latest()/history() 拉取不卡 GUI."""
    def __init__(self, port, baud=115200, maxlen=400):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        self._lock = threading.Lock()
        self._latest = None
        self._history = deque(maxlen=maxlen)
        self._buf = b""
        self._closed = False
        self._t0 = None
        self._hz = 0.0
        self._hz_win = deque(maxlen=20)

    def run(self):
        while not self._closed:
            try:
                chunk = self.ser.read(128)
            except Exception:
                break
            if not chunk:
                continue
            self._buf += chunk
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                fr = parse_frame(line.decode("utf-8", errors="ignore"))
                if fr is None:
                    continue
                now = time.monotonic()
                if self._t0 is None:
                    self._t0 = now
                else:
                    dt = now - self._t0
                    self._t0 = now
                    if dt > 0:
                        self._hz_win.append(1.0 / dt)
                        self._hz = sum(self._hz_win) / len(self._hz_win)
                fr["t"] = now
                with self._lock:
                    self._latest = fr
                    self._history.append(fr)

    def snapshot(self):
        with self._lock:
            return self._latest, list(self._history), self._hz

    def stop(self):
        self._closed = True
        try: self.ser.close()
        except Exception: pass


# ============================================================================
# 3. 姿态工具 (固件只传 RPY, 这里还原旋转矩阵)
# ============================================================================

def rpy_to_rot(roll_deg, pitch_deg, yaw_deg):
    """roll/pitch/yaw (deg) → 3x3 旋转矩阵 (body → world).

    固件的 getRoll/getPitch/getYaw 用的是标准 ZYX Tait-Bryan 分解
    (见 lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp:169-239),
    对应 R = Rz(yaw) · Ry(pitch) · Rx(roll), 这里按同一约定还原.
    """
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


# ============================================================================
# 4. 3D 模型
# ============================================================================

CUBE_V = np.array([
    [-1,-1,-1], [ 1,-1,-1], [ 1, 1,-1], [-1, 1,-1],
    [-1,-1, 1], [ 1,-1, 1], [ 1, 1, 1], [-1, 1, 1],
], dtype=float)

CUBE_FACES = [
    (0,1,2,3), (4,5,6,7),
    (0,4,7,3), (1,5,6,2),
    (3,2,6,7), (0,1,5,4),
]
FACE_COLORS = ['#b84646', '#46b86e', '#466eb8', '#e6c846', '#c846b8', '#46b8b8']

AXIS_COLORS = ['#dc5050', '#50dc50', '#5080dc']  # X 红 / Y 绿 / Z 蓝


# ============================================================================
# 5. 主程序
# ============================================================================

def pick_port(arg, preferred="COM4"):
    if arg:
        return arg
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口, 用 --port 指定", file=sys.stderr); sys.exit(1)
    for p in ports:
        if p.device.upper() == preferred.upper():
            print(f"[auto] 使用 {p.device}  ({p.description or '?'}, 命中 preferred={preferred})")
            return p.device
    p = ports[0]
    print(f"[auto] 使用 {p.device}  ({p.description or '?'}, 未找到 {preferred}, 回退到第一个)")
    return p.device


def main():
    ap = argparse.ArgumentParser(description="BNO085 matplotlib 实时姿态 + G 力")
    ap.add_argument("--port", default=None, help="串口设备名 (默认: 自动扫描, 优先 COM4)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=int, default=400, help="时序窗口长度 (采样点)")
    args = ap.parse_args()

    reader = SerialReader(pick_port(args.port), args.baud, maxlen=args.window)
    reader.start()

    # ---- Figure / 布局 ----
    fig = plt.figure(figsize=(12, 7.5))
    fig.canvas.manager.set_window_title("BNO085 Attitude + G-force (matplotlib)")
    gs = fig.add_gridspec(2, 2, height_ratios=[2.2, 1], hspace=0.32, wspace=0.25)

    # ---- 左上: 3D 立方体 (纯姿态) ----
    ax3d = fig.add_subplot(gs[0, 0], projection='3d')
    LIM = 1.8
    ax3d.set_xlim(-LIM, LIM); ax3d.set_ylim(-LIM, LIM); ax3d.set_zlim(-LIM, LIM)
    ax3d.set_box_aspect((1, 1, 1))
    ax3d.set_xlabel("X → 后/前"); ax3d.set_ylabel("Y → 右/左"); ax3d.set_zlabel("Z → 上/下")
    ax3d.view_init(elev=15, azim=180)
    ax3d.set_title("Attitude  --")

    cube_poly = Poly3DCollection(
        [CUBE_V[list(f)] for f in CUBE_FACES],
        facecolors=FACE_COLORS, edgecolors='k', linewidths=0.8, alpha=0.85,
    )
    ax3d.add_collection3d(cube_poly)

    axis_lines = [ax3d.plot([0, 0], [0, 0], [0, 0], '-', color=c, lw=2.2)[0]
                  for c in AXIS_COLORS]

    # 显示坐标系约定: 与 body frame 的 X/Y 反号, Z 同号 (驾驶舱视角).
    DISPLAY_FLIP_XY = np.diag([-1.0, -1.0, 1.0])

    # ---- 右上: RPY 时序 ----
    ax_ts = fig.add_subplot(gs[0, 1])
    ax_ts.set_xlabel("samples (oldest → newest)")
    ax_ts.set_ylabel("deg"); ax_ts.set_ylim(-180, 180); ax_ts.set_xlim(0, args.window)
    ax_ts.grid(True, alpha=0.3); ax_ts.axhline(0, color='#888', lw=0.5)
    line_r, = ax_ts.plot([], [], '-',  color='#dc5050', lw=1.3, label='Roll')
    line_p, = ax_ts.plot([], [], '-',  color='#50dc50', lw=1.3, label='Pitch')
    line_y, = ax_ts.plot([], [], '-',  color='#5080dc', lw=1.3, label='Yaw')
    ax_ts.legend(loc='upper right', fontsize=9)

    # ---- 左下: G 力时序 (三轴 + |a|) ----
    ax_g = fig.add_subplot(gs[1, 0])
    ax_g.set_xlabel("samples"); ax_g.set_ylabel("G  (m/s² ÷ 9.807)")
    ax_g.set_ylim(-2.0, 2.0); ax_g.set_xlim(0, args.window)
    ax_g.grid(True, alpha=0.3); ax_g.axhline(0, color='#888', lw=0.5)
    line_gx, = ax_g.plot([], [], '-',  color='#dc8c50', lw=1.0, label='ax (right)')
    line_gy, = ax_g.plot([], [], '-',  color='#50dc50', lw=1.0, label='ay (fwd)')
    line_gz, = ax_g.plot([], [], '-',  color='#5080dc', lw=1.0, label='az (up)')
    line_gm, = ax_g.plot([], [], '--', color='#cc44cc', lw=1.4, label='|a|')
    ax_g.legend(loc='upper right', fontsize=8, ncol=2)
    ax_g.set_title("G-force  --", fontsize=10)

    # ---- 右下: X-Y 平面加速度矢量 (sensor frame) ----
    ax_xy = fig.add_subplot(gs[1, 1])
    ax_xy.set_xlabel("ax  (right)")
    ax_xy.set_ylabel("ay  (fwd)")
    ax_xy.set_aspect('equal')
    LIM_G = 2.0
    ax_xy.set_xlim(-LIM_G, LIM_G); ax_xy.set_ylim(-LIM_G, LIM_G)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axhline(0, color='#888', lw=0.5); ax_xy.axvline(0, color='#888', lw=0.5)
    # 1 g 参考圈 + 十字
    ax_xy.plot(np.cos(np.linspace(0, 2*np.pi, 200)),
               np.sin(np.linspace(0, 2*np.pi, 200)),
               '-', color='#bbb', lw=0.8, zorder=0)
    ax_xy.text(1.03, 0, '1 g', color='#888', fontsize=8, va='center')
    xy_trail, = ax_xy.plot([], [], '-', color='#ff8c44', lw=1.5, alpha=0.85)
    xy_now, = ax_xy.plot([], [], 'o', color='#cc44cc', ms=8)

    acc_trail = deque(maxlen=120)  # ~12 s @ 10 Hz, 单位 g

    def update(_frame):
        latest, hist, hz = reader.snapshot()
        if latest is None:
            return []

        R = DISPLAY_FLIP_XY @ rpy_to_rot(latest["roll_deg"],
                                         latest["pitch_deg"],
                                         latest["yaw_deg"])

        # ---- 立方体 + body 轴 ----
        cube_poly.set_verts([(CUBE_V @ R.T)[list(f)] for f in CUBE_FACES])
        for line, vec in zip(axis_lines, R.T):
            xs, ys, zs = zip((0.0, 0.0, 0.0), tuple(vec * 1.4))
            line.set_data(xs, ys)
            line.set_3d_properties(zs)

        # ---- 标题 ----
        ax3d.set_title(
            f"Roll {latest['roll_deg']:+6.1f}°  Pitch {latest['pitch_deg']:+6.1f}°  "
            f"Yaw {latest['yaw_deg']:+6.1f}°   |a| {latest['g_mag']:.2f} g   "
            f"{hz:4.1f} Hz"
        )

        # ---- RPY 时序 ----
        n = len(hist)
        if n >= 2:
            xs = np.arange(n)
            line_r.set_data(xs, [h["roll_deg"]  for h in hist])
            line_p.set_data(xs, [h["pitch_deg"] for h in hist])
            line_y.set_data(xs, [h["yaw_deg"]   for h in hist])

            # ---- 三轴 G 力 (g) ----
            line_gx.set_data(xs, [h["ax_g"] for h in hist])
            line_gy.set_data(xs, [h["ay_g"] for h in hist])
            line_gz.set_data(xs, [h["az_g"] for h in hist])
            line_gm.set_data(xs, [h["g_mag"] for h in hist])

        ax_g.set_title(
            f"G-force   ax {latest['ax_g']:+.2f}   ay {latest['ay_g']:+.2f}   "
            f"az {latest['az_g']:+.2f}   |a| {latest['g_mag']:.2f} g",
            fontsize=10,
        )

        # ---- X-Y 加速度矢量图 (sensor frame) ----
        acc_trail.append((latest["ax_g"], latest["ay_g"]))
        if len(acc_trail) >= 2:
            t = np.asarray(acc_trail)
            xy_trail.set_data(t[:, 0], t[:, 1])
        xy_now.set_data([latest["ax_g"]], [latest["ay_g"]])

        return []

    fig._ani = FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)
    fig.canvas.mpl_connect('close_event', lambda _evt: reader.stop())

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()