# -*- coding: utf-8 -*-
"""
BNO085 实时姿态绘图器  (matplotlib 版)
=======================================
数据协议 (来自 firmware src/main.cpp):
    每行:  i j k real  roll_deg pitch_deg yaw_deg     (7 列, 空格分隔, ~100 Hz, 115200 8N1)
    或旧: i j k real                                     (4 列, RPY 由上位机反算, 兼容)

使用:
    pip install -r requirements.txt
    python app.py                       # 自动选第一个串口
    python app.py --port COM5           # 指定串口 (Windows)
    python app.py --port /dev/ttyACM0   # 指定串口 (Linux)

画图:
    左:  3D 立方体 + X/Y/Z 轴箭头跟随姿态旋转
    右:  Roll / Pitch / Yaw 实时时序曲线 (红/绿/蓝)

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

# ============================================================================
# 1. 协议解析
# ============================================================================

def parse_frame(line):
    """一行 -> dict 或 None.  支持 7 列 (新协议) 或 4 列 (兼容)."""
    parts = line.split()
    if len(parts) not in (4, 7):
        return None
    try:
        v = [float(x) for x in parts[:7]]
    except ValueError:
        return None
    fr = {"i": v[0], "j": v[1], "k": v[2], "real": v[3]}
    if len(parts) == 7:
        fr["roll_deg"], fr["pitch_deg"], fr["yaw_deg"] = v[4], v[5], v[6]
    else:
        i, j, k, r = v[:4]
        n = np.sqrt(i*i + j*j + k*k + r*r) or 1.0
        i, j, k, r = i/n, j/n, k/n, r/n
        # 标准 ZYX Tait-Bryan, 弧度 → 度
        sr = 2*(r*i + j*k);   cr = 1 - 2*(i*i + j*j);   roll  = np.degrees(np.arctan2(sr, cr))
        sp = np.clip(2*(r*j - k*i), -1.0, 1.0);         pitch = np.degrees(np.arcsin(sp))
        sy = 2*(r*k + i*j);   cy = 1 - 2*(j*j + k*k);   yaw   = np.degrees(np.arctan2(sy, cy))
        fr["roll_deg"], fr["pitch_deg"], fr["yaw_deg"] = roll, pitch, yaw
    return fr


# ============================================================================
# 2. 后台串口读取: 保留最新帧 + 滑动窗口历史
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
                fr["hz"] = self._hz
                with self._lock:
                    self._latest = fr
                    self._history.append(fr)

    def snapshot(self):
        """线程安全地拷一份 (latest, history, hz)."""
        with self._lock:
            return self._latest, list(self._history), self._hz

    def stop(self):
        self._closed = True
        try: self.ser.close()
        except Exception: pass


# ============================================================================
# 3. 四元数工具
# ============================================================================

def quat_to_rot(i, j, k, r):
    """单位四元数 (i,j,k,real) → 3x3 旋转矩阵.  不做归一化检查 (BNO085 出厂即单位四元组)."""
    return np.array([
        [1 - 2*(j*j + k*k), 2*(i*j - k*r),     2*(i*k + j*r)],
        [2*(i*j + k*r),     1 - 2*(i*i + k*k), 2*(j*k - i*r)],
        [2*(i*k - j*r),     2*(j*k + i*r),     1 - 2*(i*i + j*j)],
    ])


# ============================================================================
# 4. 3D 立方体模型
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
    """端口选择优先级:
        1) --port 显式指定
        2) preferred (默认 COM4) 在扫描列表里
        3) 扫描到的第一个
    """
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
    ap = argparse.ArgumentParser(description="BNO085 matplotlib 实时姿态")
    ap.add_argument("--port", default=None,
                    help="串口设备名 (默认: 自动扫描, 优先 COM4, 否则第一个)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=int, default=400, help="时序窗口长度 (采样点)")
    args = ap.parse_args()

    reader = SerialReader(pick_port(args.port), args.baud, maxlen=args.window)
    reader.start()

    # ---- Figure ----
    fig = plt.figure(figsize=(11, 5))
    fig.canvas.manager.set_window_title("BNO085 Attitude (matplotlib)")

    ax3d = fig.add_subplot(121, projection='3d')
    ax3d.set_xlim(-1.6, 1.6); ax3d.set_ylim(-1.6, 1.6); ax3d.set_zlim(-1.6, 1.6)
    ax3d.set_box_aspect((1, 1, 1))
    # body frame 约定 (在 BNO085 sensor frame 上做 X/Y 翻转):
    #   +X (display) = -X (body) → 后 (backward)        getRoll() 绕 X 转 = nose up/down
    #   +Y (display) = -Y (body) → 右 (rightward)        getPitch() 绕 Y 转 = banking
    #   +Z (display) = +Z (body) → 上 (upward)           getYaw() 绕 Z 转 = heading
    # 默认 "驾驶舱视角": azim=180 让静止时 红轴朝相机(后), 绿轴屏右(右), 蓝轴屏上(上).
    ax3d.set_xlabel("X → 后/前")
    ax3d.set_ylabel("Y → 右/左")
    ax3d.set_zlabel("Z → 上/下")
    ax3d.view_init(elev=15, azim=180)
    ax3d.set_title("Attitude  --")

    cube_poly = Poly3DCollection(
        [CUBE_V[list(f)] for f in CUBE_FACES],
        facecolors=FACE_COLORS, edgecolors='k', linewidths=0.8, alpha=0.85,
    )
    ax3d.add_collection3d(cube_poly)

    # X / Y / Z body axes (3 条线段, 用 set_data/set_3d_properties 原地更新)
    axis_lines = [ax3d.plot([0, 0], [0, 0], [0, 0], '-', color=c, lw=2.2)[0]
                  for c in AXIS_COLORS]

    # ---- 时序图 ----
    ax_ts = fig.add_subplot(122)
    ax_ts.set_xlabel("samples (oldest → newest)")
    ax_ts.set_ylabel("deg")
    ax_ts.set_ylim(-180, 180)
    ax_ts.set_xlim(0, args.window)
    ax_ts.grid(True, alpha=0.3)
    ax_ts.axhline(0, color='#888', lw=0.5)
    line_r, = ax_ts.plot([], [], '-',  color='#dc5050', lw=1.3, label='Roll')
    line_p, = ax_ts.plot([], [], '-',  color='#50dc50', lw=1.3, label='Pitch')
    line_y, = ax_ts.plot([], [], '-',  color='#5080dc', lw=1.3, label='Yaw')
    ax_ts.legend(loc='upper right', fontsize=9)

    # ---- 帧更新 ----
    # 显示坐标系约定: X/Y 翻转 (与 body frame 反号), Z 不变.
    # 用一个 3x3 对角矩阵左乘 R 等价于对旋转后的 X/Y 列取反.
    DISPLAY_FLIP_XY = np.diag([-1.0, -1.0, 1.0])

    def update(_frame):
        latest, hist, hz = reader.snapshot()
        if latest is None:
            return []

        i, j, k, r = latest["i"], latest["j"], latest["k"], latest["real"]
        R = DISPLAY_FLIP_XY @ quat_to_rot(i, j, k, r)
        cube_poly.set_verts([(CUBE_V @ R.T)[list(f)] for f in CUBE_FACES])

        # body 轴箭头: 旋转后的单位向量
        for line, vec in zip(axis_lines, R.T):
            xs, ys, zs = zip((0, 0, 0), tuple(vec * 1.4))
            line.set_data(xs, ys)
            line.set_3d_properties(zs)

        ax3d.set_title(
            f"Roll {latest['roll_deg']:+6.1f}°  Pitch {latest['pitch_deg']:+6.1f}°  "
            f"Yaw {latest['yaw_deg']:+6.1f}°   {hz:4.1f} Hz"
        )

        n = len(hist)
        if n >= 2:
            xs = np.arange(n)
            line_r.set_data(xs, [h["roll_deg"]  for h in hist])
            line_p.set_data(xs, [h["pitch_deg"] for h in hist])
            line_y.set_data(xs, [h["yaw_deg"]   for h in hist])

        return []

    # FuncAnimation 必须持有引用, 否则会被 GC, 动画停. 绑定到 fig 上保活.
    fig._ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    # 窗口关闭 → 停线程
    fig.canvas.mpl_connect('close_event', lambda _evt: reader.stop())

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
