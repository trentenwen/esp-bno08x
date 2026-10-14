# -*- coding: utf-8 -*-
"""
BNO085 姿态上位机 —— 极简单文件版
==================================
数据协议 (来自 firmware src/main.cpp):
    每行:  i j k real     (4 个空格分隔 float, ~10 Hz, 115200 8N1)
使用:
    pip install -r requirements.txt
    python app.py                       # GUI, 自动扫描串口
    python app.py --cli COM5            # 命令行 ASCII 姿态视图
功能:
    * 四元数 (i/j/k/real)
    * 欧拉角 Roll/Pitch/Yaw (ZYX 内旋, 度)
    * 实时采样率 Hz
    * 3D 立方体跟随姿态旋转 (QPainter 自绘, 不依赖 OpenGL)
    * 可选录制 CSV
"""
import serial
import sys, math, time, csv, argparse
from collections import deque
from datetime import datetime

# ============================================================================
# 1. 共用: 四元数解析 & 欧拉角换算
# ============================================================================

def quat_to_euler_xyz(i, j, k, real):
    """四元数 -> (Roll, Pitch, Yaw) 度, ZYX 内旋."""
    n = math.sqrt(i*i + j*j + k*k + real*real)
    if n < 1e-12:
        return 0.0, 0.0, 0.0
    i, j, k, real = i/n, j/n, k/n, real/n
    sinr_cosp = 2.0 * (real*i + j*k)
    cosr_cosp = 1.0 - 2.0 * (i*i + j*j)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (real*j - k*i)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (real*k + i*j)
    cosy_cosp = 1.0 - 2.0 * (j*j + k*k)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def parse_frame(line):
    """解析一行 -> dict 或 None.

    支持两种固件输出:
      - 新协议 (7 列): i j k real roll_deg pitch_deg yaw_deg  (RPY 由固件解算)
      - 旧协议 (4 列): i j k real                          (RPY 由上位机反算, 兼容)
    """
    try:
        parts = line.strip().split()
        if len(parts) not in (4, 7):
            return None
        ii, jj, kk, rr = (float(x) for x in parts[:4])
        fr = {"i": ii, "j": jj, "k": kk, "real": rr}
        if len(parts) == 7:
            fr["roll_deg"]  = float(parts[4])
            fr["pitch_deg"] = float(parts[5])
            fr["yaw_deg"]   = float(parts[6])
        return fr
    except Exception:
        return None

# ============================================================================
# 2. 串口读取器 —— pyserial 延迟导入
# ============================================================================

class SerialReader:
    def __init__(self, port, baud=115200, timeout=1.0):
        
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.buf = b""
        self._t_last = None
        self.hz = 0.0
        self._hz_win = deque(maxlen=20)
        self._total = 0

    def close(self):
        try: self.ser.close()
        except Exception: pass

    def read_frame(self):
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            chunk = self.ser.read(64)
            if not chunk:
                return None
            self.buf += chunk
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                fr = parse_frame(line.decode("utf-8", errors="ignore"))
                if fr is None:
                    continue
                now = time.monotonic()
                if self._t_last is not None:
                    dt = now - self._t_last
                    if dt > 0:
                        self._hz_win.append(1.0/dt)
                        self.hz = sum(self._hz_win)/len(self._hz_win)
                self._t_last = now
                fr["hz"] = self.hz
                self._total += 1
                fr["total"] = self._total
                return fr
        return None

# ============================================================================
# 3. CLI 模式
# ============================================================================

def run_cli(port):
    print("[cli] connecting " + port + " @ 115200   (Ctrl+C 退出)")
    r = SerialReader(port, baud=115200)
    try:
        while True:
            fr = r.read_frame()
            if fr is None:
                continue
            i, j, k, real = fr["i"], fr["j"], fr["k"], fr["real"]
            if "roll_deg" in fr:
                roll, pitch, yaw = fr["roll_deg"], fr["pitch_deg"], fr["yaw_deg"]
            else:
                roll, pitch, yaw = quat_to_euler_xyz(i, j, k, real)
            yaw_n = int((yaw + 180) / 360 * 40)
            bar = "." * 40
            head = max(0, min(39, yaw_n))
            bar = bar[:head] + "^" + bar[head+1:]
            out = "q=({:+.3f},{:+.3f},{:+.3f},{:+.3f})  ".format(i, j, k, real)
            out += "RPY=({:+6.1f},{:+6.1f},{:+6.1f}) deg  ".format(roll, pitch, yaw)
            out += "{:4.1f}Hz  YAW ".format(fr.get("hz", 0)) + bar
            print(out)
    except KeyboardInterrupt:
        print("\n[cli] bye")
    finally:
        r.close()

# ============================================================================
# 4. GUI 模式 —— PySide6
# ============================================================================

def run_gui():
    from PySide6.QtCore import Qt, QTimer, QSize, QPointF, QRectF
    from PySide6.QtGui import (
        QFont, QColor, QPainter, QPen, QBrush, QPolygonF, QFontDatabase,
    )
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
        QComboBox, QPushButton, QLabel, QFileDialog, QMessageBox,
        QGroupBox, QStatusBar, QFrame, QSizePolicy,
    )
    from serial.tools import list_ports

    MONO = QFontDatabase.systemFont(QFontDatabase.FixedFont)


    # ----- 4.1 3D 立方体 (QPainter + 顶点投影, 无 OpenGL) -----
    class CubeView(QWidget):
        VERTS = [
            (-1,-1,-1), ( 1,-1,-1), ( 1, 1,-1), (-1, 1,-1),
            (-1,-1, 1), ( 1,-1, 1), ( 1, 1, 1), (-1, 1, 1),
        ]
        FACES = [
            (0,1,2,3, QColor(180, 70, 70)),
            (4,5,6,7, QColor( 70,180,110)),
            (0,4,7,3, QColor( 70,110,200)),
            (1,5,6,2, QColor(230,200, 70)),
            (3,2,6,7, QColor(200,110,200)),
            (0,1,5,4, QColor(110,200,200)),
        ]
        AXES = [
            ((1.5,0,0), QColor(220, 90, 90)),
            ((0,1.5,0), QColor( 90,220, 90)),
            ((0,0,1.5), QColor( 90,130,230)),
        ]

        def __init__(self, parent=None):
            super().__init__(parent)
            self.setMinimumSize(QSize(360, 360))
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.qi = self.qj = self.qk = 0.0
            self.qr = 1.0

        def set_quat(self, i, j, k, real):
            self.qi, self.qj, self.qk, self.qr = i, j, k, real
            self.update()

        @staticmethod
        def _qrot(i, j, k, real, x, y, z):
            tx = 2*(j*z - k*y); ty = 2*(k*x - i*z); tz = 2*(i*y - j*x)
            cx = j*tz - k*ty; cy = k*tx - i*tz; cz = i*ty - j*tx
            return (x + real*tx + cx, y + real*ty + cy, z + real*tz + cz)

        def paintEvent(self, _ev):
            p = QPainter(self)
            try:
                p.setRenderHint(QPainter.Antialiasing, True)
                w, h = self.width(), self.height()
                p.fillRect(self.rect(), QColor(20, 22, 28))
                cx, cy = w/2, h/2
                scale = min(w, h) * 0.16
                i, j, k, real = self.qi, self.qj, self.qk, self.qr
                n = math.sqrt(i*i+j*j+k*k+real*real) or 1.0
                i, j, k, real = i/n, j/n, k/n, real/n
                face_polys = []
                for fi, face in enumerate(self.FACES):
                    idx0, idx1, idx2, idx3, color = face
                    pts = []
                    zavg = 0.0
                    for idx in (idx0, idx1, idx2, idx3):
                        x, y, z = self.VERTS[idx]
                        rx, ry, rz = self._qrot(i, j, k, real, x, y, z)
                        pts.append(QPointF(cx + rx*scale, cy - ry*scale))
                        zavg += rz
                    zavg /= 4.0
                    face_polys.append((zavg, color, pts))
                axes_pts = []
                for (vx, vy, vz), color in self.AXES:
                    rx, ry, rz = self._qrot(i, j, k, real, vx, vy, vz)
                    axes_pts.append((cx + rx*scale, cy - ry*scale, color))
                face_polys.sort(key=lambda t: t[0])
                for sx, sy, color in axes_pts:
                    p.setPen(QPen(color, 2))
                    p.drawLine(QPointF(cx, cy), QPointF(sx, sy))
                for _, color, pts in face_polys:
                    p.setPen(QPen(color.darker(160), 1.4))
                    p.setBrush(QBrush(color))
                    p.drawPolygon(QPolygonF(pts))
                p.setPen(QColor(200,200,200))
                p.setFont(QFont("Segoe UI", 9))
                qstr = "q=({:+.3f},{:+.3f},{:+.3f},{:+.3f})".format(i, j, k, real)
                p.drawText(QRectF(8, h-22, w-16, 18),
                           Qt.AlignLeft | Qt.AlignVCenter, qstr)
            finally:
                p.end()


    # ----- 4.2 主窗口 -----
    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("BNO085 姿态上位机  (host/)")
            self.resize(960, 620)
            central = QWidget(); self.setCentralWidget(central)
            root = QVBoxLayout(central); root.setContentsMargins(10,10,10,10)

            # 顶部控制条
            bar = QHBoxLayout()
            bar.addWidget(QLabel("端口:"))
            self.port_cb = QComboBox(); self.port_cb.setMinimumWidth(180)
            bar.addWidget(self.port_cb)
            rb = QPushButton("刷新"); rb.clicked.connect(self.refresh_ports); bar.addWidget(rb)
            self.conn_btn = QPushButton("连接"); self.conn_btn.setCheckable(True)
            self.conn_btn.clicked.connect(self.toggle_connect); bar.addWidget(self.conn_btn)
            self.rec_btn = QPushButton("开始录制 CSV"); self.rec_btn.setCheckable(True)
            self.rec_btn.clicked.connect(self.toggle_record); bar.addWidget(self.rec_btn)
            bar.addStretch(1)
            self.conn_state_lbl = QLabel("●  未连接")
            self.conn_state_lbl.setStyleSheet("color:#c66;")
            bar.addWidget(self.conn_state_lbl)
            root.addLayout(bar)

            # 中部: 立方体 | 数据面板
            mid = QHBoxLayout(); root.addLayout(mid, stretch=1)
            cube_box = QGroupBox("3D 姿态 (立方体)")
            cv = QVBoxLayout(cube_box)
            self.cube = CubeView(); cv.addWidget(self.cube)
            legend_html = (
                '<span style="color:#dc5050;">■ X</span> &nbsp; '
                '<span style="color:#50dc50;">■ Y</span> &nbsp; '
                '<span style="color:#5080dc;">■ Z</span>'
            )
            legend = QLabel(legend_html)
            legend.setAlignment(Qt.AlignRight)
            cv.addWidget(legend)
            mid.addWidget(cube_box, stretch=3)
            data_box = QGroupBox("姿态数据")
            grid = QGridLayout(data_box)
            grid.setHorizontalSpacing(14); grid.setVerticalSpacing(8)
            mid.addWidget(data_box, stretch=2)
            big = QFont(MONO.family(), 13); big.setBold(True)
            med = QFont(MONO.family(), 11)

            def row(r, label, fmt):
                l = QLabel(label); l.setFont(med)
                v = QLabel("--"); v.setFont(big)
                v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                grid.addWidget(l, r, 0); grid.addWidget(v, r, 1)
                return v, fmt

            self.lbl_i,    self.fmt_f4 = row(0, "i      ", lambda v: "{:+.4f}".format(v))
            self.lbl_j, _               = row(1, "j      ", lambda v: "{:+.4f}".format(v))
            self.lbl_k, _               = row(2, "k      ", lambda v: "{:+.4f}".format(v))
            self.lbl_real, _            = row(3, "real   ", lambda v: "{:+.4f}".format(v))
            sep = QFrame(); sep.setFrameShape(QFrame.HLine); sep.setFrameShadow(QFrame.Sunken)
            grid.addWidget(sep, 4, 0, 1, 2)
            self.lbl_roll,  self.fmt_d  = row(5, "Roll ° ",  lambda v: "{:+7.2f}°".format(v))
            self.lbl_pitch, _           = row(6, "Pitch °", lambda v: "{:+7.2f}°".format(v))
            self.lbl_yaw,   _           = row(7, "Yaw °  ", lambda v: "{:+7.2f}°".format(v))
            sep2 = QFrame(); sep2.setFrameShape(QFrame.HLine); sep2.setFrameShadow(QFrame.Sunken)
            grid.addWidget(sep2, 8, 0, 1, 2)
            self.lbl_hz, _ = row(9,  "Sample  ", lambda v: "{:5.1f} Hz".format(v))
            self.lbl_n,  _ = row(10, "Frames  ", lambda v: str(int(v)))

            self.status = QStatusBar(); self.setStatusBar(self.status)
            self.status.showMessage("就绪  ·  选择端口后点 连接")

            self.reader = None
            self.csv_fp = None
            self.csv_writer = None
            self.frame_count = 0

            self.refresh_ports()
            self.timer = QTimer(self); self.timer.setInterval(40)
            self.timer.timeout.connect(self.tick); self.timer.start()

        def refresh_ports(self):
            self.port_cb.clear()
            for p in list_ports.comports():
                self.port_cb.addItem("{}  ({})".format(p.device, p.description or "?"), p.device)
            if self.port_cb.count() == 0:
                self.port_cb.addItem("未发现串口")

        def toggle_connect(self, on):
            if on:
                port = self.port_cb.currentData() or self.port_cb.currentText()
                if not port or "未发现" in port:
                    QMessageBox.warning(self, "提示", "没有可用的串口")
                    self.conn_btn.setChecked(False); return
                try:
                    self.reader = SerialReader(port, baud=115200)
                except Exception as e:
                    QMessageBox.critical(self, "打开串口失败", str(e))
                    self.conn_btn.setChecked(False); return
                self.conn_btn.setText("断开")
                self.conn_state_lbl.setText("●  已连接 " + port)
                self.conn_state_lbl.setStyleSheet("color:#6c6;")
                self.status.showMessage("{} @ 115200  ·  数据流中".format(port))
            else:
                if self.reader: self.reader.close(); self.reader = None
                self.conn_btn.setText("连接")
                self.conn_state_lbl.setText("●  未连接")
                self.conn_state_lbl.setStyleSheet("color:#c66;")
                self.status.showMessage("已断开")

        def toggle_record(self, on):
            if on:
                fn, _ = QFileDialog.getSaveFileName(
                    self, "保存 CSV",
                    "bno085_{}.csv".format(datetime.now().strftime("%Y%m%d_%H%M%S")),
                    "CSV (*.csv)")
                if not fn:
                    self.rec_btn.setChecked(False); return
                try:
                    self.csv_fp = open(fn, "w", newline="", encoding="utf-8")
                except Exception as e:
                    QMessageBox.critical(self, "无法写入", str(e))
                    self.rec_btn.setChecked(False); return
                self.csv_writer = csv.writer(self.csv_fp)
                self.csv_writer.writerow(
                    ["time_iso","i","j","k","real",
                     "roll_deg","pitch_deg","yaw_deg","sample_hz"])
                self.rec_btn.setText("停止录制")
                self.status.showMessage("录制中 -> " + fn)
            else:
                if self.csv_fp: self.csv_fp.close()
                self.csv_fp = None; self.csv_writer = None
                self.rec_btn.setText("开始录制 CSV")
                self.status.showMessage("录制已停止")

        def tick(self):
            if not self.reader: return
            fr = self.reader.read_frame()
            if fr is None: return
            i, j, k, real = fr["i"], fr["j"], fr["k"], fr["real"]
            if "roll_deg" in fr:
                roll, pitch, yaw = fr["roll_deg"], fr["pitch_deg"], fr["yaw_deg"]
            else:
                roll, pitch, yaw = quat_to_euler_xyz(i, j, k, real)
            self.lbl_i.setText(self.fmt_f4(i))
            self.lbl_j.setText(self.fmt_f4(j))
            self.lbl_k.setText(self.fmt_f4(k))
            self.lbl_real.setText(self.fmt_f4(real))
            self.lbl_roll.setText(self.fmt_d(roll))
            self.lbl_pitch.setText(self.fmt_d(pitch))
            self.lbl_yaw.setText(self.fmt_d(yaw))
            self.lbl_hz.setText("{:5.1f} Hz".format(fr.get("hz", 0)))
            self.frame_count = fr.get("total", self.frame_count+1)
            self.lbl_n.setText(str(int(self.frame_count)))
            self.cube.set_quat(i, j, k, real)
            if self.csv_writer:
                self.csv_writer.writerow([
                    datetime.now().isoformat(timespec="milliseconds"),
                    "{:.6f}".format(i), "{:.6f}".format(j),
                    "{:.6f}".format(k), "{:.6f}".format(real),
                    "{:.4f}".format(roll), "{:.4f}".format(pitch),
                    "{:.4f}".format(yaw),
                    "{:.2f}".format(fr.get("hz", 0)),
                ])
                if self.frame_count % 10 == 0:
                    self.csv_fp.flush()

        def closeEvent(self, e):
            try:
                if self.reader: self.reader.close()
                if self.csv_fp: self.csv_fp.close()
            finally:
                super().closeEvent(e)

    app = QApplication.instance() or QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec())

# ============================================================================
# 5. 入口
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="BNO085 极简姿态上位机")
    ap.add_argument("--cli", metavar="PORT",
                    help="命令行模式: --cli COM5  或  --cli /dev/ttyACM0")
    args = ap.parse_args()
    if args.cli:
        run_cli(args.cli)
    else:
        try:
            run_gui()
        except ImportError as e:
            name = (e.name or "PySide6/pyserial").split(".")[0]
            sys.stderr.write(
                "依赖缺失: %s 未安装.\n"
                "请在 host/ 下运行: pip install -r requirements.txt\n"
                "(原始错误: %s)\n" % (name, e)
            )
            sys.exit(1)


if __name__ == "__main__":
    main()
