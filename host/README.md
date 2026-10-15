
# BNO085 姿态上位机  (host/)

单文件 matplotlib 实时姿态绘图器：显示 ESP32-C3 + BNO085 模块回传的四元数与欧拉角。

## 数据协议

固件 [src/main.cpp](../src/main.cpp) 以约 100 Hz 输出每帧 7 个 float
（前 4 列为四元数，后 3 列为固件内直接解算的 Roll/Pitch/Yaw，度）：

    i j k real  roll_deg pitch_deg yaw_deg
    例如: 0.0123 -0.0045 0.9912 0.1305   1.20  -0.40 +178.70

- 端口：115200 8N1（USB-CDC，无需额外串口驱动）
- 欧拉角由**固件**直接解算（库内 `getRoll/Pitch/Yaw`，基于 `rawQuat*`），
  上位机直接读取后 3 列；若收到旧 4 列格式，自动回退到上位机反算 RPY。

## 文件清单

| 文件 | 作用 |
|---|---|
| `app.py` | 主程序，单文件，~250 行。matplotlib + pyserial + numpy |
| `requirements.txt` | 依赖：matplotlib, numpy, pyserial |

## 安装

    cd host
    pip install -r requirements.txt

> Windows 上 matplotlib 首次安装约 30 秒。`numpy` 通常已被作为传递依赖装好。

## 运行

    python app.py                       # 自动扫描: 优先选 COM4, 否则第一个
    python app.py --port COM5           # 指定 Windows 串口 (覆盖默认)
    python app.py --port /dev/ttyACM0   # 指定 Linux 串口 (覆盖默认)
    python app.py --window 800          # 时序图窗口长度（采样点，默认 400）

> 不带 `--port` 时，`pick_port()` 会扫描所有可用串口，**优先返回 `COM4`**；
> 没插 COM4 才回退到第一个。Linux 下无 COM 设备时按 `/dev/tty*` 顺序。

启动后弹出 matplotlib 窗口：

| 区域 | 内容 |
|---|---|
| 左：3D 立方体 | 6 面着色 + XYZ 三色坐标轴，跟 BNO085 实时翻转；标题显示当前 RPY 与采样率；默认 "驾驶舱视角"（+X 朝屏幕里 = 前） |
| 右：时序图 | Roll / Pitch / Yaw 三条曲线（红/绿/蓝），横轴为采样点序号 |

### Body frame 约定（X/Y 翻转）

BNO085 sensor frame → 显示 frame 做 `diag(-1, -1, 1)` 翻转，即 +X / +Y 反号：

| 轴 | 显示正方向 | BNO085 角度 | 动作含义 |
|---|---|---|---|
| **X** | 后（向相机） | `getRoll()` 绕 X 转 | nose up / down（俯仰） |
| **Y** | 右（屏右） | `getPitch()` 绕 Y 转 | banking（倾斜） |
| **Z** | 上（屏上） | `getYaw()` 绕 Z 转 | heading（航向） |

**为什么翻 X/Y：** 用户约定 sensor 的"前"是 display frame 的 -X 方向（向后）；
sensor 的"左"是 display frame 的 -Y 方向（向右）。实现见 [host/app.py](host/app.py)
里的 `DISPLAY_FLIP_XY = np.diag([-1, -1, 1])`。

默认相机视角 `view_init(elev=15, azim=180)`（"驾驶舱视角"）：
静止时红轴朝相机（向后）、绿轴在屏右（向右）、蓝轴向上 —— 与新约定一致。
窗口里可用鼠标左键拖拽旋转查看任意角度。

关闭窗口即停止（后台读串口线程自动结束）。

## 实现要点

- **后台读串口**：`SerialReader` 继承 `threading.Thread`（daemon），
  pyserial 非阻塞读，主线程 `snapshot()` 拷一份 latest + history，不阻塞 matplotlib。
- **采样率**：滑动 20 帧时间间隔均值，平滑掉单帧抖动，显示在立方体标题上。
- **3D 旋转**：四元数 → 旋转矩阵 `quat_to_rot()` → 立方体顶点 `set_verts()`；
  X/Y/Z body 轴用 `plot3D` 线段加 `set_3d_properties` 原地更新（不每帧 new 对象）。
- **协议解析**：`parse_frame()` 按空格切，新协议 7 列直接吃，4 列旧格式回退上位机反算 RPY。
- **FuncAnimation 保活**：必须挂到 `fig._ani` 上，否则会被 GC、动画停。

## 常见问题

| 现象 | 解决 |
|---|---|
| 端口列表为空 | 检查 USB 插入；确认 CH340/CP210x 驱动；拔插一次重试 |
| 打开端口失败 | 关闭 Arduino / PlatformIO 串口监视器等已占用进程 |
| 立方体抖动 | BNO085 RV 短期噪声正常；要更平滑可做指数滑动平均后再画 |
| 数据全是 0 | 检查 BNO085 I2C 连线（SDA=8, SCL=9）与 PS1/PS0 上拉 |
| 窗口打开但不更新 | 确认固件 `enableRotationVector()` 真的开了；用 `python -m serial.tools.miniterm COM5 115200` 抓裸数据排查 |

## 进一步扩展（按需添加）

- CSV 录制：`SerialReader._history` 已经在内存里滑动存了，加一个文件 writer 即可
- 离线分析：把 `history` 写盘后丢给 numpy/scipy 做 FFT、Allan deviation
- 多设备：在 `SerialReader` 上加 ID 字段，按端口分桶
- 回放：另起一个脚本读 CSV，用同一份 `quat_to_rot` + `FuncAnimation` 离线重放
