
# BNO085 姿态上位机  (host/)

极简单文件版上位机，用于显示 ESP32-C3 + BNO085 模块回传的陀螺积分旋转矢量（四元数）。

## 数据协议

固件 src/main.cpp 以约 10 Hz 输出每帧 7 个 float (前 4 列为陀螺积分旋转矢量四元数, 后 3 列为固件内直接解算的 Roll/Pitch/Yaw, 度):

    i j k real roll_deg pitch_deg yaw_deg
    例如: 0.0123 -0.0045 0.9912 0.1305   1.20  -0.40 +178.70

- 端口: 115200 8N1 (USB-CDC, 无需额外串口驱动)
- 欧拉角 Roll / Pitch / Yaw 由**固件**直接解算 (ZYX 内旋, 航空/机器人常用),
  上位机直接读取后 3 列; 若收到 4 列旧格式, 自动回退到上位机反算 RPY.

## 文件清单

| 文件 | 作用 |
|---|---|
| app.py | 主程序, 单文件. GUI 与命令行双模式 |
| requirements.txt | 依赖: PySide6, pyserial |

## 安装

    cd host
    pip install -r requirements.txt

> Windows 首次安装 PySide6 可能耗时 1-2 分钟.
> 如果只想用 CLI 模式, 只需 pip install pyserial 即可, PySide6 可省.

## 运行

### A. GUI 模式 (推荐)

    python app.py

启动后:

1. 在"端口"下拉框里选 ESP32-C3 的 USB 串口 (例如 COM5 或 COMx - USB Serial Device)
2. 点 连接 -> 立方体与右侧数据实时更新
3. (可选) 点 开始录制 CSV 落盘

### B. 命令行模式

    python app.py --cli COM5         # Windows
    python app.py --cli /dev/ttyACM0 # Linux / WSL

输出示例:

    q=(+0.012,-0.005,+0.991,+0.131)  RPY=(+  1.2, -0.4,+178.7) deg   9.7Hz  YAW .............^......................

其中 ^ 表示航向角在 360° 标尺上的位置.

## 界面说明

| 区域 | 内容 |
|---|---|
| 顶部控制条 | 端口选择 / 刷新 / 连接 / 录制 CSV / 连接状态指示 |
| 左侧 3D 立方体 | 6 面着色 + XYZ 三色坐标轴, 跟着 BNO085 实时翻转. 无 OpenGL 依赖 |
| 右侧姿态数据 | 四元数 i j k real -> 欧拉角 Roll/Pitch/Yaw -> 采样率/帧数 |
| 状态栏 | 当前连接端口, 采样状态, 录制路径 |

## CSV 列

    time_iso, i, j, k, real, roll_deg, pitch_deg, yaw_deg, sample_hz

每 10 帧 flush 一次到磁盘, 正常退出或点 停止录制 时关闭文件.

## 实现要点

- 协议解析: parse_frame() 按空格切, 新固件 "i j k real roll_deg pitch_deg yaw_deg" 转 dict; 兼容旧 4 列格式 (上位机反算 RPY); 非法段数丢弃.
- 采样率: 滑动 20 帧时间间隔均值, 防止单帧抖动.
- 欧拉角换算: quat_to_euler_xyz() 是教科书 ZYX 内旋公式, sinp 越界回退 ±90°.
- 立方体绘制: CubeView 用 QPainter, 8 顶点逐一用 q v q^-1 旋转后投影, 按平均 z 排序实现画家算法消除 z-fighting.
- CLI/GUI 共用: pyserial 与 PySide6 都延迟 import, --cli 模式下根本不加载 Qt.

## 常见问题

| 现象 | 解决 |
|---|---|
| 端口列表为空 | 检查 USB 插入; 确认 CH340/CP210x 驱动; 拔插一次重试 |
| 打开端口失败 | 关闭 Arduino/PlatformIO 串口监视器等已占用进程 |
| 立方体看起来抖动 | Gyro Integrated RV 短期噪声较大; 要更平滑可改固件用 enableRotationVector() (卡尔曼) 替代 |
| 数据全是 0 | 检查 BNO085 I2C 连线 (SDA=8, SCL=9) 与 PS1/PS0 上拉 |
| 录制文件空白行多 | 数据行间固件夹了空行, 正常 - 解析已过滤 |

## 进一步扩展 (不在本极简版本)

- 用 Three.js 浏览器端 3D 显示, 配合 Web Serial 完全免安装
- 记录后回放: 读 CSV -> 重放动画, 便于离线分析
- 多传感器时间戳对齐 (BNO085 自带时间戳, 可加 getSensorEventTimestamp)

