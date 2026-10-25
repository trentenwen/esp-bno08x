# lib/bno08x 模块能力与 API 汇总

> 基于 [lib/bno08x/](lib/bno08x/) 源码整理。该目录是 **SparkFun BNO08x Arduino Library**，
> 内部集成了 **CEVA (原 Hillcrest) SH-2 Sensor Hub Driver**，用于驱动 BNO085/BNO080 系列
> 9 轴 IMU + 内置 ARM Sensor Hub。硬件数据手册见 [BNO080 Datasheet v1.3.pdf](BNO080%20Datasheet%20v1.3.pdf)。

---

## 1. 目录构成

| 文件 | 行数 | 层次 | 职责 |
|---|---|---|---|
| [SparkFun_BNO08x_Arduino_Library.h](lib/bno08x/SparkFun_BNO08x_Arduino_Library.h) | 344 | **API 层** | `BNO08x` C++ 类定义（对用户的主要入口）+ 常量 |
| [SparkFun_BNO08x_Arduino_Library.cpp](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp) | 1646 | **API 层** | 类实现 + I2C/SPI 两套 HAL 适配 + 事件回调 |
| [sh2.h](lib/bno08x/sh2.h) / [sh2.c](lib/bno08x/sh2.c) | 695 / 2331 | **协议层** | SH-2 传感器 Hub 标准 C API（32 个公开函数） |
| [sh2_SensorValue.h](lib/bno08x/sh2_SensorValue.h) / [.c](lib/bno08x/sh2_SensorValue.c) | 512 / 551 | **协议层** | 原始字节流 → 结构化传感器数据解码 |
| [shtp.h](lib/bno08x/shtp.h) / [shtp.c](lib/bno08x/shtp.c) | 92 / 819 | **传输层** | SHTP 传输协议（分包/重组/通道/序列号） |
| [sh2_hal.h](lib/bno08x/sh2_hal.h) | 106 | **抽象层** | HAL 接口定义（open/close/read/write/getTimeUs） |
| [sh2_util.h](lib/bno08x/sh2_util.h) / [.c](lib/bno08x/sh2_util.c) | 45 / 104 | 工具 | 大小端安全的 8/16/32 位读写 |
| [sh2_err.h](lib/bno08x/sh2_err.h) | 42 | 工具 | `SH2_OK` / `SH2_ERR_*` 错误码定义 |

### 分层调用链

```
        用户代码  (src/main.cpp)
             ↓
    ┌─────────────────────────┐
    │  BNO08x  (C++ 封装)     │  ← 打包好的高层接口，本项目只用这一层
    └─────────────────────────┘
             ↓
    ┌─────────────────────────┐
    │  sh2_*  (C 标准 API)    │  ← 传感器配置、校准、tare、FRS、元数据
    └─────────────────────────┘
             ↓
    ┌─────────────────────────┐
    │  shtp_*  (传输协议)     │  ← 组包、通道、序列号、分包重组
    └─────────────────────────┘
             ↓
    ┌─────────────────────────┐
    │  sh2_Hal_t (硬件抽象)   │  ← open/close/read/write/getTimeUs
    └─────────────────────────┘
             ↓
        I2C (TwoWire)  或  SPI (SPIClass)
```

> **注意**：HAL 的五个函数指针在 [SparkFun_BNO08x_Arduino_Library.cpp:117-121](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L117-L121)（I2C）
> 和 [:148-152](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L148-L152)（SPI）赋值，底层全部封装好了，
> 用户不需要也不应该直接碰 `sh2_*` / `shtp_*` 层，除非要做库未封装的高级功能。

---

## 2. 核心能力总览

### 2.1 通信能力

| 能力 | 说明 |
|---|---|
| **I2C** | 默认地址 `0x4B`，也可 `0x4A`（由 `begin(deviceAddress)` 指定）。SDA/SCL 可任意映射（本项目 SDA=8, SCL=9）。 |
| **SPI** | 硬件 SPI，`SPI_MODE3` + `MSBFIRST`，速率上限 **3 MHz**（超过自动钳位）。需 CS/INT/RST 三个引脚。 |
| **INT 中断驱动** | 传入 `user_INTPin` 后，库会在每次读写前阻塞等 INT 拉低（`hal_wait_for_int()`），避免轮询浪费。这是 I2C 下能跑满 100 Hz 以上的关键。 |
| **超时自动硬复位** | `hal_wait_for_int()` 最多等 500 ms；超时则自动调用 `hal_hardwareReset()` 并返回失败（[cpp:1503-1514](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L1503-L1514)）。 |
| **分块传输** | I2C 单次事务上限 32 字节（`_maxBufferSize`），大包自动循环分块读取，每块跳过重复的 4 字节 SHTP 头（[cpp:1249-1286](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L1249-L1286)）。 |
| **自动软复位握手** | I2C HAL open 时发送 `{5,0,1,0,1}` 软复位包，最多重试 5 次，之后延时 300 ms（[cpp:1182-1200](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L1182-L1200)）。 |

### 2.2 传感器数据能力

库共定义了 **38 种** SH-2 虚拟传感器（`sh2_SensorId_e`，ID 范围 0x01–0x2B），
`sh2_decodeSensorEvent()` 支持**全部 38 种**解码（[sh2_SensorValue.c:98-209](lib/bno08x/sh2_SensorValue.c#L98-L209)）。

#### 惯性测量（运动学原始量）

| 传感器 | Sensor ID | 单位 | 说明 |
|---|---|---|---|
| Raw Accelerometer | 0x14 | ADC counts | 未处理原始计数 |
| Accelerometer | 0x01 | m/s² | 含重力 |
| Linear Acceleration | 0x04 | m/s² | **已扣除重力** |
| Gravity | 0x06 | m/s² | 仅重力分量 |
| Raw Gyroscope | 0x15 | ADC counts | 含温度 |
| Gyroscope (Calibrated) | 0x02 | rad/s | 校准后角速度 |
| Gyroscope (Uncalibrated) | 0x07 | rad/s | 含 biasX/Y/Z |
| Raw Magnetometer | 0x16 | ADC counts | 原始磁力计 |
| Magnetic Field (Calibrated) | 0x03 | µT | 校准后磁场 |
| Magnetic Field (Uncalibrated) | 0x0f | µT | 含 bias |

#### 姿态融合（9DOF 方向）

| 传感器 | Sensor ID | 说明 |
|---|---|---|
| Rotation Vector | 0x05 | 陀螺 + 加速度 + 磁力计三源融合，四元数 + 精度；**本项目使用** |
| Game Rotation Vector | 0x08 | 陀螺 + 加速度（**不含磁力计**，无磁干扰，VR/游戏推荐） |
| Geomagnetic Rotation Vector | 0x09 | 与 Rotation Vector 类似，偏重磁力计 |
| AR/VR Stabilized RV | 0x28 | 为 AR/VR 场景稳定过的旋转向量 |
| AR/VR Stabilized GRV | 0x29 | 游戏版稳定旋转向量 |
| Gyro Integrated RV | 0x2A | 高刷新率陀螺积分旋转矢量（4 kHz 级），同时输出角速度 |

#### 环境/扩展传感器（需外接 BNO085 支持的扩展件）

`Pressure` 0x0a (hPa) · `Ambient Light` 0x0b (lux) · `Humidity` 0x0c (%) · `Proximity` 0x0d (cm) · `Temperature` 0x0e (°C) · `Heart Rate Monitor` 0x23 (bpm)

#### 事件/状态检测器

| 传感器 | Sensor ID | 输出 |
|---|---|---|
| Tap Detector | 0x10 | 单击/双击，flags 含 X/Y/Z、+方向、双击位 |
| Step Detector | 0x18 | 单步事件 + 延迟 |
| Step Counter | 0x11 | 累计步数（uint16） |
| Significant Motion | 0x12 | 显著运动 |
| **Stability Classifier** | 0x13 | 0=未知 1=on-table 2=stationary 3=stable 4=in-motion |
| Shake Detector | 0x19 | X/Y/Z 方向抖动 |
| Flip Detector | 0x1a | 翻转 |
| Pickup Detector | 0x1b | 拿起 / 停止 |
| Stability Detector | 0x1c | 进入/退出稳定态 |
| **Personal Activity Classifier** | 0x1e | 0=未知 1=车内 2=骑车 3=步行(站立) 4=静止 5=倾斜 6=走路 7=跑步，附 10 档置信度 |
| Sleep Detector | 0x1f | 睡眠状态 |
| Tilt Detector | 0x20 | 倾斜 |
| Pocket Detector | 0x21 | 口袋 |
| Circle Detector | 0x22 | 画圈 |

### 2.3 标定与姿态对齐能力

| 能力 | 封装函数 | 说明 |
|---|---|---|
| **Tare（归零）** | `tareNow()` / `saveTare()` / `clearTare()` | 立即把当前姿态设为"零点"。支持仅 Z 轴（`tareNow(true)`），基准可选 Rotation Vector / Game RV / Geomag RV。 |
| **动态校准开关** | `setCalibrationConfig()` | 位掩码 `SH2_CAL_ACCEL(0x01)` / `GYRO(0x02)` / `MAG(0x04)` / `PLANAR(0x08)`，控制哪些传感器参与动态校准。 |
| **保存校准到 Flash** | `saveCalibration()` | 写入 DCD（Dynamic Calibration Data）。 |
| **自校准流程** | `sh2_startCal()` / `sh2_finishCal()` | C 层简易自校准，返回 `sh2_CalStatus_t`（9 种状态码）。 |
| **DCD 管理** | C 层 `sh2_setDcdAutoSave()` / `sh2_clearDcdAndReset()` | 自动保存开关、清 DCD 并复位。 |
| **运行时重定向** | C 层 `sh2_setReorientation()` | 用四元数设置传感器朝向。 |

### 2.4 系统管理能力（主要在 C 层）

| 能力 | 函数 |
|---|---|
| 产品 ID / 固件版本 | `sh2_getProdIds()`、`getResetReason()`、`resetReason()` |
| 复位原因 | 1=POR, 2=内部复位, 3=看门狗, 4=外部复位, 5=其他 |
| 电源模式 | `modeOn()` / `modeSleep()` / `softReset()` |
| 传感器配置读写 | `enableReport()` / `sh2_getSensorConfig()` / `sh2_setSensorConfig()` |
| 元数据（Q 点/量程/分辨率/功耗） | `sh2_getMetadata()` |
| FRS 记录读写（Flash） | `sh2_getFrs()` / `sh2_setFrs()`，**约 60 个记录 ID**（见 [sh2.h:278-351](lib/bno08x/sh2.h#L278-L351)） |
| 错误日志 | `sh2_getErrors()` |
| 传感器计数器 | `sh2_getCounts()` / `sh2_clearCounts()`（offered/accepted/on/attempted） |
| 立即冲刷缓存报告 | `sh2_flush(sensorId)` |
| 时钟源查询 | `sh2_getOscType()`（内部晶振 / 外部晶振 / 外部时钟） |
| 交互式 ZRO | `sh2_setIZro()`（告诉 Hub 设备的运动意图） |

### 2.5 姿态数学

| 函数 | 返回 |
|---|---|
| `getQuatI/J/K/Real()` | 四元数 4 分量（float） |
| `getQuatRadianAccuracy()` | 朝向精度估计（弧度） |
| `getQuatAccuracy()` | 0=不可靠 1=低 2=中 3=高 |
| **`getRoll()` / `getPitch()` / `getYaw()`** | 四元数 → 欧拉角，**返回弧度**（[cpp:169-239](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L169-L239)），内部已做四元数归一化 |
| `qToFloat(fixed, qPoint)` | Q 定点数 → float，公开可用 |

---

## 3. 打包好的可用函数（`BNO08x` 类）

> 这是**推荐直接使用**的接口。全部声明见
> [SparkFun_BNO08x_Arduino_Library.h:144-344](lib/bno08x/SparkFun_BNO08x_Arduino_Library.h#L144-L344)。

### 3.1 初始化与连接

```cpp
BNO08x bno;

// I2C：可选传地址、Wire 端口、INT 引脚、RST 引脚
bool ok = bno.begin(0x4A, Wire, INT_PIN, RST_PIN);

// SPI
bool ok = bno.beginSPI(CS_PIN, INT_PIN, RST_PIN, 3000000, SPI);

bool present = bno.isConnected();   // 仅做 I2C ACK 检测
void bno.enableDebugging(Serial);   // 打开调试输出
```

| 函数 | 说明 |
|---|---|
| `begin(address, wirePort, intPin, rstPin)` | I2C 初始化。默认 `0x4B` + `Wire`。**强烈建议传入 INT 和 RST 引脚**。返回 false 表示设备无应答。 |
| `beginSPI(cs, int, rst, speed, spi)` | SPI 初始化，速度自动钳到 3 MHz。 |
| `isConnected()` | I2C ACK 探测。**仅 I2C 可用**（SPI 下 `_i2cPort` 为 NULL 会崩）。 |
| `enableDebugging(stream)` | 打开打印调试（默认 `Serial`），会打印非单位四元数等异常。 |

### 3.2 生命周期与电源

```cpp
bno.hardwareReset();   // 硬复位（拉 RST 引脚）
bno.softReset();       // 软复位（发 SHTP RESET 命令）
bno.modeOn();          // 开机
bno.modeSleep();       // 休眠
bno.serviceBus();      // 主动泵一次总线（内部 = sh2_service()）
bool r = bno.wasReset();   // 是否发生过复位（读取后自动清标志）
uint8_t why = bno.getResetReason();  // 复位原因
uint8_t why2 = bno.resetReason();    // 同上（遗留别名）
```

### 3.3 传感器使能（18 个）

> 所有 `enable*` 的 `timeBetweenReports` 参数单位是**毫秒**，内部 ×1000 转成 μs。

```cpp
bool ok = bno.enableRotationVector(10);   // 100 Hz
```

| 函数 | 对应 Sensor ID |
|---|---|
| `enableRotationVector(ms=10)` | 0x05 |
| `enableGameRotationVector(ms=10)` | 0x08 |
| `enableGeomagneticRotationVector(ms=10)` | 0x09 |
| `enableARVRStabilizedRotationVector(ms)` | 0x28 |
| `enableARVRStabilizedGameRotationVector(ms)` | 0x29 |
| `enableGyroIntegratedRotationVector(ms=10)` | 0x2A |
| `enableAccelerometer(ms=10)` | 0x01 |
| `enableLinearAccelerometer(ms=10)` | 0x04 |
| `enableGravity(ms=10)` | 0x06 |
| `enableGyro(ms=10)` | 0x02 校准后陀螺 |
| `enableUncalibratedGyro(ms=10)` | 0x07 |
| `enableMagnetometer(ms=10)` | 0x03 |
| `enableRawAccelerometer(ms=10)` | 0x14 |
| `enableRawGyro(ms=10)` | 0x15 |
| `enableRawMagnetometer(ms=10)` | 0x16 |
| `enableTapDetector(ms)` | 0x10 |
| `enableStepCounter(ms=10)` | 0x11 |
| `enableStabilityClassifier(ms=10)` | 0x13 |
| `enableActivityClassifier(ms, activitiesToEnable)` | 0x1e，可指定要启用的活动位掩码 |

**通用使能入口**（库未封装的传感器走这里）：

```cpp
bool ok = bno.enableReport(sh2_SensorId_t sensorId,
                           uint32_t interval_us,
                           uint32_t sensorSpecific = 0);
```

`interval_us` 单位是**微秒**。库未封装但可用的 Sensor ID 见 §2.2 全部表格，
配合 `sh2_decodeSensorEvent()` 直接从 `bno.sensorValue`（public 成员）读结构体数据。

### 3.4 事件读取（核心）

```cpp
while (bno.getSensorEvent()) {              // 每次只取一个事件
    switch (bno.getSensorEventID()) {       // 拿到这次是哪个传感器
        case SENSOR_REPORTID_ROTATION_VECTOR:
            float i = bno.getQuatI();
            float roll = bno.getRoll();
            break;
        ...
    }
}
```

| 函数 | 说明 |
|---|---|
| `getSensorEvent()` | **驱动整个循环的核心**。内部 `sh2_service()` 读一个 SHTP 包并解码，返回 true 表示有新事件。 |
| `getSensorEventID()` | 本次事件的 Sensor ID，用于 `switch` 分发。 |
| `getTimeStamp()` | 本次事件的硬件时间戳（uint64, μs）。 |

> ⚠️ **必须理解的一点**（本项目 `src/main.cpp:16-26` 的注释正是讲这个）：
> 库里只有**一个** `_sensor_value` 缓冲区。`getSensorEvent()` 每调用一次就把它刷新成
> "本次事件的类型"。所以**在 `switch` 分支内必须立刻把需要的值取出来缓存**，
> 一旦离开分支去读别的 sensor 类型的 getter，拿到的就是错误数据
> （例如 RV 事件之后调 `getLinAccelX()` 会返回四元数的 `.i`）。
>
> ⚠️ 另外：`_sensor_value` 只在 `getSensorEvent()` 里被赋值。**在第一次调用 `getSensorEvent()`
> 之前调用任何 getter 会解引用空指针。**

### 3.5 数据读取函数

#### 四元数 / 姿态

| 函数 | 返回 |
|---|---|
| `getQuat(i, j, k, real, radAccuracy, accuracy)` | 一次性取全部 6 个（引用出参） |
| `getQuatI()` / `getQuatJ()` / `getQuatK()` / `getQuatReal()` | 四元数分量 |
| `getQuatRadianAccuracy()` | 朝向精度（弧度） |
| `getQuatAccuracy()` | 状态 0–3 |
| `getGameQuatI/J/K/Real()` | Game Rotation Vector 四元数 |
| `getRoll()` / `getPitch()` / `getYaw()` | 欧拉角，**弧度** |

#### 加速度

| 函数 | 单位 |
|---|---|
| `getAccel(x, y, z, accuracy)` / `getAccelX/Y/Z()` / `getAccelAccuracy()` | m/s²，含重力 |
| `getLinAccel(x, y, z, accuracy)` / `getLinAccelX/Y/Z()` / `getLinAccelAccuracy()` | m/s²，**已扣重力** |
| `getGravity(x, y, z, accuracy)` / `getGravityX/Y/Z()` / `getGravityAccuracy()` | m/s² |

#### 陀螺仪

| 函数 | 单位 |
|---|---|
| `getGyro(x, y, z, accuracy)` / `getGyroX/Y/Z()` / `getGyroAccuracy()` | rad/s |
| `getUncalibratedGyro(x,y,z,bx,by,bz,accuracy)` | rad/s + bias |
| `getUncalibratedGyroX/Y/Z()` / `getUncalibratedGyroBiasX/Y/Z()` / `getUncalibratedGyroAccuracy()` | 同上，单项版 |
| `getGyroIntegratedRVI/J/K/Real()` | 陀螺积分 RV 四元数 |
| `getGyroIntegratedRVangVelX/Y/Z()` | 陀螺积分 RV 角速度 (rad/s) |

#### 磁力计

| 函数 | 单位 |
|---|---|
| `getMag(x, y, z, accuracy)` / `getMagX/Y/Z()` / `getMagAccuracy()` | µT |

#### 原始 MEMS 值

| 函数 | 说明 |
|---|---|
| `getRawAccelX/Y/Z()` → `int16_t` | 加速度计 ADC 计数 |
| `getRawGyroX/Y/Z()` → `int16_t` | 陀螺 ADC 计数 |
| `getRawMagX/Y/Z()` → `int16_t` | 磁力计 ADC 计数 |

#### 事件 / 分类器

| 函数 | 返回 |
|---|---|
| `getTapDetector()` → `uint8_t` | 轻击 flags（见 `TAPDET_*` 常量） |
| `getStepCount()` → `uint16_t` | 累计步数 |
| `getStabilityClassifier()` → `uint8_t` | 0 unknown / 1 on-table / 2 stationary / 3 stable / 4 in-motion |
| `getActivityClassifier()` → `uint8_t` | 最可能活动（0–7） |
| `getActivityConfidence(activity)` → `uint8_t` | 指定活动的置信度 0–100 |
| `getTimeStamp()` → `uint64_t` | 硬件时间戳 (μs) |

### 3.6 校准与 Tare

```cpp
bno.setCalibrationConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
bno.saveCalibration();

bno.tareNow();          // 三轴全归零
bno.tareNow(true);      // 仅 Z 轴归零
bno.saveTare();         // 持久化到 Flash
bno.clearTare();        // 清除归零
```

| 函数 | 底层 |
|---|---|
| `setCalibrationConfig(uint8_t sensors)` | `sh2_setCalConfig()` |
| `saveCalibration()` | `sh2_saveDcdNow()` |
| `tareNow(bool zAxis, sh2_TareBasis_t basis)` | `sh2_setTareNow()` |
| `saveTare()` | `sh2_persistTare()` |
| `clearTare()` | `sh2_clearTare()` |

### 3.7 Public 成员变量

```cpp
sh2_ProductIds_t   prodIds;      // 产品 ID / 固件版本 / 复位原因
sh2_SensorValue_t  sensorValue;  // 最近一次事件的完整解码结果
```

对于 §2.2 中库**未提供 getter** 的传感器（如 Significant Motion、Shake、Flip、Pickup、
Sleep、Tilt、Pocket、Circle、Heart Rate、环境传感器），直接读 `bno.sensorValue.un.<字段>` 即可。

---

## 4. C 层 API（高级用法，`sh2.h`）

32 个公开函数，用于库未封装的高级功能：

```c
// 会话
int  sh2_open(sh2_Hal_t *pHal, sh2_EventCallback_t *cb, void *cookie);
void sh2_close(void);
void sh2_service(void);
int  sh2_setSensorCallback(sh2_SensorCallback_t *cb, void *cookie);

// 设备控制
int  sh2_devReset(void);   int sh2_devOn(void);   int sh2_devSleep(void);
int  sh2_reinitialize(void);
int  sh2_getProdIds(sh2_ProductIds_t *prodIds);

// 传感器配置
int  sh2_getSensorConfig(sh2_SensorId_t, sh2_SensorConfig_t *);
int  sh2_setSensorConfig(sh2_SensorId_t, const sh2_SensorConfig_t *);
int  sh2_getMetadata(sh2_SensorId_t, sh2_SensorMetadata_t *);
int  sh2_flush(sh2_SensorId_t);

// FRS / 诊断
int  sh2_getFrs(uint16_t recordId, uint32_t *pData, uint16_t *words);
int  sh2_setFrs(uint16_t recordId, uint32_t *pData, uint16_t words);
int  sh2_getErrors(uint8_t severity, sh2_ErrorRecord_t *pErrors, uint16_t *n);
int  sh2_getCounts(sh2_SensorId_t, sh2_Counts_t *);
int  sh2_clearCounts(sh2_SensorId_t);
int  sh2_getOscType(sh2_OscType_t *);

// 校准
int  sh2_setCalConfig(uint8_t sensors);      int sh2_getCalConfig(uint8_t *);
int  sh2_saveDcdNow(void);                   int sh2_setDcdAutoSave(bool);
int  sh2_clearDcdAndReset(void);
int  sh2_startCal(uint32_t interval_us);     int sh2_finishCal(sh2_CalStatus_t *);

// Tare / 朝向
int  sh2_setTareNow(uint8_t axes, sh2_TareBasis_t basis);
int  sh2_clearTare(void);                    int sh2_persistTare(void);
int  sh2_setReorientation(sh2_Quaternion_t *orientation);

// ZRO
int  sh2_setIZro(sh2_IZroMotionIntent_t intent);
```

### 解码函数

```c
int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
```

`sh2_SensorValue_t` 结构（[sh2_SensorValue.h:434-503](lib/bno08x/sh2_SensorValue.h#L434-L503)）：

```c
typedef struct sh2_SensorValue {
    uint8_t sensorId;      // 哪个传感器
    uint8_t sequence;      // 报告序号，跳跃说明丢包
    uint8_t status;        // bit1-0: 0不可靠 1低 2中 3高；bit4-2: 延迟指数
    uint64_t timestamp;    // [uS]
    uint32_t delay;        // [uS] = delay * 2^exponent
    union { ... } un;      // 按 sensorId 取对应成员
} sh2_SensorValue_t;
```

---

## 5. 常量速查

### 5.1 Q 定点数（[头文件:332-339](lib/bno08x/SparkFun_BNO08x_Arduino_Library.h#L332-L339)）

| 传感器 | Q1 |
|---|---|
| Rotation Vector | 14 |
| 朝向精度 | 12 |
| Accelerometer / Linear Accel / Gravity | 8 |
| Gyroscope | 9 |
| Magnetometer | 4 |
| 角速度 (Gyro Integrated RV) | 10 |

转换：`qToFloat(rawValue, Q)`。也可用 `sh2_getMetadata()` 动态查询。

### 5.2 分类器枚举

```c
// Stability Classifier
STABILITY_CLASSIFIER_UNKNOWN(0) ON_TABLE(1) STATIONARY(2) STABLE(3) MOTION(4)

// Personal Activity Classifier
PAC_UNKNOWN(0) IN_VEHICLE(1) ON_BICYCLE(2) ON_FOOT(3)
PAC_STILL(4) TILTING(5) WALKING(6) RUNNING(7)

// Tap Detector flags
TAPDET_X(1) X_POS(2) Y(4) Y_POS(8) Z(16) Z_POS(32) DOUBLE(64)
```

### 5.3 其他

| 常量 | 值 |
|---|---|
| `BNO08x_DEFAULT_ADDRESS` | `0x4B` |
| `TARE_AXIS_ALL` / `TARE_AXIS_Z` | `0x07` / `0x04` |
| `SH2_CAL_ACCEL/GYRO/MAG/PLANAR` | `0x01/0x02/0x04/0x08` |
| SHTP 通道 2 报告 ID | `0xF1`–`0xFD`（命令/响应/FRS/ProductID 等） |

---

## 6. 最小可用示例

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

BNO08x bno;

void setup() {
    Serial.begin(115200);

    // I2C：地址 0x4A，带 INT 中断和 RST 引脚
    if (!bno.begin(0x4A, Wire, /*INT=*/4, /*RST=*/5)) {
        Serial.println("BNO085 not found");
        while (1);
    }

    if (bno.wasReset()) {
        bno.enableRotationVector(10);        // 100 Hz 旋转向量
        bno.enableLinearAccelerometer(10);   // 100 Hz 线性加速度
        bno.enableStabilityClassifier(10);   // 100 Hz 稳定性
    }
}

void loop() {
    while (bno.getSensorEvent()) {
        switch (bno.getSensorEventID()) {
        case SENSOR_REPORTID_ROTATION_VECTOR: {
            float roll  = bno.getRoll();      // 弧度
            float pitch = bno.getPitch();
            float yaw   = bno.getYaw();
            Serial.printf("R=%.1f P=%.1f Y=%.1f\n",
                          roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f);
            break;   // ← 必须在分支内取值
        }
        case SENSOR_REPORTID_LINEAR_ACCELERATION: {
            Serial.printf("LinAccel %.3f %.3f %.3f\n",
                          bno.getLinAccelX(), bno.getLinAccelY(), bno.getLinAccelZ());
            break;
        }
        case SENSOR_REPORTID_STABILITY_CLASSIFIER:
            Serial.printf("Stability = %d\n", bno.getStabilityClassifier());
            break;
        }
    }
}
```

---

## 7. 使用注意事项 / 已知坑

这些是从源码中确认的行为，写代码时需要留意：

1. **单缓冲区陷阱（最重要）**
   `getSensorEvent()` 每次覆盖唯一的 `_sensor_value`。必须在 `switch` 分支内立刻取值并缓存到自己的变量，否则读到的是上一种传感器的数据。

2. **首次调用前不要调 getter**
   `_sensor_value` 指针只在 `getSensorEvent()` 内被赋值。直接调 `getQuatI()` 等会解引用 NULL。

3. **`isConnected()` 只能用于 I2C**
   实现里直接用了 `_i2cPort`（[cpp:1387-1393](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L1387-L1393)），
   SPI 模式下 `_i2cPort` 为 NULL，会崩。

4. **`getGravity(x,y,z,accuracy)` 批量版本有 bug**
   三个分量都用了 `gravityX`（[cpp:510-516](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L510-L516)）。
   用单项的 `getGravityX/Y/Z()`，或直接读 `sensorValue.un.gravity`。

5. **`getTapDetector()` 实际返回 0**
   它读的是成员变量 `tapDetector`，但 `sensorHandler` 只写了 `_sensor_value`，
   从没更新过该成员（[cpp:617-622](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L617-L622)）。
   正确做法是读 `bno.sensorValue.un.tapDetector.flags`。

6. **`getGyroAccuracy()` / `getUncalibratedGyroAccuracy()` 读的是陈旧成员**
   这两个仍读 `gyroAccuracy` / `UncalibGyroAccuracy` 成员变量而非 `_sensor_value->status`，
   其他 `get*Accuracy()` 都已改成读 `status`。建议统一读 `bno.sensorValue.status`。

7. **时间戳精度只有 1 ms**
   `hal_getTimeUs()` 用 `millis() * 1000` 实现（[cpp:1344-1348](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L1344-L1348)），
   微秒位恒为 0。`getTimeStamp()` 的价值在于"是否同一份报告"和相对间隔，
   不要当高精度计时器。

8. **RST 引脚初始化方式**
   `begin()` 里把 RST 设成了 `INPUT_PULLUP`（[cpp:103-107](lib/bno08x/SparkFun_BNO08x_Arduino_Library.cpp#L103-L107)），
   而 `hal_hardwareReset()` 才切成 `OUTPUT`。能工作但与常规做法不一致。

9. **不带 INT 引脚会退化**
   传 `-1`（不配 INT）时代码会走无中断轮询路径，I2C 下大数据易丢包。
   生产使用请务必传 INT 引脚。

10. **SPI 模式固定为 MODE3**
    不可配置。速率硬上限 3 MHz。

11. **`enableReport()` 的 `interval_us` 单位与其他 `enable*` 不同**
    `enableXxx(ms)` 参数是毫秒，`enableReport(sensorId, interval_us)` 是微秒。

12. **事件可能丢**
    `sequence` 字段（`sensorValue.sequence`）会随报告递增，**出现跳号说明丢了报告**，
    需要可靠数据流时应检查。

---

## 8. 本项目的实际用法

[src/main.cpp](src/main.cpp) 的使用模式：

- I2C，地址 `0x4A`，SDA=8 / SCL=9
- 同时开启 3 个 100 Hz 报告：`Rotation Vector`（绝对姿态）+ `Linear Acceleration`（已扣重力，检测相机运动）+ `Stability Classifier`（静止/运动判定）
- 用 `pump_and_cache()` 在 `while (bno.getSensorEvent())` 里按 `getSensorEventID()` 分发并立即缓存到模块级变量 —— 正是为了规避 §7.1 的单缓冲区陷阱
- 静止时用 Stability Classifier 做零速更新（ZUPT），运动时用线性加速度 + bias 估计做泄漏积分（α=0.92）得到速度曲线