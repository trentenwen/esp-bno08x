#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

static const int SDA_PIN = 8;
static const int SCL_PIN = 9;
static const uint8_t ADDR = 0x4A; // 7-bit 地址

BNO08x bno;

static uint32_t last_print_ms = 0;
static constexpr uint32_t PRINT_PERIOD_MS = 100;

static constexpr float DEG_PER_RAD = 57.295779513082320876798154814105f;

// ============================================================================
// 多 sensor 缓存:
//   BNO085 库的 getSensorEvent() 每调用一次只刷一种 sensor 的 _sensor_value,
//   必须立刻把对应字段拷到本模块的 cache, 否则下一次读到的是上一个 sensor 的数据.
// ============================================================================
static float rv_roll = 0, rv_pitch = 0, rv_yaw = 0;              // Rotation Vector (磁力计参与)
static float la_x = 0, la_y = 0, la_z = 0;                        // Linear Accel (sensor frame, 已扣重力)
static bool rv_seen = false, la_seen = false;

static void pump_and_cache()
{
    // 一个打印周期内通常有 ~10 个事件 (RV/LA 各 100 Hz), 全部消费一遍.
    while (bno.getSensorEvent())
    {
        switch (bno.getSensorEventID())
        {
            case SENSOR_REPORTID_ROTATION_VECTOR:
                // getRoll/getPitch/getYaw 内部自行读取四元数, 无需另外缓存 i/j/k/r.
                rv_roll = bno.getRoll();
                rv_pitch = bno.getPitch();
                rv_yaw = bno.getYaw();
                rv_seen = true;
                break;

            case SENSOR_REPORTID_LINEAR_ACCELERATION:
                la_x = bno.getLinAccelX();
                la_y = bno.getLinAccelY();
                la_z = bno.getLinAccelZ();
                la_seen = true;
                break;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    delay(50);

    Wire.beginTransmission(ADDR);
    uint8_t rc = Wire.endTransmission(); // CLK 第 9 个时钟周期内 SDA 是否被 SALVE 拉低

    Serial.println("=== step 1: BNO085 ACK probe @ 0x4A ===");
    switch (rc)
    {
    case 0:
        Serial.println("  ACK  ✓  step 1 ✓");
        break;
    case 2:
        Serial.println("  NACK on ADDR  ✗");
        break;
    case 5:
        Serial.println("  TIMEOUT  ✗");
        break;
    default:
        Serial.printf("  code=%u ✗\n", rc);
    }

    bno.enableDebugging(Serial);

    Serial.println("=== step 2: BNO08x begin() ===");
    if (bno.begin(ADDR, Wire))
    { // 这里传地址 + Wire 总线句柄
        Serial.println("  Report ID 0xF0 ✓");
        Serial.println("step 2 ✓");
    }
    else
    {
        Serial.println("  begin() returned false ✗");
        Serial.println("  → PS1=HIGH? 3.3V 起来了? SDA/SCL 没反?");
    }

    // 运镜场景 2 个 sensor:
    //   RV          (含磁力计) → 姿态 (roll/pitch/yaw)
    //   LinearAccel (已扣重力, 3 轴 m/s²) → 三轴纯运动加速度
    // sensor frame (BNO085): X=右, Y=前(镜头方向), Z=上.
    Serial.println("=== step 3: BNO08x sensors @ 100 Hz ===");
    if (!bno.enableRotationVector(10))
    { // 100 Hz, 磁力计参与
        Serial.println("  enableRotationVector ✗");
        while (1)
            delay(1000);
    }
    if (!bno.enableLinearAccelerometer(10))
    { // 100 Hz, 已扣重力
        Serial.println("  enableLinearAccelerometer ✗");
        while (1)
            delay(1000);
    }
    Serial.println("  enable ✓");
    Serial.println();
    Serial.println("    roll    pitch      yaw  |     ax      ay      az");
    Serial.println("                       (deg)      (m/s^2, sensor)");
    Serial.println("------------------------------------------------------------");
}

void loop()
{
    pump_and_cache();

    uint32_t now = millis();
    if (now - last_print_ms < PRINT_PERIOD_MS)
        return;
    if (!rv_seen || !la_seen)
        return; // 启动期两类 sensor 未收齐
    last_print_ms = now;

    Serial.printf("%+7.2f %+7.2f %+8.2f  %+7.2f %+7.2f %+7.2f\n",
                  rv_roll * DEG_PER_RAD, rv_pitch * DEG_PER_RAD, rv_yaw * DEG_PER_RAD,
                  la_x, la_y, la_z);
}