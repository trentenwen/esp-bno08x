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

// 四元数 (i, j, k, real) -> (roll, pitch, yaw) [rad], ZYX 内旋 (航空/ROS 常用)
// 注意: BNO08x::getRoll/getPitch/getYaw 用的是常规 Rotation Vector 的四元数 (rawQuat*),
//       而这里只开了 Gyro Integrated RV, rawQuat* 不会被刷新, 所以必须自己解算.
// 参考: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
static void quatToEulerXYZ(float i, float j, float k, float real,
                           float &roll, float &pitch, float &yaw)
{
    // 归一化 (BNO085 给出本就是单位四元组, 此处双保险)
    const float n2 = i * i + j * j + k * k + real * real;
    if (n2 > 0.0f)
    {
        const float n = sqrtf(n2);
        i /= n;
        j /= n;
        k /= n;
        real /= n;
    }

    // roll  (X 轴旋转)
    const float sinr = 2.0f * (real * i + j * k);
    const float cosr = 1.0f - 2.0f * (i * i + j * j);
    roll  = atan2f(sinr, cosr);

    // pitch (Y 轴旋转), asin 定义域 [-1, 1] 钳位
    float sinp = 2.0f * (real * j - k * i);
    if      (sinp >  1.0f) sinp =  1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    pitch = asinf(sinp);

    // yaw   (Z 轴旋转)
    const float siny = 2.0f * (real * k + i * j);
    const float cosy = 1.0f - 2.0f * (j * j + k * k);
    yaw   = atan2f(siny, cosy);
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

    Serial.println("=== step 3: BNO08x → Gyro Integrated RV @ 100 Hz ===");
    if (!bno.enableGyroIntegratedRotationVector(10))
    { // 10 ms period = 100 Hz
        Serial.println("  enableGyroIntegratedRotationVector() ✗");
        while (1)
            delay(1000);
    }
    Serial.println("  enable 100 Hz ✓");
    Serial.println();
    Serial.println("   i       j       k      real      roll    pitch     yaw");
    Serial.println("                       (quat)           (degrees, ZYX)");
    Serial.println("----------------------------------------------------------------");
}

void loop()
{
    // 关键改动：getSensorEvent() 兼做"pump + 绑定 _sensor_value + 检查是否新数据"
    if (!bno.getSensorEvent())
    {
        delay(5); // 没新数据时稍睡，避免空转
        return;
    }

    // 如果你只关心 gyro integrated RV，加一层 eventId 过滤更省
    if (bno.getSensorEventID() != SH2_GYRO_INTEGRATED_RV)
    {
        return;
    }

    uint32_t now = millis();
    if (now - last_print_ms < PRINT_PERIOD_MS)
        return;
    last_print_ms = now;

    // 1) 取四元数
    const float qi = bno.getGyroIntegratedRVI();
    const float qj = bno.getGyroIntegratedRVJ();
    const float qk = bno.getGyroIntegratedRVK();
    const float qr = bno.getGyroIntegratedRVReal();

    // 2) 固件里直接解算 Roll/Pitch/Yaw (ZYX 内旋, 单位: rad)
    float roll, pitch, yaw;
    quatToEulerXYZ(qi, qj, qk, qr, roll, pitch, yaw);

    // 3) 一行输出: i j k real roll_deg pitch_deg yaw_deg
    Serial.print(qi, 4);                    Serial.print(' ');
    Serial.print(qj, 4);                    Serial.print(' ');
    Serial.print(qk, 4);                    Serial.print(' ');
    Serial.print(qr, 4);                    Serial.print(' ');
    Serial.print(roll  * DEG_PER_RAD, 2);    Serial.print(' ');
    Serial.print(pitch * DEG_PER_RAD, 2);    Serial.print(' ');
    Serial.println(yaw   * DEG_PER_RAD, 2);
}
