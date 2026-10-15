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

    // 开启常规 Rotation Vector (Game Rotation Vector 同源, 但 RV 含磁力计校准, yaw 更稳).
    // 库内 getRoll/Pitch/Yaw 直接基于 rawQuat* 解算 → 这里只要开了 RV 就能拿到.
    Serial.println("=== step 3: BNO08x → Rotation Vector @ 100 Hz ===");
    if (!bno.enableRotationVector(10))
    { // 10 ms period = 100 Hz
        Serial.println("  enableRotationVector() ✗");
        while (1)
            delay(1000);
    }
    Serial.println("  enable 100 Hz ✓");
    Serial.println();
    Serial.println("  i     j     k     real  |  roll    pitch     yaw");
    Serial.println("                           (degrees, ZYX, lib-converted)");
    Serial.println("----------------------------------------------------------");
}

void loop()
{
    // 关键改动：getSensorEvent() 兼做"pump + 绑定 _sensor_value + 检查是否新数据"
    if (!bno.getSensorEvent())
    {
        delay(5); // 没新数据时稍睡，避免空转
        return;
    }

    uint32_t now = millis();
    if (now - last_print_ms < PRINT_PERIOD_MS)
        return;
    last_print_ms = now;

    // 上位机 3D 立方体仍需要四元数 → 取 rawQuat* 喂给它.
    // Roll/Pitch/Yaw 直接走库的转换 (底层仍读 rawQuat*, 所以 RV 一开就同步刷新).
    const float qi = bno.getQuatI();
    const float qj = bno.getQuatJ();
    const float qk = bno.getQuatK();
    const float qr = bno.getQuatReal();

    const float roll_deg  = bno.getRoll()  * DEG_PER_RAD;
    const float pitch_deg = bno.getPitch() * DEG_PER_RAD;
    const float yaw_deg   = bno.getYaw()   * DEG_PER_RAD;

    Serial.printf("%+.4f %+.4f %+.4f %+.4f  %+7.2f %+7.2f %+7.2f\n",
                  qi, qj, qk, qr,
                  roll_deg, pitch_deg, yaw_deg);
}
