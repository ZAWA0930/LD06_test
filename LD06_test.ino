/*
 * LD06 → Arduino UNO R4 → USB → PC
 *
 * LD06 TX : Arduino D0 / RX1
 * LD06 GND: Arduino GND
 */

constexpr uint32_t LD06_BAUD_RATE = 230400;
constexpr uint32_t PC_BAUD_RATE = 230400;

void setup()
{
    // PCとのUSBシリアル
    Serial.begin(PC_BAUD_RATE);

    // LD06とのUART
    Serial1.begin(LD06_BAUD_RATE);

    // Serial待ちは不要。
    // 待つとPC未接続時に転送が始まらないことがある。
}

void loop()
{
    // LD06から受信したデータを、そのままPCに転送
    while (Serial1.available() > 0)
    {
        uint8_t data = static_cast<uint8_t>(Serial1.read());
        Serial.write(data);
    }
}