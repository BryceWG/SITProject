#include <Arduino.h>
#include <HardwareSerial.h>

void setup() {
    Serial.begin(115200); // 初始化Serial，用于串口监视器
    while (!Serial) {
        ; // 等待串口连接
    }
    Serial2.begin(115200); // 初始化Serial2，用于与新硬件通信
    Serial.println("Serial2 Test Started");
}

void loop() {
    // 检查Serial2是否有数据可读
    if (Serial2.available() > 0) {
        String dataFromNewHardware = Serial2.readStringUntil('\n'); // 假设数据以换行符结尾
        Serial.print("Received from new hardware: ");
        Serial.println(dataFromNewHardware);
    }
}