#include <Arduino.h>
#include <HardwareSerial.h>
// 定义日志前缀
const unsigned long READ_DURATION = 1000; // 读取持续时间，单位毫秒

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("磁力计原始数据测试开始...");
  
  Serial2.begin(460800); // 初始化Serial2，用于与磁力计通信
  
  delay(1000); // 等待磁力计初始化
}

void loop() {
  String rawData = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < READ_DURATION) {
    if (Serial2.available()) {
      char c = Serial2.read();
      rawData += c;
    }
  }
  
  if (rawData.length() > 0) {
    Serial.print("原始数据（");
    Serial.print(READ_DURATION);
    Serial.print("ms）: ");
    Serial.println(rawData);
  } else {
    Serial.println("在指定时间内未收到数据");
  }
  
  delay(100); // 短暂延迟，避免过于频繁的输出
}