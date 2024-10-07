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

//持续输出获取的磁力计数据  
void loop() 
{
  static unsigned long lastReadTime = 0;
  if (millis() - lastReadTime >= READ_DURATION) {
    lastReadTime = millis();
    Serial2.write(0x55); // 发送读取命令
    delay(10); // 等待磁力计响应
    if (Serial2.available()) {
      Serial.print("磁力计原始数据: ");
      while (Serial2.available()) {
        Serial.print(Serial2.read(), HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("未收到磁力计响应");
    }
  }


  delay(100); // 短暂延迟，避免过于频繁的输出
}