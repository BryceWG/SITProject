#include <Arduino.h> 
#include <SoftwareSerial.h>

SoftwareSerial magnetometerSerial(2, 3); // RX, TX

void setup() {
  Serial.begin(115200);
  magnetometerSerial.begin(9600);  // 使用磁力计的实际波特率
  Serial.println("Magnetometer Test");
}

void loop() {
  if (magnetometerSerial.available() >= 6) {  // 假设每次读取6字节数据
    int16_t x, y, z;
    
    // 读取X轴数据（假设为2字节）
    x = magnetometerSerial.read() | (magnetometerSerial.read() << 8);
    
    // 读取Y轴数据（假设为2字节）
    y = magnetometerSerial.read() | (magnetometerSerial.read() << 8);
    
    // 读取Z轴数据（假设为2字节）
    z = magnetometerSerial.read() | (magnetometerSerial.read() << 8);
    
    // 打印解析后的数据
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.print(y);
    Serial.print(", Z: ");
    Serial.println(z);
  }
}
