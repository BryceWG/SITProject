#include <Arduino.h>

void setup() {
    Serial3.begin(115200);  // 和ESP32通信
    randomSeed(analogRead(0));  // 初始化随机数生成器
}

void loop() {
    // 生成随机数据
    int randomNumber = random(1, 1000);
    float randomFloat = random(0, 10000) / 100.0;
    
    // 构建数据字符串
    String dataString = "数据包 " + String(millis()/1000) + 
                       "s: 数值1=" + String(randomNumber) + 
                       ", 数值2=" + String(randomFloat, 2);
    
    // 发送数据到ESP32
    Seria3.println(dataString);
    
    // 等待1秒
    delay(1000);
}
