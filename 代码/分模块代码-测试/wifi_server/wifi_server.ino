#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

// WiFi配置
const char* ssid = "SLAM_Robot_AP";  // WiFi名称
const char* password = "12345678";   // WiFi密码

// TCP服务器配置
WiFiServer server(8080);  // 端口8080
WiFiClient client;

// 定义串口引脚
#define ARDUINO_RX 44  // ESP32的RX引脚
#define ARDUINO_TX 43  // ESP32的TX引脚

// 发送日志到所有输出
void sendLog(String message) {
    Serial.println(message);  // 发送到USB串口
    if (client && client.connected()) {
        client.println(message);  // 发送到WiFi客户端
    }
}

void setup() {
    // 初始化串口通信
    Serial.begin(115200);      // USB串口
    Serial2.begin(115200, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX); // 硬件串口2

    pinMode(LED_BUILTIN, OUTPUT);
    
    // 明确设置WiFi模式为AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    
    sendLog("WiFi接入点已创建");
    sendLog("SSID: " + String(ssid));
    sendLog("Password: " + String(password));
    sendLog("IP地址: " + WiFi.softAPIP().toString());
    
    // 启动TCP服务器
    server.begin();
    sendLog("TCP服务器已启动");
}

void loop() {
    // 检查新的客户端连接
    if (!client || !client.connected()) {
        client = server.available();
        if (client) {
            sendLog("新客户端已连接");
        }
    }
    
    // 检查来自Arduino的数据
    if (Serial2.available()) {
        String data = Serial2.readStringUntil('\n');
        data.trim();
        sendLog(data);
        
        // LED闪烁表示收到数据
        digitalWrite(LED_BUILTIN, HIGH); 
        delay(50);  // 短暂亮灯
        digitalWrite(LED_BUILTIN, LOW); 
    }
    
    // 简单的客户端数据接收测试
    if (client && client.connected() && client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();
        sendLog("收到客户端消息: " + command);
        // 发送到Arduino
        Serial2.println(command);
    }
}