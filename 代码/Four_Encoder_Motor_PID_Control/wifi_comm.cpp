#include "wifi_comm.h"
#include "motor_control.h"
#include "sensors.h"
#include "utils.h"

// 全局变量定义
char wifiBuffer[WIFI_BUFFER_SIZE];
bool wifiConnected = false;
unsigned long lastWifiSendTime = 0;

// 位置计算变量
float posX = 0, posY = 0;
unsigned long lastPosCalcTime = 0;

void wifiInit() {
    // 初始化ESP8266串口
    WIFI_SERIAL.begin(WIFI_BAUD_RATE);
    
    // 等待ESP8266启动
    delay(1000);
    
    // 发送AT命令测试通信
    WIFI_SERIAL.println("AT");
    delay(100);
    
    if (wifiTestConnection()) {
        Serial.println(PREFIX_SYSTEM + "ESP8266 Connection Successful");
        wifiConnected = true;
    } else {
        Serial.println(PREFIX_ERROR + "ESP8266 Connection Failed");
    }
}

bool wifiTestConnection() {
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {  // 1秒超时
        if (WIFI_SERIAL.available()) {
            String response = WIFI_SERIAL.readString();
            if (response.indexOf("OK") != -1) {
                return true;
            }
        }
    }
    return false;
}

// 计算位置
void calculatePosition() {
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastPosCalcTime) / 1000000.0f;  // 转换为秒
    lastPosCalcTime = currentTime;
    
    // 计算平均速度（米/秒）
    float avgSpeed = (M1_Motor_PID.feedback + M2_Motor_PID.feedback + 
                     M3_Motor_PID.feedback + M4_Motor_PID.feedback) / 4.0f;
    avgSpeed = avgSpeed * 0.065f * PI / 60.0f;  // 转换单位：RPM -> m/s
    
    // 根据当前角度和速度更新位置
    float radYaw = yaw * PI / 180.0f;  // 转换为弧度
    posX += avgSpeed * cos(radYaw) * deltaTime * 100;  // 转换为厘米
    posY += avgSpeed * sin(radYaw) * deltaTime * 100;  // 转换为厘米
}

void wifiSendData() {
    if (!wifiConnected) return;
    
    // 每100ms发送一次数据
    if (millis() - lastWifiSendTime >= WIFI_SEND_INTERVAL) {
        lastWifiSendTime = millis();
        
        // 计算位置
        calculatePosition();
        
        // 发送数据帧
        WIFI_SERIAL.println("<START>");
        
        // 发送位置数据
        snprintf(wifiBuffer, WIFI_BUFFER_SIZE,
                "POS:%.2f,%.2f,%.2f",
                posX, posY, yaw);
        WIFI_SERIAL.println(wifiBuffer);
        
        // 发送电机数据
        snprintf(wifiBuffer, WIFI_BUFFER_SIZE,
                "MOTORS:%.2f,%.2f,%.2f,%.2f",
                M1_Motor_PID.feedback, M2_Motor_PID.feedback,
                M3_Motor_PID.feedback, M4_Motor_PID.feedback);
        WIFI_SERIAL.println(wifiBuffer);
        
        // 发送传感器数据
        snprintf(wifiBuffer, WIFI_BUFFER_SIZE,
                "SENSORS:%.2f",
                ultrasonicDistance);
        WIFI_SERIAL.println(wifiBuffer);
        
        WIFI_SERIAL.println("<END>");
    }
}

void wifiHandleCommand() {
    if (!wifiConnected) return;
    
    // 检查是否有新命令
    if (WIFI_SERIAL.available()) {
        String cmd = WIFI_SERIAL.readStringUntil('\n');
        cmd.trim();
        
        // 处理命令
        if (cmd == "STOP") {
            // 停止电机
            Motor_PWM_Set(0, 0, 0, 0);
            Serial.println(PREFIX_SYSTEM + "Received STOP command");
        } else if (cmd == "RESET_POS") {
            // 重置位置
            posX = 0;
            posY = 0;
            Serial.println(PREFIX_SYSTEM + "Position reset");
        }
    }
}