#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <Arduino.h>

// WiFi配置
#define WIFI_SERIAL Serial3  // 使用Serial3与ESP8266通信
#define WIFI_BAUD_RATE 115200
#define WIFI_BUFFER_SIZE 256

// 数据发送间隔（毫秒）
#define WIFI_SEND_INTERVAL 100

// 函数声明
void wifiInit();  // 初始化WiFi模块
void wifiSendData();  // 发送数据
void wifiHandleCommand();  // 处理接收到的命令
bool wifiTestConnection();  // 测试WiFi连接状态
void calculatePosition();  // 计算位置

// 外部变量声明
extern char wifiBuffer[WIFI_BUFFER_SIZE];
extern bool wifiConnected;
extern unsigned long lastWifiSendTime;
extern float posX, posY;  // 位置变量
extern unsigned long lastPosCalcTime;  // 上次位置计算时间

#endif