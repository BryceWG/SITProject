/*
 * 文件名: wifi_comm.cpp
 * 功能: WiFi通信相关的函数实现
 * 描述: 实现与WiFi模块的通信功能
 */

#include "wifi_comm.h"
#include "utils.h"

void wifiInit() {
    // 初始化Serial3用于与ESP32通信
    Serial3.begin(115200);
    Serial.println(PREFIX_SYSTEM + "WiFi模块通信初始化完成");
}

void wifiSendScanData(const String& data) {
    // 直接将扫描数据发送到WiFi模块
    Serial3.println(data);
}
