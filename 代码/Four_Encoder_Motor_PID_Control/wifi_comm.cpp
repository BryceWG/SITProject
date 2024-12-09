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
/*
void wifiHandleCommand() {
    // 处理来自WiFi模块的命令
    if (Serial3.available()) {
        String command = Serial3.readStringUntil('\n');
        command.trim();
        Serial.println(PREFIX_SYSTEM + "收到WiFi命令: " + command);
        // TODO: 根据需要处理特定命令
    }
}
*/

void wifiSendScanData(const String& data) {
    // 直接将扫描数据发送到WiFi模块
    Serial3.println(data);
}
