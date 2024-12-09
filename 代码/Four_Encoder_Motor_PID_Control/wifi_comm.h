/*
 * 文件名: wifi_comm.h
 * 功能: WiFi通信相关的函数声明
 * 描述: 声明WiFi通信相关的函数
 */

#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <Arduino.h>

// WiFi通信相关函数声明
void wifiInit();                    // 初始化WiFi通信
void wifiSendScanData(const String& data); // 发送扫描数据

#endif
