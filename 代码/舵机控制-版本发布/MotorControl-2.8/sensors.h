/*
 * 文件名: sensors.h
 * 功能: 传感器相关函数和变量的声明
 * 描述: 
 *   - 声明MPU6050、超声波传感器、磁力计等相关函数
 *   - 声明传感器相关的全局变量
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "MPU6050.h"
#include <Servo.h>
#include "pid_control.h"

// 函数声明
void mpuInit();                   // 初始化MPU6050
void ultrasonicInit();            // 初始化超声波传感器
void calibrateSensor();           // 校准传感器
void moveCaculate();              // 计算运动参数
float getDistance(int trig, int echo);  // 获取超声波距离
void readMagnetometerYaw();       // 读取磁力计Yaw角
float normalizeAngle(float Yaw);  // 标准化角度

// 外部变量声明
extern MPU6050 mpu;               // MPU6050对象
extern Servo myServo;             // 舵机对象
extern float ultrasonicDistance;  // 超声波测得的距离
extern float yaw;                 // 当前Yaw角
extern float magYaw;              // 磁力计读取的原始Yaw角
extern float initialMagYaw;       // 初始Yaw角
extern bool initialYawSet;        // 初始Yaw角是否已设置

#endif