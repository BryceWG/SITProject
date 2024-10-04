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

void mpuInit();
void ultrasonicInit();
void calibrateSensor();
void moveCaculate();
float getDistance(int trig, int echo);
void readMagnetometerYaw();
float normalizeAngle(float Yaw);

extern MPU6050 mpu;
extern Servo myServo;
extern float ultrasonicDistance;
extern float yaw;
extern float magYaw;
extern float initialMagYaw;
extern bool initialYawSet;

#endif