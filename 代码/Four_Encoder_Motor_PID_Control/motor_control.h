/*
 * 文件名: motor_control.h
 * 功能: 电机控制相关函数和变量的声明
 * 描述: 
 *   - 声明电机初始化、PWM设置、编码器读取等函数
 *   - 声明与电机控制相关的全局变量
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 函数声明
void motorInit();  // 电机初始化
void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM);  // 设置电机PWM
void Read_motor_M1();  // 读取电机1编码器
void Read_motor_M2();  // 读取电机2编码器
void Read_motor_M3();  // 读取电机3编码器
void Read_motor_M4();  // 读取电机4编码器
void Read_Motor_V();   // 读取所有电机速度

// 全局变量声明
extern volatile long motor_M1, motor_M2, motor_M3, motor_M4;  // 电机编码器计数
extern volatile bool needToReadMotors;  // 是否需要读取电机速度的标志
extern int motor_M1_dir, motor_M2_dir, motor_M3_dir, motor_M4_dir;  // 电机方向
extern volatile int timecnt;

#endif