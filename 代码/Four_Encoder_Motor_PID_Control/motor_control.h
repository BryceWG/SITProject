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

void motorInit();
void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM);
void Read_motor_M1();
void Read_motor_M2();
void Read_motor_M3();
void Read_motor_M4();
void Read_Motor_V();

extern volatile float motor_M1, motor_M2, motor_M3, motor_M4;
extern volatile bool needToReadMotors;
extern int motor_M1_dir, motor_M2_dir, motor_M3_dir, motor_M4_dir;

#endif