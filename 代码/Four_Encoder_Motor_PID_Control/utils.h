/*
 * 文件名: utils.h
 * 功能: 工具函数和常量定义
 * 描述: 
 *   - 定义日志前缀常量
 *   - 定义引脚常量
 *   - 声明全局变量和状态机枚举
 */

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

// 定义日志前缀
extern const String PREFIX_MOVECONTROL;
extern const String PREFIX_SENSOR;
extern const String PREFIX_SYSTEM;
extern const String PREFIX_DEBUG;
extern const String PREFIX_ERROR;

// 定义引脚
#define M1_ENCODER_A 2
#define M1_ENCODER_B 40
#define M2_ENCODER_A 3
#define M2_ENCODER_B 41
#define M3_ENCODER_A 19
#define M3_ENCODER_B 42
#define M4_ENCODER_A 18
#define M4_ENCODER_B 43

#define TB6612_M1_IN1 22
#define TB6612_M1_IN2 23
#define Motor_M1_PWM 4

#define TB6612_M2_IN1 24
#define TB6612_M2_IN2 25
#define Motor_M2_PWM 5

#define TB6612_M3_IN1 26
#define TB6612_M3_IN2 27
#define Motor_M3_PWM 6

#define TB6612_M4_IN1 28
#define TB6612_M4_IN2 29
#define Motor_M4_PWM 7

#define Trig 12
#define Echo 13

#define BUTTON_PIN 10

// 其他全局变量
extern float x, y;
extern const int numSamples;
extern float velocity_x, velocity_y;
extern float x_mm, y_mm;

// 状态机枚举
enum State { STATE_PID, STATE_READ_MOTOR };
extern volatile State currentState;

#endif