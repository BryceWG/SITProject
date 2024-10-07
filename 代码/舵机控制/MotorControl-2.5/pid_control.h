/*
 * 文件名: pid_control.h
 * 功能: PID控制相关结构体和函数的声明
 * 描述: 
 *   - 定义PID控制结构体
 *   - 声明PID初始化、计算等函数
 */

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

// PID控制结构体定义
typedef struct {
    float input;      // 输入值（目标速度）
    float output;     // 输出值（PWM）
    float feedback;   // 反馈值（实际速度）
    float k_p;        // 比例系数
    float k_i;        // 积分系数
    float k_d;        // 微分系数
    float err_1;      // 上一次误差
    float err_2;      // 上上次误差
    float err_x;      // 累积误差
    float out_max;    // 输出最大值
    float out_min;    // 输出最小值
    float err_x_max;  // 累积误差最大值
} PID;

// 函数声明
void pidInit();                   // PID初始化
void PID_Cal(PID *pid);           // PID计算
void PID_Cal_Computer_Out();      // 计算并输出PID结果

// 外部变量声明
extern PID M1_Motor_PID, M2_Motor_PID, M3_Motor_PID, M4_Motor_PID;

#endif