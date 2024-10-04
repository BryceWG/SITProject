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

typedef struct {
    float input;
    float output;
    float feedback;
    float k_p;
    float k_i;
    float k_d;
    float err_1;
    float err_2;
    float err_x;
    float out_max;
    float out_min;
    float err_x_max;
} PID;

void pidInit();
void PID_Cal(PID *pid);
void PID_Cal_Computer_Out();

extern PID M1_Motor_PID, M2_Motor_PID, M3_Motor_PID, M4_Motor_PID;

#endif