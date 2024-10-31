/*
 * 文件名: pid_control.cpp
 * 功能: PID控制相关函数的实现
 * 描述: 
 *   - 实现PID初始化、计算等函数
 *   - 定义PID控制相关的全局变量
 */

#include "pid_control.h"
#include "motor_control.h"
#include "utils.h"

PID M1_Motor_PID, M2_Motor_PID, M3_Motor_PID, M4_Motor_PID;

void pidInit() {
    // 初始化电机1的PID参数
    M1_Motor_PID.k_p = 0.2;
    M1_Motor_PID.k_i = 0.07;
    M1_Motor_PID.k_d = 0.225;
    M1_Motor_PID.out_max = 250;
    M1_Motor_PID.out_min = -250;
    M1_Motor_PID.input = 0;
    M1_Motor_PID.err_x_max = 1000;

    // 初始化电机2的PID参数（与电机1相同）
    M2_Motor_PID.k_p = 0.2;
    M2_Motor_PID.k_i = 0.07;
    M2_Motor_PID.k_d = 0.225;
    M2_Motor_PID.out_max = 250;
    M2_Motor_PID.out_min = -250;
    M2_Motor_PID.input = 0;
    M2_Motor_PID.err_x_max = 1000;

    // 初始化电机3的PID参数（与电机1相同）
    M3_Motor_PID.k_p = 0.2;
    M3_Motor_PID.k_i = 0.07;
    M3_Motor_PID.k_d = 0.225;
    M3_Motor_PID.out_max = 250;
    M3_Motor_PID.out_min = -250;
    M3_Motor_PID.input = 0;
    M3_Motor_PID.err_x_max = 1000;

    // 初始化电机4的PID参数（与电机1相同）
    M4_Motor_PID.k_p = 0.2;
    M4_Motor_PID.k_i = 0.07;
    M4_Motor_PID.k_d = 0.225;
    M4_Motor_PID.out_max = 250;
    M4_Motor_PID.out_min = -250;
    M4_Motor_PID.input = 0;
    M4_Motor_PID.err_x_max = 1000;
}

void PID_Cal(PID *pid) {
    float p, i, d;

    // 更新误差
    pid->err_2 = pid->err_1;
    pid->err_1 = pid->input - pid->feedback;

    // 计算PID各项
    p = pid->k_p * pid->err_1;
    i = pid->k_i * pid->err_x;
    d = pid->k_d * (pid->err_1 - pid->err_2);
    
    // 更新累积误差
    pid->err_x += pid->err_1;
    
    // 计算输出
    pid->output = p + i + d;

    // 限制输出和累积误差
    if(pid->output > pid->out_max)      pid->output = pid->out_max;
    if(pid->output < pid->out_min)      pid->output = pid->out_min;
    if(pid->err_x > pid->err_x_max)     pid->err_x = pid->err_x_max;
}

void PID_Cal_Computer_Out() {
    // 为每个电机计算PID输出
    PID_Cal(&M1_Motor_PID);
    PID_Cal(&M2_Motor_PID);
    PID_Cal(&M3_Motor_PID);
    PID_Cal(&M4_Motor_PID);
    
    // 设置电机PWM
    Motor_PWM_Set(M1_Motor_PID.output, M2_Motor_PID.output, M3_Motor_PID.output, M4_Motor_PID.output);
    
    // 注释掉的计时代码，可能用于调试
    // static int timecnt = 0;
    // static int time_seconds = 0;
    
    // timecnt++;
    
    // if(timecnt == 20) {
    //     time_seconds++;
    //     timecnt = 0;
    // }
}