/*
 * 文件名: motor_control.cpp
 * 功能: 电机控制相关函数的实现
 * 描述: 
 *   - 实现电机初始化、PWM设置、编码器读取等函数
 *   - 定义与电机控制相关的全局变量
 */

#include "motor_control.h"
#include "utils.h"
#include "pid_control.h"

// 全局变量定义
volatile float motor_M1 = 0, motor_M2 = 0, motor_M3 = 0, motor_M4 = 0;
volatile bool needToReadMotors = false;
int motor_M1_dir = 0, motor_M2_dir = 0, motor_M3_dir = 0, motor_M4_dir = 0;

void motorInit() {
    // 设置编码器引脚为输入
    pinMode(M1_ENCODER_A, INPUT);
    pinMode(M1_ENCODER_B, INPUT);
    pinMode(M2_ENCODER_A, INPUT);
    pinMode(M2_ENCODER_B, INPUT);
    pinMode(M3_ENCODER_A, INPUT);
    pinMode(M3_ENCODER_B, INPUT);
    pinMode(M4_ENCODER_A, INPUT);
    pinMode(M4_ENCODER_B, INPUT);

    // 设置电机控制引脚为输出
    pinMode(TB6612_M1_IN1, OUTPUT);
    pinMode(TB6612_M1_IN2, OUTPUT);
    pinMode(TB6612_M2_IN1, OUTPUT);
    pinMode(TB6612_M2_IN2, OUTPUT);
    pinMode(TB6612_M3_IN1, OUTPUT);
    pinMode(TB6612_M3_IN2, OUTPUT);
    pinMode(TB6612_M4_IN1, OUTPUT);
    pinMode(TB6612_M4_IN2, OUTPUT);

    pinMode(Motor_M1_PWM, OUTPUT);
    pinMode(Motor_M2_PWM, OUTPUT);
    pinMode(Motor_M3_PWM, OUTPUT);
    pinMode(Motor_M4_PWM, OUTPUT);

    // 初始化所有电机控制引脚为低电平
    digitalWrite(TB6612_M1_IN1, LOW);
    digitalWrite(TB6612_M1_IN2, LOW);
    digitalWrite(Motor_M1_PWM, LOW);
    digitalWrite(TB6612_M2_IN1, LOW);
    digitalWrite(TB6612_M2_IN2, LOW);
    digitalWrite(Motor_M2_PWM, LOW);
    digitalWrite(TB6612_M3_IN1, LOW);
    digitalWrite(TB6612_M3_IN2, LOW);
    digitalWrite(Motor_M3_PWM, LOW);
    digitalWrite(TB6612_M4_IN1, LOW);
    digitalWrite(TB6612_M4_IN2, LOW);
    digitalWrite(Motor_M4_PWM, LOW);
}

void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM) {
    // 设置电机1的正反转及PWM值
    if(M1_PWM > 0) {
        digitalWrite(TB6612_M1_IN1, HIGH);      
        digitalWrite(TB6612_M1_IN2, LOW);
        analogWrite(Motor_M1_PWM, M1_PWM);  
    } else {
        digitalWrite(TB6612_M1_IN1, LOW);
        digitalWrite(TB6612_M1_IN2, HIGH);
        analogWrite(Motor_M1_PWM, -1 * M1_PWM);
    }

    // 设置电机2的正反转及PWM值
    if(M2_PWM > 0) {
        digitalWrite(TB6612_M2_IN1, LOW);
        digitalWrite(TB6612_M2_IN2, HIGH);
        analogWrite(Motor_M2_PWM, M2_PWM);
    } else {
        digitalWrite(TB6612_M2_IN1, HIGH);
        digitalWrite(TB6612_M2_IN2, LOW);
        analogWrite(Motor_M2_PWM, -1 * M2_PWM);
    }

    // 设置电机3的正反转及PWM值
    if(M3_PWM > 0) {
        digitalWrite(TB6612_M3_IN1, LOW);
        digitalWrite(TB6612_M3_IN2, HIGH);
        analogWrite(Motor_M3_PWM, M3_PWM);
    } else {
        digitalWrite(TB6612_M3_IN1, HIGH);
        digitalWrite(TB6612_M3_IN2, LOW);
        analogWrite(Motor_M3_PWM, -1 * M3_PWM);
    }

    // 设置电机4的正反转及PWM值
    if(M4_PWM > 0) {
        digitalWrite(TB6612_M4_IN1, HIGH);
        digitalWrite(TB6612_M4_IN2, LOW);
        analogWrite(Motor_M4_PWM, M4_PWM);
    } else {
        digitalWrite(TB6612_M4_IN1, LOW);
        digitalWrite(TB6612_M4_IN2, HIGH);
        analogWrite(Motor_M4_PWM, -1 * M4_PWM);
    }
}

// 读取电机1编码器
void Read_motor_M1() {
    motor_M1_dir = digitalRead(M1_ENCODER_B);
    if(motor_M1_dir == 1) {
        motor_M1++;
    } else {
        motor_M1--;
    }
}

// 读取电机2编码器
void Read_motor_M2() {
    motor_M2_dir = digitalRead(M2_ENCODER_B);
    if(motor_M2_dir == 1) {
        motor_M2++;
    } else {
        motor_M2--;
    }
}

// 读取电机3编码器
void Read_motor_M3() {
    motor_M3_dir = digitalRead(M3_ENCODER_B);
    if(motor_M3_dir == 1) {
        motor_M3++;
    } else {
        motor_M3--;
    }
}

// 读取电机4编码器
void Read_motor_M4() {
    motor_M4_dir = digitalRead(M4_ENCODER_B);
    if(motor_M4_dir == 1) {
        motor_M4++;
    } else {
        motor_M4--;
    }
}

// 读取所有电机速度
void Read_Motor_V() {
    static unsigned long lastReadTime = 0;
    static float M1_Speed = 0, M2_Speed = 0, M3_Speed = 0, M4_Speed = 0;
    const float speed_k = 0.3;
    unsigned long currentTime = millis();
    
    if (currentTime - lastReadTime >= 50 && !needToReadMotors) {  
        lastReadTime = currentTime;
        
        // 重置编码器计数
        motor_M1 = 0;
        motor_M2 = 0;
        motor_M3 = 0;
        motor_M4 = 0;
        
        // 启用中断以读取编码器
        attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), Read_motor_M1, FALLING);
        attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), Read_motor_M2, FALLING);
        attachInterrupt(digitalPinToInterrupt(M3_ENCODER_A), Read_motor_M3, FALLING);
        attachInterrupt(digitalPinToInterrupt(M4_ENCODER_A), Read_motor_M4, FALLING);
        
        needToReadMotors = true;
    } else if (needToReadMotors) {    
        needToReadMotors = false;
        
        // 禁用中断
        detachInterrupt(digitalPinToInterrupt(M1_ENCODER_A));
        detachInterrupt(digitalPinToInterrupt(M2_ENCODER_A));
        detachInterrupt(digitalPinToInterrupt(M3_ENCODER_A));
        detachInterrupt(digitalPinToInterrupt(M4_ENCODER_A));
        
        float deltaTime = (currentTime - lastReadTime) / 1000.0; // 时间差，单位：s

        // 计算每个电机的速度
        float V_M1 = ((motor_M1 / 330.0) * 65.0 * PI) / deltaTime;
        float V_M2 = ((motor_M2 / 330.0) * 65.0 * PI) / deltaTime;
        float V_M3 = ((motor_M3 / 330.0) * 65.0 * PI) / deltaTime;
        float V_M4 = ((motor_M4 / 330.0) * 65.0 * PI) / deltaTime;
        
        // 获取上一次的速度
        M1_Speed = M1_Motor_PID.feedback;
        M2_Speed = M2_Motor_PID.feedback;
        M3_Speed = M3_Motor_PID.feedback;
        M4_Speed = M4_Motor_PID.feedback;

        // 使用低通滤波器更新速度
        M1_Motor_PID.feedback = (1 - speed_k) * V_M1 + speed_k * M1_Speed;
        M2_Motor_PID.feedback = -1 * (1 - speed_k) * V_M2 + speed_k * M2_Speed;
        M3_Motor_PID.feedback = -1 * (1 - speed_k) * V_M3 + speed_k * M3_Speed;
        M4_Motor_PID.feedback = (1 - speed_k) * V_M4 + speed_k * M4_Speed;
        
        //Serial.println(PREFIX_MOVECONTROL + "Read Motor V Finished");
    }
}