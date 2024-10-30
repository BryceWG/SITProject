/*
 * 文件名: Four_Encoder_Motor_PID_Control.ino
 * 功能: 主程序文件，包含setup()和loop()函数
 * 描述: 
 *   - 初始化各个模块
 *   - 实现主循环逻辑，包括按钮检测、运动控制和障碍物检测
 *   - 处理定时中断，用于PID计算和电机速度读取
 */

// 包含所需的库文件
#include <MsTimer2.h>
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "motor_control.h"
#include "pid_control.h"
#include "sensors.h"
#include "movement.h"
#include "utils.h"

void setup() {
    // 初始化电机和PID控制器
    motorInit();
    pidInit();

    // 初始化串口通信
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial2.begin(460800);

    // 初始化MPU6050和超声波传感器
    mpuInit();
    ultrasonicInit();
    
    // 设置按钮引脚为输入上拉模式
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // 初始化舵机位置
    myServo.write(90);
    delay(200);

    // 读取初始磁力计Yaw角
    Serial.println(PREFIX_SYSTEM + "Reading Initial Yaw from Magnetometer...");
    int attempts = 0;
    while (!initialYawSet && attempts < 10) {
        readMagnetometerYaw();
        if (initialYawSet) {
            Serial.print(PREFIX_SYSTEM + "Initial Yaw set to: ");
            Serial.println(initialMagYaw);
            break;
        }
        attempts++;
        delay(10);
    }
    if (!initialYawSet) {
        Serial.println(PREFIX_ERROR + "Failed to Set Initial Yaw. Defaulting to 0.");
        initialMagYaw = 0;
    }

    // 设置定时中断，用于PID计算和电机速度读取
    MsTimer2::set(40, interruptHandler);
    MsTimer2::start();
}

void loop() {
    static bool buttonPressed = true;

    // 检测按钮是否被按下
    if (!buttonPressed) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            buttonPressed = true;
            Serial.println(PREFIX_SYSTEM + "按钮已按下，小车开始运动");
            delay(200);  // 简单的消抖
        } else {
            return;
        }
    }

    // 按钮被按下后的主要逻辑
    if (buttonPressed) {
        if (frontObstacle == 0 && distanceLimit == 0) {
            // 无障碍物且未达到距离限制，继续前进
            moveForward();
            Serial.println(PREFIX_MOVECONTROL + "Loop: Move Forward");
        } else if (distanceLimit == 1) {
            // 达到距离限制，停止并重新扫描
            Serial.println(PREFIX_MOVECONTROL + "Loop: Distance Limit");
            readUltrasonicStand();
            distanceLimit = 0;
            first_call = true;
        } else if (frontObstacle == 1) {
            // 检测到前方障碍物
            Serial.println(PREFIX_MOVECONTROL + "Loop: Obstacle Detected"); 
            obstacleDetectStand();
            if (leftObstacle == 0) {
                // 左侧无障碍，向左转
                Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Left");
                turn(-90);
            } else if (rightObstacle == 0 && leftObstacle == 1) {
                // 右侧无障碍，向右转
                Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Right");
                turn(90);
            } else if (leftObstacle == 1 && rightObstacle == 1) {
                // 左右都有障碍，掉头
                Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Around");
                turn(180);
            }
            // 转向后重新扫描
            readUltrasonicStand();
            frontObstacle = 0;
            myServo.write(90);
            delay(100);
        }
    }
}

// 中断处理函数，用于PID计算和电机速度读取
void interruptHandler() {
    switch (currentState) {
    case STATE_PID:
        PID_Cal_Computer_Out();
        currentState = STATE_READ_MOTOR;
        break;
    case STATE_READ_MOTOR:
        Read_Motor_V();
        currentState = STATE_PID;
        break;
    }
}