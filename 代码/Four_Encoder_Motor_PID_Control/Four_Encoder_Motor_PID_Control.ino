/*
 * 文件名: Four_Encoder_Motor_PID_Control.ino
 * 功能: 主程序文件,包含setup()和loop()函数
 * 描述: 
 *   - 初始化各个模块
 *   - 实现主循环逻辑,包括按钮检测、运动控制和障碍物检测
 *   - 处理定时中断,用于PID计算和电机速度读取
 */

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
    motorInit();
    pidInit();

    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial2.begin(460800);

    mpuInit();
    ultrasonicInit();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    myServo.write(90);
    delay(200);

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

    MsTimer2::set(40, interruptHandler);
    MsTimer2::start();
}

void loop() {
    static bool buttonPressed = false;

    if (!buttonPressed) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            buttonPressed = true;
            Serial.println(PREFIX_SYSTEM + "按钮已按下，小车开始运动");
            delay(200);
        } else {
            return;
        }
    }

    if (buttonPressed) {
        if (frontObstacle == 0 && distanceLimit == 0) {
            moveForward();
            Serial.println(PREFIX_MOVECONTROL + "Loop: Move Forward");
        } else if (distanceLimit == 1) {
            Serial.println(PREFIX_MOVECONTROL + "Loop: Distance Limit");
            readUltrasonicStand();
            distanceLimit = 0;
            first_call = true;
        } else if (frontObstacle == 1) {
            Serial.println(PREFIX_MOVECONTROL + "Loop: Obstacle Detected"); 
            obstacleDetectStand();
            if (leftObstacle == 0) {
                Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Left");
                turn(-90);
            } else if (rightObstacle == 0 && leftObstacle == 1) {
                Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Right");
                turn(90);
            } else if (leftObstacle == 1 && rightObstacle == 1) {
                Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Around");
                turn(180);
            }
            readUltrasonicStand();
            frontObstacle = 0;
            myServo.write(90);
            delay(100);
        }
    }
}

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