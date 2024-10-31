/*
 * 文件名: movement.cpp
 * 功能: 运动控制相关函数的实现
 * 描述: 
 *   - 实现前进、转向、停止等运动控制函数
 *   - 实现障碍物检测和距离限制检测函数
 *   - 定义运动控制相关的全局变量
 */

#include "movement.h"
#include "utils.h"
#include "sensors.h"
#include "pid_control.h"

// 全局变量定义
int frontObstacle = 0;
int leftObstacle = 0;
int rightObstacle = 0;
int distanceLimit = 0;
bool first_call = true;

void moveForward() {
    static float initial_angle = 0;
    static float initial_displacement_x = 0, initial_displacement_y = 0;

    // 首次调用时初始化初始角度和位置
    if (first_call) {
        initial_angle = yaw;
        initial_displacement_x = x;
        initial_displacement_y = y;
        first_call = false;
    }

    // 计算角度偏移并进行修正
    float angle_offset = yaw - initial_angle;
    float correction = -1 * angle_offset;

    // 设置电机PID输入，加入修正值
    M1_Motor_PID.input = 200 - correction;
    M2_Motor_PID.input = 200 + correction;
    M3_Motor_PID.input = 200 - correction;
    M4_Motor_PID.input = 200 + correction;

    // 检测障碍物和计算运动参数
    obstacleDetectMoving();
    moveCaculate();
    distanceLimitDetect(initial_displacement_x, initial_displacement_y);
    Serial.println(PREFIX_MOVECONTROL + "MoveForward: Moving Forward");
}

void turn(float turnAngle) {
    moveCaculate();
    float startAngle = yaw;
    float targetAngle = startAngle + turnAngle;

    Serial.print(PREFIX_MOVECONTROL + "Turn: Start Angle: ");
    Serial.println(startAngle);
    Serial.print(PREFIX_MOVECONTROL + "Turn: Target Angle: ");
    Serial.println(targetAngle);

    // 持续转向直到达到目标角度
    while (abs(yaw - targetAngle) > 10) {
        if (turnAngle < 0) {
            // 左转
            M1_Motor_PID.input = 100;
            M2_Motor_PID.input = -100;
            M3_Motor_PID.input = -100;
            M4_Motor_PID.input = 100;
            Serial.println(PREFIX_MOVECONTROL + "Turn: Turning to Left");
        } else {
            // 右转
            M1_Motor_PID.input = -100;
            M2_Motor_PID.input = 100;
            M3_Motor_PID.input = 100;
            M4_Motor_PID.input = -100;
            Serial.println(PREFIX_MOVECONTROL + "Turn: Turning to Right");
        }
        moveCaculate();
    }
    stop();
    delay(300);
}

void stop() {
    // 停止所有电机
    M1_Motor_PID.input = 0;
    M2_Motor_PID.input = 0;
    M3_Motor_PID.input = 0;
    M4_Motor_PID.input = 0;
    Serial.println(PREFIX_MOVECONTROL + "Stop: Stopping");
}

void readUltrasonicStand() {
    myServo.write(0);
    stop();
    delay(300);
    // 输出数据开始标记
    Serial.println("<START>");
    // 输出小车位置，假设 yaw 表示小车的朝向角度（单位：度）
    Serial.print("POS:");
    Serial.print(x); // x 为全局变量，单位：cm
    Serial.print(",");
    Serial.print(y); // y 为全局变量，单位：cm
    Serial.print(",");
    Serial.println(yaw); // yaw 为全局变量，单位：度

    // 舵机从 0 度到 180 度扫描
    for (int i = 0; i <= 180; i++) {
        myServo.write(i);
        ultrasonicDistance = getDistance(Trig, Echo);
        if (ultrasonicDistance > 50) {
            ultrasonicDistance = 50;
        } else if (ultrasonicDistance < 5) {
            ultrasonicDistance = 0;
        }
        // 按要求的格式输出扫描数据
        Serial.print("SCAN:");
        Serial.print(i); // angle
        Serial.print(",");
        Serial.println(ultrasonicDistance); // distance
        delay(70);
    }

    myServo.write(90);
    delay(100);
    // 输出数据结束标记
    Serial.println("<END>");
    Serial.println(PREFIX_SENSOR + "Ultrasonic Stand Scanning Finished"); 
}

bool obstacleDistanceCheck(float maxDistance) {
    return ultrasonicDistance < maxDistance;
}

void obstacleDetectMoving() {
    ultrasonicDistance = getDistance(Trig, Echo);
    Serial.print(PREFIX_SENSOR + "Moving Distance Detecting: ");
    Serial.println(ultrasonicDistance);
    if(obstacleDistanceCheck(20.0)) {
        frontObstacle = 1;
        Serial.println(PREFIX_SENSOR + "Front Obstacle Detected");
    } else {
        frontObstacle = 0;
    }
}

void obstacleDetectStand() {
    Serial.println(PREFIX_SENSOR + "Stand Detecting"); 
    stop();
    delay(300);
     
    // 检测左侧障碍物
    myServo.write(180);
    delay(200);
    ultrasonicDistance = getDistance(Trig, Echo);
    delay(10);

    if (obstacleDistanceCheck(30.0)) {
        leftObstacle = 1;
        Serial.println(PREFIX_SENSOR + "Left Obstacle Exists");
    } else {
        leftObstacle = 0;
    }

    // 检测右侧障碍物
    myServo.write(0);
    delay(200);
    ultrasonicDistance = getDistance(Trig, Echo);
    delay(10);

    if (obstacleDistanceCheck(30.0)) {
        rightObstacle = 1;
        Serial.println(PREFIX_SENSOR + "Right Obstacle Exists");
    } else {
        rightObstacle = 0;  
    }
}

void distanceLimitDetect(float initial_displacement_x, float initial_displacement_y) {
    float current_displacement_x = x - initial_displacement_x;
    float current_displacement_y = y - initial_displacement_y;
    float current_distance = sqrt(current_displacement_x * current_displacement_x + current_displacement_y * current_displacement_y);
    
    Serial.print(PREFIX_MOVECONTROL + "Current Distance: ");
    Serial.println(current_distance);

    if (current_distance >= 50) {
        distanceLimit = 1;
        Serial.println(PREFIX_MOVECONTROL + "Attention! Distance Limit!");
    }
}