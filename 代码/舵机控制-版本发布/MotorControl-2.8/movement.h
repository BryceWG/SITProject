/*
 * 文件名: movement.h
 * 功能: 运动控制相关函数和变量的声明
 * 描述: 
 *   - 声明前进、转向、停止等运动控制函数
 *   - 声明障碍物检测和距离限制检测函数
 *   - 声明运动控制相关的全局变量
 */

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <Servo.h>
#include "utils.h"

// 运动控制函数声明
void moveForward();  // 前进
void turn(float turnAngle);  // 转向
void stop();  // 停止

// 障碍物检测和距离限制函数声明
void readUltrasonicStand();  // 静止状态下读取超声波数据
bool obstacleDistanceCheck(float maxDistance);  // 检查是否有障碍物在指定距离内
void obstacleDetectMoving();  // 移动过程中检测障碍物
void obstacleDetectStand();  // 静止状态下检测障碍物
void distanceLimitDetect(float initial_displacement_x, float initial_displacement_y);  // 检测是否达到距离限制

// 全局变量声明
extern int frontObstacle;  // 前方障碍物标志
extern int leftObstacle;   // 左侧障碍物标志
extern int rightObstacle;  // 右侧障碍物标志
extern int distanceLimit;  // 距离限制标志
extern bool first_call;    // 首次调用标志

#endif