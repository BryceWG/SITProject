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

void moveForward();
void turn(float turnAngle);
void stop();
void readUltrasonicStand();
bool obstacleDistanceCheck(float maxDistance);
void obstacleDetectMoving();
void obstacleDetectStand();
void distanceLimitDetect(float initial_displacement_x, float initial_displacement_y);

extern int frontObstacle;
extern int leftObstacle;
extern int rightObstacle;
extern int distanceLimit;
extern bool first_call;

#endif