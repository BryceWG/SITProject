/*
 * 文件名: utils.cpp
 * 功能: 工具函数和常量的实现
 * 描述: 
 *   - 定义日志前缀常量
 *   - 定义全局变量
 */

#include "utils.h"

const String PREFIX_MOVECONTROL = "1.MOVECONTROL: ";
const String PREFIX_SENSOR = "2.SENSOR: ";
const String PREFIX_SYSTEM = "3.SYSTEM: ";
const String PREFIX_DEBUG = "4.DEBUG: ";
const String PREFIX_ERROR = "5.ERROR: ";

float x = 0;
float y = 0;
const int numSamples = 1000;
float velocity_x = 0, velocity_y = 0;
float x_mm = 0, y_mm = 0;

volatile State currentState = STATE_PID;