/*
 * 文件名: utils.cpp
 * 功能: 工具函数和常量的实现
 * 描述: 
 *   - 定义日志前缀常量
 *   - 定义全局变量
 */

#include "utils.h"

// 定义日志前缀常量
const String PREFIX_MOVECONTROL = "1.MOVECONTROL: ";
const String PREFIX_SENSOR = "2.SENSOR: ";
const String PREFIX_SYSTEM = "3.SYSTEM: ";
const String PREFIX_DEBUG = "4.DEBUG: ";
const String PREFIX_ERROR = "5.ERROR: ";

// 定义全局变量
float x = 0;  // 小车在x方向的位置（厘米）
float y = 0;  // 小车在y方向的位置（厘米）
const int numSamples = 1000;  // 用于传感器校准的样本数
float velocity_x = 0, velocity_y = 0;  // 小车在x和y方向的速度（毫米/秒）
float x_mm = 0, y_mm = 0;  // 小车在x和y方向的位置（毫米）

// 初始化状态机状态
volatile State currentState = STATE_PID;