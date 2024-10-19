#include <MsTimer2.h> // 引入MsTimer2库，用于设置定时中断
#include <Arduino.h> // 引入Arduino标准库，用于Arduino的基本操作
#include "MPU6050.h" // 引入MPU6050库，用于读取MPU6050传感器数据
#include <Servo.h>    // 引入Servo库，用于控制舵机
#include <Wire.h>     // 引入Wire库，用于I2C通信    
#include "I2Cdev.h"

// 定义日志前缀
const String PREFIX_MOVECONTROL = "1.MOVECONTROL: ";
const String PREFIX_SENSOR       = "2.SENSOR: ";
const String PREFIX_SYSTEM       = "3.SYSTEM: ";
const String PREFIX_DEBUG        = "4.DEBUG: ";
const String PREFIX_ERROR        = "5.ERROR: ";

// 磁力计和Yaw角变量
float magYaw = 0;   // 磁力计Yaw角
float yaw = 0;       // 小车Yaw角
bool initialYawSet = false;
float initialMagYaw = 0;
bool turnfinish = false;
// 函数声明
void turn(float turnAngle); // 角度转向
void readMagnetometerYaw(); // 读取磁力计Yaw角数据
float normalizeAngle(float angle); // 将角度限制在-180到180之间

void setup() 
{
    Serial.begin(115200);              // 打开串口
    while (!Serial) delay(10); // 等待串口监视器打开

    Serial2.begin(460800); // 初始化Serial2，用于与磁力计通信

    // 读取并设置初始Yaw角
    Serial.println(PREFIX_SYSTEM + "Reading Initial Yaw from Magnetometer...");
    int attempts = 0;
    while (!initialYawSet && attempts < 10) { // 最多尝试10次
        readMagnetometerYaw();
        if (initialYawSet) 
        {
            Serial.print(PREFIX_SYSTEM + "Initial Yaw set to: ");
            Serial.println(initialMagYaw);
            break;
        }
        attempts++;
        delay(10); // 每次尝试后延时10ms
    }
    if (!initialYawSet) {
        Serial.println(PREFIX_ERROR + "Failed to Set Initial Yaw. Defaulting to 0.");
        initialMagYaw = 0;
    }
}

void loop() 
{
  if (!turnfinish)
  {
    turn(-90);  // 左转90度
    //turnfinish = true;
  }

}

void turn(float turnAngle)
{
    float startAngle = yaw;
    float targetAngle = startAngle + turnAngle;

    Serial.print(PREFIX_MOVECONTROL + "Turn: Start Angle: ");
    Serial.println(startAngle);
    Serial.print(PREFIX_MOVECONTROL + "Turn: Target Angle: ");
    Serial.println(targetAngle);

    while (abs(yaw - targetAngle) > 10)
    {
        readMagnetometerYaw();
        //Serial.print(PREFIX_MOVECONTROL + "Turn: Current Yaw: ");
        //Serial.println(yaw);
        delay(100);
    }

    Serial.println(PREFIX_MOVECONTROL + "Turn: Turn Completed");
    delay(300); // 停止一段时间
}

void readMagnetometerYaw()
{
    String dataFromMagnetometer = Serial2.readStringUntil('\n');
    Serial.print(PREFIX_SENSOR + "Received from new hardware: ");
    Serial.println(dataFromMagnetometer);

    // 移除所有不可打印字符
    String cleanData = "";
    for (char c : dataFromMagnetometer) {
        if (isprint(c)) {
            cleanData += c;
        }
    }
    cleanData.trim();

    int yawIndex = cleanData.indexOf("Yaw:");
    if (yawIndex != -1) 
    {
        String yawString = cleanData.substring(yawIndex + 4);
        yawString.trim();
        magYaw = yawString.toFloat();

        if (!initialYawSet) {
            initialYawSet = true;
            initialMagYaw = magYaw;
        }
        else 
        {
            Serial.print(PREFIX_SENSOR + "MagYaw: ");
            Serial.println(magYaw);

            // 调整Yaw为相对于初始Yaw的绝对角度
            yaw = normalizeAngle(magYaw);
        }
        Serial.print(PREFIX_SENSOR + "Yaw: ");
        Serial.println(yaw);
    } 
    else 
    {
        Serial.println(PREFIX_ERROR + "Error: Yaw Data Not Found in Magnetometer Output.");
    } 
}


float normalizeAngle(float Yaw) 
{
    float tempYaw = Yaw - initialMagYaw;

    // 处理角度跳变，确保在 -180 到 180 之间
    if (tempYaw > 180) 
    {
        tempYaw -= 360;
    } 
    else if (tempYaw < -180) 
    {
        tempYaw += 360;
    }
    return tempYaw;
}