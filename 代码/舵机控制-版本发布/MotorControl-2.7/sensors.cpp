/*
 * 文件名: sensors.cpp
 * 功能: 传感器相关函数的实现
 * 描述: 
 *   - 实现MPU6050、超声波传感器、磁力计等相关函数
 *   - 定义传感器相关的全局变量
 */

#include "sensors.h"
#include "utils.h"
#include <Wire.h>

// 全局变量定义
MPU6050 mpu;
Servo myServo;
float ultrasonicDistance = 0;
float yaw = 0;
float magYaw = 0;
float initialMagYaw = 0;
bool initialYawSet = false;

// 加速度计变量
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

// 简单低通滤波器参数
float filteredAccelX = 0;
float filteredAccelY = 0;

// 时间差变量
unsigned long lastTime = 0;
float dt;

void mpuInit() {
    Wire.begin();
    mpu.initialize();
    Serial.println(mpu.testConnection() ? (PREFIX_SYSTEM + "MPU6050 Connection Successful") : (PREFIX_ERROR + "MPU6050 Connection Failed"));

    // 设置陀螺仪和加速度计的量程
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    calibrateSensor();

    Serial.println(PREFIX_SYSTEM + "Setup: MPUInit Finished");
    lastTime = millis(); // 初始化 lastTime
}

void ultrasonicInit() {
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    myServo.attach(11);
    myServo.write(90);
    delay(50);
    Serial.println(PREFIX_SYSTEM + "Setup: UltrasonicInit Finished");  
}

void calibrateSensor() {
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    // 采集多次样本并求平均值作为偏移量
    for (int i = 0; i < numSamples; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        delay(2);
    }

    ax_offset = ax_sum / numSamples;
    ay_offset = ay_sum / numSamples;
    az_offset = az_sum / numSamples;
    gx_offset = gx_sum / numSamples;
    gy_offset = gy_sum / numSamples;
    gz_offset = gz_sum / numSamples;

    Serial.println(PREFIX_SYSTEM + "Sensor Calibration Complete.");
}

void moveCaculate() {
    // 获取MPU6050数据
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 应用校准偏移
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;

    // 计算加速度 (g)
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    // 调整低通滤波器系数
    const float alpha = 0.95; // 增加alpha值，使滤波器对新数据的响应更慢

    // 简单低通滤波
    filteredAccelX = alpha * filteredAccelX + (1 - alpha) * accelX;
    filteredAccelY = alpha * filteredAccelY + (1 - alpha) * accelY;

    // 计算时间增量
    unsigned long currentTime = millis();  // 使用毫秒
    dt = (currentTime - lastTime) / 1000.0;  // 转换为秒
    lastTime = currentTime;

    // 读取磁力计Yaw角
    readMagnetometerYaw();

    // 获取四个电机的速度 (mm/s)
    float v1 = M1_Motor_PID.feedback;
    float v2 = M2_Motor_PID.feedback;
    float v3 = M3_Motor_PID.feedback;
    float v4 = M4_Motor_PID.feedback;

    Serial.print(PREFIX_MOVECONTROL + "Motor Speed: ");
    Serial.print(v1);
    Serial.print(", ");
    Serial.print(v2);
    Serial.print(", ");
    Serial.print(v3);
    Serial.print(", ");
    Serial.println(v4);

    // 基于编码器的线速度
    float encoderSpeed = (v1 + v2 + v3 + v4) / 4.0;

    // 判断是否处于静止状态
    const float SPEED_THRESHOLD = 5.0; // mm/s，可以根据实际情况调整
    bool isStationary = abs(encoderSpeed) < SPEED_THRESHOLD;

    if (isStationary) {
        // 静止状态下，重置速度和加速度
        velocity_x = 0;
        velocity_y = 0;
        filteredAccelX = 0;
        filteredAccelY = 0;
    } else {
        // 基于加速度计的速度估计 (简单积分)
        velocity_x += filteredAccelX * dt * 1000.0 * 9.81; // 转换为 mm/s² -> mm/s
        velocity_y += filteredAccelY * dt * 1000.0 * 9.81; // 转换为 mm/s² -> mm/s
    }

    Serial.print(PREFIX_MOVECONTROL + "Velocity X: ");
    Serial.println(velocity_x);
    Serial.print(PREFIX_MOVECONTROL + "Velocity Y: ");
    Serial.println(velocity_y);
    
    // 融合编码器速度与加速度计速度，增加编码器数据的权重
    float fused_velocity_x = 0.9 * encoderSpeed * cos(yaw * PI / 180.0) + 0.1 * velocity_x;
    float fused_velocity_y = 0.9 * encoderSpeed * sin(yaw * PI / 180.0) + 0.1 * velocity_y;

    // 计算位移 (mm)
    float distance_x_mm = fused_velocity_x * dt;
    float distance_y_mm = fused_velocity_y * dt;

    // 更新坐标 (mm)
    x_mm += distance_x_mm;
    y_mm += distance_y_mm;

    // 将毫米转换为厘米，用于输出
    x = x_mm / 10.0;
    y = y_mm / 10.0;

    Serial.print(PREFIX_MOVECONTROL + "Position -> X: ");
    Serial.print(x);
    Serial.print(" cm, Y: ");
    Serial.print(y);
    Serial.println(" cm");
    Serial.print(PREFIX_MOVECONTROL + "Yaw: ");
    Serial.println(yaw);
}

float getDistance(int trig, int echo) {
    const int MEASUREMENTS = 2;
    float total = 0, temp2 = 0, distancecm = 0; 

    for (int i = 0; i < MEASUREMENTS; i++) {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        temp2 = (float)(pulseIn(echo, HIGH, 750000));
        distancecm = temp2 * 17 / 1000;
        total += distancecm;
        delay(10);
    }

    return total / MEASUREMENTS;
}

void readMagnetometerYaw() {
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
    if (yawIndex != -1) {
        String yawString = cleanData.substring(yawIndex + 4);
        yawString.trim();
        magYaw = yawString.toFloat();

        if (!initialYawSet) {
            initialYawSet = true;
            initialMagYaw = magYaw;
        } else {
            Serial.print(PREFIX_SENSOR + "MagYaw: ");
            Serial.println(magYaw);

            yaw = normalizeAngle(magYaw);
        }
        Serial.print(PREFIX_SENSOR + "Yaw: ");
        Serial.println(yaw);
    } else {
        Serial.println(PREFIX_ERROR + "Error: Yaw Data Not Found in Magnetometer Output.");
    } 
}

float normalizeAngle(float Yaw) {
    float temp = Yaw - initialMagYaw;

    if (temp > 180) {
        temp -= 360;
    } else if (temp < -180) {
        temp += 360;
    }
    return temp;
}