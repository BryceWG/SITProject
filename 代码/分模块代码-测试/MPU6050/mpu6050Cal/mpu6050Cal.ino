#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float temp;

float roll = 0, pitch = 0, yaw = 0;
unsigned long lastTime = 0;
float dt;

// 校准变量
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

const int numSamples = 1000; // 校准时的采样次数

// 卡尔曼滤波器参数
float Q_angle = 0.001; // 过程噪声协方差
float Q_bias = 0.003; // 过程噪声协方差
float R_measure = 0.03; // 测量噪声协方差

// Roll和Pitch的卡尔曼滤波器变量
float angle[2] = {0, 0}; // 角度
float bias[2] = {0, 0}; // 角速度偏差
float rate[2] = {0, 0}; // 角速度
float P[2][2][2] = {{{0, 0}, {0, 0}}, {{0, 0}, {0, 0}}}; // 误差协方差矩阵

void setup() {
Wire.begin();
Serial.begin(115200);

mpu.initialize();
Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

Serial.println("Starting calibration. Please keep the sensor still...");
calibrateSensor();
Serial.println("Calibration complete!");

lastTime = millis(); // 初始化 lastTime
}

void calibrateSensor() {
long ax_sum = 0, ay_sum = 0, az_sum = 0;
long gx_sum = 0, gy_sum = 0, gz_sum = 0;

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
az_offset = az_sum / numSamples - 16384; // 减去 1g 的值
gx_offset = gx_sum / numSamples;
gy_offset = gy_sum / numSamples;
gz_offset = gz_sum / numSamples;

Serial.println("Calibration results:");
Serial.print("ax_offset: "); Serial.println(ax_offset);
Serial.print("ay_offset: "); Serial.println(ay_offset);
Serial.print("az_offset: "); Serial.println(az_offset);
Serial.print("gx_offset: "); Serial.println(gx_offset);
Serial.print("gy_offset: "); Serial.println(gy_offset);
Serial.print("gz_offset: "); Serial.println(gz_offset);
}

float kalmanFilter(float newAngle, float newRate, int axis, float dt) {
// 步骤1：预测
rate[axis] = newRate - bias[axis];
angle[axis] += dt * rate[axis];

// 更新误差协方差矩阵
P[axis][0][0] += dt * (dt * P[axis][1][1] - P[axis][0][1] - P[axis][1][0] + Q_angle);
P[axis][0][1] -= dt * P[axis][1][1];
P[axis][1][0] -= dt * P[axis][1][1];
P[axis][1][1] += Q_bias * dt;

// 步骤2：更新
float y = newAngle - angle[axis];
float S = P[axis][0][0] + R_measure;
float K[2] = {P[axis][0][0] / S, P[axis][1][0] / S};

angle[axis] += K[0] * y;
bias[axis] += K[1] * y;

float P00_temp = P[axis][0][0];
float P01_temp = P[axis][0][1];

P[axis][0][0] -= K[0] * P00_temp;
P[axis][0][1] -= K[0] * P01_temp;
P[axis][1][0] -= K[1] * P00_temp;
P[axis][1][1] -= K[1] * P01_temp;

return angle[axis];
}

void loop() {
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
temp = mpu.getTemperature() / 340.0 + 36.53;

// 应用校准偏移
ax = ax - ax_offset;
ay = ay - ay_offset;
az = az - az_offset;
gx = gx - gx_offset;
gy = gy - gy_offset;
gz = gz - gz_offset;

float accelX = ax / 16384.0;
float accelY = ay / 16384.0;
float accelZ = az / 16384.0;

float gyroX = gx / 131.0;
float gyroY = gy / 131.0;
float gyroZ = gz / 131.0;

// 计算加速度计角度
float accelRoll = atan2(accelY, accelZ) * 180 / PI;
float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

// 计算时间增量
unsigned long currentTime = millis();
dt = (currentTime - lastTime) / 1000.0;
lastTime = currentTime;

// 应用卡尔曼滤波
roll = kalmanFilter(accelRoll, gyroX, 0, dt);
pitch = kalmanFilter(accelPitch, gyroY, 1, dt);
yaw += gyroZ * dt; // 简单积分，可能会有漂移

Serial.print("Roll: ");
Serial.print(roll);
Serial.print("\tPitch: ");
Serial.print(pitch);
Serial.print("\tYaw: ");
Serial.print(yaw);
Serial.print("\tTemp: ");
Serial.println(temp);

delay(10);
}