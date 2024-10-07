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

// 定义电机编码器的引脚，用于测量转速
#define M1_ENCODER_A 2  // 右前电机编码器A相引脚，用于下降沿捕获
#define M1_ENCODER_B 40 // 右前电机编码器B相引脚

#define M2_ENCODER_A 3  // 左前电机编码器A相引脚，用于下降沿捕获
#define M2_ENCODER_B 41 // 左前电机编码器B相引脚

#define M3_ENCODER_A 19 // 左后电机编码器A相引脚，用于下降沿捕获
#define M3_ENCODER_B 42 // 左后电机编码器B相引脚

#define M4_ENCODER_A 18 // 右后电机编码器A相引脚，用于下降沿捕获
#define M4_ENCODER_B 43 // 右后电机编码器B相引脚

// 定义电机驱动TB6612的控制引脚
#define TB6612_M1_IN1 22
#define TB6612_M1_IN2 23
#define Motor_M1_PWM 4 // 右前电机PWM控制引脚

#define TB6612_M2_IN1 24
#define TB6612_M2_IN2 25
#define Motor_M2_PWM 5 // 左前电机PWM控制引脚

#define TB6612_M3_IN1 26
#define TB6612_M3_IN2 27
#define Motor_M3_PWM 6 // 左后电机PWM控制引脚

#define TB6612_M4_IN1 28
#define TB6612_M4_IN2 29
#define Motor_M4_PWM 7 // 右后电机PWM控制引脚

// 定义超声波引脚
#define Trig 12 // 引脚Trig 连接 IO D2  
#define Echo 13 // 引脚Echo 连接 IO D3

MPU6050 mpu;   // 实例化MPU6050传感器对象
Servo myServo; // 实例化舵机对象

typedef struct
{
    float input;      // PID输入值（目标速度）
    float output;     // PID输出值（调整后的PWM值）
    float feedback;   // PID反馈值（实际速度）
    float k_p;        // 比例系数
    float k_i;        // 积分系数
    float k_d;        // 微分系数
    float err_1;      // 前一次误差
    float err_2;      // 前两次误差
    float err_x;      // 累计误差

    float out_max;    // 输出最大值
    float out_min;    // 输出最小值
    float err_x_max;  // 累计误差最大值
} PID;

// 定义中断变量，用于计数脉冲
volatile float motor_M1 = 0; 
volatile float motor_M2 = 0; 
volatile float motor_M3 = 0;
volatile float motor_M4 = 0;
volatile bool needToReadMotors = false;

int timecnt = 0; // 50ms计时器
int time_seconds = 0; // 秒计时器
float V_M1 = 0; // 临时存储右前电机速度变量
float V_M2 = 0; // 临时存储左前电机速度变量
float V_M3 = 0; // 临时存储左后电机速度变量
float V_M4 = 0; // 临时存储右后电机速度变量
int motor_M1_dir = 0; // 反馈的右前电机转动方向
int motor_M2_dir = 0; // 反馈的左前电机转动方向
int motor_M3_dir = 0; // 反馈的左后电机转动方向
int motor_M4_dir = 0; // 反馈的右后电机转动方向
PID M1_Motor_PID, M2_Motor_PID, M3_Motor_PID, M4_Motor_PID; // PID结构体实例

int timeinterrupt = 0;

// 定义障碍检测变量
int frontObstacle = 0;  // 前方是否存在障碍物
int leftObstacle = 0;   // 左方是否存在障碍物
int rightObstacle = 0;  // 右方是否存在障碍物

int distanceLimit = 0;  // 行驶距离是否到达上限
bool first_call = true;

// 定义位移变量
float x = 0; // 小车x轴位移 (厘米)
float y = 0; // 小车y轴位移 (厘米)
float x_mm = 0, y_mm = 0;  // 当前坐标，单位：毫米
float velocity_x = 0; // 小车x轴速度 (mm/s)
float velocity_y = 0; // 小车y轴速度 (mm/s)
float yaw = 0;
float magYaw = 0;   // 磁力计Yaw角

float ultrasonicDistance = 0; // 超声波距离变量

// 时间差变量
unsigned long lastTime = 0;
float dt;

// 加速度计变量
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

const int numSamples = 1000; // 校准时的采样次数

// 简单低通滤波器参数
float filteredAccelX = 0;
float filteredAccelY = 0;

// 新增变量：初始磁偏角
float initialMagYaw = 0;
bool initialYawSet = false;

// 函数声明
void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM); // 设置电机PWM值以控制速度和方向
void Motor_Init(void); // 初始化电机控制引脚
void PID_Init(void); // 初始化PID控制器参数
void PID_Cal(PID *pid); // 计算PID控制器的输出
void Read_Motor_V(void); // 读取电机速度（通过编码器脉冲）
void PID_Cal_Computer_Out(void); // 定期计算PID输出并更新电机PWM值
void interruptHandler(void); // 定时器中断处理函数
void ultrasonicInit(void); // 初始化超声波传感器
void mpuInit(void); // 初始化MPU6050传感器
void calibrateSensor(); // 校准MPU6050传感器
void moveCaculate(void); // 读取MPU6050传感器数据，并进行位移估算
void moveForward(void); // 向前移动
void turn(float turnAngle); // 角度转向
void stop(void); // 停止
void readUltrasonicStand(void); // 静止扫描读取超声波传感器数据
bool obstacleDistanceCheck(float maxDistance); // 障碍物距离检测
void obstacleDetectMoving(void); // 行驶时距离检测
void obstacleDetectStand(void); // 静止时障碍物检测函数
void distanceLimitDetect(float initial_displacement_x, float initial_displacement_y); // 距离上限检测
void readMagnetometerYaw(void); // 读取磁力计Yaw角数据
float getDistance(int trig, int echo); // 获取超声波距离
float normalizeAngle(float Yaw); // 将角度限制在-180到180之间

enum State { STATE_PID, STATE_READ_MOTOR }; // 定义状态机状态
volatile State currentState = STATE_PID;    // 定义状态机变量

void setup() // Arduino的setup函数，用于初始化
{
    Motor_Init();   // 初始化电机控制引脚
    PID_Init();     // 初始化PID控制器参数

    Serial.begin(115200);              // 打开串口
    while (!Serial) delay(10); // 等待串口监视器打开
    Serial2.begin(460800); // 初始化Serial2，用于与磁力计通信

    mpuInit();  // 初始化MPU6050传感器
    ultrasonicInit();   // 初始化超声波传感器
    myServo.write(90);  // 舵机转动到90度,具体角度待确定
    delay(200); // 延时200ms, 等待舵机转动到指定角度

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

    MsTimer2::set(40, interruptHandler); // 每40ms触发一次中断
    MsTimer2::start();  // 启动定时器中断
}

void loop() 
{
    if (frontObstacle == 0 && distanceLimit == 0)    // 障碍物不存在且行驶距离未到达上限时，继续行驶
    {
        moveForward();  // 向前移动
        Serial.println(PREFIX_MOVECONTROL + "Loop: Move Forward");
    }
    else if (distanceLimit == 1)   // 行驶距离到达上限
    {
        Serial.println(PREFIX_MOVECONTROL + "Loop: Distance Limit");
        readUltrasonicStand();  // 静止时超声波扫描
        distanceLimit = 0;  // 距离上限标志位清零
        first_call = true;  // 第一次调用标志位清零
    }
    else if (frontObstacle == 1)   // 障碍物检测
    {
        Serial.println(PREFIX_MOVECONTROL + "Loop: Obstacle Detected"); 
        obstacleDetectStand();      // 静止时障碍物检测
        if (leftObstacle == 0)      // 左侧无障碍物
        {
            Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Left");
            turn(-90);   // 左转
        }
        else if (rightObstacle == 0 && leftObstacle == 1)   // 右侧无障碍物
        {
            Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Right");
            turn(90);  // 右转
        }
        else if (leftObstacle == 1 && rightObstacle == 1)   // 左右均有障碍物
        {
            Serial.println(PREFIX_MOVECONTROL + "Loop: Turn Around");
            turn(180);  // 掉头
        }
        readUltrasonicStand();  // 静止时超声波雷达扫描
        frontObstacle = 0;      // 前方障碍物标志位清零
        myServo.write(90);  // 舵机转动到90度,具体角度待确定
        delay(100);
    }
}

void moveForward()  // 向前移动
{
    static float initial_angle = 0; // 记录初始角度
    static float initial_displacement_x = 0, initial_displacement_y = 0;    // 记录初始位移

    if (first_call)     // 第一次调用时初始化变量
    {
        initial_angle = yaw; // 记录切换到直行时的初始角度
        initial_displacement_x = x; // 记录初始位移
        initial_displacement_y = y;
        first_call = false; // 标记为非第一次调用
    }

    float angle_offset = yaw - initial_angle; // 计算角度偏差
    float correction = -1 * angle_offset; // 根据角度偏差计算修正值

    // 根据角度偏差调整左右电机的速度
    M1_Motor_PID.input = 200 - correction; // 右前电机
    M2_Motor_PID.input = 200 + correction; // 左前电机
    M3_Motor_PID.input = 200 - correction; // 左后电机
    M4_Motor_PID.input = 200 + correction; // 右后电机

    obstacleDetectMoving(); // 行驶时障碍物检测
    moveCaculate(); // 位移计算
    distanceLimitDetect(initial_displacement_x, initial_displacement_y);    // 行驶距离检测
    Serial.println(PREFIX_MOVECONTROL + "MoveForward: Moving Forward");
}

void turn(float turnAngle)  // 角度转向
{
    moveCaculate(); // 位移和角度计算
    float startAngle = yaw;   // 记录当前角度
    float targetAngle = startAngle + turnAngle; // 目标角度 = 当前角度 + 转向角度

    Serial.print(PREFIX_MOVECONTROL + "Turn: Start Angle: ");
    Serial.println(startAngle);
    Serial.print(PREFIX_MOVECONTROL + "Turn: Target Angle: ");
    Serial.println(targetAngle);

    while (abs(yaw - targetAngle) > 10)
    {
        if (turnAngle < 0) {
            // 左转
            M1_Motor_PID.input = 100;
            M2_Motor_PID.input = -100;
            M3_Motor_PID.input = -100;
            M4_Motor_PID.input = 100;
            Serial.println(PREFIX_MOVECONTROL + "Turn: Turning to Left");
        } 
        else 
        {
            // 右转
            M1_Motor_PID.input = -100;
            M2_Motor_PID.input = 100;
            M3_Motor_PID.input = 100;
            M4_Motor_PID.input = -100;
            Serial.println(PREFIX_MOVECONTROL + "Turn: Turning to Right");
        }
        moveCaculate(); // 位移和角度计算
    }
    stop(); // 停止
    delay(300);
}

void stop() // 停止
{
    M1_Motor_PID.input = 0;
    M2_Motor_PID.input = 0;
    M3_Motor_PID.input = 0;
    M4_Motor_PID.input = 0;
    Serial.println(PREFIX_MOVECONTROL + "Stop: Stopping");
}

void interruptHandler(void)// 定时器中断处理函数
{
    switch (currentState) // 根据当前状态机状态执行不同的操作
    {
    case STATE_PID: // 如果当前状态是STATE_PID
        PID_Cal_Computer_Out(); // 调用PID计算函数
        currentState = STATE_READ_MOTOR; // 切换状态到STATE_READ_MOTOR
        break;
    case STATE_READ_MOTOR: // 如果当前状态是STATE_READ_MOTOR
        Read_Motor_V(); // 读取电机速度
        currentState = STATE_PID; // 切换状态到STATE_PID
        break;
    }
}

void mpuInit(void)  // 初始化MPU6050传感器，设置加速度计和陀螺仪的量程、滤波器等参数
{
    Wire.begin();
    mpu.initialize();
    Serial.println(mpu.testConnection() ? (PREFIX_SYSTEM + "MPU6050 Connection Successful") : (PREFIX_ERROR + "MPU6050 Connection Failed"));

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // 设置陀螺仪量程
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // 设置加速度计量程

    calibrateSensor(); // 校准MPU6050传感器

    Serial.println(PREFIX_SYSTEM + "Setup: MPUInit Finished");
    lastTime = millis(); // 初始化 lastTime
}

void calibrateSensor() 
{
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < numSamples; i++) 
    {
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

    Serial.println(PREFIX_SYSTEM + "Sensor Calibration Complete.");
}

void ultrasonicInit(void)    // 初始化超声波雷达，设置超声波发射管的引脚
{
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    myServo.attach(11); //初始化舵机信号引脚
    myServo.write(90);  //舵机转动到90度,具体角度待确定
    delay(50);
    Serial.println(PREFIX_SYSTEM + "Setup: UltrasonicInit Finished");  
}

void moveCaculate() 
{
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

void obstacleDetectMoving(void)   // 移动中超声波距离测量函数
{
    ultrasonicDistance = getDistance(Trig, Echo); // 获取前方障碍物距离
    Serial.print(PREFIX_SENSOR + "Moving Distance Detecting: ");
    Serial.println(ultrasonicDistance);
    if(obstacleDistanceCheck(20.0)) // 当距离小于20cm时，认为有障碍物
    {
        frontObstacle = 1;
        Serial.println(PREFIX_SENSOR + "Front Obstacle Detected");
    }
    else
    {
        frontObstacle = 0;
    }
}

float getDistance(int trig, int echo)   // 获取超声波距离函数
{
    const int MEASUREMENTS = 2; // 测量次数
    float total = 0, temp2 = 0, distancecm = 0; 

    for (int i = 0; i < MEASUREMENTS; i++) {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        temp2 = (float)(pulseIn(echo, HIGH,750000));//存储回波等待时间
        distancecm = temp2 * 17 / 1000; //把回波时间换算成cm
        total += distancecm;    // 累加测量值
        delay(10);
    }

    return total / MEASUREMENTS;    // 返回平均值
}

bool obstacleDistanceCheck(float maxDistance)  // 障碍物检测
{
    if (ultrasonicDistance < maxDistance)   // 当距离小于设定值时，认为有障碍物
    {
        return true;
    }
    else
    {
        return false;
    }
}

void distanceLimitDetect(float initial_displacement_x, float initial_displacement_y)   // 超声波距离上限检测函数
{
    float current_displacement_x = x - initial_displacement_x;  // 计算当前位移
    float current_displacement_y = y - initial_displacement_y;
    float current_distance = sqrt(current_displacement_x * current_displacement_x + current_displacement_y * current_displacement_y); // 计算当前距离
    
    Serial.print(PREFIX_MOVECONTROL + "Current Distance: ");
    Serial.println(current_distance);

    if (current_distance >= 50)     // 行驶距离超过50cm时，停止
    {
        distanceLimit = 1; // 超声波距离上限标志位
        Serial.println(PREFIX_MOVECONTROL + "Attention! Distance Limit!");
    }
}

void readUltrasonicStand(void)   // 静止时超声波雷达扫描函数
{
    myServo.write(0);    // 舵机转角归零
    stop(); // 停止，等待舵机转动到指定角度
    delay(300);
    Serial.println(PREFIX_SENSOR + "Ultrasonic Stand Scanning");

    for (int i = 0; i <= 180; i++)      // 舵机转动180度
    {
        myServo.write(i);   // 舵机转动到指定角度
        ultrasonicDistance = getDistance(Trig, Echo);    // 获取超声波距离
        if (ultrasonicDistance > 50) // 当超声波测到的距离超出范围时，设定为可检测到的最远距离
        {
            ultrasonicDistance = 50;
        }
        else if (ultrasonicDistance < 5) // 当超声波测到的距离小于5时，设定为0
        {
            ultrasonicDistance = 0;
        }
        // 将数据按照一定格式发送给串口
        Serial.print('#');
        Serial.print(i);
        Serial.print('#');
        Serial.print(ultrasonicDistance);
        Serial.println('#');
        delay(70); // 70ms一次转动与采集,略大于最大测量周期66ms
    }

    myServo.write(90);  // 舵机转动到90度
    delay(100);
    Serial.println(PREFIX_SENSOR + "Ultrasonic Stand Scanning Finished"); 
}

void obstacleDetectStand(void)      // 静止时障碍物检测函数
{
    Serial.println(PREFIX_SENSOR + "Stand Detecting"); 
    stop();
    delay(300);
     
    myServo.write(180); // 舵机转动到180度，即左侧
    delay(200);
    ultrasonicDistance = getDistance(Trig, Echo);   // 获取超声波距离
    delay(10);

    if (obstacleDistanceCheck(30.0))    // 当距离小于30cm时，认为有障碍物
    {
        leftObstacle = 1;   // 左侧有障碍物
        Serial.println(PREFIX_SENSOR + "Left Obstacle Exists");
    }
    else
    {
        leftObstacle = 0;
    }

    myServo.write(0);   // 舵机转动到0度，即右侧
    delay(200);
    ultrasonicDistance = getDistance(Trig, Echo);
    delay(10);

    if (obstacleDistanceCheck(30.0))    // 当距离小于30cm时，认为有障碍物
    {
        rightObstacle = 1;  // 右侧有障碍物
        Serial.println(PREFIX_SENSOR + "Right Obstacle Exists");
    }
    else
    {
        rightObstacle = 0;  
    }
}

void PID_Cal_Computer_Out(void) // 定期计算PID输出并更新电机PWM值
{
    // 对四个电机的PID控制器进行计算
    PID_Cal(&M1_Motor_PID);
    PID_Cal(&M2_Motor_PID);
    PID_Cal(&M3_Motor_PID);
    PID_Cal(&M4_Motor_PID);
    
    // 根据PID计算结果，设置四个电机的PWM输出，以控制电机转速和方向
    Motor_PWM_Set(M1_Motor_PID.output, M2_Motor_PID.output, M3_Motor_PID.output, M4_Motor_PID.output);
    
    timecnt++;// 更新时间计数器，用于控制测试流程和PID计算频率
    
    // 每20次循环增加一秒计时，并重置时间计数器
    if(timecnt == 20)
    {
        time_seconds++;
        timecnt = 0;
    }
}

void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM)  // 设置电机PWM值以控制速度和方向
{
    // 设置电机1的正反转及PWM值
    if(M1_PWM > 0)
    {
        digitalWrite(TB6612_M1_IN1, HIGH);      
        digitalWrite(TB6612_M1_IN2, LOW);
        analogWrite(Motor_M1_PWM, M1_PWM);  
    }
    else
    {
        digitalWrite(TB6612_M1_IN1, LOW);
        digitalWrite(TB6612_M1_IN2, HIGH);
        analogWrite(Motor_M1_PWM, -1 * M1_PWM);
    }

    // 设置电机2的正反转及PWM值
    if(M2_PWM > 0)
    {
        digitalWrite(TB6612_M2_IN1, LOW);
        digitalWrite(TB6612_M2_IN2, HIGH);
        analogWrite(Motor_M2_PWM, M2_PWM);
    }
    else
    {
        digitalWrite(TB6612_M2_IN1, HIGH);
        digitalWrite(TB6612_M2_IN2, LOW);
        analogWrite(Motor_M2_PWM, -1 * M2_PWM);
    }

    // 设置电机3的正反转及PWM值
    if(M3_PWM > 0) {
        digitalWrite(TB6612_M3_IN1, LOW);
        digitalWrite(TB6612_M3_IN2, HIGH);
        analogWrite(Motor_M3_PWM, M3_PWM);
    } else {
        digitalWrite(TB6612_M3_IN1, HIGH);
        digitalWrite(TB6612_M3_IN2, LOW);
        analogWrite(Motor_M3_PWM, -1 * M3_PWM);
    }

    // 设置电机4的正反转及PWM值
    if(M4_PWM > 0) {
        digitalWrite(TB6612_M4_IN1, HIGH);
        digitalWrite(TB6612_M4_IN2, LOW);
        analogWrite(Motor_M4_PWM, M4_PWM);
    } else {
        digitalWrite(TB6612_M4_IN1, LOW);
        digitalWrite(TB6612_M4_IN2, HIGH);
        analogWrite(Motor_M4_PWM, -1 * M4_PWM);
    }
}

void Motor_Init(void)   // 初始化电机控制引脚为输入或输出模式，并将所有控制引脚设为低电平
{
    pinMode(M1_ENCODER_A, INPUT); //M1编码器A引脚，设置为输入模式
    pinMode(M1_ENCODER_B, INPUT); //M1编码器B引脚，设置为输入模式
    pinMode(M2_ENCODER_A, INPUT); //M2编码器A引脚，设置为输入模式
    pinMode(M2_ENCODER_B, INPUT); //M2编码器B引脚，设置为输入模式
    pinMode(M3_ENCODER_A, INPUT); //M3编码器A引脚，设置为输入模式
    pinMode(M3_ENCODER_B, INPUT); //M3编码器B引脚，设置为输入模式
    pinMode(M4_ENCODER_A, INPUT); //M4编码器A引脚，设置为输入模式
    pinMode(M4_ENCODER_B, INPUT); //M4编码器B引脚，设置为输入模式

    pinMode(TB6612_M1_IN1, OUTPUT); //设置两个驱动引脚为输出模式
    pinMode(TB6612_M1_IN2, OUTPUT); 
    pinMode(TB6612_M2_IN1, OUTPUT); //设置两个驱动引脚为输出模式
    pinMode(TB6612_M2_IN2, OUTPUT); 
    pinMode(TB6612_M3_IN1, OUTPUT); //设置两个驱动引脚为输出模式
    pinMode(TB6612_M3_IN2, OUTPUT); 
    pinMode(TB6612_M4_IN1, OUTPUT); //设置两个驱动引脚为输出模式
    pinMode(TB6612_M4_IN2, OUTPUT); 

    pinMode(Motor_M1_PWM, OUTPUT);  //设置使能引脚为输出模式
    pinMode(Motor_M2_PWM, OUTPUT);  //设置使能引脚为输出模式
    pinMode(Motor_M3_PWM, OUTPUT);  //设置使能引脚为输出模式
    pinMode(Motor_M4_PWM, OUTPUT);  //设置使能引脚为输出模式

    //驱动芯片控制引脚全部拉低，确保电机停止
    digitalWrite(TB6612_M1_IN1, LOW);
    digitalWrite(TB6612_M1_IN2, LOW);
    digitalWrite(Motor_M1_PWM, LOW);
    digitalWrite(TB6612_M2_IN1, LOW);
    digitalWrite(TB6612_M2_IN2, LOW);
    digitalWrite(Motor_M2_PWM, LOW);
    digitalWrite(TB6612_M3_IN1, LOW);
    digitalWrite(TB6612_M3_IN2, LOW);
    digitalWrite(Motor_M3_PWM, LOW);
    digitalWrite(TB6612_M4_IN1, LOW);
    digitalWrite(TB6612_M4_IN2, LOW);
    digitalWrite(Motor_M4_PWM, LOW);
}

void PID_Init(void) // 初始化PID控制器的参数，包括比例、积分、微分系数和输出限制
{
    M1_Motor_PID.k_p = 0.08;
    M1_Motor_PID.k_i = 0.091;
    M1_Motor_PID.k_d = 0.1;
    M1_Motor_PID.out_max = 250;
    M1_Motor_PID.out_min = -250;
    M1_Motor_PID.input = 0;
    M1_Motor_PID.err_x_max = 1000;

    M2_Motor_PID.k_p = 0.08;
    M2_Motor_PID.k_i = 0.091;
    M2_Motor_PID.k_d = 0.1;
    M2_Motor_PID.out_max = 250;
    M2_Motor_PID.out_min = -250;
    M2_Motor_PID.input = 0;
    M2_Motor_PID.err_x_max = 1000;

    M3_Motor_PID.k_p = 0.08;
    M3_Motor_PID.k_i = 0.091;
    M3_Motor_PID.k_d = 0.1;
    M3_Motor_PID.out_max = 250;
    M3_Motor_PID.out_min = -250;
    M3_Motor_PID.input = 0;
    M3_Motor_PID.err_x_max = 1000;

    M4_Motor_PID.k_p = 0.08;
    M4_Motor_PID.k_i = 0.091;
    M4_Motor_PID.k_d = 0.1;
    M4_Motor_PID.out_max = 250;
    M4_Motor_PID.out_min = -250;
    M4_Motor_PID.input = 0;
    M4_Motor_PID.err_x_max = 1000;
}

void PID_Cal(PID *pid)  // PID增量式计算函数，计算PID控制器的输出
{
    float p, i, d;

    pid->err_2 = pid->err_1;
    pid->err_1 = pid->input - pid->feedback;

    p = pid->k_p * pid->err_1;
    i = pid->k_i * pid->err_x;
    d = pid->k_d * (pid->err_1 - pid->err_2);
    pid->err_x += pid->err_1;
    pid->output = p + i + d;
    // 根据PID公式和当前误差计算输出值，然后根据输出范围限制输出值
    if(pid->output > pid->out_max)      pid->output = pid->out_max;
    if(pid->output < pid->out_min)      pid->output = pid->out_min;
    if(pid->err_x > pid->err_x_max)     pid->err_x = pid->err_x_max;
}

void Read_motor_M1(void) // 读取并计算电机的实际速度，用于PID控制器的反馈
{
    motor_M1_dir = digitalRead(M1_ENCODER_B);
    if(motor_M1_dir == 1)
    {
        motor_M1++;
    }
    else
    {
        motor_M1--;
    }
}
void Read_motor_M2(void)
{
    motor_M2_dir = digitalRead(M2_ENCODER_B);
    if(motor_M2_dir == 1)
    {
        motor_M2++;
    }
    else
    {
        motor_M2--;
    }
}
void Read_motor_M3(void)
{
    motor_M3_dir = digitalRead(M3_ENCODER_B);
    if(motor_M3_dir == 1)
    {
        motor_M3++;
    }
    else
    {
        motor_M3--;
    }
}
void Read_motor_M4(void)
{
    motor_M4_dir = digitalRead(M4_ENCODER_B);
    if(motor_M4_dir == 1)
    {
        motor_M4++;
    }
    else
    {
        motor_M4--;
    }
}

void Read_Motor_V(void) // 读取电机速度，计算电机速度
{
    static unsigned long lastReadTime = 0;
    static float M1_Speed = 0, M2_Speed = 0, M3_Speed = 0, M4_Speed = 0;    // 电机速度，单位：mm/s
    const float speed_k = 0.3;  // 速度滤波系数
    unsigned long currentTime = millis();   // 获取当前时间
    
    // 第一阶段：第一次中断调用，启动编码器中断，启动读取电机速度
    if (currentTime - lastReadTime >= 50 && !needToReadMotors) {  
        lastReadTime = currentTime;
        
        // 重置计数器
        motor_M1 = 0;
        motor_M2 = 0;
        motor_M3 = 0;
        motor_M4 = 0;
        
        // 启动编码器中断
        attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), Read_motor_M1, FALLING);
        attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), Read_motor_M2, FALLING);
        attachInterrupt(digitalPinToInterrupt(M3_ENCODER_A), Read_motor_M3, FALLING);
        attachInterrupt(digitalPinToInterrupt(M4_ENCODER_A), Read_motor_M4, FALLING);
        
        needToReadMotors = true;
    } 
    // 第二阶段：第二次中断调用，读取结果并计算速度，更新 PID 反馈值
    else if (needToReadMotors) {    
        needToReadMotors = false;
        
        // 关闭编码器中断
        detachInterrupt(digitalPinToInterrupt(M1_ENCODER_A));
        detachInterrupt(digitalPinToInterrupt(M2_ENCODER_A));
        detachInterrupt(digitalPinToInterrupt(M3_ENCODER_A));
        detachInterrupt(digitalPinToInterrupt(M4_ENCODER_A));
        
        // 计算速度 (单位: mm/s)
        V_M1 = ((motor_M1 / 330.0) * 65.0 * PI) * 10.0;   // 电机速度，单位：mm/s
        V_M2 = ((motor_M2 / 330.0) * 65.0 * PI) * 10.0;
        V_M3 = ((motor_M3 / 330.0) * 65.0 * PI) * 10.0;
        V_M4 = ((motor_M4 / 330.0) * 65.0 * PI) * 10.0;
        
        // 更新 PID 反馈值
        M1_Speed = M1_Motor_PID.feedback;
        M2_Speed = M2_Motor_PID.feedback;
        M3_Speed = M3_Motor_PID.feedback;
        M4_Speed = M4_Motor_PID.feedback;
        
        M1_Motor_PID.feedback = (1 - speed_k) * V_M1 + speed_k * M1_Speed;  //滤波后的速度
        M2_Motor_PID.feedback = -1 * (1 - speed_k) * V_M2 + speed_k * M2_Speed;
        M3_Motor_PID.feedback = -1 * (1 - speed_k) * V_M3 + speed_k * M3_Speed;
        M4_Motor_PID.feedback = (1 - speed_k) * V_M4 + speed_k * M4_Speed;
        
        Serial.println(PREFIX_MOVECONTROL + "Read Motor V Finished");
    }
}

void readMagnetometerYaw() // 读取磁力计Yaw角数据
{
    String dataFromMagnetometer = Serial2.readStringUntil('\n');
    Serial.print(PREFIX_SENSOR + "Received from new hardware: ");
    Serial.println(dataFromMagnetometer);
    // 示例数据格式: Magx:994,Magy:-7256,Magz:-5282,Yaw:-82.1
    int yawIndex = dataFromMagnetometer.indexOf("Yaw:");
    if (yawIndex != -1) 
    {
        String yawString = dataFromMagnetometer.substring(yawIndex + 4);
        yawString.trim();
        magYaw = yawString.toFloat();

        if (!initialYawSet) {
            initialYawSet = true;
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
    float temp = Yaw - initialMagYaw;

    // 处理角度跳变，确保在 -180 到 180 之间
    if (temp > 180) 
    {
        temp -= 360;
    } 
    else if (temp < -180) 
    {
        temp += 360;
    }
    return temp;
}