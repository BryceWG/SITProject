#include <MsTimer2.h> // 引入MsTimer2库，用于设置定时中断
#include <Arduino.h> // 引入Arduino标准库，用于Arduino的基本操作
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// 定义电机编码器的引脚，用于测量转速
#define M1_ENCODER_A 2 // 右前电机编码器A相引脚，用于下降沿捕获
#define M1_ENCODER_B 40 // 右前电机编码器B相引脚
#define M2_ENCODER_A 3 // 左前电机编码器A相引脚，用于下降沿捕获
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
#define Trig 12//引脚Tring 连接 IO D2  
#define Echo 13 //引脚Echo 连接 IO D3

#define BUTTON_PIN 10

Servo myServo; //实例化舵机对象
Adafruit_MPU6050 mpu;   // 实例化MPU6050传感器对象

// 定义PID结构体
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

PID M1_Motor_PID, M2_Motor_PID, M3_Motor_PID, M4_Motor_PID; // PID结构体实例
// 定义中断变量，用于计数脉冲
volatile float motor_M1 = 0; 
volatile float motor_M2 = 0; 
volatile float motor_M3 = 0;
volatile float motor_M4 = 0;

//定义定时器变量
int timecnt = 0; // 50ms计时器
int time_seconds = 0; // 秒计时器

// 定义临时速度变量
float V_M1 = 0; // 临时存储右前电机速度变量
float V_M2 = 0; // 临时存储左前电机速度变量
float V_M3 = 0; // 临时存储左后电机速度变量
float V_M4 = 0; // 临时存储右后电机速度变量

//定义方向变量
int motor_M1_dir = 0; // 反馈的右前电机转动方向
int motor_M2_dir = 0; // 反馈的左前电机转动方向
int motor_M3_dir = 0; // 反馈的左后电机转动方向
int motor_M4_dir = 0; // 反馈的右后电机转动方向
float angle = 0.0; // 小车当前旋转角度

//定义障碍检测变量
int frontObstacle = 0;  // 前方是否存在障碍物
int leftObstacle = 0;   // 左方是否存在障碍物
int rightObstacle = 0;  // 右方是否存在障碍物

int distanceLimit = 0;  // 行驶距离是否到达上限
bool first_call = true;

// 定义位移变量
float displacement_x = 0; // 小车x轴位移
float displacement_y = 0; // 小车y轴位移
float velocity_x = 0; // 小车x轴速度
float velocity_y = 0; // 小车y轴速度

float ultrasonicDistance = 60; // 超声波距离变量

// 时间差变量
unsigned long prevTime = 0;
unsigned long currentTime = 0;
float dt;

// 定义MPU6050变量
float accelXoffset, accelYoffset, accelZoffset;
float gyroXoffset, gyroYoffset, gyroZoffset;

// 卡尔曼滤波器变量
float Q_angle = 0.001; // 角度数据的不确定性
float Q_bias = 0.003;  // 陀螺仪偏差的不确定性
float R_measure = 0.03; // 测量数据的不确定性
float bias = 0;  // 估计的陀螺仪偏差
float P[2][2] = {
  { 0, 0 },
  { 0, 0 }
}; // 误差协方差矩阵

// 函数声明
void Motor_Init(void);// 初始化电机控制引脚
void PID_Init(void);// 初始化PID控制器参数
void ultrasonicInit(void); // 初始化超声波传感器
void mpuInit(void); // 初始化MPU6050传感器

void PID_Cal(PID *pid);// 计算PID控制器的输出
void Read_Motor_V(void);// 读取电机速度（通过编码器脉冲）
void PID_Cal_Computer_Out(void);// 定期计算PID输出并更新电机PWM值
void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM);// 设置电机PWM值以控制速度和方向

void calibrateMPU6050(void);// 校准MPU6050传感器

void moveCaculate(void); // 读取MPU6050传感器数据，并进行角度估算
void serialPrint(void); // 串口打印函数

void moveForward(void); // 向前移动
void turn(float turnAngle); // 角度转向
void stop(void); // 停止

void readUltrasonicStand(void); // 静止扫描读取超声波传感器数据
bool obstacleDetect(float maxDistance); // 障碍物检测
void distanceDetectMoving(void); // 行驶时距离检测
void distanceDetectStand(void); // 待机时距离检测
void distanceLimitDetect(void); // 距离上限检测
float getDistance(int trig, int echo); // 获取超声波距离

// 以下为函数实现
void setup()
{
    PID_Init();
    Motor_Init();

    Serial.begin(115200);  //打开串口
    while (!Serial) delay(10); // 等待串口监视器打开
    mpuInit();
    ultrasonicInit();

    pinMode(BUTTON_PIN, INPUT); // 设置按钮引脚为输入模式

    MsTimer2::set(50, PID_Cal_Computer_Out); // 设置定时器50ms中断调用组合函数
    MsTimer2::start(); 
    Serial.println("setup Finish");
}

void loop()
{
    static bool isRunning = false; // 用于跟踪小车是否在运行

    // 检查按钮状态
    if (digitalRead(BUTTON_PIN) == HIGH) {
        delay(50); // 去抖动
        if (digitalRead(BUTTON_PIN) == HIGH) {
            isRunning = !isRunning; // 切换运行状态
            while (digitalRead(BUTTON_PIN) == HIGH); // 等待按钮释放
        }
    }

    if (isRunning)
    {
        //障碍物不存在时，行驶
        if (frontObstacle == 0 && distanceLimit == 0)   // 前方没有障碍物，行驶距离没有到达上限
        {
            moveForward();
            Serial.println("move forward\n");
        }
        else if (distanceLimit == 1)   // 行驶距离到达上限
        {
            readUltrasonicStand();
            distanceLimit = 0;
            first_call = true;
            Serial.println("distance limit\n");
        }

        // 障碍物存在时，停止并转向
        if (frontObstacle == 1)
        {
            Serial.println("obstacle detect\n");
            distanceDetectStand();
            if (leftObstacle == 0)
            {
                Serial.println("turn left\n");
                turn(90);
            }
            else if (rightObstacle == 0 && leftObstacle == 1)
            {
                Serial.println("turn right\n");
                turn(-90);
            }
            else if (leftObstacle == 1 && rightObstacle == 1)
            {
                Serial.println("turn around\n");
                turn(180);
            }
            readUltrasonicStand();
            frontObstacle = 0;
            first_call = true;
        }
    }
    else
    {
        stop(); // 停止
    }
    Read_Motor_V(); // 读取电机速度
    moveCaculate(); // 读取MPU6050传感器数据，并进行角度估算

    serialPrint(); // 串口打印函数
    Serial.println("loop Finish");
}

void moveForward()  // 向前移动
{
    static float initial_angle = 0; // 记录初始角度
    static float initial_displacement_x = 0, initial_displacement_y = 0;

    if (first_call)     // 第一次调用时初始化变量
    {
        initial_angle = angle; // 记录初始角度
        initial_displacement_x = displacement_x; // 记录初始位移
        initial_displacement_y = displacement_y;
        first_call = false; // 标记为非第一次调用
    }

    float angle_offset = angle - initial_angle; // 计算角度偏差
    float correction = -0.1 * angle_offset; // 根据角度偏差计算修正值

    // 根据角度偏差调整左右电机的速度
    if (angle_offset < 0) {
        // 左偏，左电机速度略大于右电机
        M1_Motor_PID.input = 100 + correction; // 右前电机
        M2_Motor_PID.input = 100 - correction; // 左前电机
        M3_Motor_PID.input = 100 + correction; // 左后电机
        M4_Motor_PID.input = 100 - correction; // 右后电机
    } else {
        // 右偏，右电机速度略大于左电机
        M1_Motor_PID.input = 100 - correction; // 右前电机
        M2_Motor_PID.input = 100 + correction; // 左前电机
        M3_Motor_PID.input = 100 - correction; // 左后电机
        M4_Motor_PID.input = 100 + correction; // 右后电机
    }
    distanceDetectMoving();
    distanceLimitDetect(initial_displacement_x, initial_displacement_y);
    Serial.println("moving forward\n");
}

void turn(float turnAngle)  // 角度转向
{
    float startAngle = angle;   // 记录当前角度
    float targetAngle = startAngle + turnAngle; // 目标角度 = 当前角度 + 转向角度
    Serial.print("start angle:");
    Serial.println(startAngle);
    Serial.print("target angle:");
    Serial.println(targetAngle);

    while (abs(angle - targetAngle) > 1)     // 直到转向角度小于1度为止
    {
        moveCaculate();
        Read_Motor_V();
        if (turnAngle < 0) {
            // 左转
            M1_Motor_PID.input = 100; // 根据实际情况调整速度
            M2_Motor_PID.input = -100;
            M3_Motor_PID.input = -100;
            M4_Motor_PID.input = 100;
            Serial.println("turning to left");
        } 
        else 
        {
            // 右转
            M1_Motor_PID.input = -100; // 根据实际情况调整速度
            M2_Motor_PID.input = 100;
            M3_Motor_PID.input = 100;
            M4_Motor_PID.input = -100;
            Serial.println("turning to right");
        }
        delay(25);
    }
    stop();
    //
    float nowtime = millis();
    while (millis() - nowtime < 300) //等待300ms
    {
        moveCaculate();
        Read_Motor_V();
        delay(25);
    }
}

void stop()      // 停止
{
    M1_Motor_PID.input = 0;
    M2_Motor_PID.input = 0;
    M3_Motor_PID.input = 0;
    M4_Motor_PID.input = 0;
    Serial.println("stopping");
}

void PID_Cal_Computer_Out(void)  // 定期计算PID输出并更新电机PWM值，由定时器调用
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

void moveCaculate(void)  // 读取MPU6050传感器数据，并进行角度估算
{
    // 获取MPU6050的加速度、陀螺仪和温度数据
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    //
    a.acceleration.x -= accelXoffset;   
    a.acceleration.y -= accelYoffset;
    a.acceleration.z -= accelZoffset;

    g.gyro.x -= gyroXoffset;
    g.gyro.y -= gyroYoffset;
    g.gyro.z -= gyroZoffset;    // 减去偏移量，以获得更准确的读数

    currentTime = millis();
    dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // 从陀螺仪和加速度数据中计算角速度和初始角度
    float gyro_rate = g.gyro.z; // 从陀螺仪数据中获取角速度
    float accel_angle = atan2(a.acceleration.x, a.acceleration.y) * 180 / PI;   // 从加速度数据中获取角度

    // 计算姿态角    
    float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

    // 计算重力在 X 和 Y 轴上的分量
    float gravity_x = sin(pitch) * 9.7915;
    float gravity_y = -sin(roll) * cos(pitch) * 9.7915;

    // 去除重力影响，得到线性加速度
    float linear_accel_x = a.acceleration.x - gravity_x;
    float linear_accel_y = a.acceleration.y - gravity_y;

    // 积分计算速度
    velocity_x += linear_accel_x * dt;
    velocity_y += linear_accel_y * dt;

    // 积分计算位移
    displacement_x += velocity_x * dt;
    displacement_y += velocity_y * dt;
}


float calculateDeltaTime(unsigned long *last_time)  // 计算时间差函数
{
    unsigned long now = millis();
    float dt = (now - *last_time) / 1000.0;
    *last_time = now;
    return dt;
}

float getDistance(int trig, int echo)    // 获取超声波距离函数，通过发射和接收两个引脚，测量距离并返回距离值
{
  float temp2 = 0, distancecm = 0;

  digitalWrite(trig, LOW);   //给Trig发送一个低电平
  delayMicroseconds(2);      //等待 2微妙
  digitalWrite(trig, HIGH);  //给Trig发送一个高电平
  delayMicroseconds(10);     //等待 10微妙
  digitalWrite(trig, LOW);   //给Trig发送一个低电平

  temp2 = (float)(pulseIn(echo, HIGH,750000));//存储回波等待时间
  distancecm = temp2 * 17 / 1000; //把回波时间换算成cm
  return distancecm;
}

void readUltrasonicStand(void)   //静止时超声波雷达扫描函数
{
    myServo.write(0);    //舵机转角归零
    stop();
    // 等待300ms，使小车停止运动
    float nowtime = millis();
    while (millis() - nowtime < 300) //等待300ms
    {
        moveCaculate();
        Read_Motor_V();
        delay(25);
    }
        
    Serial.println("Ultrasonic Stand Scanning\n");  
    for (int i = 0; i <= 180; i++)  // 扫描角度范围
    {
        myServo.write(i);    // 舵机转动到当前角度
        ultrasonicDistance = getDistance(Trig, Echo);    // 获取超声波距离
        if (ultrasonicDistance > 50) //当超声波测到的距离超出范围时，设定为可检测到的最远距离
        {
            ultrasonicDistance = 50;
        }
        else if (ultrasonicDistance < 5) //当超声波测到的距离小于5时，设定为0
        {
            ultrasonicDistance = 0;
        }
        //将数据按照一定格式发送给串口
        Serial.print('#');
        Serial.print(i);
        Serial.print('#');
        Serial.print(ultrasonicDistance);
        Serial.println('#');
        delay(70); //70ms一次转动与采集,略大于最大测量周期66ms
    }
    serial.println("Ultrasonic Stand Scanning Finished\n"); // 扫描完成标志
}

void distanceDetectMoving(void)   //移动中超声波距离测量函数
{
    myServo.write(90);  // 舵机转动到90度,具体角度待确定
    ultrasonicDistance = getDistance(Trig, Echo); // 获取前方障碍物距离

    if(obstacleDetect(20.0))
    {
        frontObstacle = 1;
    }
    else
    {
        frontObstacle = 0;
    }
    Serial.print("Move detecting\n");
}

void distanceDetectStand(void)    //静止时超声波距离测量函数
{
    stop();
    float nowtime = millis();
    while (millis() - nowtime < 300) //等待300ms
    {
        moveCaculate();
        Read_Motor_V();
        delay(25);
    }

    Serial.print("Stand detecting\n");
    myServo.write(180);
    delay(100);
    ultrasonicDistance = getDistance(Trig, Echo);
    delay(10);
    if (obstacleDetect(30.0))
    {
        leftObstacle = 1;
        Serial.println("left obstacle exsist\n");
    }
    else
    {
        leftObstacle = 0;
    }
    delay(50);
    myServo.write(0); 
    delay(100);
    ultrasonicDistance = getDistance(Trig, Echo);
    delay(10);
    if (obstacleDetect(30.0))
    {
        rightObstacle = 1;
        Serial.println("right obstacle exsist\n");
    }
    else
    {
        rightObstacle = 0;  
    }
}

void distanceLimitDetect(float initial_displacement_x, float initial_displacement_y)   //超声波距离上限检测函数
{
//行驶距离超过50cm时，停止
}

bool obstacleDetect(float maxDistance)  // 障碍物检测
{
    if (ultrasonicDistance < maxDistance)   // 障碍物检测，通过超声波传感器测距
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM) // 设置电机PWM值以控制速度和方向
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

    // 设置M2\M3\M4的正反转及PWM值
}

void Motor_Init(void)  // 初始化电机控制引脚为输入或输出模式，并将所有控制引脚设为低电平
{
}

void PID_Init(void)  // 初始化PID控制器的参数，包括比例、积分、微分系数和输出限制
{
    M1_Motor_PID.k_p = 0.08;
    M1_Motor_PID.k_i = 0.091;
    M1_Motor_PID.k_d = 0.1;
    M1_Motor_PID.out_max = 250;
    M1_Motor_PID.out_min = -250;
    M1_Motor_PID.input = 0;
    M1_Motor_PID.err_x_max = 1000;
    //SET M2\M3\M4 PID
}

void mpuInit(void)  // 初始化MPU6050传感器，设置加速度计和陀螺仪的量程、滤波器等参数
{
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");} 

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);    
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // 设置滤波器带宽

    calibrateMPU6050(); // 校准MPU6050传感器

    Serial.println("mpuInit Finish\n");
}

void ultrasonicInit(void)    // 初始化超声波雷达，设置超声波发射管的引脚
{
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    myServo.attach(11); //初始化舵机信号引脚
    myServo.write(90);
    delay(50);
    Serial.println("ultrasonicInit Finish\n");
}

// 校准MPU6050传感器，计算加速度计和陀螺仪的偏移量
void calibrateMPU6050(void)
{
}

void PID_Cal(PID *pid)  // PID增量式计算函数，计算PID控制器的输出
{ 
}

// 读取并计算电机的实际速度，用于PID控制器的反馈
void Read_motor_M1(void)
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
}
void Read_motor_M3(void)
{
}
void Read_motor_M4(void)
{
}

void Read_Motor_V(void)  //下降沿捕获中断 并计算速度
{
    // 初始化四个电机的速度为0，并定义速度比例系数
    static float M1_Speed = 0, M2_Speed = 0, M3_Speed = 0, M4_Speed = 0;
    float speed_k = 0.3;
    
    // 获取四个电机的当前PID反馈值
    M1_Speed = M1_Motor_PID.feedback;
    M2_Speed = M2_Motor_PID.feedback;
    M3_Speed = M3_Motor_PID.feedback;
    M4_Speed = M4_Motor_PID.feedback;
    
    // 初始化当前时间戳，并为中断设置延迟时间
    unsigned long nowtime2 = 0;
    motor_M1 = 0;
    motor_M2 = 0;
    motor_M3 = 0;
    motor_M4 = 0;
    nowtime2 = millis() + 50; //读50毫秒
    
    // 配置四个电机的编码器中断，用于捕捉脉冲
    attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), Read_motor_M1, FALLING);
    attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), Read_motor_M2, FALLING);
    attachInterrupt(digitalPinToInterrupt(M3_ENCODER_A), Read_motor_M3, FALLING);
    attachInterrupt(digitalPinToInterrupt(M4_ENCODER_A), Read_motor_M4, FALLING);
    
    // 等待设定的延迟时间，以便中断可以采集到足够的脉冲数据
    while(millis() < nowtime2); //达到50毫秒关闭中断
    
    // 关闭四个电机的编码器中断，避免过多的中断影响性能
    detachInterrupt(digitalPinToInterrupt(M1_ENCODER_A));//右前轮脉冲关中断计数
    detachInterrupt(digitalPinToInterrupt(M2_ENCODER_A));//左前轮脉冲关中断计数
    detachInterrupt(digitalPinToInterrupt(M3_ENCODER_A));//左后轮脉冲关中断计数
    detachInterrupt(digitalPinToInterrupt(M4_ENCODER_A));//右后轮脉冲关中断计数
    
    // 根据编码器脉冲数和轮子参数计算电机的实际速度
    V_M1 = ((motor_M1 / 330) * 65 * PI) * 20; //单位mm/s 
    V_M2 = ((motor_M2 / 330) * 65 * PI) * 20; 
    V_M3 = ((motor_M3 / 330) * 65 * PI) * 20;
    V_M4 = ((motor_M4 / 330) * 65 * PI) * 20;
    
    // 根据速度比例系数和当前速度，更新电机PID的反馈值
    M1_Motor_PID.feedback = (1 - speed_k) * V_M1 + speed_k * M1_Speed;
    M2_Motor_PID.feedback = -1 * (1 - speed_k) * V_M2 + speed_k * M2_Speed;
    M3_Motor_PID.feedback = -1 * (1 - speed_k) * V_M3 + speed_k * M3_Speed;
    M4_Motor_PID.feedback = (1 - speed_k) * V_M4 + speed_k * M4_Speed;
    Serial.print("read motor v finish\n");
}

void serialPrint(void)  // 串口打印函数
{
    Serial.print(M1_Motor_PID.input);
    Serial.print(",");
    //M2\M3\M4的速度
    // 打印位移信息
}