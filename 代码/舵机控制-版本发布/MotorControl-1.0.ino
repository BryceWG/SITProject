#include <MsTimer2.h> // 引入MsTimer2库，用于设置定时中断
#include <Arduino.h> // 引入Arduino标准库，用于Arduino的基本操作
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// 定义电机编码器的引脚，用于测量转速
#define M1_ENCODER_A 2 // 右前电机编码器A相引脚，用于下降沿捕获
#define M1_ENCODER_B 40 // 右前电机编码器B相引脚
#define M2_ENCODER_A 3 // 左前电机编码器A相引脚，用于下降沿捕获
#define M2_ENCODER_B 41 // 左前电机编码器B相引脚
#define M3_ENCODER_A 20 // 左后电机编码器A相引脚，用于下降沿捕获
#define M3_ENCODER_B 42 // 左后电机编码器B相引脚
#define M4_ENCODER_A 21 // 右后电机编码器A相引脚，用于下降沿捕获
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

//定义定时器变量
int timecnt = 0; // 50ms计时器
int time_seconds = 0; // 秒计时器

// 定义临时速度变量
float V_M1 = 0; // 临时存储右前电机速度变量
float V_M2 = 0; // 临时存储左前电机速度变量
float V_M3 = 0; // 临时存储左后电机速度变量
float V_M4 = 0; // 临时存储右后电机速度变量

// 卡尔曼滤波器变量
float Q_angle = 0.001; // 角度数据的不确定性
float Q_bias = 0.003;  // 陀螺仪偏差的不确定性
float R_measure = 0.03; // 测量数据的不确定性
float angle = 0; // 估计的角度
float bias = 0;  // 估计的陀螺仪偏差
float P[2][2] = {
  { 0, 0 },
  { 0, 0 }
}; // 误差协方差矩阵

//定义方向变量
int motor_M1_dir = 0; // 反馈的右前电机转动方向
int motor_M2_dir = 0; // 反馈的左前电机转动方向
int motor_M3_dir = 0; // 反馈的左后电机转动方向
int motor_M4_dir = 0; // 反馈的右后电机转动方向
int angle = 0; // 小车当前旋转角度

PID M1_Motor_PID, M2_Motor_PID, M3_Motor_PID, M4_Motor_PID; // PID结构体实例

// 函数声明
void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM);// 设置电机PWM值以控制速度和方向
void Motor_Init(void);// 初始化电机控制引脚
void PID_Init(void);// 初始化PID控制器参数
void PID_Cal(PID *pid);// 计算PID控制器的输出
void Read_Motor_V(void);// 读取电机速度（通过编码器脉冲）
void PID_Cal_Computer_Out(void);// 定期计算PID输出并更新电机PWM值
void CombineTimer(void);    // 定时器中断调用函数，用于组合PID计算和电机PWM输出
void ReadMpu(void); // 读取MPU6050传感器数据，并进行角度估算
void moveForward(); // 向前移动
void turn(float turnAngle); // 角度转向

// Arduino的setup函数，用于初始化
void setup()
{
    Motor_Init();   //电机端口初始化
    PID_Init();    //PID参数初始化

    MsTimer2::set(50, CombineTimer); // 设置定时器50ms中断调用组合函数
    MsTimer2::start();
    Serial.begin(9600);  //打开串口

    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");} 

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);    
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // 设置滤波器带宽
}

//主循环
void loop()
{
    CombineTimer(); // 定时器中断调用函数, 实现PID输出和电机PWM值的更新
    // 串口输出电机的目标速度和实际反馈速度，用于调试
    Serial.print(M1_Motor_PID.input);
    Serial.print(",");
    Serial.print(M1_Motor_PID.feedback);
    Serial.print(",");
    Serial.print(M2_Motor_PID.input);
    Serial.print(",");
    Serial.print(M2_Motor_PID.feedback);
    Serial.print(",");
    Serial.print(M3_Motor_PID.input);
    Serial.print(",");
    Serial.print(M3_Motor_PID.feedback);
    Serial.print(",");
    Serial.print(M4_Motor_PID.input);
    Serial.print(",");
    Serial.println(M4_Motor_PID.feedback);

    delay(50); // 控制循环频率
}

void moveForward() 
{
    static float initial_angle = 0; // 记录初始角度
    static bool first_call = true; // 标记是否是第一次调用  注意重置！！！！！

    if (first_call) {
        initial_angle = angle; // 记录初始角度
        first_call = false; // 标记为非第一次调用
    }

    float angle_offset = angle - initial_angle; // 计算角度偏差
    float correction = -0.1 * angle_offset; // 根据角度偏差计算修正值，调整比例系数以适应实际情况

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

    // 可选：打印调试信息
    Serial.println("Maintaining Straight Line");
}


void turn(float turnAngle) 
{
    float startAngle = angle;
    float targetAngle = startAngle + turnAngle;

    while (abs(angle - targetAngle) > 1) {
        if (turnAngle < 0) {
            // 左转
            M1_Motor_PID.input = -50; // 根据实际情况调整速度
            M2_Motor_PID.input = 50;
            M3_Motor_PID.input = -50;
            M4_Motor_PID.input = 50;
        } else {
            // 右转
            M1_Motor_PID.input = 50; // 根据实际情况调整速度
            M2_Motor_PID.input = -50;
            M3_Motor_PID.input = 50;
            M4_Motor_PID.input = -50;
        }

        // 可选：打印调试信息
        Serial.print("Turning: ");
        Serial.println(angle);
    }
}

// 定期计算PID输出并更新电机PWM值，由定时器调用
void PID_Cal_Computer_Out(void)
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

// 读取MPU6050传感器数据，并进行角度估算
void ReadMpu(void)
{
    // 获取MPU6050的加速度、陀螺仪和温度数据
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 计算时间差，用于卡尔曼滤波中的时间更新
    // 计算偏航角（yaw）
    static unsigned long last_time = 0;
    unsigned long nowtime = millis();   //获取当前时间戳
    float dt = (nowtime - last_time) / 1000.0;  //计算时间差
    last_time = nowtime;    //更新时间戳

    // 从陀螺仪和加速度数据中计算角速度和初始角度
    // 使用卡尔曼滤波器估计角度
    float gyro_rate = g.gyro.z; // 从陀螺仪数据中获取角速度
    float accel_angle = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;   // 从加速度数据中获取角度

    // 卡尔曼滤波的时间更新步骤，预测当前角度
    // 预测步骤
    float rate = gyro_rate - bias;  // 陀螺仪角速度减去偏置
    angle += rate * dt;

    // 卡尔曼滤波的误差协方差矩阵更新
    // 更新误差协方差矩阵
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // 计算测量噪声的协方差和卡尔曼增益
    // 测量更新步骤
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 计算测量误差，更新角度和偏置估计
    float y = accel_angle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // 更新误差协方差矩阵
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    // 将估计的角度用于调整四个电机的PID输入
    // 使用估计的角度调整PID控制器的输入
    M1_Motor_PID.input += angle * 0.1;
    M2_Motor_PID.input += angle * 0.1;
    M3_Motor_PID.input += angle * 0.1;
    M4_Motor_PID.input += angle * 0.1;

}

void CombineTimer(void) // 定时器中断调用函数，用于组合PID计算和电机PWM输出
{
    Read_Motor_V(); // 读取电机速度
    PID_Cal_Computer_Out();// 调用PID计算函数
    ReadMpu();// 读取车辆姿态
}

/**
设置PWM控制电机的正反转和占空比
 * @param M1_PWM 控制电机1的PWM值，正数表示正转，负数表示反转
 * @param M2_PWM 控制电机2的PWM值，正数表示正转，负数表示反转
 * @param M3_PWM 控制电机3的PWM值，正数表示正转，负数表示反转
 * @param M4_PWM 控制电机4的PWM值，正数表示正转，负数表示反转
 * HIGH LOW（1 0）表示反转
 * 该函数根据传入的PWM值设置对应电机的转动方向和PWM占空比，实现电机的控制。
 */
void Motor_PWM_Set(float M1_PWM, float M2_PWM, float M3_PWM, float M4_PWM)
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

// 初始化电机控制引脚为输入或输出模式，并将所有控制引脚设为低电平
void Motor_Init(void)
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

// 初始化PID控制器的参数，包括比例、积分、微分系数和输出限制
void PID_Init(void)
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

// PID增量式计算函数，计算PID控制器的输出
void PID_Cal(PID *pid)
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

//下降沿捕获中断 并计算速度
/**
 * 读取电机速度并进行处理
 * 该函数用于获取四个电机（M1、M2、M3、M4）的当前速度反馈，
 * 并根据这些反馈调整电机PID控制器的反馈值。
 * 通过中断采集编码器脉冲来计算电机速度。
 */
void Read_Motor_V(void)
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
    unsigned long nowtime = 0;
    nowtime = millis() + 50; //读50毫秒
    
    // 配置四个电机的编码器中断，用于捕捉脉冲
    attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), Read_motor_M1, FALLING);
    attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), Read_motor_M2, FALLING);
    attachInterrupt(digitalPinToInterrupt(M3_ENCODER_A), Read_motor_M3, FALLING);
    attachInterrupt(digitalPinToInterrupt(M4_ENCODER_A), Read_motor_M4, FALLING);
    
    // 等待设定的延迟时间，以便中断可以采集到足够的脉冲数据
    while(millis() < nowtime); //达到50毫秒关闭中断
    
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
}