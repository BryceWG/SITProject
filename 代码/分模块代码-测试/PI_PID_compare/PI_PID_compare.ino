#include <MsTimer2.h>
#include <Arduino.h>
// 省略未使用的头文件，简化代码

// 定义电机编码器的引脚
#define M1_ENCODER_A 2
#define M1_ENCODER_B 40

// 定义电机驱动TB6612的控制引脚
#define TB6612_M1_IN1 22
#define TB6612_M1_IN2 23
#define Motor_M1_PWM 4

// PID结构体定义
typedef struct
{
    float input;        // 目标值
    float output;       // 输出值
    float feedback;     // 反馈值
    float k_p;          // P增益
    float k_i;          // I增益
    float k_d;          // D增益
    float err_1;        // 当前误差
    float err_2;        // 上一次误差
    float err_x;        // 误差累计
    float out_max;      // 输出最大值
    float out_min;      // 输出最小值
    float err_x_max;    // 误差累计最大值
} PID;

volatile long motor_M1_count = 0;    // 编码器计数，使用 long 类型防止溢出
volatile bool needToReadMotors = false;

unsigned int timecnt = 0;
float V_M1 = 0.0;
int motor_M1_dir = 0;
PID M1_Motor_PID;

enum ControlType { PID_CONTROL, PI_CONTROL };
ControlType currentControl = PID_CONTROL;

enum MotionState { 
    ACCELERATE,          // 加速阶段
    DECELERATE,          // 减速阶段
    STATIONARY,          // 静止阶段
    SUDDEN_ACCELERATE,   // 突变加速
    SUDDEN_STOP          // 突变停止
};
MotionState currentMotion = ACCELERATE;

float targetSpeed = 0.0;
const float accelerationRate = 100.0; // 加速度，单位：mm/s^2
unsigned long motionStartTime = 0;

// 定义控制器参数的范围和步长
const float kp_values[] = {0.08, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14}; // P增益测试值
const float ki_values[] = {0.091, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12}; // I增益测试值
const float kd_values[] = {0.1, 0.05, 0.1, 0.15, 0.2}; // D增益测试值

int kp_index = 0;
int ki_index = 0;
int kd_index = 0;

bool parametersExhausted = false; // 标记参数是否遍历完毕

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Motor_Init();
    PID_Init();
    
    MsTimer2::set(40, interruptHandler); // 定时器中断周期为 20ms
    MsTimer2::start();
    
    // 输出数据标题，增加Phase列
    Serial.println("Timestamp_ms,ControlType,k_p,k_i,k_d,TargetSpeed_mm_s,ActualSpeed_mm_s,Error_mm_s,Phase");
    
    motionStartTime = millis();
}

void loop() {
    // 检查是否需要切换运动状态
    updateMotionState();
    
    // 更新目标速度
    updateTargetSpeed();
    M1_Motor_PID.input = targetSpeed;

    // 如果所有参数都测试完毕，停止程序
    if (parametersExhausted) {
        Serial.println("All parameters tested. Experiment completed.");
        while (1); // 停止程序运行
    }
}

void updateMotionState() {
    unsigned long elapsedTime = millis() - motionStartTime;
    
    switch(currentMotion) {
        case ACCELERATE:
            if (elapsedTime >= 2000) {  // 加速2秒
                currentMotion = DECELERATE;
                motionStartTime = millis();
            }
            break;
        case DECELERATE:
            if (elapsedTime >= 2000) {  // 减速2秒
                currentMotion = STATIONARY;
                motionStartTime = millis();
            }
            break;
        case STATIONARY:
            if (elapsedTime >= 3000) {  // 静止3秒
                currentMotion = SUDDEN_ACCELERATE;
                motionStartTime = millis();
            }
            break;
        case SUDDEN_ACCELERATE:
            if (elapsedTime >= 6000) {  // 匀速6秒
                currentMotion = SUDDEN_STOP;
                motionStartTime = millis();
            }
            break;
        case SUDDEN_STOP:
            if (elapsedTime >= 3000) {  // 停止3秒
                // 完成一个循环，调整参数
                adjustParameters();
                if (!parametersExhausted) {
                    currentMotion = ACCELERATE;
                    motionStartTime = millis();
                }
            }
            break;
    }
}

void adjustParameters() {
    // 调整控制器参数
    if (currentControl == PID_CONTROL) {
        // 遍历PID参数组合
        kd_index++;
        if (kd_index >= sizeof(kd_values)/sizeof(kd_values[0])) {
            kd_index = 0;
            ki_index++;
            if (ki_index >= sizeof(ki_values)/sizeof(ki_values[0])) {
                ki_index = 0;
                kp_index++;
                if (kp_index >= sizeof(kp_values)/sizeof(kp_values[0])) {
                    // PID参数测试完毕，切换到PI控制
                    kp_index = 0;
                    ki_index = 0;
                    kd_index = 0;
                    currentControl = PI_CONTROL;
                    // 移除切换控制参数时的提示输出
                    // Serial.println("Switched to PI Control");
                }
            }
        }

        // 更新PID参数
        M1_Motor_PID.k_p = kp_values[kp_index];
        M1_Motor_PID.k_i = ki_values[ki_index];
        M1_Motor_PID.k_d = kd_values[kd_index];
        
        // 移除打印当前参数设置的输出语句
    } else if (currentControl == PI_CONTROL) {
        // 遍历PI参数组合
        kd_index = 0;
        ki_index++;
        if (ki_index >= sizeof(ki_values)/sizeof(ki_values[0])) {
            ki_index = 0;
            kp_index++;
            if (kp_index >= sizeof(kp_values)/sizeof(kp_values[0])) {
                // PI参数测试完毕，实验结束
                parametersExhausted = true;
            }
        }

        // 更新PI参数
        M1_Motor_PID.k_p = kp_values[kp_index];
        M1_Motor_PID.k_i = ki_values[ki_index];
        M1_Motor_PID.k_d = 0.0;
        
        // 移除打印当前参数设置的输出语句
    }

    // 重置PID状态
    resetPID(&M1_Motor_PID);
}

void updateTargetSpeed() {
    unsigned long elapsedTime = millis() - motionStartTime;
    
    switch(currentMotion) {
        case ACCELERATE:
            targetSpeed = min(200.0, accelerationRate * (elapsedTime / 1000.0));
            break;
        case DECELERATE:
            targetSpeed = max(0.0, 200.0 - accelerationRate * (elapsedTime / 1000.0));
            break;
        case STATIONARY:
            targetSpeed = 0.0;
            break;
        case SUDDEN_ACCELERATE:
            targetSpeed = 200.0;
            break;
        case SUDDEN_STOP:
            targetSpeed = 0.0;
            break;
    }
}

void interruptHandler() {
    PID_Cal_Computer_Out();
    Read_Motor_V();
}

void Motor_Init() {
    pinMode(M1_ENCODER_A, INPUT);
    pinMode(M1_ENCODER_B, INPUT);
    pinMode(TB6612_M1_IN1, OUTPUT);
    pinMode(TB6612_M1_IN2, OUTPUT);
    pinMode(Motor_M1_PWM, OUTPUT);
    
    digitalWrite(TB6612_M1_IN1, LOW);
    digitalWrite(TB6612_M1_IN2, LOW);
    analogWrite(Motor_M1_PWM, LOW);
}

void PID_Init() {
    // 初始化PID参数
    M1_Motor_PID.k_p = kp_values[kp_index];
    M1_Motor_PID.k_i = ki_values[ki_index];
    M1_Motor_PID.k_d = kd_values[kd_index];
    M1_Motor_PID.out_max = 255.0;  
    M1_Motor_PID.out_min = -255.0;
    M1_Motor_PID.input = 0.0;
    M1_Motor_PID.err_x_max = 1000.0;
    resetPID(&M1_Motor_PID);
}

void resetPID(PID *pid) {
    pid->err_1 = 0.0;
    pid->err_2 = 0.0;
    pid->err_x = 0.0;
    pid->output = 0.0;
}

void PID_Cal_Computer_Out() {
    PID_Cal(&M1_Motor_PID);
    Motor_PWM_Set(M1_Motor_PID.output);
    
    // 数据输出间隔为20ms
    if (++timecnt >= 1) { 
        timecnt = 0;

        // 数据输出，CSV格式
        Serial.print(millis()); // 时间戳
        Serial.print(",");
        Serial.print(currentControl == PID_CONTROL ? "PID" : "PI"); // 控制类型
        Serial.print(",");
        Serial.print(M1_Motor_PID.k_p, 5); // k_p
        Serial.print(",");
        Serial.print(M1_Motor_PID.k_i, 5); // k_i
        Serial.print(",");
        if (currentControl == PI_CONTROL) {
            Serial.print("NaN"); // PI控制时，k_d输出NaN
        } else {
        Serial.print(M1_Motor_PID.k_d, 5); // k_d
        }
        Serial.print(",");
        Serial.print(M1_Motor_PID.input, 5); // 目标速度
        Serial.print(",");
        Serial.print(M1_Motor_PID.feedback, 5); // 实际速度
        Serial.print(",");
        Serial.print(M1_Motor_PID.input - M1_Motor_PID.feedback, 5); // 误差
        Serial.print(",");
        Serial.println(getPhaseName(currentMotion)); // 输出当前阶段
    }
}

// 添加getPhaseName函数
String getPhaseName(MotionState state) {
    switch (state) {
        case ACCELERATE:
            return "ACCELERATE"; // 加速阶段
        case DECELERATE:
            return "DECELERATE"; // 减速阶段
        case STATIONARY:
            return "STATIONARY"; // 静止阶段
        case SUDDEN_ACCELERATE:
            return "SUDDEN_ACCELERATE"; // 突变加速
        case SUDDEN_STOP:
            return "SUDDEN_STOP"; // 突变停止
        default:
            return "Unknown";
    }
}

void PID_Cal(PID *pid) {
    float p, i, d;

    // 更新误差
    pid->err_2 = pid->err_1;
    pid->err_1 = pid->input - pid->feedback;

    // 计算PID各项
    p = pid->k_p * pid->err_1;
    i = pid->k_i * pid->err_x;
    d = pid->k_d * (pid->err_1 - pid->err_2);
    
    // 更新累积误差
    pid->err_x += pid->err_1;
    
    // 计算输出
    pid->output = p + i + d;

    // 限制输出和累积误差
    if(pid->output > pid->out_max)      pid->output = pid->out_max;
    if(pid->output < pid->out_min)      pid->output = pid->out_min;
    if(pid->err_x > pid->err_x_max)     pid->err_x = pid->err_x_max;
}

void Motor_PWM_Set(float pwm_value) {
    if(pwm_value >= 0) {
        digitalWrite(TB6612_M1_IN1, HIGH);      
        digitalWrite(TB6612_M1_IN2, LOW);
        analogWrite(Motor_M1_PWM, pwm_value);  
    } else {
        digitalWrite(TB6612_M1_IN1, LOW);
        digitalWrite(TB6612_M1_IN2, HIGH);
        analogWrite(Motor_M1_PWM, pwm_value * -1);
    }
}

void Read_motor_M1() {
    motor_M1_dir = digitalRead(M1_ENCODER_B);
    if(motor_M1_dir == 1) {
        motor_M1_count++;
    } else {
        motor_M1_count--;
    }
}

void Read_Motor_V() {
    static unsigned long lastReadTime = 0;
    static float last_V_M1 = 0.0;
    const float speed_filter_k = 0.7; // 一阶低通滤波系数
    unsigned long currentTime = millis();
    unsigned int readInterval = 50; // 编码器读取间隔，单位：ms
    
    if ((currentTime - lastReadTime) >= readInterval && !needToReadMotors) {  
        lastReadTime = currentTime;
        motor_M1_count = 0;
        attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), Read_motor_M1, FALLING);
        needToReadMotors = true;
    } else if (needToReadMotors) {    
        needToReadMotors = false;
        detachInterrupt(digitalPinToInterrupt(M1_ENCODER_A));
        
        // 计算速度，单位：mm/s
        float deltaTime = (currentTime - lastReadTime) / 1000.0; // 时间差，单位：s
        float revolutions = (float)motor_M1_count / 330.0; // 330为编码器每转的脉冲数
        float distance = revolutions * 65.0 * 3.1416; // 65.0为轮子直径，单位：mm

        V_M1 = distance / deltaTime; // 速度，单位：mm/s

        last_V_M1 = M1_Motor_PID.feedback;
        // 对速度进行一阶低通滤波，减少噪声
        M1_Motor_PID.feedback = speed_filter_k * V_M1 + (1 - speed_filter_k) * last_V_M1;
        
    }
}
