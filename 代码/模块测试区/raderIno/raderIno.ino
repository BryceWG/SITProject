#include<Servo.h>
#define Trig 12//引脚Tring 连接 IO D2  
#define Echo 13 //引脚Echo 连接 IO D3
Servo myServo; //实例化舵机对象
float distance;

void setup() {
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  myServo.attach(11); //初始化舵机信号引脚
  Serial.begin(9600); //初始化通信波特率
  myServo.write(0);//设定舵机的初始角度
  delay(100);
}

void loop() {
  for (int i = 0; i <= 180; i++) {
    myServo.write(i);
    distance = getDistance(Trig, Echo);
    if (distance > 50) //当超声波测到的距离超出范围时，设定为可检测到的最远距离
      distance = 50;
    else if (distance < 5) //当超声波测到的距离小于5时，设定为0
      distance = 0;
	//将数据按照一定格式发送给串口
    Serial.print('#');
    Serial.print(i);
    Serial.print('#');
    Serial.print(distance);
    Serial.println('#');
    delay(70); //70ms一次转动与采集,略大于最大测量周期66ms
  }
  myServo.write(0);
  delay(200);
}

float getDistance(int trig, int echo) {
  float temp = 0, distancecm = 0;
  digitalWrite(trig, LOW);   //给Trig发送一个低电平
  delayMicroseconds(2);      //等待 2微妙
  digitalWrite(trig, HIGH);  //给Trig发送一个高电平
  delayMicroseconds(10);     //等待 10微妙
  digitalWrite(trig, LOW);   //给Trig发送一个低电平
  temp = (float)(pulseIn(echo, HIGH,750000));//存储回波等待时间
  distancecm = temp * 17 / 1000; //把回波时间换算成cm
  return distancecm;
}