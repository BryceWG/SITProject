#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor pins
const int motorLeftForward = 3;
const int motorLeftBackward = 4;
const int motorRightForward = 5;
const int motorRightBackward = 6;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Set motor pins as output
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate the angle
  float accelAngleX = atan2(ay, az) * RAD_TO_DEG;
  float accelAngleY = atan2(ax, az) * RAD_TO_DEG;

  Serial.print("Angle X: ");
  Serial.print(accelAngleX);
  Serial.print("  |  Angle Y: ");
  Serial.println(accelAngleY);

  // Control the motors based on the angle
  if (accelAngleX > 10) {
    // Move forward
    digitalWrite(motorLeftForward, HIGH);
    digitalWrite(motorLeftBackward, LOW);
    digitalWrite(motorRightForward, HIGH);
    digitalWrite(motorRightBackward, LOW);
  } else if (accelAngleX < -10) {
    // Move backward
    digitalWrite(motorLeftForward, LOW);
    digitalWrite(motorLeftBackward, HIGH);
    digitalWrite(motorRightForward, LOW);
    digitalWrite(motorRightBackward, HIGH);
  } else {
    // Stop
    digitalWrite(motorLeftForward, LOW);
    digitalWrite(motorLeftBackward, LOW);
    digitalWrite(motorRightForward, LOW);
    digitalWrite(motorRightBackward, LOW);
  }

  delay(100);
}
