#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <math.h>

#define I2C_SDA     21
#define I2C_SCL     22
#define SERVO_PIN   18

#define SERIAL_BAUD 9600
#define ANGLE_LIMIT 45.0f                         

Adafruit_MPU6050 mpu;
Servo servo1;

float pitch_f = 0.0f;
float roll_f  = 0.0f;

float g = 0.0f;   // yaw (rad)
float b = 0.0f;   // pitch (rad)
float a = 0.0f;   // roll (rad)

uint32_t last_us = 0;

const float SERVO_GAIN = 90.0f / ANGLE_LIMIT;    

float wrap_pi(float x) {
  float PI_F = acosf(-1.0f);
  float two_pi = 2.0f * PI_F;
  while (x <= -PI_F) x += two_pi;
  while (x >   PI_F) x -= two_pi;
  return x;
}

int16_t mapAngleToServo(float pitch_deg) {
  if (pitch_deg >  ANGLE_LIMIT) pitch_deg =  ANGLE_LIMIT;
  if (pitch_deg < -ANGLE_LIMIT) pitch_deg = -ANGLE_LIMIT;

  float servo = 90.0f + pitch_deg * SERVO_GAIN;

  int16_t angle = (int16_t)lroundf(servo);
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;

  return angle;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("ERRO: MPU6050 nao encontrado.");
    while (1) delay(1000);
  }
  Serial.println("MPU6050 OK.");

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  servo1.setPeriodHertz(50);
  servo1.attach(SERVO_PIN);
  servo1.write(90); // posição central

  sensors_event_t acc, gyr, tmp;
  delay(300);
  mpu.getEvent(&acc, &gyr, &tmp);

  float ax = acc.acceleration.x;
  float ay = acc.acceleration.y;
  float az = acc.acceleration.z;

  float pitch_acc = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  float roll_acc  = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / PI;

  pitch_f = pitch_acc;
  roll_f  = roll_acc;

  b = pitch_f * (PI / 180.0f);
  a = roll_f  * (PI / 180.0f);
  g = 0.0f;

  last_us = micros();
}


void loop() {
  uint32_t now_us = micros();

  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;

  sensors_event_t acc, gyr, tmp;
  mpu.getEvent(&acc, &gyr, &tmp);

  float ax = acc.acceleration.x;
  float ay = acc.acceleration.y;
  float az = acc.acceleration.z;

  // Ângulos "brutos" do acelerômetro (em graus)
  float pitch_acc = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  float roll_acc  = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / PI;

  float gx_rad = gyr.gyro.x;
  float gy_rad = gyr.gyro.y;
  float gz_rad = gyr.gyro.z;

  float gx_deg = gx_rad * 180.0f / PI;
  float gy_deg = gy_rad * 180.0f / PI;
  
  float pitch_gyro = pitch_f + gy_deg * dt;
  float roll_gyro  = roll_f  + gx_deg * dt;

  const float alpha_c = 0.92f;
  pitch_f = alpha_c * pitch_gyro + (1.0f - alpha_c) * pitch_acc;
  roll_f  = alpha_c * roll_gyro  + (1.0f - alpha_c) * roll_acc;

  g += gz_rad * dt;
  g = wrap_pi(g);

  b = pitch_f * (PI / 180.0f);
  a = roll_f  * (PI / 180.0f);
  
  int16_t servo_angle = mapAngleToServo(pitch_f);
  servo1.write(servo_angle);

  // Saída no formato (yaw:pitch:roll) em radianos
  Serial.print(g);
  Serial.print(':');
  Serial.print(b);
  Serial.print(':');
  Serial.println(a);
}
