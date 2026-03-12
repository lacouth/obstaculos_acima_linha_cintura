#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SERVO_PIN 18

Adafruit_MPU6050 mpu;
Servo servo;

float sensibilidade = 2.0; 
float anguloBase = 0;      

// Limites do servo
const int servoMin = 60;   
const int servoMax = 150;  

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 nao encontrado.");
    while (1);
  }

  servo.attach(SERVO_PIN, 500, 2400);

  // Calibração 

  Serial.println("Segure a bengala na altura normal para calibrar...");
  delay(2000);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  anguloBase = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

  Serial.print("Angulo base calibrado: ");
  Serial.println(anguloBase);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accY = a.acceleration.y;
  float accZ = a.acceleration.z;

  float angulo = atan2(accY, accZ) * 180 / PI;

  // diferença em relação à posição neutra + sensibilidade
  float anguloRelativo = (angulo - anguloBase) * sensibilidade;

  // câmera corrige pra baixo quando a bengala sobe
  int servoPos = 90 - anguloRelativo;

  // limitar faixa de movimento
  servoPos = constrain(servoPos, servoMin, servoMax);

  servo.write(servoPos);

  Serial.print("Angulo relativo: ");
  Serial.print(anguloRelativo);
  Serial.print("  Servo: ");
  Serial.println(servoPos);

  delay(20);
}