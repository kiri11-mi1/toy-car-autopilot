#include <Servo.h>
#include <NewPing.h>

// Пины для мотора
#define E1 3
#define I1 A0
#define I2 A1

// Пины для серво-двигателей колёс и сканера дальности
#define WHEELS_SERVO_PIN 5
#define SCANNER_SERVO_PIN 6

// Пины для сканера дальности
#define ECHO_PIN 9
#define TRIGGER_PIN 10
#define SONAR_MAX_DISTANCE 500

Servo wheels; // Объект серво-двигателя колёс
Servo scanner; // Объёкт серво-двигателя сканера дальности
NewPing sonar(TRIGGER_PIN, ECHO_PIN, SONAR_MAX_DISTANCE); // Объект сканера дальности


void setup() {
  Serial.begin(9600); // Порт ввода/вывода
  wheels.attach(WHEELS_SERVO_PIN); // Подключаем серво-двигатель колёс
  scanner.attach(SCANNER_SERVO_PIN); // Подключаем серво-двигатель сканера дальности

  //Подключаем мотор машинки
  pinMode(E1, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);

}

void loop() {
//  int distance = sonar.ping();
//  int speedCar = 200;
//  Serial.println(distance);
//  if (distance == 0) {
//    moveForward(speedCar, 1000);
//  }
//  if (distance <= 20 && distance != 0){
//    stopMove(1000);
//    moveBack(200, 1000);
//    
//  } else {
//    int t = (distance / speedCar) * 100;
//    moveForward(speedCar, t);
//  }
  moveForward();
}

void moveForward(int v, int t) {
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW); 
  digitalWrite(E1, LOW);
  delay(100);
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW); 
  analogWrite(E1,v);
  delay(t);
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW); 
  digitalWrite(E1, LOW);
  delay(100);
}

void moveBack(int v,int t) {
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW); 
  digitalWrite(E1, LOW);
  delay(100);
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH); 
  analogWrite(E1, v);
  delay(t);
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW); 
  digitalWrite(E1, LOW);
  delay(100);
}

void stopMove(int t) {
  digitalWrite(I1,LOW);
  digitalWrite(I2,LOW); 
  digitalWrite(E1,LOW);
  delay(t);
}
 
