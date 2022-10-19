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

#define MAX_WHEELS_ANGLE 90      // Максимальный угол серво-двигателя колёс
#define MAX_SCANNER_ANGLE 180    // Максимальный угол серво-двигателя сканера дальности
#define SCANNER_STEP_DEGREES 15  // Шаг поворота серво-двигателя сканера дальности в градусах

#define WHEELS_FORWRD 50    // Угол серво-двигателя колёс - ПРЯМО
#define SCANNER_FORWARD 90  // Угол серво-двигателя сканера - ПРЯМО

#define DISTANCE_TO_BLOCKAGE_CM 50  // Дистанция до препятcтвия

#define SPEED_CAR_MAX 240  // Максимальная скорость движения машинки
#define SPEED_CAR_MIN 200  // Минимальная скорость движения машинки
#define TIME_MOVING 1000   // Время движения по заданной траектории



struct ScannerData {
  long unsigned int Distance;  // Дистанция до препятсвия
  int Angle;                   // Угол поворота серво-двигателя сканера дальности
};

Servo wheels;                                              // Объект серво-двигателя колёс
Servo scanner;                                             // Объёкт серво-двигателя сканера дальности
NewPing sonar(TRIGGER_PIN, ECHO_PIN, SONAR_MAX_DISTANCE);  // Объект сканера дальности


void setup() {
  Serial.begin(9600);                 // Порт ввода/вывода
  wheels.attach(WHEELS_SERVO_PIN);    // Подключаем серво-двигатель колёс
  scanner.attach(SCANNER_SERVO_PIN);  // Подключаем серво-двигатель сканера дальности

  //Подключаем мотор машинки
  pinMode(E1, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
}

void loop() {
  stopMove(1000);  // Остановиться на секунду

  const int lenDataArray = MAX_SCANNER_ANGLE / SCANNER_STEP_DEGREES + 1;  // Рассчитываем длину массива в зависимости от вводных данных
  struct ScannerData dataArray[lenDataArray];                             // Создаём массив заданной длины

  Serial.println("SCANNER DATA ARRAY");
  for (int i = 0; i < lenDataArray; i++) {
    Serial.print("Number: ");
    Serial.print(i);

    scanner.write(i * SCANNER_STEP_DEGREES);                       // Поварачиваем сканер дальности на угол с шагом
    struct ScannerData tmp = { sonar.ping_cm(), scanner.read() };  // Считываем данные и создаём структуру
    if (tmp.Distance == 1149) {                                    // В данных сканера дальности проскакивает это магическое число, не разобрался почему так
      tmp.Distance = 0;
    }
    dataArray[i] = tmp;           // Добвляем в массив данных нашу структуру
    FullPrintData(dataArray[i]);  // Выводим в монитор порта данные
    delay(500);
  }

  Serial.println("---------------------------------------");
  struct ScannerData maxScannerDataByDistance = GetMaxScannerDataByDistanceField(dataArray, lenDataArray);  // Получаем максимальное значение по дистанции
  Serial.print("Max Scanner Data:");
  FullPrintData(maxScannerDataByDistance);
  Serial.println("---------------------------------------");

  int currentWheelsAngle = wheels.read();  // Текущее положение серво двигателя колёс
  delay(500);
  int scannerAngleByWheels = GetScannerAngle(currentWheelsAngle);  // Конвертируем значения угла сканера в угол колёс
  scanner.write(scannerAngleByWheels);                             // Поворот сканерра на новый угол
  delay(500);
  int currentDistance = sonar.ping_cm();            // Текушая дистанция до препятствия
  if (currentDistance < DISTANCE_TO_BLOCKAGE_CM) {  // Если текущая дистанция меньше максимальной дистанции до препятсвия
    wheels.write(WHEELS_FORWRD);                    // Колёса прямо
    moveBack(SPEED_CAR_MAX, TIME_MOVING);           // Отъезжаем назад
  }

  if (maxScannerDataByDistance.Distance > DISTANCE_TO_BLOCKAGE_CM) {         // Если максимальная дистанция больше максимальной дистанции до препятсвия
    int wheelsRotateAngle = GetWheelsAngle(maxScannerDataByDistance.Angle);  // Считаем угол поворота колёс относительно максимальных углов серво-двигателей колёс и сканера дальности
    Serial.print("Wheels degrees: ");
    Serial.println(wheelsRotateAngle);
    Serial.println("---------------------------------------");

    wheels.write(WHEELS_FORWRD);           // Колёса прямо
    moveBack(SPEED_CAR_MIN, TIME_MOVING);  // Чуть чуть отъзжаем назад из-за возможно больших габаритов машины

    wheels.write(wheelsRotateAngle);          // Поварачиваем серво двигатель колёс на угол с максимальной дистанцией до препятствия
    moveForward(SPEED_CAR_MAX, TIME_MOVING);  // Движемся вперёд
  } else {                                    // в противном случае
    wheels.write(WHEELS_FORWRD);              // Колёса прямо
    moveBack(SPEED_CAR_MAX, TIME_MOVING);     // Едем назад на полной скорости
  }
}

void FullPrintData(struct ScannerData item) {
  Serial.print(" | ");
  Serial.print("Distance: ");
  Serial.print(item.Distance);
  Serial.print(" | ");
  Serial.print("Angle: ");
  Serial.println(item.Angle);
  Serial.println();
}

int GetWheelsAngle(int scannerAngle) {  // Конвертация угла сканера-дальности в угол поворота колёс
  return (scannerAngle * MAX_WHEELS_ANGLE) / MAX_SCANNER_ANGLE;
}

int GetScannerAngle(int wheelsAngle) {  // Конвертация угла поворота колёс в угол сканера дальности
  return (wheelsAngle * MAX_SCANNER_ANGLE) / MAX_WHEELS_ANGLE;
}

struct ScannerData GetMaxScannerDataByDistanceField(struct ScannerData array[], int lenArray) {  // Получение данных со сканера по максимальной дистанции
  struct ScannerData maxValue = array[0];
  for (int i = 0; i < lenArray; i++) {
    if (array[i].Distance > maxValue.Distance) {
      maxValue = array[i];
    }
  }
  return maxValue;
}

void moveForward(int v, int t) {  // Движение вперёд
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  digitalWrite(E1, LOW);
  delay(100);
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  analogWrite(E1, v);
  delay(t);
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  digitalWrite(E1, LOW);
  delay(100);
}

void moveBack(int v, int t) {  // Движение назад
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

void stopMove(int t) {  // Остановка движения
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  digitalWrite(E1, LOW);
  delay(t);
}