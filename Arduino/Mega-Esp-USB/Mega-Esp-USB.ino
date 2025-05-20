
#include <AFMotor.h>     // подключаем библиотеку для шилда

AF_DCMotor motor1(3); // подключаем мотор к клеммникам M1
AF_DCMotor motor2(4); // подключаем мотор к клеммникам M2

int maxSpeed = 255;
int iterr = 0;
char c;
String inString;
String firstpart;
String secondpart;
// Настройка
void setup() {
  // Инициализация портов и выходов
  Serial.begin(115200);
  Serial3.begin(115200);
}

// Выполнение
void loop() {
  serialEvent3();

}

// Проверка события на порту Serial3
void serialEvent3() {
  while (Serial3.available()) {
    inString = Serial3.readStringUntil(':');
    if(inString[0] == ';')
    {
      if(inString.substring(1, inString.indexOf('.')).toInt() >= iterr)
      {
      Serial.println(inString);
      firstpart = inString.substring(inString.indexOf('.') + 1, inString.indexOf(','));
      secondpart = inString.substring(inString.indexOf(',') + 1, inString.indexOf('/'));
      // Поиск команды в полученных данных (команда должна быть в квадратных скобках)
      Serial.println("----------------------------------------------");
      Serial.println(firstpart.toInt());
      Serial.println("++++++++++++++++++++++++++++++++++++++++++++++");
      Serial.println(secondpart.toInt());
      Serial.println("==============================================");
      setMotorSpeed(motor1,firstpart.toInt());
      setMotorSpeed(motor2,secondpart.toInt());
      inString = "";
      delay(100);
      }
      else
      {
      Serial.println("command " + inString + "have old iterr");  
      }
    }
    else
    {
    Serial.println("unknown command " + inString);
    }
    Serial3.read();
    Serial3.read();
  }
}


void setMotorSpeed(AF_DCMotor motor, int persent)
{
if (-100 <= persent <= 100){
if (persent >=0)
{
 motor.setSpeed((maxSpeed*persent)/100);
}
else
{
 motor.setSpeed((maxSpeed*(-persent))/100);
}
}

 if (persent > 0)
 {
  motor.run(FORWARD);
 }
 else if (persent < 0)
 {
  motor.run(BACKWARD);
 }
 else if (persent == 0)
 {
   motor.run(RELEASE);
 }
 else
 {
  motor.setSpeed(20);
  motor.run(BACKWARD);
 }
}

