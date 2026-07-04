#include <Arduino.h>
#include <ESP32Servo.h>

Servo servo1, servo2, servo3, servo4, servo5, servo6, servoPrueba;

const int servoPin1 = 26;
const int servoPin2 = 22;
const int servoPin3 = 21;
const int servoPin4 = 25;
const int servoPin5 = 19;
const int servoPin6 = 18;

const int servoPinPrueba = 27;

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);

  servoPrueba.attach(servoPinPrueba);
}

void loop() {
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
  servo6.write(90); 

  delay(1000);

  servoPrueba.write(90); 

}