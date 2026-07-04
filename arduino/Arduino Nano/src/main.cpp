#include <Arduino.h>
#include <Servo.h>


const int NUM_SERVOS = 6;

const int SERVO_PINS[NUM_SERVOS] = {A5, A3, A4, A2, A0, A1};

Servo servos[NUM_SERVOS];

int actual_positions[NUM_SERVOS]= {90, 115, 110, 175, 90, 90};


void apply_movement(int positions[]) {

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(positions[i]);
  }

} 

void read_positions_from_serial(int *positions);




void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
  }

  apply_movement(actual_positions);
}


void loop() {

  if(!Serial.available()) {
    return;
  }

  read_positions_from_serial(actual_positions);

  apply_movement(actual_positions);

}


void read_positions_from_serial(int *positions) {

  String line = Serial.readStringUntil('\n');
  line.trim();


  int parsed = sscanf(
    line.c_str(),
    "%d,%d,%d,%d,%d,%d",
    &positions[0],
    &positions[1],
    &positions[2],
    &positions[3],
    &positions[4],
    &positions[5]
  );

  if (parsed != NUM_SERVOS) {
    Serial.println(F("ERR"));
    return;
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    positions[i] = constrain(positions[i], 0, 180);
  }

  Serial.println(F("OK"));

} 