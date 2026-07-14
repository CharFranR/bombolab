#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// FABRI Creator — ESP32 servo control via serial protocol
// ---------------------------------------------------------------------------
// Protocol: "a1,a2,a3,a4,a5,a6\n" (6 comma-separated integers, degrees)
// Response: "OK\n" on success, "ERR\n" on parse or range error
// ---------------------------------------------------------------------------

const int NUM_SERVOS = 6;

// Pin mapping (ESP32 GPIO → servo)
// 26 → S1 (J1, Base yaw)
// 22 → S2 (J2, Shoulder pitch)
// 21 → S3 (J3, Elbow pitch)
// 25 → S4 (J4, Wrist roll)
// 19 → S5 (J5, Wrist pitch)
// 18 → S6 (Gripper)
const int SERVO_PINS[NUM_SERVOS] = {26, 22, 21, 25, 19, 18};

Servo servos[NUM_SERVOS];

int positions[NUM_SERVOS] = {90, 115, 110, 170, 90, 90};

// ---------------------------------------------------------------------------
// Character-by-character serial parser (same as Arduino Nano firmware)
// ---------------------------------------------------------------------------
// Validates: exactly 6 fields, digits and commas only, range 10–170.
// Rejects: extra commas, non-numeric chars, wrong field count, empty lines.
// ---------------------------------------------------------------------------
bool read_positions_serial(int out[6]) {
    int idx = 0;
    int value = 0;
    bool has_digit = false;
    unsigned long start = millis();

    while (true) {
        if (millis() - start > 100) return false;
        if (!Serial.available()) continue;

        char c = Serial.read();

        if (c == '\n') {
            if (!has_digit || idx != 5) return false;
            out[5] = value;
            for (int i = 0; i < NUM_SERVOS; i++) {
                if (out[i] < 10 || out[i] > 170) return false;
            }
            return true;
        }

        if (c == '\r') continue;

        if (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            has_digit = true;
        } else if (c == ',') {
            if (!has_digit || idx >= 5) return false;
            out[idx++] = value;
            value = 0;
            has_digit = false;
        } else {
            while (Serial.available()) {
                if (Serial.read() == '\n') break;
            }
            return false;
        }
    }
}


void apply_movement(const int pos[]) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(pos[i]);
    }
}


void setup() {
    Serial.begin(115200);

    // Allow a brief window for ESP32-specific PWM allocation
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].setPeriodHertz(50);        // Standard 50 Hz servo signal
        servos[i].attach(SERVO_PINS[i], 500, 2500); // 500–2500 μs pulse range
    }

    apply_movement(positions);
    Serial.println(F("READY"));
}


void loop() {
    if (!Serial.available()) return;

    // Skip empty lines (stray newlines)
    if (Serial.peek() == '\n' || Serial.peek() == '\r') {
        Serial.read();
        return;
    }

    int new_positions[NUM_SERVOS];

    if (read_positions_serial(new_positions)) {
        apply_movement(new_positions);
        Serial.println(F("OK"));
    } else {
        Serial.println(F("ERR"));
    }
}
