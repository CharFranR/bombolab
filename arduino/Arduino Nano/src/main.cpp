#include <Arduino.h>
#include <Servo.h>


const int NUM_SERVOS = 6;

// Pin mapping: Arduino analog pins → servo indices
// A5 → S1 (J1, Base yaw)
// A3 → S2 (J2, Shoulder)
// A4 → S3 (J3, Elbow)
// A2 → S4 (J4, Wrist roll)
// A0 → S5 (J5, Wrist pitch)
// A1 → S6 (Gripper)
const int SERVO_PINS[NUM_SERVOS] = {A5, A3, A4, A2, A0, A1};

Servo servos[NUM_SERVOS];

int actual_positions[NUM_SERVOS] = {90, 115, 110, 170, 90, 90};

// ---------------------------------------------------------------------------
// Serial protocol parser — character-by-character, no sscanf
// ---------------------------------------------------------------------------
// Expects: "a1,a2,a3,a4,a5,a6\n" — exactly 6 comma-separated integers.
// Rejects: extra commas, non-numeric chars, wrong field count, empty lines.
// Returns true on success, false on any parse error.
// ---------------------------------------------------------------------------
bool read_positions_serial(int positions[6]) {
    int idx = 0;
    int value = 0;
    bool has_digit = false;
    unsigned long start = millis();

    while (true) {
        // Timeout — prevent blocking forever on a partial line
        if (millis() - start > 100) {
            return false;
        }

        if (!Serial.available()) {
            continue;
        }

        char c = Serial.read();

        // Line terminator — validate and store the last value
        if (c == '\n') {
            if (!has_digit || idx != 5) return false;
            positions[5] = value;

            // Range validation
            for (int i = 0; i < NUM_SERVOS; i++) {
                if (positions[i] < 10 || positions[i] > 170) return false;
            }
            return true;
        }

        // Skip carriage returns (CRLF tolerance)
        if (c == '\r') continue;

        // Digit — accumulate the current value
        if (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            has_digit = true;
        }
        // Comma — store current value and advance to next field
        else if (c == ',') {
            if (!has_digit || idx >= 5) return false;
            positions[idx++] = value;
            value = 0;
            has_digit = false;
        }
        // Invalid character — flush rest of line and fail
        else {
            while (Serial.available()) {
                char discard = Serial.read();
                if (discard == '\n') break;
            }
            return false;
        }
    }
}


void apply_movement(int positions[]) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(positions[i]);
    }
}


void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(SERVO_PINS[i]);
    }

    apply_movement(actual_positions);
}


void loop() {
    if (!Serial.available()) {
        return;
    }

    // Peek at the first byte — skip empty lines (stray newlines)
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
