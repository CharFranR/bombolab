#include <Arduino.h>
#include <Servo.h>


const int NUM_SERVOS = 6;

// Pin mapping — ordered to match ServoCommand wire format:
//   J1(yaw), J2(shoulder), J3(elbow), J4(roll), J5(pitch), Gripper
// Wire pos 0 = J1 yaw    → pin A1
// Wire pos 1 = J2 shoulder → pin A0
// Wire pos 2 = J3 elbow   → pin A2
// Wire pos 3 = J4 roll    → pin A4
// Wire pos 4 = J5 pitch   → pin 13
// Wire pos 5 = Gripper    → pin A5
const int SERVO_PINS[NUM_SERVOS] = {A1, A0, A2, A4, 13, A5};

Servo servos[NUM_SERVOS];

int actual_positions[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

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
