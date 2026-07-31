#include <Arduino.h>
#include <Servo.h>

// Failsafe (watchdog): if no valid frame is received within HOLD_TIMEOUT_MS,
// the servos are parked at the home pose (90,90,90,90,90,90) once per timeout
// period. A new accepted frame resumes normal control. This prevents the arm
// from holding torque forever if the host dies or the cable is unplugged.


const int NUM_SERVOS = 6;
const unsigned long HOLD_TIMEOUT_MS = 5000;

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

// Last accepted frame timestamp + failsafe parking state (see loop()).
unsigned long last_command_ms = 0;
bool parked = true;  // boot pose is the home pose; nothing to park yet

// ---------------------------------------------------------------------------
// Serial protocol parser — character-by-character, no sscanf
// ---------------------------------------------------------------------------
// Expects: "a1,a2,a3,a4,a5,a6\n" — exactly 6 comma-separated integers.
// Rejects: extra commas, non-numeric chars, wrong field count, empty lines.
// Returns true on success, false on any parse error.
// ---------------------------------------------------------------------------
// Drain the serial RX buffer until a newline is consumed (or the buffer is
// empty), so a rejected frame cannot corrupt the next one.
static void drain_rx_until_newline() {
    while (Serial.available()) {
        char discard = Serial.read();
        if (discard == '\n') break;
    }
}

bool read_positions_serial(int positions[6]) {
    int idx = 0;
    long value = 0;  // long is 32-bit on AVR; int is 16-bit and a 5-digit field would wrap
    int digit_count = 0;
    bool has_digit = false;
    unsigned long start = millis();

    while (true) {
        // Timeout — prevent blocking forever on a partial line, and drain the
        // rest of the line so residual bytes cannot corrupt the next frame
        if (millis() - start > 100) {
            drain_rx_until_newline();
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
            digit_count++;
            // No angle can have more than 3 digits (max 170). Rejecting here
            // also prevents a 5-digit field from wrapping the 16-bit range.
            if (digit_count > 3) {
                drain_rx_until_newline();
                return false;
            }
        }
        // Comma — store current value and advance to next field
        else if (c == ',') {
            if (!has_digit || idx >= 5) return false;
            positions[idx++] = value;
            value = 0;
            has_digit = false;
            digit_count = 0;
        }
        // Invalid character — flush rest of line and fail
        else {
            drain_rx_until_newline();
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
        // Explicit Arduino Servo defaults (544–2400 µs) — no behavior change,
        // but makes the pulse range visible. Per-servo µs calibration
        // (SG90 nominal 500–2400 µs) is future work.
        servos[i].attach(SERVO_PINS[i], 544, 2400);
    }

    apply_movement(actual_positions);
}


void loop() {
    // Failsafe: park at home pose once per timeout period when no valid
    // frame has arrived (host died, cable unplugged, etc.).
    if (millis() - last_command_ms > HOLD_TIMEOUT_MS && !parked) {
        int park_positions[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
        apply_movement(park_positions);
        parked = true;
    }

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
        last_command_ms = millis();
        parked = false;
        Serial.println(F("OK"));
    } else {
        Serial.println(F("ERR"));
    }
}
