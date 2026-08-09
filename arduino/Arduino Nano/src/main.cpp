#include <Arduino.h>
#include <Servo.h>
#include "protocol_v2.h"

// Failsafe (watchdog): if no valid frame is received within HOLD_TIMEOUT_MS,
// the servos are parked at the home pose (90,90,81,95,60,90) once per timeout
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
// VERIFICAR contra el cableado físico real
const int SERVO_PINS[NUM_SERVOS] = {A1, A0, A2, A4, 13, A5};

// Wire units (auto-detected per frame):
//   all values in [5, 175]     → degrees, written with servo.write()
//   all values in [500, 2400]  → microseconds, written with
//                                writeMicroseconds() (0.1° resolution —
//                                the web viewer sends µs so the drawing
//                                granularity is not limited to 1°)
// Mixed frames are rejected.
const int DEG_MIN = 5;
const int DEG_MAX = 175;
const int US_MIN = 500;
const int US_MAX = 2400;

Servo servos[NUM_SERVOS];

// Home pose (µs): 90°, 90°, 81°, 95°, 60°, 90°
int actual_positions[NUM_SERVOS] = {1472, 1472, 1379, 1524, 1163, 1472};

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

            // Range validation: all values in ONE unit mode —
            // degrees [5,175] OR microseconds [500,2400]; mixed frames fail.
            bool all_deg = true;
            bool all_us = true;
            for (int i = 0; i < NUM_SERVOS; i++) {
                if (positions[i] < DEG_MIN || positions[i] > DEG_MAX) all_deg = false;
                if (positions[i] < US_MIN || positions[i] > US_MAX) all_us = false;
            }
            if (!all_deg && !all_us) return false;
            return true;
        }

        // Skip carriage returns (CRLF tolerance)
        if (c == '\r') continue;

        // Digit — accumulate the current value
        if (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            has_digit = true;
            digit_count++;
            // No value can have more than 4 digits (max 2400). Rejecting here
            // also prevents a 5-digit field from wrapping the 16-bit range.
            if (digit_count > 4) {
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
    // Auto-detect units per frame: degrees (all ≤175) or microseconds
    // (all ≥500). Mixed frames never reach here (rejected by the parser).
    bool all_deg = true;
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (positions[i] > DEG_MAX) { all_deg = false; break; }
    }
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (all_deg) {
            servos[i].write(positions[i]);
        } else {
            servos[i].writeMicroseconds(positions[i]);
        }
    }
}

V2Protocol g_v2;

static void apply_v2_servos(const uint16_t* joints) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].writeMicroseconds(joints[i]);
    }
}

static void reply_v2(const char* line) {
    Serial.println(line);
}

static void trace_v2(uint32_t t_us, const uint16_t* joints) {
    char buf[64];
    char tmp[11];
    int n = 0;
    unsigned long v = t_us;
    int i = 0;
    do {
        tmp[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v > 0);
    buf[n++] = 'T';
    buf[n++] = ' ';
    while (i > 0) buf[n++] = tmp[--i];
    for (int j = 0; j < NUM_SERVOS; j++) {
        buf[n++] = ' ';
        unsigned w = joints[j];
        int k = 0;
        do {
            tmp[k++] = (char)('0' + (w % 10u));
            w /= 10u;
        } while (w > 0);
        while (k > 0) buf[n++] = tmp[--k];
    }
    buf[n] = '\0';
    Serial.println(buf);
}

static char g_v2_line[64];
static int g_v2_line_len = 0;
static unsigned long g_v2_line_start = 0;

static bool v2_pump_line(char* out, int cap) {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            for (int i = 0; i < g_v2_line_len; i++) {
                out[i] = g_v2_line[i];
            }
            out[g_v2_line_len] = '\0';
            bool ok = g_v2_line_len > 0;
            g_v2_line_len = 0;
            return ok;
        }
        if (c == '\r') {
            continue;
        }
        if (g_v2_line_len == 0) {
            g_v2_line_start = millis();
        }
        if (g_v2_line_len >= cap - 1) {
            g_v2_line_len = 0;
            drain_rx_until_newline();
            return false;
        }
        g_v2_line[g_v2_line_len++] = c;
    }
    if (g_v2_line_len > 0 && millis() - g_v2_line_start > 100) {
        g_v2_line_len = 0;
        drain_rx_until_newline();
    }
    return false;
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
    v2_init(&g_v2, micros, apply_v2_servos, reply_v2);
    v2_executor_set_trace(&g_v2.executor, trace_v2);
    Serial.println("FW V2.1");
}


void loop() {
    // Failsafe: park at home pose once per timeout period when no valid
    // frame has arrived (host died, cable unplugged, etc.).
    if (millis() - last_command_ms > HOLD_TIMEOUT_MS && !parked) {
        int park_positions[NUM_SERVOS] = {1472, 1472, 1379, 1524, 1163, 1472};
        apply_movement(park_positions);
        v2_abort(&g_v2);
        parked = true;
    }

    bool activity = false;
    char line[64];

    if (v2_state(&g_v2) == V2_STATE_IDLE) {
        if (g_v2_line_len > 0) {
            while (v2_pump_line(line, sizeof(line))) {
                activity = v2_process_line(&g_v2, line) || activity;
            }
        } else if (Serial.available()) {
        if (Serial.peek() == '\n' || Serial.peek() == '\r') {
            Serial.read();
        } else if (Serial.peek() >= '0' && Serial.peek() <= '9') {
            int new_positions[NUM_SERVOS];

            if (read_positions_serial(new_positions)) {
                apply_movement(new_positions);
                activity = true;
                Serial.println(F("OK"));
            } else {
                Serial.println(F("ERR"));
            }
        } else {
            while (v2_pump_line(line, sizeof(line))) {
                activity = v2_process_line(&g_v2, line) || activity;
            }
        }
        }
    } else if (v2_state(&g_v2) != V2_STATE_IDLE) {
        while (v2_pump_line(line, sizeof(line))) {
            activity = v2_process_line(&g_v2, line) || activity;
        }
    }

    activity = v2_tick(&g_v2) || activity;

    if (activity) {
        last_command_ms = millis();
        parked = false;
    }
}
