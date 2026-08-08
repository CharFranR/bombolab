#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "validator.h"
#include "executor.h"

typedef enum {
    V2_STATE_IDLE = 0,
    V2_STATE_RECEIVING,
    V2_STATE_READY,
    V2_STATE_RUNNING
} V2State;

typedef struct {
    V2State state;
    V2Validator validator;
    V2Executor executor;
    bool manifest_ready;
    void (*reply)(const char* line);
} V2Protocol;

void v2_init(V2Protocol* p,
             uint32_t (*now)(void),
             void (*apply)(const uint16_t* joints),
             void (*reply)(const char* line));
V2State v2_state(const V2Protocol* p);
bool v2_process_line(V2Protocol* p, const char* line);
bool v2_tick(V2Protocol* p);
