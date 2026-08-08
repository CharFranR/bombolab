#pragma once
#include <stdint.h>
#include <stdbool.h>

#define V2_JOINT_COUNT 6
#define V2_US_MIN 500u
#define V2_US_MAX 2400u
#define V2_DT_MAX 4294967295u

typedef enum {
    V2_OK = 0,
    V2_ERR_BAD_LINE,
    V2_ERR_OUT_OF_RANGE,
    V2_ERR_BAD_DT,
    V2_ERR_BAD_STATE,
    V2_ERR_COUNT_MISMATCH,
    V2_ERR_DURATION_MISMATCH,
    V2_ERR_NOT_READY
} V2Result;

typedef struct {
    uint16_t declared_count;
    uint32_t declared_duration_us;
    uint32_t received_count;
    uint32_t sum_dt_us;
    bool saw_first;
} V2Validator;

void v2_validator_begin(V2Validator* v, uint16_t count, uint32_t duration_us);
V2Result v2_validator_sample(V2Validator* v, const uint16_t joints[V2_JOINT_COUNT], uint32_t dt_us);
V2Result v2_validator_end(V2Validator* v);
uint32_t v2_validator_tolerance_us(const V2Validator* v);
