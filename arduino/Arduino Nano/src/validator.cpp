#include "validator.h"

uint32_t v2_validator_tolerance_us(const V2Validator* v) {
    uint32_t pct = v->declared_duration_us / 100u;
    return pct > 1000u ? pct : 1000u;
}

void v2_validator_begin(V2Validator* v, uint16_t count, uint32_t duration_us) {
    v->declared_count = count;
    v->declared_duration_us = duration_us;
    v->received_count = 0;
    v->sum_dt_us = 0;
    v->saw_first = false;
}

V2Result v2_validator_sample(V2Validator* v, const uint16_t joints[V2_JOINT_COUNT], uint32_t dt_us) {
    for (int i = 0; i < V2_JOINT_COUNT; i++) {
        if (joints[i] < V2_US_MIN || joints[i] > V2_US_MAX) {
            return V2_ERR_OUT_OF_RANGE;
        }
    }
    if (!v->saw_first) {
        if (dt_us != 0) {
            return V2_ERR_BAD_DT;
        }
        v->saw_first = true;
    } else if (dt_us == 0) {
        return V2_ERR_BAD_DT;
    }
    if (dt_us > (V2_DT_MAX - v->sum_dt_us)) {
        return V2_ERR_DURATION_MISMATCH;
    }
    v->sum_dt_us += dt_us;
    if (v->sum_dt_us > v->declared_duration_us) {
        uint32_t tol = v2_validator_tolerance_us(v);
        if (v->sum_dt_us - v->declared_duration_us > tol) {
            return V2_ERR_DURATION_MISMATCH;
        }
    }
    v->received_count++;
    return V2_OK;
}

V2Result v2_validator_end(V2Validator* v) {
    if (v->received_count != v->declared_count) {
        return V2_ERR_COUNT_MISMATCH;
    }
    uint32_t tol = v2_validator_tolerance_us(v);
    if (v->sum_dt_us > v->declared_duration_us) {
        if (v->sum_dt_us - v->declared_duration_us > tol) {
            return V2_ERR_DURATION_MISMATCH;
        }
    } else if (v->declared_duration_us - v->sum_dt_us > tol) {
        return V2_ERR_DURATION_MISMATCH;
    }
    return V2_OK;
}
