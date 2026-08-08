#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "validator.h"

#define V2_CHUNK_MAX 24
#define V2_RING_SIZE (2 * V2_CHUNK_MAX)

typedef struct {
    uint16_t joints[V2_RING_SIZE][V2_JOINT_COUNT];
    uint32_t dt[V2_RING_SIZE];
    uint8_t head;
    uint8_t play;
    uint8_t in_use;
    uint16_t consumed_total;
    uint16_t declared_total;
    bool running;
    uint32_t exec_start;
    uint32_t target_time;
    uint32_t (*now)(void);
    void (*apply)(const uint16_t* joints);
    void (*ack)(uint8_t free_slots);
} V2Executor;

void v2_executor_init(V2Executor* e,
                      uint32_t (*now)(void),
                      void (*apply)(const uint16_t* joints),
                      void (*ack)(uint8_t free_slots));
V2Result v2_executor_store(V2Executor* e, const uint16_t* joints, uint32_t dt_us);
V2Result v2_executor_start(V2Executor* e);
V2Result v2_executor_tick(V2Executor* e);
void v2_executor_stop(V2Executor* e);
void v2_executor_discard(V2Executor* e);
uint8_t v2_executor_free_slots(const V2Executor* e);
bool v2_executor_finished(const V2Executor* e);
