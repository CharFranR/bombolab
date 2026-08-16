#include "executor.h"

void v2_executor_init(V2Executor* e,
                      uint32_t (*now)(void),
                      void (*apply)(const uint16_t* joints),
                      void (*ack)(uint8_t free_slots)) {
    e->head = 0;
    e->play = 0;
    e->in_use = 0;
    e->consumed_total = 0;
    e->declared_total = 0;
    e->running = false;
    e->exec_start = 0;
    e->target_time = 0;
    e->now = now;
    e->apply = apply;
    e->ack = ack;
    e->trace = 0;
}

void v2_executor_set_trace(V2Executor* e, void (*trace)(uint32_t t_us, const uint16_t* joints)) {
    e->trace = trace;
}

uint8_t v2_executor_free_slots(const V2Executor* e) {
    return (uint8_t)(V2_RING_SIZE - e->in_use);
}

V2Result v2_executor_store(V2Executor* e, const uint16_t* joints, uint32_t dt_us) {
    if (e->in_use == V2_RING_SIZE) {
        return V2_ERR_BAD_STATE;
    }
    bool was_empty = (e->in_use == 0);
    for (int i = 0; i < V2_JOINT_COUNT; i++) {
        e->joints[e->head][i] = joints[i];
    }
    e->dt[e->head] = dt_us;
    e->head = (uint8_t)((e->head + 1) % V2_RING_SIZE);
    e->in_use++;
    if (was_empty && e->running) {
        // Re-anclar el pacing tras un ring vacío: v2_executor_tick deja de
        // avanzar target_time cuando in_use llega a 0 (no hay dt siguiente), y
        // si el refill llega MUCHO después, el próximo tick consume TODAS las
        // muestras almacenadas de una (modo ráfaga): el brazo "lata" los
        // samples y los ACK densos que emite desbordan el ring web-side.
        // La muestra recién almacenada se consume ya (el timeline está
        // atrasado de todas formas) y las siguientes respetan su dt.
        e->target_time = (uint32_t)(e->now() - e->exec_start);
    }
    return V2_OK;
}

V2Result v2_executor_start(V2Executor* e) {
    if (e->in_use == 0) {
        return V2_ERR_NOT_READY;
    }
    e->running = true;
    e->exec_start = e->now();
    e->target_time = 0;
    return V2_OK;
}

V2Result v2_executor_tick(V2Executor* e) {
    if (!e->running) {
        return V2_OK;
    }
    while (e->in_use > 0) {
        uint32_t delta = (uint32_t)(e->now() - e->exec_start);
        if (delta < e->target_time) {
            break;
        }
        uint8_t idx = e->play;
        e->play = (uint8_t)((e->play + 1) % V2_RING_SIZE);
        e->in_use--;
        e->consumed_total++;
        e->apply(e->joints[idx]);
        if (e->trace) {
            e->trace((uint32_t)(e->now() - e->exec_start), e->joints[idx]);
        }
        if (e->ack) {
            e->ack(v2_executor_free_slots(e));
        }
        if (e->in_use > 0) {
            e->target_time += e->dt[(uint8_t)((idx + 1) % V2_RING_SIZE)];
        }
    }
    return V2_OK;
}

void v2_executor_stop(V2Executor* e) {
    e->running = false;
}

void v2_executor_discard(V2Executor* e) {
    e->head = 0;
    e->play = 0;
    e->in_use = 0;
    e->running = false;
    e->target_time = 0;
}

bool v2_executor_finished(const V2Executor* e) {
    return e->running && e->in_use == 0 && e->consumed_total >= e->declared_total;
}
