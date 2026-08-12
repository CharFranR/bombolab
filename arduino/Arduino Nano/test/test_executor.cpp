#include <unity.h>
#include "../src/executor.h"

static uint32_t g_now = 0;
static uint16_t g_applied[V2_JOINT_COUNT];
static int g_apply_count = 0;
static int g_ack_count = 0;
static uint8_t g_last_ack = 0;
static int g_trace_count = 0;
static uint32_t g_last_trace_t = 0;

static uint32_t fake_now(void) { return g_now; }

static void fake_apply(const uint16_t* joints) {
    for (int i = 0; i < V2_JOINT_COUNT; i++) g_applied[i] = joints[i];
    g_apply_count++;
}

static void fake_ack(uint8_t free_slots) {
    g_ack_count++;
    g_last_ack = free_slots;
}

static void fake_trace(uint32_t t_us, const uint16_t* joints) {
    g_trace_count++;
    g_last_trace_t = t_us;
    for (int i = 0; i < V2_JOINT_COUNT; i++) g_applied[i] = joints[i];
}

void setUp() {
    g_now = 0;
    g_apply_count = 0;
    g_ack_count = 0;
    g_last_ack = 0;
    g_trace_count = 0;
    g_last_trace_t = 0;
}

void tearDown() {}

static void fill(uint16_t j[V2_JOINT_COUNT], uint16_t v) {
    for (int i = 0; i < V2_JOINT_COUNT; i++) j[i] = v;
}

static void test_pacing_follows_dt_of_next_sample() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 0));
    j[0] = 1600;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    j[0] = 1700;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    e.declared_total = 3;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_start(&e));
    g_now = 1000;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(1, g_apply_count);
    TEST_ASSERT_EQUAL(1500, g_applied[0]);
    g_now = 51000;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(2, g_apply_count);
    TEST_ASSERT_EQUAL(1600, g_applied[0]);
    g_now = 101000;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(3, g_apply_count);
    TEST_ASSERT_EQUAL(1700, g_applied[0]);
}

static void test_first_sample_applies_at_start() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 0));
    e.declared_total = 1;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_start(&e));
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(1, g_apply_count);
    TEST_ASSERT_EQUAL(1, g_ack_count);
}

static void test_ring_full_rejects_store() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    for (int i = 0; i < V2_RING_SIZE; i++) {
        TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 1000));
    }
    TEST_ASSERT_EQUAL(V2_ERR_BAD_STATE, v2_executor_store(&e, j, 1000));
}

static void test_consume_frees_slots_and_finishes() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    for (int i = 0; i < V2_RING_SIZE; i++) {
        TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 1000));
    }
    e.declared_total = V2_RING_SIZE;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_start(&e));
    g_now = 1000000;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(V2_RING_SIZE, g_apply_count);
    TEST_ASSERT_EQUAL(V2_RING_SIZE, g_ack_count);
    TEST_ASSERT_EQUAL(V2_RING_SIZE, v2_executor_free_slots(&e));
    TEST_ASSERT_TRUE(v2_executor_finished(&e));
}

static void test_not_finished_while_samples_pending() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 0));
    j[0] = 1600;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    e.declared_total = 2;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_start(&e));
    g_now = 500;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(1, g_apply_count);
    TEST_ASSERT_FALSE(v2_executor_finished(&e));
    g_now = 51000;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(2, g_apply_count);
    TEST_ASSERT_TRUE(v2_executor_finished(&e));
}

static void test_stop_holds_and_discard_resets() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 0));
    v2_executor_stop(&e);
    v2_executor_discard(&e);
    TEST_ASSERT_EQUAL(0, e.in_use);
    TEST_ASSERT_EQUAL(V2_RING_SIZE, v2_executor_free_slots(&e));
}

static void test_wrap_safe_delta() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 0));
    j[0] = 1600;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    e.declared_total = 2;
    g_now = 4294967000u;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_start(&e));
    g_now = 100u;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(1, g_apply_count);
}

static void test_trace_hook_fires_with_elapsed_time() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    v2_executor_set_trace(&e, fake_trace);
    fill(j, 1500);
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 0));
    j[0] = 1600;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    e.declared_total = 2;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_start(&e));
    g_now = 51000;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(2, g_trace_count);
    TEST_ASSERT_EQUAL(51000u, g_last_trace_t);
    TEST_ASSERT_EQUAL(1600, g_applied[0]);
}

static void test_pacing_survives_ring_empty() {
    V2Executor e;
    uint16_t j[V2_JOINT_COUNT];
    v2_executor_init(&e, fake_now, fake_apply, fake_ack);
    fill(j, 1500);
    // Fase 1: cargar y ejecutar 2 muestras (dt 0 y 50000) → el ring se vacía
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 0));
    j[0] = 1600;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    e.declared_total = 4;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_start(&e));
    g_now = 50000;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(2, g_apply_count);
    TEST_ASSERT_EQUAL(0, e.in_use); // ring vacío → target_time queda stale
    // Fase 2: refill MUCHO después (el web tardó: ventana de lectura de 3s).
    // Con target_time stale el tick consumiría AMBAS muestras de una (modo
    // ráfaga → el brazo "lata" y los ACK densos desbordan el ring web-side).
    g_now = 200000;
    j[0] = 1700;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    j[0] = 1800;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_store(&e, j, 50000));
    // Tick inmediato tras el refill: solo la primera se consume ya ("está
    // atrasada"); la segunda espera su dt de 50000us.
    g_now = 200100;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(3, g_apply_count);
    TEST_ASSERT_EQUAL(1700, g_applied[0]);
    TEST_ASSERT_EQUAL(1, e.in_use);
    g_now = 250100;
    TEST_ASSERT_EQUAL(V2_OK, v2_executor_tick(&e));
    TEST_ASSERT_EQUAL(4, g_apply_count);
    TEST_ASSERT_EQUAL(1800, g_applied[0]);
    TEST_ASSERT_TRUE(v2_executor_finished(&e));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pacing_follows_dt_of_next_sample);
    RUN_TEST(test_first_sample_applies_at_start);
    RUN_TEST(test_ring_full_rejects_store);
    RUN_TEST(test_consume_frees_slots_and_finishes);
    RUN_TEST(test_not_finished_while_samples_pending);
    RUN_TEST(test_stop_holds_and_discard_resets);
    RUN_TEST(test_wrap_safe_delta);
    RUN_TEST(test_trace_hook_fires_with_elapsed_time);
    RUN_TEST(test_pacing_survives_ring_empty);
    return UNITY_END();
}
