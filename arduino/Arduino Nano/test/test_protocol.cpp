#include <unity.h>
#include <string.h>
#include "../src/protocol_v2.h"

static uint32_t g_now = 0;
static uint16_t g_applied[V2_JOINT_COUNT];
static int g_apply_count = 0;
static char g_replies[16][40];
static int g_reply_count = 0;
static V2Protocol g_proto;

static uint32_t fake_now(void) { return g_now; }

static void fake_apply(const uint16_t* joints) {
    for (int i = 0; i < V2_JOINT_COUNT; i++) g_applied[i] = joints[i];
    g_apply_count++;
}

static void fake_reply(const char* line) {
    if (g_reply_count < 16) {
        strncpy(g_replies[g_reply_count], line, 39);
    }
    g_reply_count++;
}

static const char* last_reply(void) {
    if (g_reply_count == 0) return "";
    return g_replies[g_reply_count - 1];
}

static char sample_line[64];

static void make_sample(uint16_t v, unsigned long dt) {
    char tmp[8];
    sample_line[0] = '\0';
    strcat(sample_line, "SAMPLE ");
    for (int i = 0; i < V2_JOINT_COUNT; i++) {
        if (i > 0) strcat(sample_line, " ");
        sprintf(tmp, "%u", (unsigned)v);
        strcat(sample_line, tmp);
    }
    strcat(sample_line, " ");
    sprintf(tmp, "%lu", (unsigned long)dt);
    strcat(sample_line, tmp);
}

void setUp() {
    g_now = 0;
    g_apply_count = 0;
    g_reply_count = 0;
    v2_init(&g_proto, fake_now, fake_apply, fake_reply);
}

void tearDown() {}

static void test_hello_handshake() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_EQUAL(V2_STATE_RECEIVING, v2_state(&g_proto));
    TEST_ASSERT_EQUAL_STRING("HELLO 2 OK CHUNK_MAX 24", last_reply());
}

static void test_hello_with_wrong_version() {
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, "HELLO 1"));
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
    TEST_ASSERT_EQUAL_STRING("ERR BAD_LINE", last_reply());
}

static void test_sample_without_manifest() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    make_sample(1500, 0);
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
    TEST_ASSERT_EQUAL_STRING("ERR BAD_STATE", last_reply());
}

static void test_happy_flow_with_done() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 3 100000"));
    make_sample(1500, 0);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    make_sample(1600, 50000);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    make_sample(1700, 50000);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "END_UPLOAD"));
    TEST_ASSERT_EQUAL(V2_STATE_READY, v2_state(&g_proto));
    TEST_ASSERT_EQUAL_STRING("ACK 3", last_reply());
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "EXECUTE"));
    TEST_ASSERT_EQUAL(V2_STATE_RUNNING, v2_state(&g_proto));
    TEST_ASSERT_TRUE(v2_tick(&g_proto));
    TEST_ASSERT_EQUAL(1, g_apply_count);
    TEST_ASSERT_EQUAL(1500, g_applied[0]);
    g_now = 51000;
    TEST_ASSERT_TRUE(v2_tick(&g_proto));
    TEST_ASSERT_EQUAL(2, g_apply_count);
    TEST_ASSERT_EQUAL(1600, g_applied[0]);
    g_now = 101000;
    TEST_ASSERT_TRUE(v2_tick(&g_proto));
    TEST_ASSERT_EQUAL(3, g_apply_count);
    TEST_ASSERT_EQUAL(1700, g_applied[0]);
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
    TEST_ASSERT_EQUAL_STRING("DONE", last_reply());
}

static void test_execute_before_end_upload() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 50000"));
    make_sample(1500, 0);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    make_sample(1600, 50000);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "EXECUTE"));
    TEST_ASSERT_EQUAL(V2_STATE_RUNNING, v2_state(&g_proto));
    g_now = 60000;
    TEST_ASSERT_TRUE(v2_tick(&g_proto));
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
}

static void test_execute_without_samples() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 50000"));
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, "EXECUTE"));
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
    TEST_ASSERT_EQUAL_STRING("ERR NOT_READY", last_reply());
}

static void test_execute_while_running() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 100000"));
    make_sample(1500, 0);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    make_sample(1600, 100000);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "EXECUTE"));
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, "EXECUTE"));
    TEST_ASSERT_EQUAL_STRING("ERR NOT_READY", last_reply());
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
}

static void test_stop_returns_to_idle() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 100000"));
    make_sample(1500, 0);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "STOP"));
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
}

static void test_status_reports_state() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "STATUS"));
    TEST_ASSERT_EQUAL_STRING("STATUS RECEIVING", last_reply());
}

static void test_garbage_line() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, "GARBAGE 1 2 3"));
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
    TEST_ASSERT_EQUAL_STRING("ERR BAD_LINE", last_reply());
}

static void test_first_sample_dt_nonzero() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 50000"));
    make_sample(1500, 50000);
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_EQUAL_STRING("ERR BAD_DT", last_reply());
}

static void test_chunk_ack_every_24() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 48 4700000"));
    for (int i = 0; i < 24; i++) {
        make_sample(1500, i == 0 ? 0 : 100000);
        TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    }
    TEST_ASSERT_EQUAL_STRING("ACK 24", last_reply());
    TEST_ASSERT_EQUAL(2, g_reply_count);
    for (int i = 24; i < 48; i++) {
        make_sample(1500, 100000);
        TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    }
    TEST_ASSERT_EQUAL(2, g_reply_count);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "EXECUTE"));
    TEST_ASSERT_EQUAL(V2_STATE_RUNNING, v2_state(&g_proto));
}

static void test_ring_overflow_bad_state() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 49 4800000"));
    for (int i = 0; i < 48; i++) {
        make_sample(1500, i == 0 ? 0 : 100000);
        TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    }
    make_sample(1500, 100000);
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_EQUAL_STRING("ERR BAD_STATE", last_reply());
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
}

static void test_duration_mismatch_on_end() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 100000"));
    make_sample(1500, 0);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    make_sample(1500, 500000);
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_EQUAL_STRING("ERR DURATION_MISMATCH", last_reply());
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
}

static void test_abort_resets_to_idle() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 100000"));
    make_sample(1500, 0);
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, sample_line));
    v2_abort(&g_proto);
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_EQUAL(V2_STATE_RECEIVING, v2_state(&g_proto));
}

static void test_dt_overflow_bad_dt() {
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "HELLO 2"));
    TEST_ASSERT_TRUE(v2_process_line(&g_proto, "MANIFEST 2 1000000"));
    make_sample(1500, 5000000000ul);
    TEST_ASSERT_FALSE(v2_process_line(&g_proto, sample_line));
    TEST_ASSERT_EQUAL_STRING("ERR BAD_DT", last_reply());
    TEST_ASSERT_EQUAL(V2_STATE_IDLE, v2_state(&g_proto));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_hello_handshake);
    RUN_TEST(test_hello_with_wrong_version);
    RUN_TEST(test_sample_without_manifest);
    RUN_TEST(test_happy_flow_with_done);
    RUN_TEST(test_execute_before_end_upload);
    RUN_TEST(test_execute_without_samples);
    RUN_TEST(test_execute_while_running);
    RUN_TEST(test_stop_returns_to_idle);
    RUN_TEST(test_status_reports_state);
    RUN_TEST(test_garbage_line);
    RUN_TEST(test_first_sample_dt_nonzero);
    RUN_TEST(test_chunk_ack_every_24);
    RUN_TEST(test_ring_overflow_bad_state);
    RUN_TEST(test_duration_mismatch_on_end);
    RUN_TEST(test_abort_resets_to_idle);
    RUN_TEST(test_dt_overflow_bad_dt);
    return UNITY_END();
}
