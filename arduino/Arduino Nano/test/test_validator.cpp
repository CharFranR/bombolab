#include <unity.h>
#include "../src/validator.h"

void setUp() {}
void tearDown() {}

static void fill(uint16_t j[V2_JOINT_COUNT], uint16_t v) {
    for (int i = 0; i < V2_JOINT_COUNT; i++) j[i] = v;
}

static void test_begin_and_valid_sequence() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 3, 100000);
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 0));
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 50000));
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 50000));
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_end(&v));
}

static void test_first_dt_must_be_zero() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 2, 100000);
    TEST_ASSERT_EQUAL(V2_ERR_BAD_DT, v2_validator_sample(&v, j, 50000));
}

static void test_zero_dt_after_first_rejected() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 3, 100000);
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 0));
    TEST_ASSERT_EQUAL(V2_ERR_BAD_DT, v2_validator_sample(&v, j, 0));
}

static void test_out_of_range_joint_rejected() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 2, 100000);
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 0));
    j[0] = 3000;
    TEST_ASSERT_EQUAL(V2_ERR_OUT_OF_RANGE, v2_validator_sample(&v, j, 50000));
}

static void test_count_mismatch_on_end() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 3, 100000);
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 0));
    TEST_ASSERT_EQUAL(V2_ERR_COUNT_MISMATCH, v2_validator_end(&v));
}

static void test_duration_over_tolerance_early() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 2, 5000);
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 0));
    TEST_ASSERT_EQUAL(V2_ERR_DURATION_MISMATCH, v2_validator_sample(&v, j, 20000));
}

static void test_duration_within_tolerance() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 2, 5000);
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 0));
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 5900));
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_end(&v));
}

static void test_duration_under_tolerance() {
    V2Validator v;
    uint16_t j[V2_JOINT_COUNT];
    fill(j, 1500);
    v2_validator_begin(&v, 2, 5000);
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 0));
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_sample(&v, j, 4100));
    TEST_ASSERT_EQUAL(V2_OK, v2_validator_end(&v));
}

static void test_tolerance_floor_is_1000us() {
    V2Validator v;
    v2_validator_begin(&v, 2, 500);
    TEST_ASSERT_EQUAL(1000u, v2_validator_tolerance_us(&v));
}

static void test_tolerance_percent() {
    V2Validator v;
    v2_validator_begin(&v, 2, 1000000);
    TEST_ASSERT_EQUAL(10000u, v2_validator_tolerance_us(&v));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_begin_and_valid_sequence);
    RUN_TEST(test_first_dt_must_be_zero);
    RUN_TEST(test_zero_dt_after_first_rejected);
    RUN_TEST(test_out_of_range_joint_rejected);
    RUN_TEST(test_count_mismatch_on_end);
    RUN_TEST(test_duration_over_tolerance_early);
    RUN_TEST(test_duration_within_tolerance);
    RUN_TEST(test_duration_under_tolerance);
    RUN_TEST(test_tolerance_floor_is_1000us);
    RUN_TEST(test_tolerance_percent);
    return UNITY_END();
}
