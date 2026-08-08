#include "protocol_v2.h"

static V2Protocol* g_active = 0;

static void reply_ack(V2Protocol* p, uint32_t k);

static void executor_ack(uint8_t free_slots) {
    if (g_active) {
        reply_ack(g_active, free_slots);
    }
}

static const char* V2_STATE_NAMES[4] = {"IDLE", "RECEIVING", "READY", "RUNNING"};
static const char* V2_TOKEN_NAMES[8] = {
    "BAD_LINE", "OUT_OF_RANGE", "BAD_DT", "BAD_STATE",
    "COUNT_MISMATCH", "DURATION_MISMATCH", "NOT_READY", "UNKNOWN"
};

static void u32_to_str(uint32_t v, char* out) {
    char tmp[11];
    int i = 0;
    do {
        tmp[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v > 0);
    int j = 0;
    while (i > 0) {
        out[j++] = tmp[--i];
    }
    out[j] = '\0';
}

static void reply_ack(V2Protocol* p, uint32_t k) {
    if (!p->reply) return;
    char buf[16];
    buf[0] = 'A';
    buf[1] = 'C';
    buf[2] = 'K';
    buf[3] = ' ';
    u32_to_str(k, buf + 4);
    p->reply(buf);
}

static void reply_err(V2Protocol* p, V2Result r) {
    if (!p->reply) return;
    char buf[24];
    buf[0] = 'E';
    buf[1] = 'R';
    buf[2] = 'R';
    buf[3] = ' ';
    const char* token = V2_TOKEN_NAMES[(int)r - 1];
    int i = 0;
    while (token[i] != '\0' && i < 19) {
        buf[4 + i] = token[i];
        i++;
    }
    buf[4 + i] = '\0';
    p->reply(buf);
}

static const char* skip_spaces(const char* s) {
    while (*s == ' ') s++;
    return s;
}

static bool parse_u32_field(const char** pp, uint32_t* out, int max_digits, bool* overflowed) {
    const char* s = skip_spaces(*pp);
    if (*s < '0' || *s > '9') return false;
    uint32_t v = 0;
    int digits = 0;
    while (*s >= '0' && *s <= '9') {
        digits++;
        if (digits > max_digits) {
            if (overflowed) *overflowed = true;
            return false;
        }
        uint32_t d = (uint32_t)(*s - '0');
        if (v > (V2_DT_MAX - d) / 10u) {
            if (overflowed) *overflowed = true;
            return false;
        }
        v = v * 10u + d;
        s++;
    }
    *pp = s;
    *out = v;
    return true;
}

static bool line_has_word(const char** pp, const char* word) {
    const char* s = skip_spaces(*pp);
    int i = 0;
    while (word[i] != '\0') {
        if (s[i] != word[i]) return false;
        i++;
    }
    if (s[i] != ' ' && s[i] != '\0') return false;
    *pp = s + i;
    return true;
}

static bool line_only_trailing_spaces(const char* s) {
    s = skip_spaces(s);
    return *s == '\0';
}

void v2_init(V2Protocol* p,
             uint32_t (*now)(void),
             void (*apply)(const uint16_t* joints),
             void (*reply)(const char* line)) {
    g_active = p;
    p->state = V2_STATE_IDLE;
    p->manifest_ready = false;
    p->reply = reply;
    v2_executor_init(&p->executor, now, apply, executor_ack);
}

V2State v2_state(const V2Protocol* p) {
    return p->state;
}

static void discard_to_idle(V2Protocol* p) {
    v2_executor_discard(&p->executor);
    p->manifest_ready = false;
    p->state = V2_STATE_IDLE;
}

void v2_abort(V2Protocol* p) {
    discard_to_idle(p);
}

static bool handle_hello(V2Protocol* p, const char* line) {
    const char* pp = line;
    uint32_t ver;
    if (!line_has_word(&pp, "HELLO") ||
        !parse_u32_field(&pp, &ver, 2, 0) || ver != 2 || !line_only_trailing_spaces(pp)) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    if (p->state != V2_STATE_IDLE) {
        reply_err(p, V2_ERR_BAD_STATE);
        discard_to_idle(p);
        return false;
    }
    p->state = V2_STATE_RECEIVING;
    if (p->reply) p->reply("HELLO 2 OK CHUNK_MAX 24");
    return true;
}

static bool handle_manifest(V2Protocol* p, const char* line) {
    const char* pp = line;
    if (!line_has_word(&pp, "MANIFEST")) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    if (p->state == V2_STATE_IDLE || p->state == V2_STATE_RUNNING) {
        reply_err(p, V2_ERR_BAD_STATE);
        discard_to_idle(p);
        return false;
    }
    uint32_t count, duration;
    if (!parse_u32_field(&pp, &count, 5, 0) || count == 0 || count > 65535u ||
        !parse_u32_field(&pp, &duration, 10, 0) || !line_only_trailing_spaces(pp)) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    v2_executor_discard(&p->executor);
    v2_validator_begin(&p->validator, (uint16_t)count, duration);
    p->executor.declared_total = (uint16_t)count;
    p->executor.consumed_total = 0;
    p->manifest_ready = true;
    return true;
}

static bool handle_sample(V2Protocol* p, const char* line) {
    const char* pp = line;
    if (!line_has_word(&pp, "SAMPLE")) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    if (p->state == V2_STATE_IDLE || !p->manifest_ready) {
        reply_err(p, V2_ERR_BAD_STATE);
        discard_to_idle(p);
        return false;
    }
    uint16_t joints[V2_JOINT_COUNT];
    for (int i = 0; i < V2_JOINT_COUNT; i++) {
        uint32_t j;
        if (!parse_u32_field(&pp, &j, 4, 0) || j > 2400u) {
            reply_err(p, V2_ERR_BAD_LINE);
            discard_to_idle(p);
            return false;
        }
        joints[i] = (uint16_t)j;
    }
    uint32_t dt;
    bool dt_overflow = false;
    if (!parse_u32_field(&pp, &dt, 10, &dt_overflow) || !line_only_trailing_spaces(pp)) {
        reply_err(p, dt_overflow ? V2_ERR_BAD_DT : V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    V2Result r = v2_validator_sample(&p->validator, joints, dt);
    if (r != V2_OK) {
        reply_err(p, r);
        discard_to_idle(p);
        return false;
    }
    r = v2_executor_store(&p->executor, joints, dt);
    if (r != V2_OK) {
        reply_err(p, r);
        discard_to_idle(p);
        return false;
    }
    uint32_t received = p->validator.received_count;
    if (received % V2_CHUNK_MAX == 0) {
        uint8_t free = v2_executor_free_slots(&p->executor);
        if (free > 0) {
            reply_ack(p, free);
        }
    }
    return true;
}

static bool handle_end_upload(V2Protocol* p, const char* line) {
    const char* pp = line;
    if (!line_has_word(&pp, "END_UPLOAD") || !line_only_trailing_spaces(pp)) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    if (p->state == V2_STATE_IDLE || !p->manifest_ready) {
        reply_err(p, V2_ERR_BAD_STATE);
        discard_to_idle(p);
        return false;
    }
    V2Result r = v2_validator_end(&p->validator);
    if (r != V2_OK) {
        reply_err(p, r);
        discard_to_idle(p);
        return false;
    }
    reply_ack(p, p->validator.received_count);
    p->state = V2_STATE_READY;
    return true;
}

static bool handle_execute(V2Protocol* p, const char* line) {
    const char* pp = line;
    if (!line_has_word(&pp, "EXECUTE") || !line_only_trailing_spaces(pp)) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    if (p->state == V2_STATE_IDLE) {
        reply_err(p, V2_ERR_BAD_STATE);
        discard_to_idle(p);
        return false;
    }
    if (p->state == V2_STATE_RUNNING) {
        reply_err(p, V2_ERR_NOT_READY);
        discard_to_idle(p);
        return false;
    }
    if (p->validator.received_count == 0) {
        reply_err(p, V2_ERR_NOT_READY);
        discard_to_idle(p);
        return false;
    }
    if (v2_executor_start(&p->executor) != V2_OK) {
        reply_err(p, V2_ERR_NOT_READY);
        discard_to_idle(p);
        return false;
    }
    p->state = V2_STATE_RUNNING;
    return true;
}

static bool handle_stop(V2Protocol* p, const char* line) {
    const char* pp = line;
    if (!line_has_word(&pp, "STOP") || !line_only_trailing_spaces(pp)) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    if (p->state == V2_STATE_IDLE) {
        return false;
    }
    v2_executor_stop(&p->executor);
    v2_executor_discard(&p->executor);
    p->manifest_ready = false;
    p->state = V2_STATE_IDLE;
    return true;
}

static bool handle_status(V2Protocol* p, const char* line) {
    const char* pp = line;
    if (!line_has_word(&pp, "STATUS") || !line_only_trailing_spaces(pp)) {
        reply_err(p, V2_ERR_BAD_LINE);
        discard_to_idle(p);
        return false;
    }
    if (!p->reply) return true;
    char buf[20];
    buf[0] = 'S';
    buf[1] = 'T';
    buf[2] = 'A';
    buf[3] = 'T';
    buf[4] = 'U';
    buf[5] = 'S';
    buf[6] = ' ';
    const char* name = V2_STATE_NAMES[(int)p->state];
    int i = 0;
    while (name[i] != '\0' && i < 12) {
        buf[7 + i] = name[i];
        i++;
    }
    buf[7 + i] = '\0';
    p->reply(buf);
    return true;
}

bool v2_process_line(V2Protocol* p, const char* line) {
    const char* pp = line;
    pp = skip_spaces(pp);
    if (*pp == '\0') {
        return false;
    }
    if (line_has_word(&pp, "HELLO")) {
        return handle_hello(p, line);
    }
    if (line_has_word(&pp, "MANIFEST")) {
        return handle_manifest(p, line);
    }
    if (line_has_word(&pp, "SAMPLE")) {
        return handle_sample(p, line);
    }
    if (line_has_word(&pp, "END_UPLOAD")) {
        return handle_end_upload(p, line);
    }
    if (line_has_word(&pp, "EXECUTE")) {
        return handle_execute(p, line);
    }
    if (line_has_word(&pp, "STOP")) {
        return handle_stop(p, line);
    }
    if (line_has_word(&pp, "STATUS")) {
        return handle_status(p, line);
    }
    reply_err(p, V2_ERR_BAD_LINE);
    discard_to_idle(p);
    return false;
}

bool v2_tick(V2Protocol* p) {
    if (p->state != V2_STATE_RUNNING) {
        return false;
    }
    uint16_t before = p->executor.consumed_total;
    v2_executor_tick(&p->executor);
    bool consumed = p->executor.consumed_total > before;
    if (v2_executor_finished(&p->executor)) {
        p->executor.running = false;
        p->state = V2_STATE_IDLE;
        p->manifest_ready = false;
        if (p->reply) p->reply("DONE");
    }
    return consumed;
}
