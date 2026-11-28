#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

extern "C" {
#include "trans_frame.h"
#include "trans_msg.h"
#include "trans_ringbuf.h"
}

namespace
{

int g_failures = 0;

#define CHECK(cond)                                                              \
    do                                                                           \
    {                                                                            \
        if (!(cond))                                                             \
        {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

#define CHECK_EQ(a, b)                                                           \
    do                                                                           \
    {                                                                            \
        const auto va = (a);                                                     \
        const auto vb = (b);                                                     \
        if (!(va == vb))                                                         \
        {                                                                        \
            std::printf("FAIL %s:%d: %s != %s (%lld vs %lld)\n", __FILE__,       \
                        __LINE__, #a, #b, (long long)va, (long long)vb);         \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                    \
    do                                                                           \
    {                                                                            \
        const double va = (a);                                                   \
        const double vb = (b);                                                   \
        if (std::fabs(va - vb) > (eps))                                          \
        {                                                                        \
            std::printf("FAIL %s:%d: %s != %s (%f vs %f)\n", __FILE__, __LINE__, \
                        #a, #b, va, vb);                                         \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

// 固定向量，由协议文档的字段表手工核对后固化。
const std::uint8_t kGimbalFrame[] = {
    0xFF, 0x05, 0x23, 0x19, 0x01, 0x10, 0x27, 0x00,
    0x00, 0x78, 0xEC, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFC, 0x0A,
    0x00, 0x00, 0xD3, 0xCE, 0xFE, 0xFF, 0xB8,
};

const std::uint8_t kShootFrame[] = {
    0xFF, 0x05, 0x24, 0x07, 0x01, 0x07, 0x15, 0xCD,
    0x5B, 0x07, 0x01, 0x7C, 0x91,
};

const std::uint8_t kVisionFrame[] = {
    0xFF, 0x05, 0x22, 0x11, 0x01, 0x10, 0x27, 0x00,
    0x00, 0x88, 0x13, 0x00, 0x00, 0x18, 0xFC, 0xFF,
    0xFF, 0x78, 0xEC, 0xFF, 0xFF, 0x7E, 0xCC,
};

const std::uint8_t kChassisFrame[] = {
    0xFF, 0x05, 0x12, 0x18, 0x10, 0x27, 0x00, 0x00,
    0x88, 0x13, 0x00, 0x00, 0xC4, 0x09, 0x00, 0x00,
    0xE8, 0x03, 0x00, 0x00, 0x30, 0xF8, 0xFF, 0xFF,
    0x20, 0x4E, 0x00, 0x00, 0x4C, 0x10,
};

std::vector<std::uint8_t> to_bytes(const std::uint8_t *data, std::size_t len)
{
    return std::vector<std::uint8_t>(data, data + len);
}

std::vector<trans_frame_t> feed_all(const std::vector<std::uint8_t> &bytes,
                                    trans_frame_parser_t *parser)
{
    std::vector<trans_frame_t> frames;
    for (std::uint8_t b : bytes)
    {
        trans_frame_t frame{};
        if (trans_frame_parser_feed(parser, b, &frame) != 0U)
        {
            frames.push_back(frame);
        }
    }
    return frames;
}

std::int32_t read_i32(const std::uint8_t *p)
{
    const std::uint32_t raw = static_cast<std::uint32_t>(p[0]) |
                              (static_cast<std::uint32_t>(p[1]) << 8U) |
                              (static_cast<std::uint32_t>(p[2]) << 16U) |
                              (static_cast<std::uint32_t>(p[3]) << 24U);
    return static_cast<std::int32_t>(raw);
}

void test_known_frames()
{
    trans_frame_parser_t parser;

    trans_frame_parser_init(&parser);
    auto frames = feed_all(to_bytes(kVisionFrame, sizeof(kVisionFrame)), &parser);
    CHECK_EQ(frames.size(), 1U);
    if (frames.size() == 1U)
    {
        trans_vision_cmd_t vision{};
        CHECK_EQ(trans_msg_parse_vision(&frames[0], &vision, 100U), 1U);
        CHECK_EQ(vision.mode, 1U);
        CHECK_EQ(vision.yaw_mdeg, 10000);
        CHECK_EQ(vision.yaw_vel_mdeg_s, 5000);
        CHECK_EQ(vision.yaw_acc_mdeg_s2, -1000);
        CHECK_EQ(vision.pitch_mdeg, -5000);
        CHECK_NEAR(vision.yaw_rad, 0.174532925f, 1e-6);
        CHECK_NEAR(vision.pitch_rad, -0.0872664626f, 1e-6);
        CHECK_EQ(vision.tick_ms, 100U);
        CHECK_EQ(vision.valid, 1U);
    }

    trans_frame_parser_init(&parser);
    frames = feed_all(to_bytes(kChassisFrame, sizeof(kChassisFrame)), &parser);
    CHECK_EQ(frames.size(), 1U);
    if (frames.size() == 1U)
    {
        trans_pc_chassis_t chassis{};
        CHECK_EQ(trans_msg_parse_pc_chassis(&frames[0], &chassis, 200U), 1U);
        CHECK_NEAR(chassis.vx_mps, 1.0f, 1e-6);
        CHECK_NEAR(chassis.vy_mps, -0.5f, 1e-6);
        CHECK_NEAR(chassis.vw_rps, -2.0f, 1e-6);
        CHECK_EQ(chassis.tick_ms, 200U);
        CHECK_EQ(chassis.valid, 1U);
    }

    trans_frame_parser_init(&parser);
    frames = feed_all(to_bytes(kGimbalFrame, sizeof(kGimbalFrame)), &parser);
    CHECK_EQ(frames.size(), 1U);
    if (frames.size() == 1U)
    {
        CHECK_EQ(frames[0].id, 0x23U);
        CHECK_EQ(frames[0].len, 25U);
        CHECK_EQ(frames[0].data[0], 1U);
        CHECK_EQ(read_i32(&frames[0].data[1]), 10000);
        CHECK_EQ(read_i32(&frames[0].data[5]), -5000);
        CHECK_EQ(read_i32(&frames[0].data[17]), 720000);
        CHECK_EQ(read_i32(&frames[0].data[21]), -20000000);
        for (int i = 9; i < 17; i++)
        {
            CHECK_EQ(frames[0].data[i], 0U);
        }
    }

    trans_frame_parser_init(&parser);
    frames = feed_all(to_bytes(kShootFrame, sizeof(kShootFrame)), &parser);
    CHECK_EQ(frames.size(), 1U);
    if (frames.size() == 1U)
    {
        CHECK_EQ(frames[0].id, 0x24U);
        CHECK_EQ(frames[0].len, 7U);
        CHECK_EQ(frames[0].data[0], 1U);
        CHECK_EQ(frames[0].data[1], 7U);
        CHECK_EQ(read_i32(&frames[0].data[2]), 123456789);
        CHECK_EQ(frames[0].data[6], 1U);
    }
}

void test_build_matches_known_frames()
{
    std::uint8_t payload[TRANS_GIMBAL_FB_PAYLOAD_LEN];
    std::uint8_t frame[TRANS_FRAME_MAX_SIZE];

    const std::uint8_t plen = trans_msg_build_gimbal_fb(
        payload, 1U, 10.0f, -5.0f, 720.0f, -20000.0f);
    const std::uint16_t flen = trans_frame_build(
        TRANS_ID_GIMBAL_FB, payload, plen, frame, sizeof(frame));
    CHECK_EQ(plen, 25U);
    CHECK_EQ(flen, sizeof(kGimbalFrame));
    CHECK(std::memcmp(frame, kGimbalFrame, sizeof(kGimbalFrame)) == 0);

    std::uint8_t shoot[TRANS_SHOOT_PAYLOAD_LEN];
    const std::uint8_t slen = trans_msg_build_shoot_status(
        shoot, 1U, 7U, 123456789U, 1U);
    const std::uint16_t sflen = trans_frame_build(
        TRANS_ID_SHOOT_STATUS, shoot, slen, frame, sizeof(frame));
    CHECK_EQ(slen, 7U);
    CHECK_EQ(sflen, sizeof(kShootFrame));
    CHECK(std::memcmp(frame, kShootFrame, sizeof(kShootFrame)) == 0);
}

void test_parser_resync()
{
    const std::vector<std::uint8_t> frame =
        to_bytes(kGimbalFrame, sizeof(kGimbalFrame));

    trans_frame_parser_t parser;
    trans_frame_parser_init(&parser);

    auto bad_checksum = frame;
    bad_checksum.back() ^= 0x5AU;
    CHECK_EQ(feed_all(bad_checksum, &parser).size(), 0U);
    CHECK(parser.checksum_errors > 0U);
    CHECK_EQ(feed_all(frame, &parser).size(), 1U);

    auto bad_address = frame;
    bad_address[1] = 0x04U;
    trans_frame_parser_init(&parser);
    CHECK_EQ(feed_all(bad_address, &parser).size(), 0U);
    CHECK(parser.address_errors > 0U);
    CHECK_EQ(feed_all(frame, &parser).size(), 1U);

    const std::uint8_t overlong[] = {0xFFU, 0x05U, 0x23U, 65U, 0x00U, 0x00U};
    trans_frame_parser_init(&parser);
    CHECK_EQ(feed_all(to_bytes(overlong, sizeof(overlong)), &parser).size(), 0U);
    CHECK(parser.length_errors > 0U);
    CHECK_EQ(feed_all(frame, &parser).size(), 1U);

    trans_frame_parser_init(&parser);
    std::vector<std::uint8_t> partial(frame.begin(), frame.end() - 1);
    CHECK_EQ(feed_all(partial, &parser).size(), 0U);
    trans_frame_t out{};
    CHECK_EQ(trans_frame_parser_feed(&parser, frame.back(), &out), 1U);

    std::vector<std::uint8_t> two = frame;
    two.insert(two.end(), frame.begin(), frame.end());
    trans_frame_parser_init(&parser);
    CHECK_EQ(feed_all(two, &parser).size(), 2U);
}

void test_message_limits()
{
    std::uint8_t payload[TRANS_GIMBAL_FB_PAYLOAD_LEN];
    const std::uint8_t len = trans_msg_build_gimbal_fb(
        payload, 9U, 0.0f, 70.0f, 1000.0f, 30000.0f);
    CHECK_EQ(len, 25U);
    CHECK_EQ(payload[0], 0U);
    CHECK_EQ(read_i32(&payload[5]), 60000);
    CHECK_EQ(read_i32(&payload[17]), 720000);
    CHECK_EQ(read_i32(&payload[21]), 20000000);

    const std::uint8_t nan_len = trans_msg_build_gimbal_fb(
        payload, 0U, std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN());
    CHECK_EQ(nan_len, 25U);
    CHECK_EQ(read_i32(&payload[1]), 0);
    CHECK_EQ(read_i32(&payload[5]), 0);
    CHECK_EQ(read_i32(&payload[17]), 0);
    CHECK_EQ(read_i32(&payload[21]), 0);

    trans_frame_t invalid{};
    invalid.id = TRANS_ID_VISION_CMD;
    invalid.len = 16U;
    trans_vision_cmd_t vision{};
    CHECK_EQ(trans_msg_parse_vision(&invalid, &vision, 0U), 0U);

    std::uint8_t bad_mode[17] = {0};
    bad_mode[0] = 3U;
    invalid.len = 17U;
    std::memcpy(invalid.data, bad_mode, sizeof(bad_mode));
    CHECK_EQ(trans_msg_parse_vision(&invalid, &vision, 0U), 0U);

    trans_frame_t bad_chassis{};
    bad_chassis.id = TRANS_ID_CHASSIS_CMD;
    bad_chassis.len = 23U;
    trans_pc_chassis_t chassis{};
    CHECK_EQ(trans_msg_parse_pc_chassis(&bad_chassis, &chassis, 0U), 0U);

    std::uint8_t shoot[TRANS_SHOOT_PAYLOAD_LEN];
    CHECK_EQ(trans_msg_build_shoot_status(shoot, 9U, 0U, 0U, 4U), 7U);
    CHECK_EQ(shoot[0], 1U);
    CHECK_EQ(shoot[6], 255U);
}

void test_yaw_unwrap()
{
    trans_yaw_unwrap_t u;
    trans_yaw_unwrap_reset(&u);

    CHECK_NEAR(trans_yaw_unwrap_update(&u, 0.0f), 0.0f, 1e-5);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, 10.0f), 10.0f, 1e-5);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, 20.0f), 20.0f, 1e-5);

    trans_yaw_unwrap_reset(&u);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, 170.0f), 170.0f, 1e-5);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, -170.0f), 190.0f, 1e-5);

    trans_yaw_unwrap_reset(&u);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, -170.0f), -170.0f, 1e-5);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, 170.0f), -190.0f, 1e-5);

    trans_yaw_unwrap_reset(&u);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, 179.0f), 179.0f, 1e-5);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, -179.0f), 181.0f, 1e-5);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, -178.0f), 182.0f, 1e-5);

    trans_yaw_unwrap_reset(&u);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, 100.0f), 100.0f, 1e-5);
    trans_yaw_unwrap_reset(&u);
    CHECK_NEAR(trans_yaw_unwrap_update(&u, 5.0f), 5.0f, 1e-5);

    const float held = trans_yaw_unwrap_update(
        &u, std::numeric_limits<float>::quiet_NaN());
    CHECK_NEAR(held, 5.0f, 1e-5);
}

void test_freshness()
{
    CHECK_EQ(trans_msg_is_fresh(1000U, 1000U, 150U), 1U);
    CHECK_EQ(trans_msg_is_fresh(1149U, 1000U, 150U), 1U);
    CHECK_EQ(trans_msg_is_fresh(1150U, 1000U, 150U), 0U);
    CHECK_EQ(trans_msg_is_fresh(1151U, 1000U, 150U), 0U);
    CHECK_EQ(trans_msg_is_fresh(0x00000005U, 0xFFFFFFF0U, 150U), 1U);
    CHECK_EQ(trans_msg_is_fresh(1000U, 1000U, 0U), 0U);
}

void test_frame_build_limits()
{
    std::uint8_t payload[TRANS_GIMBAL_FB_PAYLOAD_LEN] = {0};
    std::uint8_t out[80];

    CHECK_EQ(trans_frame_build(TRANS_ID_GIMBAL_FB, payload,
                               TRANS_GIMBAL_FB_PAYLOAD_LEN, out, 30U), 0U);
    CHECK_EQ(trans_frame_build(TRANS_ID_GIMBAL_FB, payload,
                               TRANS_GIMBAL_FB_PAYLOAD_LEN, out, 31U), 31U);
    CHECK_EQ(trans_frame_build(TRANS_ID_GIMBAL_FB, payload, 65U, out, 80U), 0U);
    CHECK_EQ(trans_frame_build(TRANS_ID_GIMBAL_FB, nullptr, 1U, out, 80U), 0U);
    CHECK_EQ(trans_frame_build(TRANS_ID_GIMBAL_FB, nullptr, 0U, out, 80U), 6U);

    trans_frame_t frame{};
    CHECK_EQ(trans_frame_parser_feed(nullptr, 0U, &frame), 0U);
    trans_frame_parser_t parser;
    trans_frame_parser_init(&parser);
    CHECK_EQ(trans_frame_parser_feed(&parser, 0xFFU, nullptr), 0U);
}

void test_chassis_six_axis()
{
    std::uint8_t payload[TRANS_CHASSIS_PAYLOAD_LEN];
    const std::int32_t raw[6] = {-12345, 6789, 1111, -2222, 3333, -4444};
    for (int i = 0; i < 6; i++)
    {
        const std::uint32_t v = static_cast<std::uint32_t>(raw[i]);
        payload[i * 4 + 0] = static_cast<std::uint8_t>(v & 0xFFU);
        payload[i * 4 + 1] = static_cast<std::uint8_t>((v >> 8U) & 0xFFU);
        payload[i * 4 + 2] = static_cast<std::uint8_t>((v >> 16U) & 0xFFU);
        payload[i * 4 + 3] = static_cast<std::uint8_t>((v >> 24U) & 0xFFU);
    }

    std::uint8_t frame[TRANS_FRAME_MAX_SIZE];
    const std::uint16_t len = trans_frame_build(
        TRANS_ID_CHASSIS_CMD, payload, sizeof(payload), frame, sizeof(frame));
    CHECK_EQ(len, 30U);

    trans_frame_parser_t parser;
    trans_frame_parser_init(&parser);
    const auto frames = feed_all(to_bytes(frame, len), &parser);
    CHECK_EQ(frames.size(), 1U);
    if (frames.size() == 1U)
    {
        trans_pc_chassis_t chassis{};
        CHECK_EQ(trans_msg_parse_pc_chassis(&frames[0], &chassis, 1U), 1U);
        CHECK_NEAR(chassis.vx_mps, -1.2345f, 1e-6);
        CHECK_NEAR(chassis.vy_mps, -0.6789f, 1e-6);
        CHECK_NEAR(chassis.vw_rps, 0.4444f, 1e-6);
    }
}

void test_ringbuf()
{
    std::uint8_t storage[8];
    trans_ringbuf_t rb;
    trans_ringbuf_init(&rb, storage, sizeof(storage));

    for (std::uint8_t i = 0; i < 7U; i++)
    {
        CHECK_EQ(trans_ringbuf_push(&rb, i), 1U);
    }
    CHECK_EQ(trans_ringbuf_count(&rb), 7U);
    CHECK_EQ(trans_ringbuf_push(&rb, 99U), 0U);
    CHECK_EQ(rb.overflow, 1U);

    for (std::uint8_t i = 0; i < 7U; i++)
    {
        std::uint8_t b = 0U;
        CHECK_EQ(trans_ringbuf_pop(&rb, &b), 1U);
        CHECK_EQ(b, i);
    }
    CHECK_EQ(trans_ringbuf_count(&rb), 0U);
}

void test_fuzz()
{
    std::mt19937 rng(0xC0FFEEU);
    trans_frame_parser_t parser;
    trans_frame_parser_init(&parser);

    for (int i = 0; i < 200000; i++)
    {
        trans_frame_t frame{};
        (void)trans_frame_parser_feed(&parser, static_cast<std::uint8_t>(rng() & 0xFFU),
                                      &frame);
        CHECK(parser.len <= TRANS_FRAME_PARSER_BUF);
    }
}

}  // namespace

int main()
{
    test_known_frames();
    test_build_matches_known_frames();
    test_parser_resync();
    test_message_limits();
    test_yaw_unwrap();
    test_freshness();
    test_frame_build_limits();
    test_chassis_six_axis();
    test_ringbuf();
    test_fuzz();

    if (g_failures == 0)
    {
        std::printf("ALL PROTOCOL TESTS PASSED\n");
        return 0;
    }

    std::printf("FAILURES: %d\n", g_failures);
    return 1;
}
