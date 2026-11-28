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

#include "ul_bcp_bridge/bcp_codec.hpp"

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

std::int32_t read_i32(const std::uint8_t *p)
{
    const std::uint32_t raw = static_cast<std::uint32_t>(p[0]) |
                              (static_cast<std::uint32_t>(p[1]) << 8U) |
                              (static_cast<std::uint32_t>(p[2]) << 16U) |
                              (static_cast<std::uint32_t>(p[3]) << 24U);
    return static_cast<std::int32_t>(raw);
}

std::vector<trans_frame_t> feed_all(const std::vector<std::uint8_t> & bytes,
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

std::vector<std::uint8_t> feed_split(const std::vector<std::uint8_t> & bytes,
                                     std::size_t split,
                                     trans_frame_parser_t *parser)
{
    std::vector<std::uint8_t> ids;
    for (std::size_t i = 0; i < bytes.size(); i++)
    {
        trans_frame_t frame{};
        if (i == split)
        {
            (void)frame;
        }
        if (trans_frame_parser_feed(parser, bytes[i], &frame) != 0U)
        {
            ids.push_back(frame.id);
        }
    }
    return ids;
}

void test_frame_lengths_and_bytes()
{
    ul_bcp::VisionCommand vision;
    vision.mode = ul_bcp::VisionMode::kAim;
    vision.yaw_mdeg = 10000;
    vision.yaw_velocity_mdeg_s = 5000;
    vision.yaw_acceleration_mdeg_s2 = -1000;
    vision.pitch_mdeg = -5000;
    const auto vision_bytes = ul_bcp::Codec::encode_vision_command(vision);
    CHECK_EQ(vision_bytes.size(), 23U);
    CHECK_EQ(vision_bytes[1], 0x05U);
    CHECK_EQ(vision_bytes[2], 0x22U);

    ul_bcp::GimbalFeedback fb;
    fb.gimbal_mode = 1U;
    fb.yaw_mdeg = 10000;
    fb.pitch_mdeg = -5000;
    fb.yaw_velocity_mdeg_s = 720000;
    fb.yaw_acceleration_mdeg_s2 = -20000000;
    const auto fb_bytes = ul_bcp::Codec::encode_gimbal_feedback(fb);
    CHECK_EQ(fb_bytes.size(), 31U);
    CHECK_EQ(fb_bytes[1], 0x05U);
    CHECK_EQ(fb_bytes[2], 0x23U);
    CHECK_EQ(fb_bytes[3], 25U);

    ul_bcp::ShootStatus shoot;
    shoot.can_fire = false;
    shoot.reject_reason = 0U;
    shoot.bullet_speed_mmps = 0U;
    shoot.launcher_state = ul_bcp::LauncherState::kUnknownOrFault;
    const auto shoot_bytes = ul_bcp::Codec::encode_shoot_status_provisional(shoot);
    CHECK_EQ(shoot_bytes.size(), 13U);
    CHECK_EQ(shoot_bytes[1], 0x05U);
    CHECK_EQ(shoot_bytes[2], 0x24U);
    CHECK_EQ(shoot_bytes[3], 7U);
    CHECK_EQ(shoot_bytes[10], 255U);

    ul_bcp::ChassisCommand cmd;
    cmd.linear_x = 1.0;
    cmd.linear_y = -0.5;
    cmd.linear_z = 0.25;
    cmd.angular_x = 0.1;
    cmd.angular_y = -0.2;
    cmd.angular_z = -2.0;
    const auto cmd_bytes = ul_bcp::Codec::encode_chassis_command(cmd);
    CHECK_EQ(cmd_bytes.size(), 30U);
    CHECK_EQ(cmd_bytes[1], 0x05U);
    CHECK_EQ(cmd_bytes[2], 0x12U);
    CHECK_EQ(cmd_bytes[3], 24U);
}

void test_frame_build_matches_pc()
{
    // 0x23
    ul_bcp::GimbalFeedback fb;
    fb.gimbal_mode = 1U;
    fb.yaw_mdeg = 10000;
    fb.pitch_mdeg = -5000;
    fb.yaw_velocity_mdeg_s = 720000;
    fb.yaw_acceleration_mdeg_s2 = -20000000;
    const auto pc_fb = ul_bcp::Codec::encode_gimbal_feedback(fb);

    std::uint8_t payload[TRANS_GIMBAL_FB_PAYLOAD_LEN];
    std::uint8_t frame[TRANS_FRAME_MAX_SIZE];
    const std::uint8_t plen = trans_msg_build_gimbal_fb(
        payload, fb.gimbal_mode, 10.0f, -5.0f, 720.0f, -20000.0f);
    const std::uint16_t flen = trans_frame_build(
        TRANS_ID_GIMBAL_FB, payload, plen, frame, sizeof(frame));
    CHECK_EQ(plen, 25U);
    CHECK_EQ(flen, pc_fb.size());
    CHECK(std::memcmp(frame, pc_fb.data(), flen) == 0);

    // 0x24 normal
    ul_bcp::ShootStatus shoot;
    shoot.can_fire = true;
    shoot.reject_reason = 7U;
    shoot.bullet_speed_mmps = 123456789U;
    shoot.launcher_state = ul_bcp::LauncherState::kInstalledUnlocked;
    const auto pc_shoot = ul_bcp::Codec::encode_shoot_status_provisional(shoot);

    std::uint8_t sp[TRANS_SHOOT_PAYLOAD_LEN];
    const std::uint8_t splen = trans_msg_build_shoot_status(
        sp, 1U, 7U, 123456789U, 1U);
    const std::uint16_t sflen = trans_frame_build(
        TRANS_ID_SHOOT_STATUS, sp, splen, frame, sizeof(frame));
    CHECK_EQ(splen, 7U);
    CHECK_EQ(sflen, pc_shoot.size());
    CHECK(std::memcmp(frame, pc_shoot.data(), sflen) == 0);

    // 0x24 invalid enum coercion
    ul_bcp::ShootStatus invalid;
    invalid.can_fire = true;
    invalid.reject_reason = 7U;
    invalid.bullet_speed_mmps = 123456789U;
    invalid.launcher_state = ul_bcp::LauncherState::kUnknownOrFault;
    const auto pc_invalid = ul_bcp::Codec::encode_shoot_status_provisional(invalid);
    const std::uint8_t invalid_len = trans_msg_build_shoot_status(
        sp, 9U, 7U, 123456789U, 4U);
    const std::uint16_t invalid_flen = trans_frame_build(
        TRANS_ID_SHOOT_STATUS, sp, invalid_len, frame, sizeof(frame));
    CHECK_EQ(invalid_flen, pc_invalid.size());
    CHECK(std::memcmp(frame, pc_invalid.data(), invalid_flen) == 0);
}

void test_parser_accepts_pc_frames()
{
    ul_bcp::VisionCommand vision;
    vision.mode = ul_bcp::VisionMode::kAim;
    vision.yaw_mdeg = 10000;
    vision.yaw_velocity_mdeg_s = 5000;
    vision.yaw_acceleration_mdeg_s2 = -1000;
    vision.pitch_mdeg = -5000;
    const auto vision_bytes = ul_bcp::Codec::encode_vision_command(vision);

    trans_frame_parser_t parser;
    trans_frame_parser_init(&parser);
    const auto frames = feed_all(vision_bytes, &parser);
    CHECK_EQ(frames.size(), 1U);
    if (frames.size() == 1U)
    {
        CHECK_EQ(frames[0].id, 0x22U);
        CHECK_EQ(frames[0].len, 17U);
        trans_vision_cmd_t out{};
        CHECK_EQ(trans_msg_parse_vision(&frames[0], &out, 1234U), 1U);
        CHECK_EQ(out.mode, 1U);
        CHECK_EQ(out.yaw_mdeg, 10000);
        CHECK_EQ(out.yaw_vel_mdeg_s, 5000);
        CHECK_EQ(out.yaw_acc_mdeg_s2, -1000);
        CHECK_EQ(out.pitch_mdeg, -5000);
        CHECK_NEAR(out.yaw_rad, 0.174532925f, 1e-6);
        CHECK_NEAR(out.pitch_rad, -0.0872664626f, 1e-6);
        CHECK_EQ(out.tick_ms, 1234U);
        CHECK_EQ(out.valid, 1U);
    }

    ul_bcp::ChassisCommand cmd;
    cmd.linear_x = 1.0;
    cmd.linear_y = 0.5;
    cmd.linear_z = 0.25;
    cmd.angular_x = 0.1;
    cmd.angular_y = -0.2;
    cmd.angular_z = -2.0;
    const auto cmd_bytes = ul_bcp::Codec::encode_chassis_command(cmd);

    trans_frame_parser_init(&parser);
    const auto cmd_frames = feed_all(cmd_bytes, &parser);
    CHECK_EQ(cmd_frames.size(), 1U);
    if (cmd_frames.size() == 1U)
    {
        trans_pc_chassis_t out{};
        CHECK_EQ(trans_msg_parse_pc_chassis(&cmd_frames[0], &out, 55U), 1U);
        CHECK_NEAR(out.vx_mps, 1.0f, 1e-6);
        CHECK_NEAR(out.vy_mps, 0.5f, 1e-6);
        CHECK_NEAR(out.vw_rps, -2.0f, 1e-6);
        CHECK_EQ(out.tick_ms, 55U);
        CHECK_EQ(out.valid, 1U);
    }
}

void test_parser_robustness()
{
    ul_bcp::GimbalFeedback fb;
    fb.gimbal_mode = 1U;
    fb.yaw_mdeg = 12345;
    fb.pitch_mdeg = -6789;
    fb.yaw_velocity_mdeg_s = 720000;
    fb.yaw_acceleration_mdeg_s2 = -20000000;
    const auto frame_bytes = ul_bcp::Codec::encode_gimbal_feedback(fb);

    trans_frame_parser_t parser;
    trans_frame_parser_init(&parser);

    // Split at every byte boundary.
    for (std::size_t split = 0; split <= frame_bytes.size(); split++)
    {
        trans_frame_parser_init(&parser);
        const auto ids = feed_split(frame_bytes, split, &parser);
        CHECK_EQ(ids.size(), 1U);
        if (!ids.empty())
        {
            CHECK_EQ(ids[0], 0x23U);
        }
    }

    // Garbage prefix and suffix.
    std::vector<std::uint8_t> noisy;
    noisy.insert(noisy.end(), {0x00U, 0x11U, 0xFFU, 0x22U, 0x33U});
    noisy.insert(noisy.end(), frame_bytes.begin(), frame_bytes.end());
    noisy.insert(noisy.end(), {0x00U, 0x7FU, 0xFFU});
    trans_frame_parser_init(&parser);
    const auto noisy_frames = feed_all(noisy, &parser);
    CHECK_EQ(noisy_frames.size(), 1U);

    // Bad checksum, then recovery.
    auto bad_checksum = frame_bytes;
    bad_checksum.back() ^= 0x5AU;
    trans_frame_parser_init(&parser);
    CHECK_EQ(feed_all(bad_checksum, &parser).size(), 0U);
    CHECK(parser.checksum_errors > 0U);
    CHECK_EQ(feed_all(frame_bytes, &parser).size(), 1U);

    // Bad address, then recovery.
    auto bad_address = frame_bytes;
    bad_address[1] = 0x04U;
    trans_frame_parser_init(&parser);
    CHECK_EQ(feed_all(bad_address, &parser).size(), 0U);
    CHECK(parser.address_errors > 0U);
    CHECK_EQ(feed_all(frame_bytes, &parser).size(), 1U);

    // Overlong LEN, then recovery.
    const std::uint8_t overlong[] = {0xFFU, 0x05U, 0x23U, 65U, 0x00U, 0x00U};
    trans_frame_parser_init(&parser);
    CHECK_EQ(feed_all(std::vector<std::uint8_t>(overlong, overlong + sizeof(overlong)),
                      &parser)
                 .size(),
             0U);
    CHECK(parser.length_errors > 0U);
    CHECK_EQ(feed_all(frame_bytes, &parser).size(), 1U);

    // Truncated frame, then completion.
    trans_frame_parser_init(&parser);
    std::vector<std::uint8_t> partial(frame_bytes.begin(), frame_bytes.end() - 1);
    CHECK_EQ(feed_all(partial, &parser).size(), 0U);
    const std::uint8_t last = frame_bytes.back();
    trans_frame_t out{};
    CHECK_EQ(trans_frame_parser_feed(&parser, last, &out), 1U);

    // Two frames back-to-back.
    std::vector<std::uint8_t> two = frame_bytes;
    two.insert(two.end(), frame_bytes.begin(), frame_bytes.end());
    trans_frame_parser_init(&parser);
    CHECK_EQ(feed_all(two, &parser).size(), 2U);
}

void test_differential_with_pc_parser()
{
    std::mt19937 rng(0x5A5A1234U);

    auto random_frame = [&rng]() {
        const std::uint32_t pick = rng() % 4U;
        if (pick == 0U)
        {
            ul_bcp::VisionCommand v;
            v.mode = static_cast<ul_bcp::VisionMode>(rng() % 3U);
            v.yaw_mdeg = static_cast<std::int32_t>(rng() % 2000000U) - 1000000;
            v.yaw_velocity_mdeg_s = static_cast<std::int32_t>(rng() % 2000000U) - 1000000;
            v.yaw_acceleration_mdeg_s2 = static_cast<std::int32_t>(rng() % 40000000U) - 20000000;
            v.pitch_mdeg = static_cast<std::int32_t>(rng() % 120000U) - 60000;
            return ul_bcp::Codec::encode_vision_command(v);
        }
        if (pick == 1U)
        {
            ul_bcp::GimbalFeedback f;
            f.gimbal_mode = static_cast<std::uint8_t>(rng() % 4U);
            f.yaw_mdeg = static_cast<std::int32_t>(rng() % 20000000U) - 10000000;
            f.pitch_mdeg = static_cast<std::int32_t>(rng() % 120000U) - 60000;
            f.yaw_velocity_mdeg_s = static_cast<std::int32_t>(rng() % 2000000U) - 1000000;
            f.yaw_acceleration_mdeg_s2 = static_cast<std::int32_t>(rng() % 40000000U) - 20000000;
            return ul_bcp::Codec::encode_gimbal_feedback(f);
        }
        if (pick == 2U)
        {
            ul_bcp::ShootStatus s;
            s.can_fire = (rng() & 1U) != 0U;
            s.reject_reason = static_cast<std::uint8_t>(rng() % 8U);
            s.bullet_speed_mmps = rng() % 40000U;
            const std::uint32_t states[] = {0U, 1U, 2U, 3U, 255U};
            s.launcher_state = static_cast<ul_bcp::LauncherState>(states[rng() % 5U]);
            return ul_bcp::Codec::encode_shoot_status_provisional(s);
        }
        ul_bcp::ChassisCommand c;
        c.linear_x = (static_cast<double>(rng() % 20000U) - 10000.0) / 10000.0;
        c.linear_y = (static_cast<double>(rng() % 20000U) - 10000.0) / 10000.0;
        c.linear_z = 0.0;
        c.angular_x = 0.0;
        c.angular_y = 0.0;
        c.angular_z = (static_cast<double>(rng() % 20000U) - 10000.0) / 10000.0;
        return ul_bcp::Codec::encode_chassis_command(c);
    };

    std::vector<std::uint8_t> stream;
    for (int i = 0; i < 200; i++)
    {
        const std::size_t noise_len = rng() % 9U;
        for (std::size_t j = 0; j < noise_len; j++)
        {
            stream.push_back(static_cast<std::uint8_t>(rng() & 0xFFU));
        }

        auto frame = random_frame();
        const std::uint32_t mutation = rng() % 100U;
        if (mutation < 10U && frame.size() > 6U)
        {
            const std::size_t pos = 4U + (rng() % (frame.size() - 6U));
            frame[pos] ^= static_cast<std::uint8_t>(1U << (rng() % 8U));
        }
        else if (mutation < 15U)
        {
            frame[1] = static_cast<std::uint8_t>(rng() & 0xFFU);
        }
        else if (mutation < 20U && frame.size() > 10U)
        {
            frame.resize(frame.size() - (1U + (rng() % 5U)));
        }

        stream.insert(stream.end(), frame.begin(), frame.end());
    }

    ul_bcp::StreamParser pc_parser;
    const auto pc_frames = pc_parser.feed(stream);

    trans_frame_parser_t parser;
    trans_frame_parser_init(&parser);
    const auto our_frames = feed_all(stream, &parser);

    CHECK_EQ(our_frames.size(), pc_frames.size());
    const std::size_t count = std::min(our_frames.size(), pc_frames.size());
    for (std::size_t i = 0; i < count; i++)
    {
        CHECK_EQ(our_frames[i].id, pc_frames[i].id);
        CHECK_EQ(our_frames[i].len, pc_frames[i].payload.size());
        if (our_frames[i].len == pc_frames[i].payload.size())
        {
            CHECK(std::memcmp(our_frames[i].data, pc_frames[i].payload.data(),
                              our_frames[i].len) == 0);
        }
    }

    CHECK_EQ(parser.frames, pc_parser.stats().frames);
    CHECK_EQ(parser.checksum_errors, pc_parser.stats().checksum_errors);
    CHECK_EQ(parser.length_errors, pc_parser.stats().length_errors);
    CHECK_EQ(parser.address_errors, pc_parser.stats().address_errors);
}

void test_roundtrip_pc_decode()
{
    std::uint8_t payload[TRANS_GIMBAL_FB_PAYLOAD_LEN];
    std::uint8_t frame[TRANS_FRAME_MAX_SIZE];
    const std::uint8_t plen = trans_msg_build_gimbal_fb(
        payload, 2U, 12.345f, -6.789f, 123.456f, -4567.89f);
    const std::uint16_t flen = trans_frame_build(
        TRANS_ID_GIMBAL_FB, payload, plen, frame, sizeof(frame));

    std::vector<std::uint8_t> bytes(frame, frame + flen);
    const auto raw = ul_bcp::Codec::decode_raw_frame(bytes);
    CHECK(raw.has_value());
    if (raw.has_value())
    {
        CHECK_EQ(raw->destination, 0x05U);
        CHECK_EQ(raw->id, 0x23U);
        const auto fb = ul_bcp::Codec::decode_gimbal_feedback(*raw);
        CHECK(fb.has_value());
        if (fb.has_value())
        {
            CHECK_EQ(fb->gimbal_mode, 2U);
            CHECK_EQ(fb->yaw_mdeg, 12345);
            CHECK_EQ(fb->pitch_mdeg, -6789);
            CHECK_EQ(fb->yaw_velocity_mdeg_s, 123456);
            CHECK_EQ(fb->yaw_acceleration_mdeg_s2, -4567890);
        }
    }
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
    for (int i = 9; i < 17; i++)
    {
        CHECK_EQ(payload[i], 0U);
    }

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
    invalid.id = 0x22U;
    invalid.len = 16U;
    trans_vision_cmd_t vision{};
    CHECK_EQ(trans_msg_parse_vision(&invalid, &vision, 0U), 0U);

    std::uint8_t bad_mode[17] = {0};
    bad_mode[0] = 3U;
    invalid.len = 17U;
    std::memcpy(invalid.data, bad_mode, sizeof(bad_mode));
    CHECK_EQ(trans_msg_parse_vision(&invalid, &vision, 0U), 0U);

    trans_frame_t bad_chassis{};
    bad_chassis.id = 0x12U;
    bad_chassis.len = 23U;
    trans_pc_chassis_t chassis{};
    CHECK_EQ(trans_msg_parse_pc_chassis(&bad_chassis, &chassis, 0U), 0U);
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

    for (std::uint8_t i = 0; i < 5U; i++)
    {
        CHECK_EQ(trans_ringbuf_push(&rb, static_cast<std::uint8_t>(10U + i)), 1U);
    }
    for (std::uint8_t i = 0; i < 5U; i++)
    {
        std::uint8_t b = 0U;
        CHECK_EQ(trans_ringbuf_pop(&rb, &b), 1U);
        CHECK_EQ(b, static_cast<std::uint8_t>(10U + i));
    }
}

void test_fuzz_no_crash()
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
    test_frame_lengths_and_bytes();
    test_frame_build_matches_pc();
    test_parser_accepts_pc_frames();
    test_parser_robustness();
    test_differential_with_pc_parser();
    test_roundtrip_pc_decode();
    test_message_limits();
    test_ringbuf();
    test_fuzz_no_crash();

    if (g_failures == 0)
    {
        std::printf("ALL OFFLINE TESTS PASSED\n");
        return 0;
    }

    std::printf("FAILURES: %d\n", g_failures);
    return 1;
}
