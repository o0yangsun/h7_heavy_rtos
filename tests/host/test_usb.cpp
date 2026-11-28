#include <cstdio>
#include <cstring>

extern "C" {
#include "trans_usb.h"
#include "usbd_def.h"
}

extern "C"
{
USBD_HandleTypeDef hUsbDeviceHS;
}

static USBD_CDC_HandleTypeDef s_cdc;
static uint8_t s_tx_result = USBD_OK;
static const uint8_t *s_last_ptr = nullptr;
static uint16_t s_last_len = 0;
static int s_failures = 0;

#define CHECK(cond)                                                              \
    do                                                                           \
    {                                                                            \
        if (!(cond))                                                             \
        {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            s_failures++;                                                        \
        }                                                                        \
    } while (0)

extern "C" uint8_t CDC_Transmit_HS(uint8_t *Buf, uint16_t Len)
{
    s_last_ptr = Buf;
    s_last_len = Len;
    return s_tx_result;
}

int main()
{
    const uint8_t frame[31] = {0};

    CHECK(trans_usb_ready() == 0U);
    CHECK(trans_usb_tx(frame, sizeof(frame)) == 0U);
    CHECK(trans_usb_tx_error_count() == 1U);

    hUsbDeviceHS.pClassData = &s_cdc;
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    CHECK(trans_usb_ready() == 1U);

    s_cdc.TxState = 0U;
    s_tx_result = USBD_OK;
    CHECK(trans_usb_tx(frame, sizeof(frame)) == 1U);
    CHECK(s_last_len == sizeof(frame));

    // 驱动异步持有指针：调用者栈被改写后，发送缓冲内容必须仍然正确。
    uint8_t caller[31];
    std::memset(caller, 0x11, sizeof(caller));
    CHECK(trans_usb_tx(caller, sizeof(caller)) == 1U);
    std::memset(caller, 0xAA, sizeof(caller));
    CHECK(s_last_ptr != caller);
    for (std::size_t i = 0; i < sizeof(caller); i++)
    {
        CHECK(s_last_ptr[i] == 0x11U);
    }

    // 驱动正在发送时，必须先判忙，不能覆盖静态缓冲。
    s_cdc.TxState = 1U;
    s_tx_result = USBD_OK;
    CHECK(trans_usb_tx(frame, sizeof(frame)) == 0U);
    CHECK(trans_usb_tx_busy_count() == 1U);
    s_cdc.TxState = 0U;

    s_tx_result = USBD_BUSY;
    CHECK(trans_usb_tx(frame, sizeof(frame)) == 0U);
    CHECK(trans_usb_tx_busy_count() == 2U);

    s_tx_result = 3U;
    CHECK(trans_usb_tx(frame, sizeof(frame)) == 0U);
    CHECK(trans_usb_tx_error_count() == 2U);

    CHECK(trans_usb_tx(frame, 0U) == 0U);
    CHECK(trans_usb_tx(frame, 71U) == 0U);

    const uint8_t input[5] = {1U, 2U, 3U, 4U, 5U};
    uint8_t output[8] = {0};
    trans_usb_rx_isr(input, sizeof(input));
    CHECK(trans_usb_read(output, sizeof(output)) == 5U);
    CHECK(std::memcmp(input, output, sizeof(input)) == 0);

    trans_usb_rx_isr(nullptr, 5U);
    CHECK(trans_usb_read(nullptr, 5U) == 0U);

    uint8_t bulk[512];
    for (int i = 0; i < 512; i++)
    {
        bulk[i] = static_cast<uint8_t>(i);
    }
    trans_usb_rx_isr(bulk, sizeof(bulk));
    CHECK(trans_usb_overflow_count() == 1U);
    CHECK(trans_usb_read(output, 0U) == 0U);
    for (int i = 0; i < 511; i++)
    {
        uint8_t b = 0U;
        CHECK(trans_usb_read(&b, 1U) == 1U);
        CHECK(b == static_cast<uint8_t>(i));
    }
    CHECK(trans_usb_read(output, 1U) == 0U);

    if (s_failures == 0)
    {
        std::printf("ALL USB BACKEND TESTS PASSED\n");
        return 0;
    }

    std::printf("FAILURES: %d\n", s_failures);
    return 1;
}
