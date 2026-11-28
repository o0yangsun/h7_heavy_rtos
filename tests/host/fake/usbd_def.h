#ifndef TEST_USBD_DEF_H
#define TEST_USBD_DEF_H

#include <stdint.h>

typedef struct
{
    void *pClassData;
    uint8_t dev_state;
} USBD_HandleTypeDef;

typedef struct
{
    volatile uint8_t TxState;
} USBD_CDC_HandleTypeDef;

#define USBD_STATE_CONFIGURED 4U
#define USBD_OK               0U
#define USBD_BUSY             2U

uint8_t CDC_Transmit_HS(uint8_t *Buf, uint16_t Len);

#endif
