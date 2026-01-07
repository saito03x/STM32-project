#include "main.h"
#include "usart.h"
#include "circular_buffer.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

uint8_t UART_TxBuf[UART_TXBUF_LEN];
uint8_t UART_RxBuf[UART_RXBUF_LEN];

volatile int UART_TX_Empty = 0;
volatile int UART_TX_Busy = 0;
volatile int UART_RX_Empty = 0;
volatile int UART_RX_Busy = 0;

uint8_t UART_RX_IsEmpty(void) {
	return (UART_RX_Empty == UART_RX_Busy);
}

int16_t UART_RX_GetChar(void) {
	int16_t tmp;
	if (!UART_RX_IsEmpty()) {
		tmp = UART_RxBuf[UART_RX_Busy];
		UART_RX_Busy = (UART_RX_Busy + 1) % UART_RXBUF_LEN;
		return tmp;
	} else {
		return -1;
	}
}

void UART_TX_FSend(char *format, ...) {
	char tmp_rs[128];
	int i;
	volatile int idx;
	va_list arglist;
	va_start(arglist, format);
	vsprintf(tmp_rs, format, arglist);
	va_end(arglist);
	idx = UART_TX_Empty;
	for (i = 0; i < strlen(tmp_rs); i++) {
		UART_TxBuf[idx] = tmp_rs[i];
		idx++;
		if (idx >= UART_TXBUF_LEN)
			idx = 0;
	}
	__disable_irq();
	if ((UART_TX_Empty == UART_TX_Busy) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
		UART_TX_Empty = idx;
		uint8_t tmp = UART_TxBuf[UART_TX_Busy];
		UART_TX_Busy++;
		if (UART_TX_Busy >= UART_TXBUF_LEN)
			UART_TX_Busy = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	} else {
		UART_TX_Empty = idx;
	}
	__enable_irq();
}


// BUFER KOLOROWY
ColorBufferEntry_t ColorBuffer[COLOR_BUFFER_SIZE];
volatile uint32_t ColorBuffer_WritePos = 0;  // POZYCJA ZAPISU
volatile uint8_t ColorBuffer_DataAvailable = 0;  // FLAGA CZY ROZPOCZETO ZBIERANIE DANYCH

uint8_t ColorBuffer_Put(TCS34725_Data_t *data, uint32_t timestamp) {
    __disable_irq();

    ColorBuffer[ColorBuffer_WritePos].data = *data;
    ColorBuffer[ColorBuffer_WritePos].timestamp = timestamp;
    ColorBuffer_WritePos = (ColorBuffer_WritePos + 1) % COLOR_BUFFER_SIZE;

    ColorBuffer_DataAvailable = 1;

    __enable_irq();

    return 1;
}

ColorBufferEntry_t* ColorBuffer_GetLatest(void) {

    if (!ColorBuffer_DataAvailable) {
        return NULL;
    }

    uint32_t latest_index;
    if (ColorBuffer_WritePos == 0) {
        latest_index = COLOR_BUFFER_SIZE - 1;
    } else {
        latest_index = ColorBuffer_WritePos - 1;
    }

    return &ColorBuffer[latest_index];
}

ColorBufferEntry_t* ColorBuffer_GetByTimeOffset(uint32_t timeOffsetMs) {

    uint32_t maxOffset = COLOR_BUFFER_SIZE * timer_interval;
    if (timeOffsetMs == 0 || timeOffsetMs > maxOffset) {
        return NULL;
    }

    uint32_t currentTime = HAL_GetTick();
    uint32_t targetTime = currentTime - timeOffsetMs;

    uint32_t index;
    if (ColorBuffer_WritePos == 0) {
        index = COLOR_BUFFER_SIZE - 1;
    } else {
        index = ColorBuffer_WritePos - 1;
    }

    for (uint32_t i = 0; i < COLOR_BUFFER_SIZE; i++) {
        if (ColorBuffer[index].timestamp <= targetTime) {
            return &ColorBuffer[index];
        }

        if (index == 0) {
            index = COLOR_BUFFER_SIZE - 1;
        } else {
            index--;
        }
    }
    return NULL;
}


