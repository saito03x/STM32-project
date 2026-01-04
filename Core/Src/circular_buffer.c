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

// Color sensor data buffer implementation
ColorBufferEntry_t ColorBuffer[COLOR_BUFFER_SIZE];
volatile uint32_t ColorBuffer_Head = 0;
volatile uint32_t ColorBuffer_Tail = 0;
volatile uint32_t ColorBuffer_Count = 0;

/**
 * @brief Check if color buffer is empty
 * @return 1 if empty, 0 otherwise
 */
uint8_t ColorBuffer_IsEmpty(void) {
    return (ColorBuffer_Count == 0);
}

/**
 * @brief Check if color buffer is full
 * @return 1 if full, 0 otherwise
 */
uint8_t ColorBuffer_IsFull(void) {
    return (ColorBuffer_Count == COLOR_BUFFER_SIZE);
}

/**
 * @brief Get number of entries in color buffer
 * @return Number of entries
 */
uint32_t ColorBuffer_GetCount(void) {
    return ColorBuffer_Count;
}

/**
 * @brief Add new color data entry to buffer
 * @param data Pointer to TCS34725_Data_t structure
 * @param timestamp Timestamp in milliseconds
 * @return 1 if successful, 0 if buffer is full
 */
uint8_t ColorBuffer_Put(TCS34725_Data_t *data, uint32_t timestamp) {
    if (ColorBuffer_IsFull()) {
        return 0; // Buffer is full
    }

    __disable_irq();
    ColorBuffer[ColorBuffer_Head].data = *data;
    ColorBuffer[ColorBuffer_Head].timestamp = timestamp;
    ColorBuffer_Head = (ColorBuffer_Head + 1) % COLOR_BUFFER_SIZE;
    ColorBuffer_Count++;
    __enable_irq();

    return 1;
}

/**
 * @brief Get latest (most recent) color data entry
 * @return Pointer to ColorBufferEntry_t or NULL if buffer is empty
 */
ColorBufferEntry_t* ColorBuffer_GetLatest(void) {
    if (ColorBuffer_IsEmpty()) {
        return NULL;
    }

    // Latest entry is at head-1 position (circular buffer)
    uint32_t latest_index;
    if (ColorBuffer_Head == 0) {
        latest_index = COLOR_BUFFER_SIZE - 1;
    } else {
        latest_index = ColorBuffer_Head - 1;
    }

    return &ColorBuffer[latest_index];
}

/**
 * @brief Get color data entry by time offset from current time
 * @param timeOffsetMs Time offset in milliseconds (0 = latest, higher = older)
 * @return Pointer to ColorBufferEntry_t or NULL if not found or out of range
 */
ColorBufferEntry_t* ColorBuffer_GetByTimeOffset(uint32_t timeOffsetMs) {
    if (ColorBuffer_IsEmpty()) {
        return NULL;
    }

    // Validate time offset range: 00001 - Tmax = 600 * Tint
    uint32_t maxOffset = COLOR_BUFFER_SIZE * current_collection_interval;
    if (timeOffsetMs == 0 || timeOffsetMs > maxOffset) {
        return NULL; // Invalid range
    }

    // Get current timestamp (assuming HAL_GetTick() gives current time)
    uint32_t currentTime = HAL_GetTick();
    uint32_t targetTime = currentTime - timeOffsetMs;

    // Start from latest entry and go backwards
    uint32_t index;
    if (ColorBuffer_Head == 0) {
        index = COLOR_BUFFER_SIZE - 1;
    } else {
        index = ColorBuffer_Head - 1;
    }

    uint32_t checked = 0;

    while (checked < ColorBuffer_Count) {
        if (ColorBuffer[index].timestamp <= targetTime) {
            // Found entry that is at or before target time
            return &ColorBuffer[index];
        }

        // Move to previous entry
        if (index == 0) {
            index = COLOR_BUFFER_SIZE - 1;
        } else {
            index--;
        }
        checked++;
    }

    // If no exact match found, return oldest available entry
    if (ColorBuffer_Count > 0) {
        // Return oldest entry (tail)
        return &ColorBuffer[ColorBuffer_Tail];
    }

    return NULL;
}

/**
 * @brief Clear color buffer
 */
void ColorBuffer_Clear(void) {
    __disable_irq();
    ColorBuffer_Head = 0;
    ColorBuffer_Tail = 0;
    ColorBuffer_Count = 0;
    __enable_irq();
}

