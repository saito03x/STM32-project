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

/**
 * @brief Sprawdzenie czy bufor jest pusty
 * @return 1 jeśli pusty, 0 jeśli wystęoują w nim pomiary
 */
uint8_t ColorBuffer_IsEmpty(void) {
    return 0;
}

/**
 * @brief Dodanie nowego pomiaru do bufora
 * @param data Wskaźnik na strukturę TCS34725_Data_t
 * @param timestamp Czas w milisekundach
 * @return 
 */
uint8_t ColorBuffer_Put(TCS34725_Data_t *data, uint32_t timestamp) {
    __disable_irq();

    // Zapisuje dane i przesuwa pozycję zapisu
    ColorBuffer[ColorBuffer_WritePos].data = *data;
    ColorBuffer[ColorBuffer_WritePos].timestamp = timestamp;
    ColorBuffer_WritePos = (ColorBuffer_WritePos + 1) % COLOR_BUFFER_SIZE;

    ColorBuffer_DataAvailable = 1;

    __enable_irq();

    return 1;
}

/**
 * @brief Pobranie najnowszego pomiaru
 * @return Wskaźnik na strukturę ColorBufferEntry_t
 */
ColorBufferEntry_t* ColorBuffer_GetLatest(void) {
    // Sprawdzenie czy zbieranie danych zostało rozpoczęte
    if (!ColorBuffer_DataAvailable) {
        return NULL; // Brak danych
    }

    uint32_t latest_index;
    if (ColorBuffer_WritePos == 0) {
        latest_index = COLOR_BUFFER_SIZE - 1;
    } else {
        latest_index = ColorBuffer_WritePos - 1;
    }

    return &ColorBuffer[latest_index];
}

/**
 * @brief Get color data entry by time offset from current time
 * @param timeOffsetMs Time offset in milliseconds (0 = latest, higher = older)
 * @return Pointer to ColorBufferEntry_t or NULL if not found or out of range
 */
ColorBufferEntry_t* ColorBuffer_GetByTimeOffset(uint32_t timeOffsetMs) {
    // Validate time offset range: 00001 - Tmax = 600 * Tint
    uint32_t maxOffset = COLOR_BUFFER_SIZE * timer_interval;
    if (timeOffsetMs == 0 || timeOffsetMs > maxOffset) {
        return NULL; // Invalid range
    }

    // Get current timestamp (assuming HAL_GetTick() gives current time)
    uint32_t currentTime = HAL_GetTick();
    uint32_t targetTime = currentTime - timeOffsetMs;

    // Start from latest entry (write_pos - 1) and search backwards through entire buffer
    uint32_t index;
    if (ColorBuffer_WritePos == 0) {
        index = COLOR_BUFFER_SIZE - 1;
    } else {
        index = ColorBuffer_WritePos - 1;
    }

    // Search through all buffer entries (circular)
    for (uint32_t i = 0; i < COLOR_BUFFER_SIZE; i++) {
        if (ColorBuffer[index].timestamp <= targetTime) {
            // Found entry that is at or before target time
            return &ColorBuffer[index];
        }

        // Move to previous entry (circular)
        if (index == 0) {
            index = COLOR_BUFFER_SIZE - 1;
        } else {
            index--;
        }
    }
    return NULL;
}

/**
 * @brief Clear color buffer (reset write position)
 */
void ColorBuffer_Clear(void) {
    __disable_irq();
    ColorBuffer_WritePos = 0;
    __enable_irq();
}

