#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include "tcs34725.h"

// UART buffers
#define UART_TXBUF_LEN 1512
#define UART_RXBUF_LEN 128

extern uint8_t UART_TxBuf[UART_TXBUF_LEN];
extern uint8_t UART_RxBuf[UART_RXBUF_LEN];
extern volatile int UART_TX_Empty;
extern volatile int UART_TX_Busy;
extern volatile int UART_RX_Empty;
extern volatile int UART_RX_Busy;

// Color sensor data buffer
#define COLOR_BUFFER_SIZE 600  // Minimum 600 entries as per requirements

typedef struct {
    TCS34725_Data_t data;
    uint32_t timestamp;  // Timestamp in milliseconds
} ColorBufferEntry_t;

extern ColorBufferEntry_t ColorBuffer[COLOR_BUFFER_SIZE];
extern volatile uint32_t ColorBuffer_Head;
extern volatile uint32_t ColorBuffer_Tail;
extern volatile uint32_t ColorBuffer_Count;

// External reference to current collection interval (defined in main.c)
extern volatile uint32_t timer_interval;

uint8_t UART_RX_IsEmpty(void);

int16_t UART_RX_GetChar(void);

uint8_t UART_RX_GetLine(char *buf);

void UART_TX_FSend(char* format, ...);

// Color buffer functions
uint8_t ColorBuffer_IsEmpty(void);
uint8_t ColorBuffer_IsFull(void);
uint32_t ColorBuffer_GetCount(void);
uint8_t ColorBuffer_Put(TCS34725_Data_t *data, uint32_t timestamp);
ColorBufferEntry_t* ColorBuffer_GetLatest(void);
ColorBufferEntry_t* ColorBuffer_GetByTimeOffset(uint32_t timeOffsetMs);
void ColorBuffer_Clear(void);



#endif
