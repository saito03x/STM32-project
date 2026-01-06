#include "tcs34725.h"
#include "circular_buffer.h"
#include "i2c.h"
#include <stdint.h>

// State machine variable
volatile TCS_State_t sensor_state = TCS_STATE_READY;

// DMA buffer for sensor data (8 bytes: C, R, G, B - 16-bit each, little endian)
uint8_t dma_buffer[8];

// Funkcja pomocnicza do zapisu rejestru
// Zawsze dodajemy TCS34725_COMMAND_BIT do adresu rejestru
void TCS34725_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = TCS34725_COMMAND_BIT | reg;
    data[1] = value;
    HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDRESS, data, 2, 10);
}

// Funkcja pomocnicza do odczytu rejestru (8-bit)
static uint8_t TCS34725_ReadReg8(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t cmd = TCS34725_COMMAND_BIT | reg;
    uint8_t data = 0;

    HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDRESS, &cmd, 1, 10);
    HAL_I2C_Master_Receive(hi2c, TCS34725_ADDRESS, &data, 1, 10);

    return data;
}

/**
 * @brief Inicjalizacja czujnika
 * @return 1 jeśli OK (znaleziono ID 0x44), 0 jeśli błąd
 */
uint8_t TCS34725_Init(I2C_HandleTypeDef *hi2c) {
    // 1. Sprawdź ID układu [cite: 813]
    // Oczekiwana wartość dla TCS34725 to 0x44 (dla TCS34727 byłoby 0x4D)
    uint8_t id = TCS34725_ReadReg8(hi2c, TCS34725_ID);
    if (id != 0x44) {
        return 0; // Błąd: Nie wykryto czujnika lub zły model
    }

    // 2. Konfiguracja czasu integracji (np. 154ms) [cite: 720]
    TCS34725_WriteReg(hi2c, TCS34725_ATIME, TCS34725_INTEGRATIONTIME_154MS);

    // 3. Konfiguracja wzmocnienia (np. 4x) [cite: 794]
    TCS34725_WriteReg(hi2c, TCS34725_CONTROL, TCS34725_GAIN_4X);

    // 4. Włączenie zasilania (PON) [cite: 705]
    TCS34725_WriteReg(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON);
    HAL_Delay(3); // Wymagane > 2.4ms opóźnienia po PON [cite: 708]

    // 5. Włączenie przetwornika ADC (AEN) [cite: 705]
    // Utrzymujemy PON i dodajemy AEN
    TCS34725_WriteReg(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);

    // Opcjonalnie: Poczekaj na pierwszy pomiar (czas integracji)
    HAL_Delay(154);

    return 1; // Sukces
}

/**
 * @brief Odczytuje wszystkie 4 kanały (C, R, G, B)
 * Wykorzystuje Auto-Increment do szybkiego odczytu.
 */
void TCS34725_ReadRawData(I2C_HandleTypeDef *hi2c, TCS34725_Data_t *data) {
    // Ustawiamy bit COMMAND (0x80) oraz bit AUTO-INCREMENT (0x20)
    // Adres początkowy to 0x14 (CDATAL) [cite: 839]
    uint8_t cmd = TCS34725_COMMAND_BIT | 0x20 | TCS34725_CDATAL;
    uint8_t buffer[8];

    // Zapisz adres rejestru, od którego chcemy zacząć czytać
    HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDRESS, &cmd, 1, 10);

    // Odczytaj 8 bajtów (CDATAL, CDATAH, RDATAL, RDATAH, GDATAL, GDATAH, BDATAL, BDATAH)
    HAL_I2C_Master_Receive(hi2c, TCS34725_ADDRESS, buffer, 8, 10);

    // Złożenie bajtów (Little Endian - najpierw młodszy bajt) [cite: 839]
    data->c = (uint16_t)(buffer[1] << 8) | buffer[0];
    data->r = (uint16_t)(buffer[3] << 8) | buffer[2];
    data->g = (uint16_t)(buffer[5] << 8) | buffer[4];
    data->b = (uint16_t)(buffer[7] << 8) | buffer[6];
}

/**
 * @brief Start DMA read of sensor data
 * @param hi2c I2C handle
 * @note Non-blocking function with state machine protection
 */
void TCS34725_Start_DMA_Read(I2C_HandleTypeDef *hi2c) {
    // Check if sensor is ready for new DMA operation
    if (sensor_state == TCS_STATE_READY) {
        // Set state to busy to prevent concurrent operations
        sensor_state = TCS_STATE_BUSY;

        // Start DMA read: 8 bytes from CDATAL register with auto-increment
        // Register address: COMMAND_BIT (0x80) | AUTO_INCREMENT (0x20) | CDATAL (0x14)
        uint8_t reg_addr = TCS34725_COMMAND_BIT | 0x20 | TCS34725_CDATAL;

        HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c,
                                                       TCS34725_ADDRESS,
                                                       reg_addr,
                                                       I2C_MEMADD_SIZE_8BIT,
                                                       dma_buffer,
                                                       8);

        if (status != HAL_OK) {
            sensor_state = TCS_STATE_READY;
        }
    }
    
}

/**
 * @brief DMA completion callback for I2C memory read
 * @param hi2c I2C handle
 * @note Called automatically when DMA transfer completes
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Check if this callback is for our sensor (STM32F446RE I2C1)
    if (hi2c->Instance == hi2c1.Instance) {
        // Parse DMA buffer into sensor data structure
        TCS34725_Data_t sensor_data;

        // Little Endian: LSB first, then MSB
        sensor_data.c = (uint16_t)(dma_buffer[1] << 8) | dma_buffer[0];  // CDATAL, CDATAH
        sensor_data.r = (uint16_t)(dma_buffer[3] << 8) | dma_buffer[2];  // RDATAL, RDATAH
        sensor_data.g = (uint16_t)(dma_buffer[5] << 8) | dma_buffer[4];  // GDATAL, GDATAH
        sensor_data.b = (uint16_t)(dma_buffer[7] << 8) | dma_buffer[6];  // BDATAL, BDATAH

        // Add data to circular buffer with current timestamp
        ColorBuffer_Put(&sensor_data, HAL_GetTick());
        // Reset state to ready for next operation
        sensor_state = TCS_STATE_READY;
    }
}
