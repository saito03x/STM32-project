#include "tcs34725.h"
#include <stdint.h>

uint8_t dma_buffer[8];

// Funkcja pomocnicza do zapisu rejestru
// Zawsze dodajemy TCS34725_COMMAND_BIT do adresu rejestru
static void TCS34725_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
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
