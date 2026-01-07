#ifndef INC_TCS34725_H_
#define INC_TCS34725_H_

#include "stm32f4xx_hal.h"
#include "main.h"

//BUFOR KOLOWY
#define TCS34725_BUFFER_SIZE 600

//ADRES URZĄDZENIA
#define TCS34725_ADDRESS	(0x29 << 1)

//WYMAGANY BIT PRZED KAŻDYM ADRESEM
#define TCS34725_COMMAND_BIT	0x80

// Rejestry
#define TCS34725_ENABLE           0x00
#define TCS34725_ATIME            0x01 // Czas integracji
#define TCS34725_WTIME            0x03
#define TCS34725_AILTL            0x04
#define TCS34725_AILTH            0x05
#define TCS34725_AIHTL            0x06
#define TCS34725_AIHTH            0x07
#define TCS34725_PERS             0x0C
#define TCS34725_CONFIG           0x0D
#define TCS34725_CONTROL          0x0F // Wzmocnienie (Gain)
#define TCS34725_ID               0x12 // ID układu
#define TCS34725_STATUS           0x13
#define TCS34725_CDATAL           0x14 // Poczatek danych kolorów

// Maski bitowe dla rejestru
#define TCS34725_ENABLE_AIEN      0x10 // RGBC Interrupt Enable
#define TCS34725_ENABLE_WEN       0x08 // Wait enable
#define TCS34725_ENABLE_AEN       0x02 // RGBC Enable
#define TCS34725_ENABLE_PON       0x01 // Power on

// Czas integracji ATIME
#define TCS34725_INTEGRATIONTIME_2_4MS  0xFF
#define TCS34725_INTEGRATIONTIME_24MS   0xF6
#define TCS34725_INTEGRATIONTIME_101MS  0xD5
#define TCS34725_INTEGRATIONTIME_154MS  0xC0
#define TCS34725_INTEGRATIONTIME_700MS  0x00

// Wzmocnienie GAIN
#define TCS34725_GAIN_1X    0x00
#define TCS34725_GAIN_4X    0x01
#define TCS34725_GAIN_16X   0x02
#define TCS34725_GAIN_60X   0x03

//DMA STANY
typedef enum {
    TCS_STATE_POWERUP_WAIT,  //Czekanie na start PON
    TCS_STATE_READY,
    TCS_STATE_BUSY
} TCS_State_t;


typedef struct {
    uint16_t c; // Clear
    uint16_t r; // Red
    uint16_t g; // Green
    uint16_t b; // Blue
} TCS34725_Data_t;

// Funkcje
uint8_t TCS34725_Init(I2C_HandleTypeDef *hi2c);
void TCS34725_ReadRawData(I2C_HandleTypeDef *hi2c, TCS34725_Data_t *data);
void TCS34725_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);


void TCS34725_HandleLoop(I2C_HandleTypeDef *hi2c);
void TCS34725_Start_DMA_Read(I2C_HandleTypeDef *hi2c);

extern volatile TCS_State_t sensor_state;

#endif /* INC_TCS34725_H_ */


