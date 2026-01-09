#include "tcs34725.h"
#include "circular_buffer.h"
#include "protocol.h"
#include "i2c.h"
#include <stdint.h>

volatile TCS_State_t sensor_state = TCS_STATE_INIT_READ_ID;

uint8_t dma_buffer[8];

static uint32_t tcs_poweron_tick = 0;
static uint8_t config_step = 0;

void TCS34725_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    static uint8_t data[2];
    data[0] = TCS34725_COMMAND_BIT | reg;
    data[1] = value;

    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
        return;
    }

    HAL_I2C_Master_Transmit_DMA(hi2c, TCS34725_ADDRESS, data, 2);
}


void TCS34725_Init(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;
    config_step = 0;
    sensor_state = TCS_STATE_INIT_READ_ID;
}

// Glowna petla obslugi czujnika
void TCS34725_HandleLoop(I2C_HandleTypeDef *hi2c) {
    
    //Oczyt id
    if (sensor_state == TCS_STATE_INIT_READ_ID) {
        if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {
            uint8_t reg_addr = TCS34725_COMMAND_BIT | TCS34725_ID;
            HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c,
                                                           TCS34725_ADDRESS,
                                                           reg_addr,
                                                           I2C_MEMADD_SIZE_8BIT,
                                                           dma_buffer,
                                                           1);
            if (status == HAL_OK) {
                sensor_state = TCS_STATE_BUSY;
            }
        }
    }
   //Konfiguracja czujnika
    else if (sensor_state == TCS_STATE_CONFIGURING) {
        if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {
            switch (config_step) {
                case 0:
                    TCS34725_WriteReg(hi2c, TCS34725_ATIME, TIME_TABLE[current_time_index]);
                    config_step = 1;
                    break;

                case 1:
                    TCS34725_WriteReg(hi2c, TCS34725_CONTROL, GAIN_TABLE[current_gain_index]);
                    config_step = 2;
                    break;

                case 2:
                    TCS34725_WriteReg(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON);
                    tcs_poweron_tick = HAL_GetTick();
                    sensor_state = TCS_STATE_POWERUP_WAIT;
                    break;
            }
        }
    }
    //Czekanie 3ms na rozruch oscylatora
    else if (sensor_state == TCS_STATE_POWERUP_WAIT) {
        if ((HAL_GetTick() - tcs_poweron_tick) >= 3) {
            if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {
                TCS34725_WriteReg(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
                sensor_state = TCS_STATE_READY;
            }
        }
    }
}

// Rozpoczecie odczytu danych kolorow przez DMA
void TCS34725_Start_DMA_Read(I2C_HandleTypeDef *hi2c) {
    if (sensor_state != TCS_STATE_READY) {
        return;
    }

    sensor_state = TCS_STATE_BUSY;

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

// Callback po zakonczeniu odczytu DMA
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != hi2c1.Instance) {
        return;
    }

    // Jesli bylismy w trakcie odczytu ID (inicjalizacja)
    if (sensor_state == TCS_STATE_BUSY && config_step == 0) {
        if (dma_buffer[0] == TCS34725_EXPECTED_ID) {
            sensor_state = TCS_STATE_CONFIGURING;
        } else {
            sensor_state = TCS_STATE_ERROR;
        }
        return;
    }

    // Normalny odczyt danych kolorow
    TCS34725_Data_t sensor_data;
    sensor_data.c = (uint16_t)(dma_buffer[1] << 8) | dma_buffer[0];
    sensor_data.r = (uint16_t)(dma_buffer[3] << 8) | dma_buffer[2];
    sensor_data.g = (uint16_t)(dma_buffer[5] << 8) | dma_buffer[4];
    sensor_data.b = (uint16_t)(dma_buffer[7] << 8) | dma_buffer[6];

    ColorBuffer_Put(&sensor_data, HAL_GetTick());
    sensor_state = TCS_STATE_READY;
}
