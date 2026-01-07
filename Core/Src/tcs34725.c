#include "tcs34725.h"
#include "circular_buffer.h"
#include "protocol.h"
#include "i2c.h"
#include <stdint.h>

volatile TCS_State_t sensor_state = TCS_STATE_READY;

uint8_t dma_buffer[8];

static uint32_t tcs_poweron_tick = 0;

void TCS34725_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = TCS34725_COMMAND_BIT | reg;
    data[1] = value;
    HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDRESS, data, 2, 10);
}

static uint8_t TCS34725_ReadReg8(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t cmd = TCS34725_COMMAND_BIT | reg;
    uint8_t data = 0;

    HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDRESS, &cmd, 1, 10);
    HAL_I2C_Master_Receive(hi2c, TCS34725_ADDRESS, &data, 1, 10);

    return data;
}


uint8_t TCS34725_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t id = TCS34725_ReadReg8(hi2c, TCS34725_ID);
    if (id != 0x44) {
        return 0;
    }

    TCS34725_WriteReg(hi2c, TCS34725_ATIME, TIME_TABLE[current_time_index]);

    TCS34725_WriteReg(hi2c, TCS34725_CONTROL, GAIN_TABLE[current_gain_index]);

    TCS34725_WriteReg(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON);

    tcs_poweron_tick = HAL_GetTick();
    sensor_state = TCS_STATE_POWERUP_WAIT;

    return 1;
}

void TCS34725_ReadRawData(I2C_HandleTypeDef *hi2c, TCS34725_Data_t *data) {
    uint8_t cmd = TCS34725_COMMAND_BIT | 0x20 | TCS34725_CDATAL;
    uint8_t buffer[8];

    HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDRESS, &cmd, 1, 10);
    HAL_I2C_Master_Receive(hi2c, TCS34725_ADDRESS, buffer, 8, 10);

    data->c = (uint16_t)(buffer[1] << 8) | buffer[0];
    data->r = (uint16_t)(buffer[3] << 8) | buffer[2];
    data->g = (uint16_t)(buffer[5] << 8) | buffer[4];
    data->b = (uint16_t)(buffer[7] << 8) | buffer[6];
}

void TCS34725_HandleLoop(I2C_HandleTypeDef *hi2c) {
    if (sensor_state == TCS_STATE_POWERUP_WAIT) {
        if ((HAL_GetTick() - tcs_poweron_tick) >= 3) {
            TCS34725_WriteReg(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
            sensor_state = TCS_STATE_READY;
        }
    }
}

void TCS34725_Start_DMA_Read(I2C_HandleTypeDef *hi2c) {
    if (sensor_state != TCS_STATE_READY) {
        return;
    }

    if (sensor_state == TCS_STATE_READY) {
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
    
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        TCS34725_Data_t sensor_data;

        sensor_data.c = (uint16_t)(dma_buffer[1] << 8) | dma_buffer[0];
        sensor_data.r = (uint16_t)(dma_buffer[3] << 8) | dma_buffer[2];
        sensor_data.g = (uint16_t)(dma_buffer[5] << 8) | dma_buffer[4];
        sensor_data.b = (uint16_t)(dma_buffer[7] << 8) | dma_buffer[6];

        ColorBuffer_Put(&sensor_data, HAL_GetTick());
        sensor_state = TCS_STATE_READY;
    }
}
