/*
 * mpm3801.c
 *
 *  Created on: Oct 2, 2025
 *      Author: cli
 */




#include "mpm3801.h"
#include "string.h"

//extern I2C_HandleTypeDef hi2c1;
//
//HAL_StatusTypeDef MPM3801_ReadData(MPM3801_Data_t *data)
//{
//    if (data == NULL) return HAL_ERROR;
//
//    uint8_t rx_buf[MPM3801_DATA_SIZE] = {0};
//
//    // MPM3801 auto-streams 4 bytes when read (no register address needed)
//    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c1,
//                                                   (MPM3801_I2C_ADDR << 1),
//                                                   rx_buf,
//                                                   MPM3801_DATA_SIZE,
//                                                   100); // 100 ms timeout
//
//    if (ret != HAL_OK) {
//        // Optional: log error code
//        // Error_Handler();
//        return ret;
//    }
//
//    // Pressure: 14-bit, big-endian
//    data->raw_pressure = ((uint16_t)(rx_buf[0] & 0x3F) << 8) | rx_buf[1];
//    // Temperature: 11-bit unsigned
//	// T_H = rx_buf[2] = T[10:3]
//	// T_L = rx_buf[3] = T[2:0]xxxxx
//	data->raw_temperature = ((rx_buf[2] << 3) | (rx_buf[3] >> 5)) & 0x7FF;
//
//    return HAL_OK;
//}

HAL_StatusTypeDef MPM3801_Convert(MPM3801_Data_t *data)
{
	if (!data) return HAL_ERROR;

    // Clamp raw pressure to valid range [1638, 14746]
    uint16_t raw = data->raw_pressure;
    if (raw < PRESSURE_ZERO_OUTPUT) raw = PRESSURE_ZERO_OUTPUT;
    if (raw > PRESSURE_FULL_SCALE_OUTPUT) raw = PRESSURE_FULL_SCALE_OUTPUT;

    // Linear mapping: 1638 → 0%, 14746 → 100%
    float percent = (raw - PRESSURE_ZERO_OUTPUT) / DIGITAL_SPAN;
    data->pressure_kPa = SENSOR_PRESSURE_SPAN * percent + SENSOR_PRESSURE_ZERO;

    // Temperature: 11-bit → -50°C to +150°C
	if (data->raw_temperature > 2047) data->raw_temperature = 2047; // safety
	data->temperature_C = (data->raw_temperature / 2047.0f) * 200.0f - 50.0f;

    return HAL_OK;
}
