/*
 * mpm3801.h
 *
 *  Created on: Oct 3, 2025
 *      Author: cli
 */

#ifndef INC_MPM3801_H_
#define INC_MPM3801_H_

#include "main.h"

#define MPM3801_I2C_ADDR        0x28  // 7-bit address [[1]]
#define MPM3801_DATA_SIZE       4     // P_H, P_L, T_H, T_L

// Digital output values from the data sheet
#define PRESSURE_ZERO_OUTPUT      1638.0f
#define PRESSURE_FULL_SCALE_OUTPUT 14746.0f
#define DIGITAL_SPAN              (PRESSURE_FULL_SCALE_OUTPUT - PRESSURE_ZERO_OUTPUT)

// MPM3801 K020: for a 0 to 20 kPa sensor
#define SENSOR_PRESSURE_ZERO        0.0f
#define SENSOR_PRESSURE_FULL_SCALE  20.0f
#define SENSOR_PRESSURE_SPAN        (SENSOR_PRESSURE_FULL_SCALE - SENSOR_PRESSURE_ZERO)

typedef struct {
    uint16_t raw_pressure;   // 14-bit: 1638 (0%) to 14746 (100%)
    int16_t  raw_temperature; // 11-bit unsigned (0–2047)
    float pressure_kPa;      // Converted to kPa
    float temperature_C;     // Approx. temperature in °C
} MPM3801_Data_t;

HAL_StatusTypeDef MPM3801_ReadData(MPM3801_Data_t *data);
HAL_StatusTypeDef MPM3801_Convert(MPM3801_Data_t *data);

#endif /* INC_MPM3801_H_ */
