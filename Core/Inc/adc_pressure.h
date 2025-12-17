#ifndef INC_ADC_PRESSURE_H_
#define INC_ADC_PRESSURE_H_

#include "main.h"
#include "adc.h"

// STM32L0 Internal Reference Voltage Calibration Value address
#define VREFINT_CAL_ADDR ((uint16_t*)((uint32_t)0x1FF80078))

float Get_MCU_Voltage(void);
float Get_Current(void);
float Get_Pressure_kPa(void);
float Get_Height_from_Pressure(void);
void ADC_Manual_Calibration(void);
float Get_Average_Current(uint8_t samples);
float Get_Battery_Voltage(void);

#endif /* INC_ADC_PRESSURE_H_ */
