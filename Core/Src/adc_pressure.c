#include "adc_pressure.h"
#include <math.h>

// --- SYSTEM CONSTANTS ---
// The voltage of the LDO powering the CURRENT SENSOR (Fixed)
#define SENSOR_SUPPLY_VOLTAGE   3.0f

// The rated voltage in the sensor datasheet (usually 3.3V or 5.0V)
// We need this to scale the sensitivity correctly.
#define SENSOR_DATASHEET_VOLTAGE 3.3f

// The sensitivity listed in the datasheet at the rated voltage (e.g., 44mV/A)
#define SENSOR_ORIGIN_SENSITIVITY 0.044f

void ADC_Manual_Calibration(void)
{
    // CRITICAL: STM32L0 requires calibration after power on
    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
}
// Helper to read a specific channel safely on STM32L0
uint32_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    HAL_ADC_Stop(&hadc);
    hadc.Instance->CHSELR = 0; // Clear selection

    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
        return HAL_ADC_GetValue(&hadc);
    }
    return 0;
}

// Calculates the actual MCU Power Supply Voltage (Battery Voltage)
// We need this to convert the raw ADC number (0-4095) into actual Volts.

float Get_MCU_Voltage(void)
{
    uint32_t raw_vref = Read_ADC_Channel(ADC_CHANNEL_17);

    if (raw_vref == 0) return 3.3f;

    // Factory Calibrated VREFINT at 3.0V (Address for STM32L0)
    uint32_t vref_cal = *VREFINT_CAL_ADDR;

    // Formula: VDDA = 3.0V * VREF_CAL / VREF_DATA
    return (3.0f * (float)vref_cal) / (float)raw_vref;
}

float Get_Battery_Voltage(void)
{
    // 1. Get the current MCU voltage (The "Ruler")
    // This will be ~3.3V usually, but might drop if battery is dead.
    float mcu_voltage = Get_MCU_Voltage();

    // 2. Read the Raw ADC from Channel 4
    uint32_t battery_adc = Read_ADC_Channel(ADC_CHANNEL_4);

    // 3. Convert Raw ADC to Pin Voltage
    float pin_voltage = ((float)battery_adc * mcu_voltage) / 4095.0f;

    return pin_voltage * 2.35f;
}

float Get_Current(void)
{
    // 1. Get the actual voltage of the MCU (e.g., 3.6V or 3.55V)
    float mcu_voltage = Get_MCU_Voltage();

    // 2. Read the Raw ADC value (e.g., 1936)
    uint32_t adc_val = Read_ADC_Channel(ADC_CHANNEL_1);

    // 3. Convert Raw ADC to Actual Voltage on the Pin
    // This is the voltage RELATIVE to the MCU ground
    float pin_voltage = ((float)adc_val * mcu_voltage) / 4095.0f;

    float sensitivity = SENSOR_ORIGIN_SENSITIVITY * (SENSOR_SUPPLY_VOLTAGE / SENSOR_DATASHEET_VOLTAGE);
    float zero_point = SENSOR_SUPPLY_VOLTAGE / 2.0f; // 1.50V

    float current = (pin_voltage - zero_point) / sensitivity;

    current = fabsf(current);

    // 5. Deadband Filter (Optional: Force 0 if reading is very small noise)
    if (current < 0.15f) {
        return 0.0f;
    }

    return current;
}

float Get_Average_Current(uint8_t samples)
{
    float total_current = 0.0f;
    for(int i = 0; i < samples; i++)
    {
        total_current += Get_Current();

    }
    return total_current / (float)samples;
}

float Get_Average_voltage(uint8_t samples_v)
{
    float total_voltage = 0.0f;
    for(int i = 0; i < samples_v; i++)
    {
        total_voltage += Get_Battery_Voltage();
    }
    return total_voltage / (float)samples_v;
}
float Get_Pressure_kPa(void)
{
    float mcu_voltage = Get_MCU_Voltage();
    uint32_t adc_val = Read_ADC_Channel(ADC_CHANNEL_0);

    // 1. Convert Raw ADC to Voltage
    float pin_voltage = ((float)adc_val * mcu_voltage) / 4095.0f;

    // 2. Pressure Calculation
    // Assuming a standard 3.3V sensor mapping:
    // 0 kPa = 0.0V (or offset) -> Check your sensor datasheet!
    // If your sensor is 0.5V to 2.5V output:
    // return (pin_voltage - 0.5f) * ScaleFactor;

    // Based on your previous code, assuming simple linear scaling:
    return pin_voltage / 0.33f; // Adjust this formula based on your specific sensor datasheet
}

float Get_Height_from_Pressure(void)
{
	float pressure_kpa = Get_Pressure_kPa(); // Get pressure in kPa from channel 0
	return (pressure_kpa * 100 / 10.0); // convert to cm 1m == 100cm, 10.0 is maximum kpa
}
