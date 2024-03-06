

#ifndef SENSORBME280_H_
#define SENSORBME280_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "msp432.h"
#include "i2c.h"
#include "interrupt.h"
#include "gpio.h"
#include <stdbool.h>
#include <msp.h>
#include "eusci.h"

// DEFINE IC SPECIFIC REGISTERS, REFER TO DATASHEET
//#define BME280_REG_HUM_LSB          0xFE   // hum_lsb <7:0> burst read, not used
#define BME280_REG_HUM_MSB          0xFD   // hum_msb <7:0>
#define BME280_REG_TEMP_XLSB        0xFC   // temp_xlsb <7:4>
//#define BME280_REG_TEMP_LSB         0xFB   // temp_lsb <7:0> burst read, not used
#define BME280_REG_TEMP_MSB         0xFA   // temp_msb <7:0>
#define BME280_REG_PRESS_XLSB       0xF9   // press_xlsb <7:4>
//#define BME280_REG_PRESS_LSB        0xF8   // press_lsb <7:0> burst read, not used
#define BME280_REG_PRESS_MSB        0xF7   // press_msb <7:0>
#define BME280_REG_CTRL_MEAS        0xF4   // Control Register Humi
#define BME280_REG_CTRL_HUM         0xF2   // Control Register Humi
#define BME280_REG_RESET            0xE0   // Reset
#define BME280_REG_DEV_ID           0xD0   // DEVICE ID: 0x60 <7:0>
#define BME280_REG_TRIM_T1          0x88   // BME280Trim_T1
#define BME280_REG_TRIM_T2          0x8A   // BME280Trim_T2
#define BME280_REG_TRIM_T3          0x8C   // BME280Trim_T3



bool iniBME280(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress);
void getBME280trim(uint32_t EUSCI_XX_BASE);
void getBME280status(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress);
void getBME280temp(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress);

#endif /* SENSORBME280_H_ */
