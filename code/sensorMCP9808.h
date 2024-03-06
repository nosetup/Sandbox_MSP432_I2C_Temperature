#ifndef SENSORMCP9808_H_
#define SENSORMCP9808_H_


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
#define MCP9808_REG_CONFIG           0x01   // Reading the CONFIG Register:
#define MCP9808_REG_TUPPER           0x02   // Reading from the TUPPER Register:
#define MCP9808_REG_TLOWER           0x03   // Reading from the TLOWER Register:
#define MCP9808_REG_TCRIT            0x04   // Reading from the TCRIT Critical Temperature Register:
#define MCP9808_REG_TEMP             0x05   // AMBIENT TEMPERATURE REGISTER
#define MCP9808_REG_MFG_ID           0x06   // Manufacturer ID register
#define MCP9808_REG_REVISION         0x07   // MSB ID (MCP9808 is 0x04 (hex)) LSB(Revision)
#define MCP9808_REG_RESOLUTION       0x08   // RESOLUTION

bool iniMCP9808(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress);
void getMCP9808status(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress);
void getMCP9808temp(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress);

#endif /* SENSORMCP9808_H_ */
