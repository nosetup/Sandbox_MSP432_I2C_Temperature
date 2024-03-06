/*
 *  Created on: Aug 2023
 *      Author: Nelson D.
 *
 *      Target: MSP432P401R
 *      Compiler: Code Composer Studio 12
 *      Description: MICROCHIP MCP9808
 *      I2C TEMPERATURE SENSOR DRIVER
 */

#include <stdio.h>
#include <stdint.h>
#include "msp.h"

// I2C examples
#include "i2c.h"
#include "interrupt.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include <msp.h>
#include "eusci.h"
#include "sensorMCP9808.h"

// #define DEBUG


// DEFINE IC SPECIFIC VALUES
#define MFG_ID                       0x54   // MICROCHIP MCP9808
#define MCP9808_VAL_RESET            0xB6   // Soft Reset Value
#define MCP9808_VAL_CTRLMEAS         0x23   // 00100111 3=EN_Dev, 4=PRESS, 20=TEMP
#define MCP9808_VAL_CTRLHUM          0x00   // set bit0, 0=DIS_HUM; 1=EN_HUM

#define I2C_RW_TIMEOUT      10000

/* Variables */
uint16_t MCP9808response = 0;
uint8_t MCP9808lsb = 0;
uint8_t MCP9808msb = 0;

// printf
// %d = signed decimal integer
// %c = character
// %s = string
// %X = hexdecimal integer
// %p = pointer address

bool iniMCP9808(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress)
{
    printf(" \nSearching for MICROCHIP MCP9808 address:  0x%X \n", slaveAddress );
    //Setup Address
    I2C_setSlaveAddress(EUSCI_XX_BASE,slaveAddress);
    /* Wait until byte is sent before receive.*/
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );     /* Wait until last byte received */

    // The Manufacturer ID for the MCP9808 is 0x0054 (hexadecimal).
    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, MCP9808_REG_MFG_ID))
    {
        MCP9808msb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE); // returns 0
        MCP9808lsb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE); // returns 54
    }
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );     /* Wait until last byte received */

    if(MCP9808msb == 0x00 && MCP9808lsb == MFG_ID)
    {
        printf("SUCCESS, MCP9808 MFG ID register, read: 0x%X, expected 0x%X \n", MCP9808lsb, MFG_ID); //
        MCP9808lsb = 0;
        MCP9808msb = 0;
        return true;
    }
    else
    {
        printf("ERROR, MCP9808 NOT FOUND at address: 0x%X \n", slaveAddress);
        printf("ERROR, MCP9808 MFG ID register, read: 0x%X, expected: 0x%X \n", MCP9808msb, MFG_ID); //
        MCP9808lsb = 0;
        MCP9808msb = 0;
        return false;
    }
}

void getMCP9808status(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress)
{
    MCP9808msb = 0;
    MCP9808lsb = 0;

    // START TRANSMITTING CODE HERE
    I2C_setSlaveAddress(EUSCI_XX_BASE,slaveAddress);
    //Writing +90°C to the TUPPER Register w0x02,w0x05,w0xA0
    writeI2C2BYTESTimeout(EUSCI_XX_BASE, MCP9808_REG_TUPPER, 0x05, 0xA0, I2C_RW_TIMEOUT);

    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );     /* Wait until last byte received */

    //MCP9808
    // Reading the CONFIG Register:
    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, MCP9808_REG_TUPPER))
    {
        MCP9808msb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE);
        MCP9808lsb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);
    }

#ifdef DEBUG
    printf("MICROCHIP MCP9808msb TUPPER: %d \n", MCP9808msb);
    printf("MICROCHIP MCP9808lsb TUPPER: %d \n", MCP9808lsb);
#endif
}


void getMCP9808temp(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress)
{
    MCP9808msb = 0;
    MCP9808lsb = 0;
    // START TRANSMITTING CODE HERE
    I2C_setSlaveAddress(EUSCI_XX_BASE,slaveAddress);

    // GET TEMP VALUES
    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, MCP9808_REG_TEMP))
    {
        MCP9808msb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE);
        MCP9808lsb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);
    }
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE ); // WAIT FOR STOP

    #ifdef DEBUG
        printf("MICROCHIP MCP9808msb TEMP: %d \n", MCP9808msb);
        printf("MICROCHIP MCP9808lsb TEMP: %d \n", MCP9808lsb);
    #endif

    // PROCESS TEMP VALUES
    //TA TCRIT
    if ((MCP9808msb & 0x80) == 0x80)
    {
        printf("MICROCHIP MCP9808 Trip register status = CRITICAL\n");
    }

    //TA > TUPPER
    if ((MCP9808msb & 0x40) == 0x40)
    {
        printf("MICROCHIP MCP9808 Trip register status = TUPPER\n");
    }

    //TA < TLOWER
    if ((MCP9808msb & 0x20) == 0x20)
    {
        printf("MICROCHIP MCP9808 Trip register status = TLOWER\n");
    }

    MCP9808msb = MCP9808msb & 0x1F; //Clear flag bits
    //TA < 0°C
    if ((MCP9808msb & 0x10) == 0x10)
    {
        MCP9808msb = MCP9808msb & 0x0F;//Clear SIGN
        MCP9808response = 256 - ((MCP9808msb * 16) + (MCP9808lsb / 16));
    }
    //TA 0°C
    else
    {
        //Temperature = Ambient Temperature (°C)
        MCP9808response = ((MCP9808msb * 16) + (MCP9808lsb / 16));
     }

#ifdef DEBUG
   printf("MICROCHIP MCP9808msb TEMP: %d \n", MCP9808msb);
   printf("MICROCHIP MCP9808lsb TEMP: %d \n", MCP9808lsb);
#endif

    printf("MICROCHIP MCP9808 TEMP: %d °C\n", MCP9808response);
    printf("MICROCHIP MCP9808 TEMP: %d °F \n", (MCP9808response*9/5) + 32); //Divide by 100
}

