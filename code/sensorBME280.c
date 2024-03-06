/*
 *  Created on: Aug 2023
 *      Author: Nelson D.
 *
 *    Target: MSP432P401R
 *    Compiler: Code Composer Studio 12
 *    Description: BOSCH BME280
 *    I2C TEMPERATURE SENSOR DRIVER
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
#include "sensorBME280.h"

// #define DEBUG


// DEFINE IC SPECIFIC VALUES
#define MFG_ID                      0x60   // BOSCH BME280
#define BME280_VAL_RESET            0xB6   // Soft Reset Value
#define BME280_VAL_CTRLMEAS         0x23   // 00100111 3=EN_Dev, 4=PRESS, 20=TEMP
#define BME280_VAL_CTRLHUM          0x00   // set bit0, 0=DIS_HUM; 1=EN_HUM

/* Variables */
volatile uint8_t BME280lsb = 0;
volatile uint8_t BME280msb = 0;
int BME280Trim_T1;
int BME280Trim_T2;
int BME280Trim_T3;
// printf
// %d = signed decimal integer
// %c = character
// %s = string
// %X = hexdecimal integer
// %p = pointer address

bool iniBME280(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress)
{
    printf(" \nSearching for BOSCH BME280 address:  0x%X \n", slaveAddress );

    // SET ADDRESS
    I2C_setSlaveAddress(EUSCI_XX_BASE,slaveAddress);

    // START TRANSMITTING CODE HERE
    // Ini device
    while (I2C_isBusBusy(EUSCI_XX_BASE) == EUSCI_B_I2C_BUS_BUSY);


    // TODO: BUG: We may need to revisit this for we still getting random values for the initial reading.
    // Perform Soft Reset to avoid random boot states
    I2C_masterSendMultiByteStart(EUSCI_XX_BASE, BME280_REG_RESET);
    if(!I2C_masterSendMultiByteFinish(EUSCI_XX_BASE, BME280_VAL_RESET))
    {
        printf("ERROR, BME280 NOT FOUND at address: 0x%X \n", slaveAddress);
        printf("ERROR, BME280 SOFTRESET set reg: 0x%X, val: 0x%X \n", BME280_REG_RESET, BME280_VAL_RESET);
    }

    // Get MFG ID, Expected value ID: 0x60 <7:0>
    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_DEV_ID))
    {
        BME280lsb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE);
        BME280msb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE); // returns 0?
    }
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE ); // Wait until last byte received

    // Verify MFG ID then continue to initialize device.
    if(BME280lsb == MFG_ID && BME280msb == 0)
    {
        printf("SUCCESS, BME280 MFG ID register, read: 0x%X, expected: 0x%X \n", BME280lsb, MFG_ID);
        BME280lsb = 0;
        BME280msb = 0;

        while (I2C_isBusBusy(EUSCI_XX_BASE) == EUSCI_B_I2C_BUS_BUSY); //DEVICE FOUND, START
        I2C_masterSendMultiByteStart(EUSCI_XX_BASE, BME280_REG_CTRL_MEAS); // CTRL REGISTER

        // SETUP DEFAULT VALUES FOR CTRL REGISTER
        if(!I2C_masterSendMultiByteFinish(EUSCI_XX_BASE, BME280_VAL_CTRLMEAS))
        {
            printf("ERROR, BME280 CTRL_MEAS setup: 0x%X \n", BME280_REG_CTRL_MEAS);
        }
        while (I2C_isBusBusy(EUSCI_XX_BASE) == EUSCI_B_I2C_BUS_BUSY);

        if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_CTRL_MEAS))
        {  // ctrl_meas Expected value ID: 0x27 <7:0> BME280_VAL_CTRL_MEAS
            BME280lsb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE);
            BME280msb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);
        }
        while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE ); // Wait until last byte received

        // HUM SUB-CTRL REGISTER
        I2C_masterSendMultiByteStart(EUSCI_XX_BASE, BME280_REG_CTRL_HUM);

        // SETUP DEFAULT VALUES FOR CTRL REGISTER
        if(!I2C_masterSendMultiByteFinish(EUSCI_XX_BASE, BME280_VAL_CTRLHUM))
        {
            printf("ERROR, BME280 HUM SUB-CTRL set reg: 0x%X, val: 0x%X  \n", BME280_REG_CTRL_HUM, BME280_VAL_CTRLHUM);
        }
        while (I2C_isBusBusy(EUSCI_XX_BASE) == EUSCI_B_I2C_BUS_BUSY);

        if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_CTRL_HUM))
        {
            BME280lsb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE); // 0x88 BME280lsb
            BME280msb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);// 0x89 BME280msb *FROM BUST READ
        }
        // Wait until last byte received */
        while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );
        // Capture trim values
        getBME280trim(EUSCI_XX_BASE);

        return true;
    }
    else
    {
        printf("ERROR, BME280 MFG ID register, read: 0x%X, expected: 0x%X \n", (BME280msb << 2) & BME280lsb, MFG_ID); //
        BME280msb = 0;
        BME280lsb = 0;
        return false;
    }
}


void getBME280status(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress)
{
    // SET ADDRESS
    I2C_setSlaveAddress(EUSCI_XX_BASE,slaveAddress);
}

void getBME280trim(uint32_t EUSCI_XX_BASE)
{
    //Get BME280Trim_T1
    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_TRIM_T1))
    {
         BME280lsb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE); // 0x88 BME280lsb
         BME280msb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);// 0x89 BME280msb *FROM BUS READ
    }

    // Wait until last byte received
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );
    BME280Trim_T1 = (BME280msb << 8) + BME280lsb;

    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_TRIM_T2))
    {  // Get BME280Trim_T2
        BME280lsb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE); // 0x8A BME280lsb
        BME280msb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);// 0x8B BME280msb *FROM BUS READ
    }

    // Wait until last byte received
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );
    BME280Trim_T2 = (BME280msb << 8) + BME280lsb;


    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_TRIM_T3))
    {  // Get BME280Trim_T3
        BME280lsb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE); // 0x8C BME280lsb
        BME280msb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);// 0x8D BME280msb *FROM BUS READ
    }
    // Wait until last byte received
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );
    BME280Trim_T3 = (BME280msb << 8) + BME280lsb;
}



void getBME280temp(uint32_t EUSCI_XX_BASE, uint_fast16_t slaveAddress){

    int adc_T;
    int var1;
    int var2;
    int T;
    int t_fine;
    uint8_t temp_lsb = 0;
    uint8_t temp_msb = 0;
    BME280msb = 0;
    BME280lsb = 0;

    // SET ADDRESS
    I2C_setSlaveAddress(EUSCI_XX_BASE,slaveAddress);

    //0xFA temp_msb <19:12>, 0xFB temp_lsb <11:4>
    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_TEMP_MSB))
    {
        temp_msb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE); // 0xFA temp_msb
        temp_lsb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);// 0xFB temp_lsb
    }

    // Wait until last byte received
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );

    // temp_xlsb
    if(I2C_masterSendSingleByte(EUSCI_XX_BASE, BME280_REG_TEMP_XLSB))
    {
        BME280msb = I2C_masterReceiveMultiByteStart(EUSCI_XX_BASE);// <3:0> Temp_lsb
        BME280lsb = I2C_masterReceiveMultiByteFinish(EUSCI_XX_BASE);// Discard data, Burst read
    }

    // Wait until last byte received
    while( I2C_masterIsStopSent(EUSCI_XX_BASE) != EUSCI_B_I2C_STOP_SEND_COMPLETE );

    // PROCESS VALUES
    adc_T = (temp_msb << 12) + (temp_lsb << 4) + BME280msb ;

    var1 = adc_T >> 3;
    var2 = BME280Trim_T1 << 1;
    var1 = (var1 - var2) * BME280Trim_T2;
    var1 = var1 >> 11;

    var2 = adc_T >> 4;
    var2 = var2 - BME280Trim_T1;
    var2 = var2 * var2;
    var2 = var2 >> 12;
    var2 = var2 * BME280Trim_T3;
    var2 = var2 >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    printf("BOSCH BME280 TEMP: %d °C \n", T/100); //Divide by 100
    printf("BOSCH BME280 TEMP: %d °F \n", (T*9/500) + 32); //Divide by 100
}



