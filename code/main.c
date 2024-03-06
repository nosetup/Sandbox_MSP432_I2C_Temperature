/*
 * main.h
 *
 *  Created on: Aug 2023
 *      Author: Nelson D.
 *
 *  Target: MSP432P401R
 *  Compiler: Code Composer Studio 12
 *  Example interface with Bosch BME280 and Microchip MCP9808
 *  Press button and Read Temp values from Console.
 */

#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include <stdarg.h>
#include <clock.h>

// I2C examples
#include "i2c.h"
#include "interrupt.h"
#include "gpio.h"

// Sensor Examples
#include "sensorMCP9808.h"
#include "sensorBME280.h"

// #define DEBUG

/* Slave Address */
#define SLAVE_ADDRESS_1         0x00    //
#define BME280_1_ADDRESS        0x76    // BME280 0x76=GND, 0x77=HIGH SDO
#define MCP9808_1_ADDRESS       0x18    // 0x18 MCP9808 001-1XXX PULL DOWN ON A0-A3

#define NUM_OF_REC_BYTES        10
#define NUM_OF_RX_BYTES         1
#define NUM_OF_TX_BYTES         10

#define I2C_PINS GPIO_PORT_P6, GPIO_PIN4 + GPIO_PIN5
#define EUSCI_XX_BASE EUSCI_B1_BASE // configures P6.4 DATA and P6.5 CLK

//#define I2C_PINS GPIO_PORT_P1, GPIO_PIN6 + GPIO_PIN7
//#define EUSCI_XX_BASE EUSCI_B0_BASE // configures P1.7 SCLK and P1.6 SDATA

/* Variables */
volatile uint16_t response = 0;
volatile uint8_t lsb = 0;
volatile uint8_t msb = 0;
static volatile uint32_t xferIndex = 0;
static volatile bool stopSent;
//const uint8_t TXData[2] = {0x04, 0x00};
//static uint8_t RXData[NUM_OF_REC_BYTES]; // todo:


//------------LaunchPad_Input------------
// Input from Switches
// Input: none
// Output: 0x00 none
//         0x01 Button1
//         0x02 Button2
//         0x03 both Button1 and Button2
uint8_t LaunchPadButton_Input(void)
{
  return ((((~(P1->IN))&0x10)>>3)|(((~(P1->IN))&0x02)>>1));   // read P1.4,P1.1 inputs
}

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    #ifdef DEBUG
    printf ("\r start of test \r");
    #endif


    // INI SIDE BUTTON INPUT
    // Press button to transmit IR
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;      // 1) Configure P1.4, P1.1 as GPIO
    P1->DIR  &= ~0x12;      // 2) Set P1.4 and P1.1 input
    P1->REN  |=  0x12;      // 3) Enable pull resistors on P1.4 and P1.1
    P1->OUT  |=  0x12;      //    P1.4 and P1.1 are pull-up


    Clock_Init48MHz();

    eUSCI_I2C_MasterConfig eUSCB1_I2C;
    eUSCB1_I2C.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK; //uint_fast8_t SMCLK
    eUSCB1_I2C.i2cClk = 3000000; //uint32_t
    eUSCB1_I2C.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS; //uint32_t 100khz
    eUSCB1_I2C.byteCounterThreshold = 0; //uint_fast8_t
    eUSCB1_I2C.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP; //uint_fast8_t no autostop

    // configures P6.4 DATA and P6.5 CLK
    //P6->SEL0 |= 0x30;              // set SEL0, 0011 0000
    //P6->SEL1 &= ~0x30;             // clr SEL0, 0011 0000

    // alternate configures P6.4 DATA and P6.5 CLK
    GPIO_setAsPeripheralModuleFunctionInputPin(I2C_PINS, GPIO_PRIMARY_MODULE_FUNCTION);

    I2C_disableModule(EUSCI_XX_BASE); //Change 1
    // Initializing I2C Master
    I2C_initMaster(EUSCI_XX_BASE, &eUSCB1_I2C);
    I2C_setSlaveAddress(EUSCI_XX_BASE,SLAVE_ADDRESS_1);

    /* Set in receive mode */
    //I2C_setMode(EUSCI_XX_BASE, EUSCI_B_I2C_RECEIVE_MODE);

    /* Set Master in transmit mode */
    I2C_setMode(EUSCI_XX_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

    /* Enable and clear the interrupt flag */
    I2C_enableModule(EUSCI_XX_BASE);

    I2C_clearInterruptFlag(EUSCI_XX_BASE,
    EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);

    //Enable master Receive interrupt
    I2C_enableInterrupt(EUSCI_XX_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
    //Interrupt_enableSleepOnIsrExit(); //change2
    Interrupt_enableInterrupt(INT_EUSCIB1);  // interrupt 37 = IRQ21

    // priority for IRQ21  EUSCIB1 Interrupt note: didn't see difference between this and I_EI above.
    //NVIC->IP[5] = (NVIC->IP[2]&0xFFFF00FF)|0x00004000; // priority 2
    //NVIC->ISER[0] = 0x00100000; // enable interrupt 21 (EUSCIB1 Interrupt) in NVIC


    //INI TEMP SENSORS
    if(iniBME280(EUSCI_XX_BASE, BME280_1_ADDRESS))
    {
        getBME280status(EUSCI_XX_BASE, BME280_1_ADDRESS);
        getBME280temp(EUSCI_XX_BASE, BME280_1_ADDRESS);
    }

    if(iniMCP9808(EUSCI_XX_BASE, MCP9808_1_ADDRESS))
    {
        getMCP9808temp(EUSCI_XX_BASE, MCP9808_1_ADDRESS);
    }

    while(1){

        if(LaunchPadButton_Input())
        {
            getBME280temp(EUSCI_XX_BASE, BME280_1_ADDRESS);
            getMCP9808temp(EUSCI_XX_BASE, MCP9808_1_ADDRESS);
            Clock_Delay1ms(1000);//add delay
        }
        else
        {

        }
    }
}



