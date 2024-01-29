/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_LED_GPIO     BOARD_USER_LED_GPIO
#define EXAMPLE_LED_GPIO_PIN BOARD_USER_LED_PIN


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#include "st7735.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter;
/* The PIN status */
volatile bool g_pinSet = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}
uint8_t masterTxData[64] = {0U};
/*!
 * @brief Main function
 */
int main(void)
{
    /* Board pin init */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    BOARD_InitBootPeripherals();
//    BOARD_InitDebugConsole();
    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }
    PRINTF("APP START !\n");
//    ST7735_Init();
//    ST7735_FillScreen(ST7735_BLUE);
//    uint8_t masterTxData[64] = {0U};
	// #define TRANSFER_SIZE 64
//    uint32_t i;
//    for (i = 0U; i < TRANSFER_SIZE; i++)
//    {
//    	masterTxData[i] = (i + 1) % 256U;
//        GPIO_ClearPinsOutput(BOARD_LCD_RES_PORT,BOARD_LCD_RES_PIN_MASK);
//        SysTick_DelayTicks(1);
//        GPIO_SetPinsOutput(BOARD_LCD_RES_PORT,BOARD_LCD_RES_PIN_MASK);
//    }
//    lpspi_transfer_t masterXfer;
//    /*Start master transfer, transfer data to slave.*/
//	masterXfer.txData   = masterTxData;
//	masterXfer.rxData   = NULL;
//	masterXfer.dataSize = TRANSFER_SIZE;
//	masterXfer.configFlags =
//			kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
//
//	LPSPI_MasterTransferBlocking(LPSPI4, &masterXfer);
	ST7735_Init();
	ST7735_FillScreen(ST7735_BLUE);
	ST7735_FillScreen(ST7735_RED);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_FillScreen(ST7735_GREEN);
    while (1)
    {
        /* Delay 1000 ms */
        SysTick_DelayTicks(100U);
        if (g_pinSet)
        {
            GPIO_PinWrite(BOARD_U_LED_GPIO, BOARD_U_LED_GPIO_PIN, 0U);
            g_pinSet = false;
//            PRINTF("LED OFF !\n");
        }
        else
        {
            GPIO_PinWrite(BOARD_U_LED_GPIO, BOARD_U_LED_GPIO_PIN, 1U);
            g_pinSet = true;
//            PRINTF("LED ON !\n");
        }
    }
}
