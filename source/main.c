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
#define EXAMPLE_LED_GPIO BOARD_USER_LED_GPIO
#define EXAMPLE_LED_GPIO_PIN BOARD_USER_LED_PIN

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#include "st7735.h"

#include "sensor_measure.h"
#include "com_delay.h"
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
// uint8_t masterTxData[64] = {0U};
void testSPI()
{
    lpspi_transfer_t masterXfer;

    uint8_t send_data[2] = {0x8FU, 00};
    uint8_t rx_data[2];
    //    uint8_t bufp;
    masterXfer.txData = send_data;
    masterXfer.rxData = rx_data;
    masterXfer.dataSize = 2;
    masterXfer.configFlags =
        kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap; //
    LPSPI_MasterTransferBlocking(LPSPI2, &masterXfer);

    //    masterXfer.txData   = NULL;
    //    masterXfer.rxData   = &bufp;
    //    masterXfer.dataSize = 1;
    //     masterXfer.configFlags =
    //    		 kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    //    LPSPI_MasterTransferBlocking(LPSPI4, &masterXfer);

    PRINTF("test rx_data=%x %x\n", rx_data[0], rx_data[1]);
}

void I2C_SENSOR_TEST()
{
#define LPI2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define LPI2C_DATA_LENGTH 33U

    uint8_t g_master_txBuff[LPI2C_DATA_LENGTH];
    status_t reVal = kStatus_Fail;
    size_t txCount = 0xFFU;
    uint8_t deviceAddress = 0x01U;
    /* Send master blocking data to slave */
    if (kStatus_Success == LPI2C_MasterStart(LPI2C1_PERIPHERAL, LPI2C_MASTER_SLAVE_ADDR_7BIT, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts(LPI2C1_PERIPHERAL, NULL, &txCount);
        while (txCount)
        {
            LPI2C_MasterGetFifoCounts(LPI2C1_PERIPHERAL, NULL, &txCount);
        }
        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags(LPI2C1_PERIPHERAL) & kLPI2C_MasterNackDetectFlag)
        {
            return kStatus_LPI2C_Nak;
        }

        /* subAddress = 0x01, data = g_master_txBuff - write to slave.
          start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
        reVal = LPI2C_MasterSend(LPI2C1_PERIPHERAL, &deviceAddress, 1);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(LPI2C1_PERIPHERAL);
            }
            return -1;
        }

        reVal = LPI2C_MasterSend(LPI2C1_PERIPHERAL, g_master_txBuff, LPI2C_DATA_LENGTH);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(LPI2C1_PERIPHERAL);
            }
            return -1;
        }

        reVal = LPI2C_MasterStop(LPI2C1_PERIPHERAL);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
    }
}
/*!
 * @brief Main function
 */
int main(void)
{
    /* Board pin init */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    /* Set the PWM Fault inputs to a low value */
    XBARA_Init(XBARA);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm2Fault0);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm2Fault1);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm12Fault2);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm12Fault3);

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
//    PRINTF("float:%.2f\n", 3.145);
    //    ST7735_Init();
    //    ST7735_FillScreen(ST7735_BLUE);
    //	ST7735_Init();
    //	ST7735_FillScreen(ST7735_BLUE);
    //	ST7735_FillScreen(ST7735_RED);
    //	ST7735_FillScreen(ST7735_BLACK);
    //	ST7735_FillScreen(ST7735_WHITE);
    //	ST7735_FillScreen(ST7735_GREEN);
    //    testSPI();
    //    SysTick_DelayTicks(2U);
    extern void lsm6ds3tr_c_read_data_polling(void);
    //    lsm6ds3tr_c_read_data_polling();

    extern void lis2mdl_read_data_simple(void);
    //    lis2mdl_read_data_simple();
    uint8_t hell_str[] = "hello ya\n";
    serial_tx(hell_str, strlen(hell_str));

    /* Set the load okay bit for all submodules to load registers from their buffer */
    PWM_SetPwmLdok(PWM2, kPWM_Control_Module_0, true);

    /* Start the PWM generation from Submodules 0, 1 and 2 */
    PWM_StartTimer(PWM2, kPWM_Control_Module_0);
    uint32_t pwmVal = 1;
    //	ENC_DoSoftwareLoadInitialPositionValue(ENC1); /* Update the position counter with initial value. */
    //	uint32_t mCurPosValue;

    BH1730_test();
    double rth[2];
    float BH1730_light;
    while (1)
    {

        if(0==read_temp_rh_1ch(rth))
        {
            PRINTF("NSHT30 READ:%3.4f,%3.6f%%\r\n",rth[0],rth[1]);

        }
        BH1730_light=BH1730_readLux();
        PRINTF("BH1730_light=%.3f\n", BH1730_light);
        //    	 /* This read operation would capture all the position counter to responding hold registers. */
        //		mCurPosValue = ENC_GetPositionValue(ENC1);
        //
        //		/* Read the position values. */
        //		PRINTF("Current position value: %ld\r\n", mCurPosValue);
        //		PRINTF("Position differential value: %d\r\n", (int16_t)ENC_GetHoldPositionDifferenceValue(ENC1));
        //		PRINTF("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(ENC1));
        /* Delay 1000 ms */
        SysTick_DelayTicks(1000U);
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

        pwmVal = pwmVal + 10;

        /* Reset the duty cycle percentage */
        if (pwmVal > 100)
        {
            pwmVal = 1;
        }

        /* Update duty cycles for all 3 PWM signals */
        PWM_UpdatePwmDutycycle(PWM2, kPWM_Module_0, kPWM_PwmB, kPWM_SignedCenterAligned, pwmVal);

        /* Set the load okay bit for all submodules to load registers from their buffer */
        PWM_SetPwmLdok(PWM2, kPWM_Control_Module_0, true);


    }
}
