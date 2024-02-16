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

#include "fsl_xbara.h"
#include "sensor_measure.h"
#include "com_delay.h"


#include "camera.h"
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
/* BOARD_CAM_VS_handle callback function */
void BOARD_CAM_VS_callback(void *param) {
  /* Place your code here */
	/* clear the interrupt status */
	GPIO_PortClearInterruptFlags(BOARD_CAM_VS_GPIO, BOARD_CAM_VS_GPIO_PIN_MASK);
}
uint16_t   ov7670_finish_flag = 0;    //一场图像采集完成标志位
//uint16_t CAM_BUFFER[160*128];
void CAM_DMA_COMPLETE(FLEXIO_CAMERA_Type *base, flexio_camera_edma_handle_t *handle, status_t status, void *userData)
{

//	ov7670_finish_flag=1;
	ov7670_finish_flag++;

}

/* PIT_IRQn interrupt handler */
void PIT_IRQHANDLER(void) {
  /*  Place your code here */
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


void ADC_DEMO()
{
#define DEMO_ADC_BASE          ADC1
#define DEMO_ADC_USER_CHANNEL  0U
#define DEMO_ADC_CHANNEL_GROUP 0U
    uint8_t ticks=10;
    while(ticks--)
    {
        /*
         When in software trigger mode, each conversion would be launched once calling the "ADC_SetChannelConfig()"
         function, which works like writing a conversion command and executing it. For another channel's conversion,
         just to change the "channelNumber" field in channel's configuration structure, and call the
         "ADC_SetChannelConfig() again.
        */
          ADC_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP, &ADC1_channels_config[0]);
        while (0U == ADC_GetChannelStatusFlags(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP))
        {
        }
        PRINTF("ADC Value: %d\r\n", ADC_GetChannelConversionValue(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP));
        HAL_Delay(300);
    }

    }
/*!
 * @brief Main function
 */

edma_transfer_config_t transferConfig;
void CAM_OV7670_DEMO()
{
	/* lcd init  */

	/*camera ov7670 init ,12M XCLK,PWDN LOW,RST HIGH*/
	GPIO_PinWrite(BOARD_CAM_PWDN_GPIO, BOARD_CAM_PWDN_GPIO_PIN, 0U);
	GPIO_PinWrite(BOARD_CAM_RES_GPIO, BOARD_CAM_RES_GPIO_PIN, 0U);
	HAL_Delay(20);
	GPIO_PinWrite(BOARD_CAM_RES_GPIO, BOARD_CAM_RES_GPIO_PIN, 1U);
	Camera_Init_Device(LPI2C3_PERIPHERAL, FRAMESIZE_QQVGA2);
    /* Set the load okay bit for all submodules to load registers from their buffer */
    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_0, true);
    /* Start the PWM generation from Submodules 0, 1 and 2 */
    PWM_StartTimer(PWM1, kPWM_Control_Module_0);
	/*cam dma transfer start */
    EDMA_PrepareTransfer(&transferConfig,
        (void *)FLEXIO_CAMERA_GetRxBufferAddress(&FLEXIO1_peripheralConfig),
        8,
        (void *)(FLEXIO1_Camera_Buffer),
        8,
        8,
		FLEXIO1_FRAME_WIDTH *FLEXIO1_FRAME_HEIGHT,
        kEDMA_MemoryToMemory);


        EDMA_SubmitTransfer(&FLEXIO1_FLEXIO_0_Handle, &transferConfig);

//        switch(4*flexio_shift_count)
//        {
//            case 4:     s_addr_modulo = kEDMA_Modulo4bytes;break;
//            case 8:     s_addr_modulo = kEDMA_Modulo8bytes;break;
//            case 16:    s_addr_modulo = kEDMA_Modulo16bytes;break;
//            case 32:    s_addr_modulo = kEDMA_Modulo32bytes;break;
//            default:assert(0);  //参数有误
//        }


        EDMA_SetModulo(DMA0,FLEXIO1_FLEXIO_0_DMA_CHANNEL,kEDMA_Modulo8bytes,kEDMA_ModuloDisable);
        EDMA_StartTransfer(&FLEXIO1_FLEXIO_0_Handle);
    PRINTF("OV7670 START\n");
	/*loop frame show*/
    while(1)
    {


    }
}
int main(void)
{
    /* Board pin init */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    /* Set the PWM Fault inputs to a low value */
    XBARA_Init(XBARA);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault0);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault1);
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
//       ST7735_Init();
//       ST7735_FillScreen(ST7735_BLUE);
    	ST7735_Init();
    	ST7735_FillScreen(ST7735_BLUE);
    	ST7735_FillScreen(ST7735_RED);
    	ST7735_FillScreen(ST7735_BLACK);
    	ST7735_FillScreen(ST7735_WHITE);
    	ST7735_FillScreen(ST7735_GREEN);
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

//    CAM_OV7670_DEMO();

    BH1730_test();
    double rth[2];
    float BH1730_light;
    ADC_DEMO();

    while (1)
    {

        if (0 == read_temp_rh_1ch(rth))
        {
            PRINTF("NSHT30 READ:%3.4f,%3.6f%%\r\n", rth[0], rth[1]);
        }
        BH1730_light = BH1730_readLux();
        PRINTF("BH1730_light=%.3f\n", BH1730_light);
        //    	 /* This read operation would capture all the position counter to responding hold registers. */
        //		mCurPosValue = ENC_GetPositionValue(ENC1);
        //
        //		/* Read the position values. */
        //		PRINTF("Current position value: %ld\r\n", mCurPosValue);
        //		PRINTF("Position differential value: %d\r\n", (int16_t)ENC_GetHoldPositionDifferenceValue(ENC1));
        //		PRINTF("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(ENC1));
        /* Delay 1000 ms */
        HAL_Delay(1000U);
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
