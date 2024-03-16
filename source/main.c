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
#include "stdio.h"

#include "fsl_xbara.h"
#include "sensor_measure.h"
#include "com_delay.h"

#include "camera.h"
#include "bmp_coder.h"
#include "multi_button.h"

#include "fsl_sd_disk.h"
#include "sdmmc_config.h"
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

uint8_t fram_choice = 0;
uint16_t ov2640_finish_flag = 0, ov2640_frame_conter = 0; // 一场图像采集完成标志位
edma_transfer_config_t transferConfig;
flexio_camera_transfer_t cam_xfer;

uint8_t encoder_key_pressed = 0;
uint8_t im_par_chose = 0;
uint8_t work_mode = 0;
/* BOARD_CAM_VS_handle callback function */
void BOARD_CAM_VS_callback(void *param)
{
    /* Place your code here */
    /* clear the interrupt status */
    GPIO_PortClearInterruptFlags(BOARD_CAM_VS_GPIO, BOARD_CAM_VS_GPIO_PIN_MASK);

    //	ov2640_finish_flag++;
    //    ov2640_frame_conter++;
    ov2640_finish_flag = 1;
    //	fram_choice=fram_choice ? 0:1;
    //	DMA0->TCD[0].DADDR = (uint32_t)FLEXIO1_Camera_Buffer[0];

    FLEXIO_CAMERA_ClearStatusFlags(&FLEXIO1_peripheralConfig,
                                   kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
    //	    ov2640_finish_flag=1;
    /* Enable DMA channel request. */
    //    DMA0->SERQ = DMA_SERQ_SERQ(0);

    //	 cam_xfer.dataAddress=(uint32_t)FLEXIO1_Camera_Buffer[fram_choice];
    if (0 == work_mode)
        FLEXIO_CAMERA_TransferReceiveEDMA(&FLEXIO1_peripheralConfig, &FLEXIO1_Camera_eDMA_Handle, &cam_xfer);

    __DSB();
}

void CAM_DMA_COMPLETE(FLEXIO_CAMERA_Type *base, flexio_camera_edma_handle_t *handle, status_t status, void *userData)
{

    //	ov2640_finish_flag++;
    /* Enable DMA channel request. */
    //    DMA0->SERQ = DMA_SERQ_SERQ(FLEXIO_CAMERA_DMA_CHN);
}

/* PIT_IRQn interrupt handler */
void PIT_IRQHANDLER(void)
{
    /*  Place your code here */
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    // 5ms
    button_ticks();
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
   Store immediate overlapping exception return operation might vector to incorrect interrupt. */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void ADC_DEMO()
{
#define DEMO_ADC_BASE ADC1
#define DEMO_ADC_USER_CHANNEL 0U
#define DEMO_ADC_CHANNEL_GROUP 0U
    uint8_t ticks = 10;
    while (ticks--)
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

struct _image_parameters
{
    uint8_t quality;   // 2-59
    int8_t brightness; //-2 - +2
    int8_t contrast    //-2 - +2
};

extern uint8_t OV2640_image_param_set(uint8_t par1, int8_t value);

void image_parameters_display(struct _image_parameters param)
{
    char d_buf[14];
    sprintf(d_buf, "Q:%2d B:%2d C:%2d", param.quality, param.brightness, param.contrast);
//    ST7735_WriteString_lucency(10, 10, d_buf, Font_7x10, ST7735_RED);
    ST7735_WriteString(10, 10, d_buf, Font_11x18, ST7735_RED,ST7735_BLACK);

    // char d_buf[2];
    // d_buf[0]=param.quality/10 +'0' ;
    // d_buf[1]=param.quality%10 +'0' ;
    // ST7735_WriteString_lucency(30,0,d_buf,Font_7x10,ST7735_RED);

    // d_buf[0]=param.brightness> 0? '+' :(param.brightness==0 ? '0':'-');
    // d_buf[1]=(param.brightness>0 ?param.brightness:-param.brightness) +'0' ;
    // ST7735_WriteString_lucency(30,20,d_buf,Font_7x10,ST7735_RED);
    // d_buf[0]=param.contrast> 0? '+' :(param.contrast==0 ? '0':'-');
    // d_buf[1]=(param.contrast>0 ?param.contrast:-param.contrast) +'0' ;
    // ST7735_WriteString_lucency(30,40,d_buf,Font_7x10,ST7735_RED);
}
struct Button encoder_KEY;
uint8_t btn1_id = 0;

uint8_t read_button_GPIO(uint8_t button_id)
{
    return GPIO_ReadPinInput(BOARD_ENC_Buttion_GPIO, BOARD_ENC_Buttion_GPIO_PIN);
}
void BTN1_PRESS_UP_Handler(void *btn)
{
    PRINTF("BTN1_PRESS_UP\n");

    // encoder_key_pressed=1;
    if (im_par_chose < 2)
        im_par_chose++;
    else
        im_par_chose = 0;
    PRINTF("im_par_chose=%d\n",im_par_chose);

}
void BTN1_DOUBLE_CLICK_Handler(void *btn)
{
    PRINTF("BTN1_DOUBLE_CLICK\n");
    PRINTF("SCREEN SHOT\n");
    encoder_key_pressed = 1;
}
void BTN1_LONG_PRESS_START_Handler(void *btn)
{
    PRINTF("BTN1_LONG_PRESS_START\n");
    work_mode = !work_mode;
}
uint8_t enc_rotate()
{
    static uint32_t mCur_temp = 0;
    uint8_t ret = 0;
    uint32_t mCurPosValue = ENC_GetPositionValue(ENC1);
    if (mCur_temp < mCurPosValue)
        ret = 1;
    else if (mCur_temp > mCurPosValue)
        ret = 2;
    else
        ret = 0;
    mCur_temp = mCurPosValue;
    return ret;
}
void im_par_change_handler(struct _image_parameters *param)
{
    uint8_t menc_r = enc_rotate();
    switch (im_par_chose)
    {
    case 0:
        if (1 == menc_r && param->quality < 59)
            param->quality++;
        else if (2 == menc_r && param->quality > 2)
            param->quality--;
        break;
    case 1:
        if (1 == menc_r && param->brightness < 2)
            param->brightness++;
        else if (2 == menc_r && param->brightness > -2)
            param->brightness--;
        break;
    case 2:
        if (1 == menc_r && param->contrast < 2)
            param->contrast++;
        else if (2 == menc_r && param->contrast > -2)
            param->contrast--;
        break;

    default:
        break;
    }
}
void dis_pic_change_handler(uint16_t *param)
{
    uint8_t menc_r = enc_rotate();
    uint16_t temp=*param;
    if (1 == menc_r)
    {
        *param= temp+1;
    }
    else if (2 == menc_r)
    {
    	*param= temp-1;
    }
}
void CAM_OV2640_DEMO()
{
    struct _image_parameters m_image_parameters;
    /* lcd init  */
    m_image_parameters.brightness=1;
    m_image_parameters.contrast=1;
    m_image_parameters.quality=20;
    /*camera OV2640 init ,,PWDN LOW,RST HIGH*/
    GPIO_PinWrite(BOARD_CAM_PWDN_GPIO, BOARD_CAM_PWDN_GPIO_PIN, 0U);
    GPIO_PinWrite(BOARD_CAM_RES_GPIO, BOARD_CAM_RES_GPIO_PIN, 0U);
    HAL_Delay(20);
    GPIO_PinWrite(BOARD_CAM_RES_GPIO, BOARD_CAM_RES_GPIO_PIN, 1U);

    /* Clear all the flag. */
    FLEXIO_CAMERA_ClearStatusFlags(&FLEXIO1_peripheralConfig, kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
    /* Enable FlexIO. */
    FLEXIO_CAMERA_Enable(&FLEXIO1_peripheralConfig, true);

    /*cam dma transfer start */
    cam_xfer.dataAddress = (uint32_t)FLEXIO1_Camera_Buffer[0];
    cam_xfer.dataNum = 2 * FLEXIO1_FRAME_WIDTH * FLEXIO1_FRAME_HEIGHT;

    FLEXIO_CAMERA_TransferReceiveEDMA(&FLEXIO1_peripheralConfig, &FLEXIO1_Camera_eDMA_Handle, &cam_xfer);
    // 160x120
    Camera_Init_Device(LPI2C3_PERIPHERAL, FRAMESIZE_QQVGA);
    PRINTF("OV camera init ok\n");

    /*因为用的OV2640模块上自带24M晶振，故不需要PWM做XCLK*/
    /* Set the load okay bit for all submodules to load registers from their buffer */
    //    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_0, true);
    /* Start the PWM generation from Submodules 0, 1 and 2 */
    //    PWM_StartTimer(PWM1, kPWM_Control_Module_0);

    PRINTF("OV2640 START\n");
    /*loop frame show*/
    char picName[20];
    uint16_t picID = 1;
    uint16_t picID_read = 1, picID_read_last = 0;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
    FRESULT error;
#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        //        return -1;
    }
#endif

    static PressEvent encoder_KEY_event_val;

    button_init(&encoder_KEY, read_button_GPIO, 0, btn1_id);
    button_start(&encoder_KEY);
    button_attach(&encoder_KEY, PRESS_UP, BTN1_PRESS_UP_Handler);

    button_attach(&encoder_KEY, DOUBLE_CLICK, BTN1_DOUBLE_CLICK_Handler);
    button_attach(&encoder_KEY, LONG_PRESS_START, BTN1_LONG_PRESS_START_Handler);

    ENC_DoSoftwareLoadInitialPositionValue(ENC1); /* Update the position counter with initial value. */

    while (1)
    {
        if (!work_mode)
        {
            im_par_change_handler(&m_image_parameters);
//            PRINTF("m_image_parameters B:%d C:%d Q:%d\n",m_image_parameters.brightness,m_image_parameters.contrast,m_image_parameters.quality);
        }
        else
        {
            dis_pic_change_handler(&picID_read);
            if (picID_read < 1)
                picID_read = 1;
            if (picID_read > picID)
                picID_read = picID;

            if (picID_read_last != picID_read)
            {
                sprintf(picName, "/m_dir/screen%d.bmp", picID_read);
                bmp_pic_display(picName);
//                picID_read_last = picID_read;
                PRINTF("pic read ok.:%d\r\n",picID_read);
                HAL_Delay(100);
            }
            picID_read_last = picID_read;
        }

        if (ov2640_finish_flag && !work_mode)
        {
            ov2640_finish_flag = 0;
            /* PRINTF("OV2640 frame get\n");*/
            ST7735_FillRGBRect(0, 0, (uint8_t *)&FLEXIO1_Camera_Buffer[0][0][0], 160, 120);
            image_parameters_display(m_image_parameters);
            if (encoder_key_pressed)
            {
                encoder_key_pressed = 0;
                sprintf(picName, "/m_dir/screen%d.bmp", picID);
                picID++;
                bmp_pic_write(_T(picName), (uint8_t *)&FLEXIO1_Camera_Buffer[0][0][0]);
            }


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
    ST7735_Init();
    ST7735_FillScreen(ST7735_BLACK);
    //    	ST7735_Init();
    //    	ST7735_FillScreen(ST7735_BLUE);
    //    	ST7735_FillScreen(ST7735_RED);
    //    	ST7735_FillScreen(ST7735_BLACK);
    //    	ST7735_FillScreen(ST7735_WHITE);
    //    	ST7735_FillScreen(ST7735_GREEN);
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
    FRESULT error;
#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return -1;
    }
#endif

    // char picName[20];
    // for (uint16_t a = 0; a < 100; a++)
    // {
    //     sprintf(picName, "/m_dir/BB%d.bmp", a);
    //     bmp_pic_display(picName);
    //     HAL_Delay(500);
    // }
//    char d_buf[14];
//    char quality=8,brightness=9,contrast=10;
//    	sprintf(d_buf, "Q:%2d B:%2d C:%2d", quality,brightness,contrast);
//        ST7735_WriteString_lucency(10, 10, d_buf, Font_7x10, ST7735_RED);

    CAM_OV2640_DEMO();

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
    	uint32_t mCurPosValue;

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
        		mCurPosValue = ENC_GetPositionValue(ENC1);
        //
        //		/* Read the position values. */
        		PRINTF("Current position value: %ld\r\n", mCurPosValue);
        		PRINTF("Position differential value: %d\r\n", (int16_t)ENC_GetHoldPositionDifferenceValue(ENC1));
        		PRINTF("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(ENC1));
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

void FATFS_DiskInit(void)
{
    //	extern void BOARD_SD_Config(void *card, sd_cd_t cd, uint32_t hostIRQPriority, void *userData);
    BOARD_SD_Config(&g_sd, NULL, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        //	        return kStatus_Fail;
    }

    /* wait card insert */
    if (SD_PollingCardInsert(&g_sd, kSD_Inserted) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power off card */
        SD_SetCardPower(&g_sd, false);
        /* power on the card */
        SD_SetCardPower(&g_sd, true);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        //	        return kStatus_Fail;
    }

    //	f_mount(&FATFS_System_0, (const TCHAR*)"2:", 1)
    if (f_mount(&FATFS_System_0, (const TCHAR *)"2:", 1))
    {
        PRINTF("Mount volume failed.\r\n");
    }
    //	PRINTF("Mount success.\r\n");
    //	    return kStatus_Success;
}
