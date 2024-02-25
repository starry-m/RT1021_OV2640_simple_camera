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

uint16_t   ov7670_finish_flag = 0,ov7670_frame_conter=0;    //一场图像采集完成标志位
edma_transfer_config_t transferConfig;
flexio_camera_transfer_t cam_xfer;
//uint16_t CAM_BUFFER[160*128];
/* BOARD_CAM_VS_handle callback function */
void BOARD_CAM_VS_callback(void *param) {
  /* Place your code here */
	/* clear the interrupt status */
	GPIO_PortClearInterruptFlags(BOARD_CAM_VS_GPIO, BOARD_CAM_VS_GPIO_PIN_MASK);


//	ov7670_finish_flag++;
    ov7670_frame_conter++;

//	DMA0->TCD[0].DADDR = (uint32_t)FLEXIO1_Camera_Buffer[0];

	 FLEXIO_CAMERA_ClearStatusFlags(&FLEXIO1_peripheralConfig,
	                                   kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
//	    ov7670_finish_flag=1;
    /* Enable DMA channel request. */
//    DMA0->SERQ = DMA_SERQ_SERQ(0);
	 FLEXIO_CAMERA_TransferReceiveEDMA(&FLEXIO1_peripheralConfig,&FLEXIO1_Camera_eDMA_Handle,&cam_xfer);

    __DSB();
}

void CAM_DMA_COMPLETE(FLEXIO_CAMERA_Type *base, flexio_camera_edma_handle_t *handle, status_t status, void *userData)
{

	ov7670_finish_flag=1;
//	ov7670_finish_flag++;
    /* Enable DMA channel request. */
//    DMA0->SERQ = DMA_SERQ_SERQ(FLEXIO_CAMERA_DMA_CHN);
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



/* DMA channel assigements. */
#define FLEXIO_CAMERA_DMA_CHN           0u
#define FLEXIO_CAMERA_DMA_MUX_SRC       (kDmaRequestMuxFlexIO1Request0Request1 & 0xFF)

#define OV7670_FRAME_BYTES FLEXIO1_FRAME_WIDTH *FLEXIO1_FRAME_HEIGHT
#define FXIO_SHFT_COUNT         4u          /* 4 shifters */
#define DMA_TRSF_SIZE           8u          /* 8 bytes */
#define DMA_MINOR_LOOP_SIZE     16u         /* 16 bytes */
#define DMA_MAJOR_LOOP_SIZE     (OV7670_FRAME_BYTES / DMA_MINOR_LOOP_SIZE)


void CAM_OV7670_DEMO()
{
//	for(uint8_t i=0;i<160;i++)
//	{
//		for(uint8_t j=0;j<128;j++)
//		{
//			FLEXIO1_Camera_Buffer[0][i][j]=ST7735_RED;
//			ST7735_DrawPixel(i,j,FLEXIO1_Camera_Buffer[0][i][j]);
//		}
//	}
////	ST7735_FillRGBRect(0,0,(uint8_t *)FLEXIO1_Camera_Buffer[0][0][0],128,160);
//	PRINTF("FLEXIO1_Camera_Buffer[0][1][119]=%x\n",FLEXIO1_Camera_Buffer[0][1][119]);
//	ST7735_WriteString(10,10,"hello",Font_11x18,0xff00,0x0000);
//	PRINTF("temp OV7670 START\n");
//    while(1)
//    {
//    	HAL_Delay(500);
//    	PRINTF("dididid\n");
//
//    }
	/* lcd init  */

	/*camera ov7670 init ,12M XCLK,PWDN LOW,RST HIGH*/
	GPIO_PinWrite(BOARD_CAM_PWDN_GPIO, BOARD_CAM_PWDN_GPIO_PIN, 0U);
//	GPIO_PinWrite(BOARD_CAM_RES_GPIO, BOARD_CAM_RES_GPIO_PIN, 0U);
//	HAL_Delay(20);
	GPIO_PinWrite(BOARD_CAM_RES_GPIO, BOARD_CAM_RES_GPIO_PIN, 1U);

    /* Clear all the flag. */
    FLEXIO_CAMERA_ClearStatusFlags(&FLEXIO1_peripheralConfig, kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
    /* Enable FlexIO. */
    FLEXIO_CAMERA_Enable(&FLEXIO1_peripheralConfig, true);

	/*cam dma transfer start */

    cam_xfer.dataAddress=(uint32_t)FLEXIO1_Camera_Buffer[0];
    cam_xfer.dataNum=2*FLEXIO1_FRAME_WIDTH *FLEXIO1_FRAME_HEIGHT;

    FLEXIO_CAMERA_TransferReceiveEDMA(&FLEXIO1_peripheralConfig,&FLEXIO1_Camera_eDMA_Handle,&cam_xfer);
    //// 160x120 x
	Camera_Init_Device(LPI2C3_PERIPHERAL, FRAMESIZE_QQVGA);
	PRINTF("OV camera init ok\n");

    /* Set the load okay bit for all submodules to load registers from their buffer */
//    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_0, true);
//    /* Start the PWM generation from Submodules 0, 1 and 2 */
//    PWM_StartTimer(PWM1, kPWM_Control_Module_0);



    PRINTF("OV7670 START\n");
//    PRINTF("FLEXIO1_Camera_Buffer[0][1][119]=%x\n",FLEXIO1_Camera_Buffer[0][20][60]);
//    PRINTF("FLEXIO1_Camera_Buffer[0][100][60]=%x\n",FLEXIO1_Camera_Buffer[0][100][60]);
	/*loop frame show*/
    while(1)
    {
//    	PRINTF("ov7670_finish_flag=%d,ov7670_frame_conter=%d\n",ov7670_finish_flag,ov7670_frame_conter);
//    	PRINTF("FLEXIO1_Camera_Buffer=%d\n",FLEXIO1_Camera_Buffer[0][10][10]);
//    	HAL_Delay(100);
    	if(ov7670_finish_flag)
    	{
    		ov7670_finish_flag=0;
//    		 PRINTF("OV7670 frame get\n");
    		ST7735_FillRGBRect(0,0,(uint8_t *)&FLEXIO1_Camera_Buffer[0][0][0],160,120);
//    			for(uint8_t i=0;i<160;i++)
//    			{
//    				for(uint8_t j=0;j<128;j++)
//    				{
////    					FLEXIO1_Camera_Buffer[0][i][j]=ST7735_RED;
//    					ST7735_DrawPixel(j,i,FLEXIO1_Camera_Buffer[0][i][j]);
//    				}
//    			}
    	}

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
	if (f_mount(&FATFS_System_0, (const TCHAR*)"2:", 1))
	{
		PRINTF("Mount volume failed.\r\n");

	}
//	PRINTF("Mount success.\r\n");
//	    return kStatus_Success;
}
static FIL g_fileObject;
#define BUFFER_SIZE (513U)
/*! @brief Data written to the card */
SDK_ALIGN(uint8_t g_bufferWrite[BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
/*! @brief Data read from the card */
SDK_ALIGN(uint8_t g_bufferRead[BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
int SD_operate_demo()
{
	FRESULT error;
	DIR directory; /* Directory object */
	FILINFO fileInformation;
    UINT bytesWritten;
    UINT bytesRead;
    volatile bool failedFlag           = false;
    char ch                            = '0';
    BYTE work[FF_MAX_SS];
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};

#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return -1;
    }
#endif

//#if FF_USE_MKFS
//    PRINTF("\r\nMake file system......The time may be long if the card capacity is big.\r\n");
//    if (f_mkfs(driverNumberBuffer, 0, work, sizeof work))
//    {
//        PRINTF("Make file system failed.\r\n");
//        return -1;
//    }
//#endif /* FF_USE_MKFS */

	PRINTF("\r\nCreate directory......\r\n");
	error = f_mkdir(_T("/m_dir"));
	if (error)
	{
		if (error == FR_EXIST)
		{
			PRINTF("Directory exists.\r\n");
		}
		else
		{
			PRINTF("Make directory failed.\r\n");
			return -1;
		}
	}

    PRINTF("\r\nCreate a file in that directory......\r\n");
    error = f_open(&g_fileObject, _T("/m_dir/f_1.dat"), (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
    if (error)
    {
        if (error == FR_EXIST)
        {
            PRINTF("File exists.\r\n");
        }
        else
        {
            PRINTF("Open file failed.\r\n");
            return -1;
        }
    }

    PRINTF("\r\nCreate a directory in that directory......\r\n");
        error = f_mkdir(_T("/m_dir/dir_2"));
        if (error)
        {
            if (error == FR_EXIST)
            {
                PRINTF("Directory exists.\r\n");
            }
            else
            {
                PRINTF("Directory creation failed.\r\n");
                return -1;
            }
        }

        PRINTF("\r\nList the file in that directory......\r\n");
        if (f_opendir(&directory, "/m_dir"))
        {
            PRINTF("Open directory failed.\r\n");
            return -1;
        }

        for (;;)
        {
            error = f_readdir(&directory, &fileInformation);

            /* To the end. */
            if ((error != FR_OK) || (fileInformation.fname[0U] == 0U))
            {
                break;
            }
            if (fileInformation.fname[0] == '.')
            {
                continue;
            }
            if (fileInformation.fattrib & AM_DIR)
            {
                PRINTF("Directory file : %s.\r\n", fileInformation.fname);
            }
            else
            {
                PRINTF("General file : %s.\r\n", fileInformation.fname);
            }
        }
        memset(g_bufferWrite, 'a', sizeof(g_bufferWrite));
        g_bufferWrite[BUFFER_SIZE - 2U] = '\r';
        g_bufferWrite[BUFFER_SIZE - 1U] = '\n';

        PRINTF("\r\nWrite/read file until encounters error......\r\n");
        while (true)
        {
            if (failedFlag || (ch == 'q'))
            {
                break;
            }

            PRINTF("\r\nWrite to above created file.\r\n");
            error = f_write(&g_fileObject, g_bufferWrite, sizeof(g_bufferWrite), &bytesWritten);
            if ((error) || (bytesWritten != sizeof(g_bufferWrite)))
            {
                PRINTF("Write file failed. \r\n");
                failedFlag = true;
                continue;
            }

            /* Move the file pointer */
            if (f_lseek(&g_fileObject, 0U))
            {
                PRINTF("Set file pointer position failed. \r\n");
                failedFlag = true;
                continue;
            }

            PRINTF("Read from above created file.\r\n");
            memset(g_bufferRead, 0U, sizeof(g_bufferRead));
            error = f_read(&g_fileObject, g_bufferRead, sizeof(g_bufferRead), &bytesRead);
            if ((error) || (bytesRead != sizeof(g_bufferRead)))
            {
                PRINTF("Read file failed. \r\n");
                failedFlag = true;
                continue;
            }

            PRINTF("Compare the read/write content......\r\n");
            if (memcmp(g_bufferWrite, g_bufferRead, sizeof(g_bufferWrite)))
            {
                PRINTF("Compare read/write content isn't consistent.\r\n");
                failedFlag = true;
                continue;
            }
            PRINTF("The read/write content is consistent.\r\n");

            PRINTF("\r\nInput 'q' to quit read/write.\r\nInput other char to read/write file again.\r\n");
            ch = GETCHAR();
            PUTCHAR(ch);
        }
        PRINTF("\r\nThe example will not read/write file again.\r\n");

        if (f_close(&g_fileObject))
        {
            PRINTF("\r\nClose file failed.\r\n");
            return -1;
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
       ST7735_FillScreen(ST7735_BLUE);
//    	ST7735_Init();
//    	ST7735_FillScreen(ST7735_BLUE);
//    	ST7735_FillScreen(ST7735_RED);
//    	ST7735_FillScreen(ST7735_BLACK);
//    	ST7735_FillScreen(ST7735_WHITE);
//    	ST7735_FillScreen(ST7735_GREEN);
    //    testSPI();
    //    SysTick_DelayTicks(2U);

//    CAM_OV7670_DEMO();

       SD_operate_demo();
       while(1);
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
static void configDMA(void)
{
    uint32_t soff, smod = 0u, size=0u;

    while(1u << size < DMA_TRSF_SIZE) /* size = log2(DMA_TRSF_SIZE) */
    {
        size++;
    }

    if(DMA_TRSF_SIZE == DMA_MINOR_LOOP_SIZE)
    {
        soff = 0u;
    }
    else
    {
        soff = DMA_TRSF_SIZE;
        while(1u << smod < DMA_MINOR_LOOP_SIZE) /* smod = log2(DMA_MINOR_LOOP_SIZE) */
        {
            smod++;
        }
    }

    /* Configure DMA TCD */
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].SADDR = FLEXIO_CAMERA_GetRxBufferAddress(&FLEXIO1_peripheralConfig);
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].SOFF = soff;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].ATTR = DMA_ATTR_SMOD(smod) |
                                            DMA_ATTR_SSIZE(size) |
                                            DMA_ATTR_DMOD(0u) |
                                            DMA_ATTR_DSIZE(size);
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].NBYTES_MLNO = DMA_MINOR_LOOP_SIZE;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].SLAST = 0u;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].DADDR = (uint32_t)(*FLEXIO1_Camera_Buffer[0]);
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].DOFF = DMA_TRSF_SIZE;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].CITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].DLAST_SGA = -OV7670_FRAME_BYTES;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].CSR = 0u;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].CSR |= DMA_CSR_DREQ_MASK;
    DMA0->TCD[FLEXIO_CAMERA_DMA_CHN].BITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;

    /* Configure DMA MUX Source */
    DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] = DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] &
                                            (~DMAMUX_CHCFG_SOURCE_MASK) |
                                            DMAMUX_CHCFG_SOURCE(FLEXIO_CAMERA_DMA_MUX_SRC);
    /* Enable DMA channel. */
    DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] |= DMAMUX_CHCFG_ENBL_MASK;
}

/*    EDMA_PrepareTransfer(&transferConfig,
        (void *)FLEXIO_CAMERA_GetRxBufferAddress(&FLEXIO1_peripheralConfig),
        8,
        (void *)(FLEXIO1_Camera_Buffer[0]),
        8,
        8*8,
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


        EDMA_SetModulo(DMA0,FLEXIO1_FLEXIO_0_DMA_CHANNEL,kEDMA_Modulo64bytes,kEDMA_ModuloDisable);
        EDMA_StartTransfer(&FLEXIO1_FLEXIO_0_Handle);
//        configDMA();
         Enable FlexIO DMA request.
        FLEXIO_CAMERA_EnableRxDMA(&FLEXIO1_peripheralConfig, true);*/
