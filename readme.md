# RT1021_OV2640_simple_camera

[带调试器的i.MX RT1021开发板](https://www.eetree.cn/platform/2581)
[完成电子森林任务：自备摄像头模块采集图像](https://www.eetree.cn/task/420)  


# 使用i.MX-RT1021开发板+OV2640实现的简易照相机

## 一、任务要求
*  将开发板连接摄像头采集图像，并显示在屏幕上。
*  能使用旋钮控制图像亮度等参数
*  可按下旋钮截图
*  将采集后的图片存储并回放
  
## 二、设计思路
*  任务要求连接摄像头采集图片，这一部分可使用rt1021的flexio接口实现、屏幕上显示可以使用LPSPI、旋钮控制可使用ENC、按下按键用GPIO输入、保存图片使用SDIO FatFS保存并读取。
*  因此处理流程为：
![](./pic/流程图.png)

硬件连接为：
![](./pic/硬件连接.png)  


## 三、实现过程

### 1、开发环境的搭建
使用NXP官方提供的MCUXpresso IDE，以及下载RT1021的SDK。先跑通demo。再学会使用IDE中的图形化配置，轻松配置外设驱动~

### 2、屏幕显示
开发板上的屏幕为1.8寸的160*128像素的ST7735。其使用SPI的方式驱动，因此我们只需要替换通用的ST7735屏幕显示的驱动接口即可。

``` c
static void ST7735_WriteCommand(uint8_t cmd)
{
  GPIO_ClearPinsOutput(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN_MASK);
  uint8_t masterTxData = cmd;
  masterXfer.txData = &masterTxData;
  masterXfer.rxData = NULL;
  masterXfer.dataSize = 1;
  masterXfer.configFlags =
      LCD_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
  LPSPI_MasterTransferBlocking(LCD_LPSPI_MASTER_BASEADDR, &masterXfer);
}

static void ST7735_WriteData(uint8_t *buff, size_t buff_size)
{
  GPIO_SetPinsOutput(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN_MASK);
  masterXfer.txData = buff;
  masterXfer.rxData = NULL;
  masterXfer.dataSize = buff_size;
  masterXfer.configFlags =
      LCD_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
  LPSPI_MasterTransferBlocking(LCD_LPSPI_MASTER_BASEADDR, &masterXfer);
}
```
基本使用默认的配置即可，我只是把速度调高了。
![](./pic/SPI配置.png)
然后我们就可以愉快的在屏幕上显示任何东西了。需要注意的是我们使用的是阻塞的方式传输，如果想要更加高效可以使用DMA传输，我这里没有太快的屏幕显示需求，就没有研究了。

### 3、摄像头的选择与驱动
在开发板上的摄像头接口如下：
![](./pic/摄像头接口.png)

淘宝上可以直插的是0V7670（大概10块钱的那款）。我有买过一个来使用，发现我一直驱动不起来，怀疑是XCLK信号问题或者摄像头坏了。所以我干脆换成了自带晶振的OV2640摄像头，不需要提供XCLK信号了，而且是我自己之前用过的（确定是好的）。所以我最终使用的这个。
有关摄像头的参数和原理叙述我跳过了（因为之前做过FPGA的摄像头输入图像处理，所以之前对摄像头的原理和时序有一定的了解，不需要再复习了~~）。
只需要关注使用I2C来配置摄像头的工作模式，FLEX IO来获取图像即可。
RT1021的I2C配置有个需要注意的地方是它的GPIO要配置成下面这样
![](./pic/I2C%20IO.png)

**要打开software Input On,这个是我踩过的坑，当时奇怪了很久，还是后来对比SDK中的I2C使用示例才发现的。**
接下来一样是找到通用的OV2640驱动，然后修改他的I2C驱动接口
``` c
int32_t Camera_WriteReg(Camera_HandleTypeDef *hov, uint8_t regAddr, const uint8_t *pData)
{
	uint8_t tt[2];
	tt[0] = regAddr;
	tt[1] = pData[0];

	status_t reVal = kStatus_Fail;
	reVal = LPI2C_MasterStart(hov->hi2c, hov->addr, kLPI2C_Write);
	if (reVal != kStatus_Success)
		return kStatus_Fail;
	while (LPI2C_MasterGetStatusFlags(hov->hi2c) & kLPI2C_MasterNackDetectFlag)
		;
	reVal = LPI2C_MasterSend(hov->hi2c, tt, 2);
	if (reVal != kStatus_Success)
		return kStatus_Fail;
	reVal = LPI2C_MasterStop(hov->hi2c);
	if (reVal != kStatus_Success)
		return kStatus_Fail;
	return Camera_OK;
}

int32_t Camera_ReadReg(Camera_HandleTypeDef *hov, uint8_t regAddr, uint8_t *pData)
{
	   status_t reVal = kStatus_Fail;

	    reVal = LPI2C_MasterStart(hov->hi2c,hov->addr, kLPI2C_Write);
	    if (reVal != kStatus_Success)
	        return kStatus_Fail;
	    while (LPI2C_MasterGetStatusFlags(hov->hi2c) & kLPI2C_MasterNackDetectFlag)
	        ;
	    reVal = LPI2C_MasterSend(hov->hi2c, &regAddr, 1);
	    if (reVal != kStatus_Success)
	        return kStatus_Fail;
	    reVal = LPI2C_MasterRepeatedStart(hov->hi2c, hov->addr, kLPI2C_Read);
	    if (reVal != kStatus_Success)
	        return kStatus_Fail;
	    reVal = LPI2C_MasterReceive(hov->hi2c, pData, 1);
	    if (reVal != kStatus_Success)
	        return kStatus_Fail;
	    reVal = LPI2C_MasterStop(hov->hi2c);
	    if (reVal != kStatus_Success)
	        return kStatus_Fail;
	    return Camera_OK;
}
```
在rt1021的sdk中并没有封装好的I2C读写函数，只能自己来组合每一步操作，这个是用的示例中的I2C阻塞传输方式。然后需要注意的是这里需要传输的是I2C从机设备7位地址，别自作聪明的给改成读写的8位。
![](./pic/摄像头初始化.png)
这里能读到正确的摄像头ID，说明i2c配置基本就没问题了。
最后是flex io的配置
![](./pic/flexio配置.png)
针对接入的引脚，进行相应设置。流程是，摄像头输入的pclk会驱动这个外设中的一个定时器，然后每得到8位数据就进行移位，等待下一8位，组合成16位rgb565数组，通过DMA搬运到定义好的缓存数组。当一幅图像接收完后，会触发DMA完成中断，可在这个中断函数中来进行一些标志位置位的操作。但我实际用的是VS信号触发GPIO中断，在这里面进行的其他操作。
``` c
/* BOARD_CAM_VS_handle callback function */
void BOARD_CAM_VS_callback(void *param)
{
    /* clear the interrupt status */
    GPIO_PortClearInterruptFlags(BOARD_CAM_VS_GPIO, BOARD_CAM_VS_GPIO_PIN_MASK);

    ov2640_finish_flag = 1;

    FLEXIO_CAMERA_ClearStatusFlags(&FLEXIO1_peripheralConfig,
                                   kFLEXIO_CAMERA_RxDataRegFullFlag | kFLEXIO_CAMERA_RxErrorFlag);
    if (0 == work_mode)
        FLEXIO_CAMERA_TransferReceiveEDMA(&FLEXIO1_peripheralConfig, &FLEXIO1_Camera_eDMA_Handle, &cam_xfer);

    __DSB();
}
```
在rt1021的图形化DMA配置中我没看到有DMA自动循环搬运的配置，得要每次手动开启。
最后就是整个的初始化过程：
``` c
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
```
**图像参数的改变**
这个通过向摄像头写入对应的参数实现
``` c
typedef enum  {
    im_quality,
    im_brightness,
    im_contrast
}im_params;

uint8_t OV2640_image_param_set(uint8_t par1,int8_t value)
{
    if(im_quality==par1)
    {
    return     set_quality(value);
    }
    else     if(im_brightness==par1)
    {
    return     set_brightness(value);
    }
    else     if(im_contrast==par1)
    {
    return     set_contrast(value);
    }
}
void im_par_change_handler(struct _image_parameters *param)
{
    uint8_t menc_r = enc_rotate();
    static struct _image_parameters temp_param;

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
    if (param->quality != temp_param.quality)
        OV2640_image_param_set(0, param->quality);
    if (param->brightness != temp_param.brightness)
        OV2640_image_param_set(1, param->brightness);
    if (param->contrast != temp_param.contrast)
        OV2640_image_param_set(2, param->contrast);
    temp_param.brightness = param->brightness;
    temp_param.contrast = param->contrast;
    temp_param.quality = param->quality;
}
```


### 4、文件系统的配置和图片存储读取

rt1021的sdk自带很多丰富的组件，只需要在图形界面中选择，就可以将他们添加进去。
直接在组件中搜索FATFS，导入，然后把示例的一些配置移过来。下面是文件系统添加后，需要往FATFS_DiskInit这个物理驱动函数增加设置。然后对照着示例，改sdmmc_config.c文件，基本就可以了。
``` c
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

```
现在我们的文件系统已经有了，我们可以使用fatfs的通用文件操作了。
接下来需要考虑我们摄像头的图像用什么格式存储了。常用的有jpg、png、bmp等。虽然我们也可以直接把图像数据用二进制写到文件中，但为了方便我们用其他设备，如电脑查看拍下的图片。还是选择使用通用的图片格式存储。
JPG方式需要用到LibJpg库来对图片进行编码，这种方式虽然得到的图片占用空间比较小，但是在图片编解码时需要大量的堆空间，还可能会有其他问题。所以我选了了简单的bmp格式存储，bmp格式只有前面的66个字节用来保存格式信息，后面的空间全是图片原始像素数据，不管是存入还是读取都比较方便。
有关bmp的具体信息可以看这个[BMP file format](https://en.wikipedia.org/wiki/BMP_file_format#)。
bmp文件保存实现如下：
``` c
uint8_t bmp_pic_write(char *filename, uint8_t *pdata)
{
	PRINTF("start screenshot\r\n");
	uint16_t w_color;
	uint8_t *rgb_data = pdata;
	// 打开文件，若不存在就创建
	res_sd = f_open(&fnew, filename, FA_CREATE_ALWAYS | FA_WRITE);

	// 文件打开成功
	if (res_sd == FR_OK)
	{
		// 填写文件信息头信息
		bmp.bmfHeader.bfType = 0x4D42;											// bmp类型  "BM"
		bmp.bmfHeader.bfSize = 54 + BMP_PICTURE_WIDTH * BMP_PICTURE_HEIGHT * 2; // 文件大小（信息结构体+像素数据）
		bmp.bmfHeader.bfReserved1 = 0x0000;										// 保留，必须为0
		bmp.bmfHeader.bfReserved2 = 0x0000;
		bmp.bmfHeader.bfOffBits = 54; // 位图信息结构体所占的字节数

		// 填写位图信息头信息
		bmp.bmiHeader.biSize = 40;												// 位图信息头的大小
		bmp.bmiHeader.biWidth = BMP_PICTURE_WIDTH;								// 位图的宽度
		bmp.bmiHeader.biHeight = BMP_PICTURE_HEIGHT;							// 图像的高度
		bmp.bmiHeader.biPlanes = 1;												// 目标设别的级别，必须是1
		bmp.bmiHeader.biBitCount = 16;											// 每像素位数
		bmp.bmiHeader.biCompression = 3;										// RGB555格式
		bmp.bmiHeader.biSizeImage = BMP_PICTURE_WIDTH * BMP_PICTURE_HEIGHT * 2; // 实际位图所占用的字节数（仅考虑位图像素数据）
		bmp.bmiHeader.biXPelsPerMeter = 0;										// 水平分辨率
		bmp.bmiHeader.biYPelsPerMeter = 0;										// 垂直分辨率
		bmp.bmiHeader.biClrImportant = 0;										// 说明图像显示有重要影响的颜色索引数目，0代表所有的颜色一样重要
		bmp.bmiHeader.biClrUsed = 0;											// 位图实际使用的彩色表中的颜色索引数，0表示使用所有的调色板项

		// RGB565格式掩码
		bmp.RGB_MASK[0].rgbBlue = 0;
		bmp.RGB_MASK[0].rgbGreen = 0xF8;
		bmp.RGB_MASK[0].rgbRed = 0;
		bmp.RGB_MASK[0].rgbReserved = 0;

		bmp.RGB_MASK[1].rgbBlue = 0xE0;
		bmp.RGB_MASK[1].rgbGreen = 0x07;
		bmp.RGB_MASK[1].rgbRed = 0;
		bmp.RGB_MASK[1].rgbReserved = 0;

		bmp.RGB_MASK[2].rgbBlue = 0x1F;
		bmp.RGB_MASK[2].rgbGreen = 0;
		bmp.RGB_MASK[2].rgbRed = 0;
		bmp.RGB_MASK[2].rgbReserved = 0;

		res_sd = f_write(&fnew, &bmp, sizeof(bmp), &fnum);

		uint8_t mpdata[320];
		uint8_t *rgb_data = pdata;
		if (res_sd == FR_OK)
		{

			for (uint8_t j = 0; j < BMP_PICTURE_HEIGHT; j++)
			{
				for (uint8_t i = 0; i < BMP_PICTURE_WIDTH; i++)
				{
					mpdata[2U * i] = (uint8_t)(*(rgb_data));
					mpdata[(2U * i) + 1U] = (uint8_t)(*(rgb_data + 1));
					rgb_data += 2;
				}
				res_sd = f_write(&fnew, mpdata, sizeof(mpdata), &fnum);
				if (res_sd != FR_OK)
				{
					PRINTF("pic write error\r\n");

					return 1;
				}
			}
			PRINTF("pic write ok!\r\n");
			if (f_close(&fnew))
			{
				PRINTF("\r\nClose file failed.\r\n");
				return 1;
			}
			return 0;
		}
	}
	return 1;
}
```
我在完成上面的处理后，碰到了电脑无法正确读取bmp图片的情况。经群里老哥提醒，使用HxD软件查看bmp文件数据。发现前面写入的数据多了两个0x00，我手动给他删掉，图片就能正常读取了。经研究搜索发现是数据对齐的问题，编译器默认结构体是32位对齐，一个结构体的大小是32位的倍数。因此我们需要告诉编译器我们的结构体是8位对齐的，加入如下操作即可。
``` c
#pragma pack(1)
\*结构体定义*\
#pragma pack()
```
然后是bmp文件读取到屏幕显示。
``` c
uint8_t bmp_pic_display(char *filename)
{
	res_sd = f_open(&fnew, filename, FA_OPEN_EXISTING | FA_READ);
	if (res_sd != FR_OK)
	{
		PRINTF("OPEN error\r\n");
		return 1;
	}
	res_sd= f_read(&fnew, &bmp, sizeof(bmp), &fnum);
	if (res_sd != FR_OK)
	{
		PRINTF("READ header error\r\n");
		return 1;
	}
	if(0x4D42 !=bmp.bmfHeader.bfType)
	{
		f_close(&fnew);
		return 1;
	}
	if (f_lseek(&fnew, 66U))
	{
		PRINTF("Set file pointer position failed. \r\n");
		f_close(&fnew);
		return 1;
	}
	uint8_t mpdata[320];
	uint8_t TTmpdata[320];
	uint8_t col=0;
	uint16_t j=159;
	/*一次显示一行的图像*/
	while(col<120)
	{
		res_sd= f_read(&fnew, &mpdata, sizeof(mpdata), &fnum);
		for(uint16_t i=0;i<160;i++)
		{
			TTmpdata[i*2]=mpdata[i*2+1];
			TTmpdata[i*2+1]=mpdata[i*2];
		}
		j=159;
		ST7735_DrawImage(0,119-col,160,1,(uint16_t*)TTmpdata);
		col++;
	}
	f_close(&fnew);
	return 0;
}

```

### 5、按键与旋转编码器
按键我使用了一个开源的驱动[multibutton](https://github.com/0x1abin/MultiButton)来进行操作。只需要提供一个5ms的定时就可以，这里使用PIT定时器来是实现
![](./pic/PIT.png)
按键只需要注册对应事件的回调函数就可以。
``` c
struct Button encoder_KEY;
uint8_t btn1_id = 0;
button_init(&encoder_KEY, read_button_GPIO, 0, btn1_id);
button_start(&encoder_KEY);
button_attach(&encoder_KEY, PRESS_UP, BTN1_PRESS_UP_Handler);

button_attach(&encoder_KEY, DOUBLE_CLICK, BTN1_DOUBLE_CLICK_Handler);
button_attach(&encoder_KEY, LONG_PRESS_START, BTN1_LONG_PRESS_START_Handler);

```
编码器使用enc外设，我用如下处理来判断左右旋转，
``` c
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
```


### 6、主逻辑处理
这里只需要拍照和读取照片两种模式，使用按键双击来切换。单击配合旋转编码器来实现参数调节。长按实现模式切换。

``` c
void BTN1_PRESS_UP_Handler(void *btn)
{
    PRINTF("BTN1_PRESS_UP\n");
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

while (1)
    {
        if (!work_mode)
        {
            im_par_change_handler(&m_image_parameters);
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
```



## 四、效果展示与遇到的问题

### 效果展示

摄像头获取图像显示
![](./pic/2.jpg)
改变图像参数
![](./pic/3.jpg)
![](./pic/4.jpg)
![](./pic/5.jpg)
![](./pic/6.jpg)

查看保存的截图
![](./pic/8.jpg)

![](./pic/9.jpg)

![](./pic/11.jpg)

### 遇到的问题
* 一开始调摄像头显示时一直只显示一半，然后发现flex io DMA传送的单位是字节，最后乘以2就可以了。
* rgb565的高低2个字节好像是反的，我得在显示时交换下才显示正常。
* SPI读写同步，在做摄像头屏幕前我把板上的那些传感器都读了一遍，发现SPI读的时候，会有一个空字节，需要丢掉，才是正确的数据。
* 加入文件系统后，启动很慢，应该是文件系统物理层那里的配置没做好。
* 前面提过的bmp写入字节对齐。
* 旋转编码器读到的很不准，待解决。导致我图像参数改变和选取图片有问题。

## 五、未来的计划与感想
* 这次体验了下NXP的高性能MCU和IDE，图形化真的好评。但是这个基于eclipse的IDE有时候是真的卡。
* 之前一直有个想法，基于这个MCU的480M bps USB，做一个电脑的扩展屏幕。有时间一定要实现下。
* 这次收获满满，第一次用NXP的单片机，了解了新的一套软件框架。
* 感觉这个花的时间还挺多的，看我的git提交记录，哈哈。
![](./pic/git提交记录.png)
* 以上！
