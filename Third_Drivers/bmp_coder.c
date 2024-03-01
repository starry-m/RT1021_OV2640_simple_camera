/*
 * bmp_coder.c
 *
 *  Created on: 2024年2月26日
 *      Author: starry
 */

#include "bmp_coder.h"

#include "fsl_debug_console.h"
#include "fsl_sd_disk.h"
#include "sdmmc_config.h"

#include "st7735.h"

// BMP头文件

#if defined(__GNUC__) // LPCXpresso Tools
#define PRE_PACK
#define POST_PACK __attribute__((packed))
#else // Other toolchain
#define PRE_PACK __packed
#define POST_PACK
#endif

#pragma pack(1)

typedef struct
{
	u16 bfType;		 // 文件标志.只对'BM',用来识别BMP位图类型
	u32 bfSize;		 // 文件大小,占四个字节
	u16 bfReserved1; // 保留
	u16 bfReserved2; // 保留
	u32 bfOffBits;	 // 从文件开始到位图数据(bitmap data)开始之间的的偏移量
} BITMAPFILEHEADER;
// BMP信息头
typedef struct
{
	u32 biSize;		// 说明BITMAPINFOHEADER结构所需要的字数。
	long biWidth;	// 说明图象的宽度，以象素为单位
	long biHeight;	// 说明图象的高度，以象素为单位
	u16 biPlanes;	// 为目标设备说明位面数，其值将总是被设为1
	u16 biBitCount; // 说明比特数/象素，其值为1、4、8、16、24、或32
	/*说明图象数据压缩的类型。其值可以是下述值之一：
	BI_RGB：没有压缩；
	BI_RLE8：每个象素8比特的RLE压缩编码，压缩格式由2字节组成(重复象素计数和颜色索引)；
	BI_RLE4：每个象素4比特的RLE压缩编码，压缩格式由2字节组成
	BI_BITFIELDS：每个象素的比特由指定的掩码决定。*/
	u32 biCompression;
	u32 biSizeImage;	  // 说明图象的大小，以字节为单位。当用BI_RGB格式时，可设置为0
	long biXPelsPerMeter; // 说明水平分辨率，用象素/米表示
	long biYPelsPerMeter; // 说明垂直分辨率，用象素/米表示
	u32 biClrUsed;		  // 说明位图实际使用的彩色表中的颜色索引数
	u32 biClrImportant;	  // 说明对图象显示有重要影响的颜色索引的数目，如果是0，表示都重要。
} BITMAPINFOHEADER;
// 彩色表
typedef struct
{
	u8 rgbBlue;		// 指定蓝色强度
	u8 rgbGreen;	// 指定绿色强度
	u8 rgbRed;		// 指定红色强度
	u8 rgbReserved; // 保留，设置为0
} RGBQUAD;
// 整体信息头
typedef struct
{
	BITMAPFILEHEADER bmfHeader;
	BITMAPINFOHEADER bmiHeader;
	RGBQUAD RGB_MASK[3]; // 调色板用于存放RGB掩码.
} BITMAPINFO;

#pragma pack()

// SDK_ALIGN(static BITMAPINFO bmp,8);
static BITMAPINFO bmp;

FRESULT res_sd;	 // 操作结果
static FIL fnew; // 文件对象
UINT fnum;
#define BMP_PICTURE_WIDTH 160
#define BMP_PICTURE_HEIGHT 120

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
		//		u16  bfType= 0x4D42;
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

		// 写文件头进文件
		//		res_sd= f_write(&fnew, &bfType, sizeof(bfType), &fnum);
		res_sd = f_write(&fnew, &bmp, sizeof(bmp), &fnum);

		uint8_t mpdata[320];
		uint8_t *rgb_data = pdata;
		//		  uint32_t i, j;
		if (res_sd == FR_OK)
		{
			//			res_sd= f_write(&fnew, pdata, sizeof(pdata)/2, &fnum);
			//			if(res_sd != FR_OK)
			//			{
			//				PRINTF("pic write error\r\n");
			//
			//				return 1;
			//			}
			for (uint8_t j = 0; j < BMP_PICTURE_HEIGHT; j++)
			{
				for (uint8_t i = 0; i < BMP_PICTURE_WIDTH; i++)
				{
					//					w_color=(uint16_t)(*(rgb_data + i*(0+BMP_PICTURE_HEIGHT)+j));

					mpdata[2U * i] = (uint8_t)(*(rgb_data));
					mpdata[(2U * i) + 1U] = (uint8_t)(*(rgb_data + 1));
					rgb_data += 2;

					//					res_sd= f_write(&fnew, &w_color, sizeof(w_color), &fnum);
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
			//			PRINTF("\r\nClose file ok.\r\n");
			return 0;
		}
	}
	return 1;
}


uint8_t bmp_pic_display(char *filename)
{
	// 打开文件
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
