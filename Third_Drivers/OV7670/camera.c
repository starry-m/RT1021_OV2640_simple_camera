#include "camera.h"
//#include "tim.h"
#include "ov7670.h"
//#include "ov2640.h"
//#include "ov7725.h"
//#include "ov5640.h"

Camera_HandleTypeDef hcamera;
// Resolution table
//----------------------------------------
const uint16_t dvp_cam_resolution[][2] = {
	{0, 0},
	// C/SIF Resolutions
	{88, 72},	/* QQCIF     */
	{176, 144}, /* QCIF      */
	{352, 288}, /* CIF       */
	{88, 60},	/* QQSIF     */
	{176, 120}, /* QSIF      */
	{352, 240}, /* SIF       */
	// VGA Resolutions
	{40, 30},	/* QQQQVGA   */
	{80, 60},	/* QQQVGA    */
	{160, 120}, /* QQVGA     */
	{320, 240}, /* QVGA      */
	{640, 480}, /* VGA       */
	{60, 40},	/* HQQQVGA   */
	{120, 80},	/* HQQVGA    */
	{240, 160}, /* HQVGA     */
	{480, 320}, /* HVGA      */
	// FFT Resolutions
	{64, 32},	/* 64x32     */
	{64, 64},	/* 64x64     */
	{128, 64},	/* 128x64    */
	{128, 128}, /* 128x64    */
	// Other
	{128, 160},	  /* LCD       */
	{128, 160},	  /* QQVGA2    */
	{720, 480},	  /* WVGA      */
	{752, 480},	  /* WVGA2     */
	{800, 600},	  /* SVGA      */
	{1024, 768},  /* XGA       */
	{1280, 1024}, /* SXGA      */
	{1600, 1200}, /* UXGA      */
	{1280, 720},  /* 720P      */
	{1920, 1080}, /* 1080P     */
	{1280, 960},  /* 960P      */
	{2592, 1944}, /* 5MP       */
};

int32_t Camera_WriteReg(Camera_HandleTypeDef *hov, uint8_t regAddr, const uint8_t *pData)
{
	uint8_t tt[2];
	tt[0] = regAddr;
	tt[1] = pData[0];
//	if (HAL_I2C_Master_Transmit(hov->hi2c, hov->addr, tt, 2, hov->timeout) == HAL_OK)
//	{
//		return Camera_OK;
//	}
//	else
//	{
//		return camera_ERROR;
//	}

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
//	HAL_I2C_Master_Transmit(hov->hi2c, hov->addr + 1, &regAddr, 1, hov->timeout);
//	if (HAL_I2C_Master_Receive(hov->hi2c, hov->addr + 1, pData, 1, hov->timeout) == HAL_OK)
//	{
//		return Camera_OK;
//	}
//	else
//	{
//		return camera_ERROR;
//	}

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

int32_t Camera_WriteRegb2(Camera_HandleTypeDef *hov, uint16_t reg_addr, uint8_t reg_data)
{
//	if (HAL_I2C_Mem_Write(hov->hi2c, hov->addr + 1, reg_addr,
//						  I2C_MEMADD_SIZE_16BIT, &reg_data, 1, hov->timeout) == HAL_OK)
//	{
//		return Camera_OK;
//	}
//	else
//	{
//		return camera_ERROR;
//	}
	return Camera_OK;
}

int32_t Camera_ReadRegb2(Camera_HandleTypeDef *hov, uint16_t reg_addr, uint8_t *reg_data)
{
//	if (HAL_I2C_Mem_Read(hov->hi2c, hov->addr + 1, reg_addr,
//						 I2C_MEMADD_SIZE_16BIT, reg_data, 1, hov->timeout) == HAL_OK)
//	{
//		return Camera_OK;
//	}
//	else
//	{
//		return camera_ERROR;
//	}
	 status_t reVal = kStatus_Fail;
	 uint8_t reg_read_addr[2]={reg_addr>>8,reg_addr&0xff};
	reVal = LPI2C_MasterStart(hov->hi2c,hov->addr, kLPI2C_Write);
	if (reVal != kStatus_Success)
		return kStatus_Fail;
	while (LPI2C_MasterGetStatusFlags(hov->hi2c) & kLPI2C_MasterNackDetectFlag)
		;

	reVal = LPI2C_MasterSend(hov->hi2c, reg_read_addr, 2);
	if (reVal != kStatus_Success)
		return kStatus_Fail;

	reVal = LPI2C_MasterRepeatedStart(hov->hi2c, hov->addr, kLPI2C_Read);
	if (reVal != kStatus_Success)
		return kStatus_Fail;

	reVal = LPI2C_MasterReceive(hov->hi2c, reg_data, 1);
	if (reVal != kStatus_Success)
		return kStatus_Fail;

	reVal = LPI2C_MasterStop(hov->hi2c);
	if (reVal != kStatus_Success)
		return kStatus_Fail;
	return Camera_OK;
}

int32_t Camera_WriteRegList(Camera_HandleTypeDef *hov, const struct regval_t *reg_list)
{
	const struct regval_t *pReg = reg_list;
	while (pReg->reg_addr != 0xFF && pReg->value != 0xFF)
	{
		int write_result = Camera_WriteReg(hov, pReg->reg_addr, &(pReg->value));
		if (write_result != Camera_OK)
		{
			return write_result;
		}
		pReg++;
	}
	return Camera_OK;
}

int32_t Camera_read_id(Camera_HandleTypeDef *hov)
{
	uint8_t temp[2];
	temp[0] = 0x01;
	if (hov->addr != OV5640_ADDRESS)
	{
//		Camera_WriteReg(hov, 0xFF, temp);
//		Camera_ReadReg(hov, 0x1C, &temp[0]);
//		Camera_ReadReg(hov, 0x1D, &temp[1]);
//		hov->manuf_id = ((uint16_t)temp[0] << 8) | temp[1];

//		Camera_ReadReg(hov, 0x0A, &temp[0]);
//		Camera_ReadReg(hov, 0x0B, &temp[1]);
//		PRINTF("temp[0]=%x \n",temp[0]);
//		hov->manuf_id = ((uint16_t)temp[0] << 8) | temp[1];
//		Camera_ReadReg(hov, 0x0A, &temp[0]);
//		Camera_ReadReg(hov, 0x0B, &temp[1]);
//		PRINTF("temp[0]=%x \n",temp[0]);

		Camera_WriteReg(hov, 0xFF, temp);
//		Camera_ReadReg(hov, 0x1C, &temp[0]);
//		Camera_ReadReg(hov, 0x1D, &temp[1]);
		Camera_ReadReg(hov, 0x0A, &temp[0]);
		Camera_ReadReg(hov, 0x0B, &temp[1]);
		hov->manuf_id = ((uint16_t)temp[0] << 8) | temp[1];

	}
	else
	{
#define OV5640_CHIP_IDH 0x300A
#define OV5640_CHIP_IDL 0x300B
		Camera_ReadRegb2(&hcamera, OV5640_CHIP_IDH, &temp[0]);
		Camera_ReadRegb2(&hcamera, OV5640_CHIP_IDL, &temp[1]);
		hov->manuf_id = 0;
	}
	hov->device_id = ((uint16_t)temp[0] << 8) | temp[1];
	return 0;
}

void Camera_Reset(Camera_HandleTypeDef *hov)
{
	uint8_t temp;
	temp = 0x01;
	Camera_WriteReg(hov, 0xFF, &temp);
	temp = 0x80;
	Camera_WriteReg(hov, 0x12, &temp);
	HAL_Delay(100);
}

void Camera_XCLK_Set(uint8_t xclktype)
{

}

void Camera_Init_Device(LPI2C_Type *hi2c, framesize_t framesize)
{
	hcamera.hi2c = hi2c;
//	hcamera.addr =0X21 ;//   0X21
	hcamera.timeout = 100;


	{
		hcamera.addr = OV2640_ADDRESS>>1;
		Camera_read_id(&hcamera);
		PRINTF("id=%x \n",hcamera.manuf_id);
		ov2640_init(framesize);
		if (hcamera.manuf_id == 0x7fa2 && ((hcamera.device_id - 0x2641) <= 2))
		{
			// ov2640 当使用高帧率寄存器配置 XCLK时钟采用MCO1输出可能存在异常(花屏)，可以使用TIM1 Channel 1 PWM模式 产生12Mhz方波时钟
			// 但是使用TIM1输出XCLK时钟后与LCD的背光PWM冲突，故该函数自动设置LCD使用软件PWM控制
			// Camera_XCLK_Set(XCLK_TIM);
			PRINTF("OV2640 address get\n");
//			ov2640_init(framesize);
		}
		else
		{

		}
	}
}
