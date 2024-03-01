/*
 * bmp_coder.h
 *
 *  Created on: 2024年2月26日
 *      Author: starry
 */

#ifndef BMP_CODER_H_
#define BMP_CODER_H_

#include "com_delay.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t



uint8_t bmp_pic_write(char *filename,uint8_t *pdata);
uint8_t bmp_pic_display(char *filename);

#endif /* BMP_CODER_H_ */
