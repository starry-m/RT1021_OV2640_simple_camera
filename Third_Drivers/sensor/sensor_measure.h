#ifndef __SENSOR_MEASURE_H__
#define __SENSOR_MEASURE_H__


#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define CMD_MEAS_SINGLE_H 0x2400 // measurement: SINGLE Mode high repeatability
#define CMD_MEAS_SINGLE_M 0x240B // measurement: SINGLE Mode medium repeatability
#define CMD_MEAS_SINGLE_L 0x2416 // measurement: SINGLE Mode low repeatability
#define CMD_MEAS_PERI_05_H 0x2032 // measurement: periodic Mode 0.5 mps high repeatability
#define CMD_MEAS_PERI_05_M 0x2024 // measurement: periodic Mode 0.5 mps medium repeatability
#define CMD_MEAS_PERI_05_L 0x202F // measurement: periodic Mode 0.5 mps low repeatability
#define CMD_MEAS_PERI_1_H 0x2130 // measurement: periodic Mode 1 mps high repeatability
#define CMD_MEAS_PERI_1_M 0x2126 // measurement: periodic Mode 1 mps medium repeatability
#define CMD_MEAS_PERI_1_L 0x212D // measurement: periodic Mode 1 mps low repeatability
#define CMD_MEAS_PERI_2_H 0x2236 // measurement: periodic Mode 2 mps high repeatability
#define CMD_MEAS_PERI_2_M 0x2220 // measurement: periodic Mode 2 mps medium repeatability
#define CMD_MEAS_PERI_2_L 0x222B // measurement: periodic Mode 2 mps low repeatability
#define CMD_MEAS_PERI_4_H 0x2334 // measurement: periodic Mode 4 mps high repeatability
#define CMD_MEAS_PERI_4_M 0x2322 // measurement: periodic Mode 4 mps medium repeatability
#define CMD_MEAS_PERI_4_L 0x2329 // measurement: periodic Mode 4 mps low repeatability
#define CMD_MEAS_PERI_10_H 0x2737 // measurement: periodic Mode 10 mps high repeatability
#define CMD_MEAS_PERI_10_M 0x2721 // measurement: periodic Mode 10 mps medium repeatability
#define CMD_MEAS_PERI_10_L 0x272A // measurement: periodic Mode 10 mps low repeatability


uint8_t read_temp_rh_1ch(double *pout);



#define BH1730_ADDR 0x29
#define BH1730_PART_NUMBER 0x7

#define BH1730_CMD 0x80
#define BH1730_CMD_SPECIAL 0x60
#define BH1730_CMD_SPECIAL_SOFT_RESET 0x4

#define BH1730_REG_CONTROL 0x00
#define BH1730_REG_GAIN 0x7
#define BH1730_REG_TIMING 0x01
#define BH1730_REG_PART_ID 0x12
#define BH1730_REG_DATA0_LOW 0x14
#define BH1730_REG_DATA0_HIGH 0x15
#define BH1730_REG_DATA1_LOW 0x16
#define BH1730_REG_DATA1_HIGH 0x17

#define BH1730_REG_CONTROL_POWER 0x1
#define BH1730_REG_CONTROL_ADC_EN 0x2
#define BH1730_REG_CONTROL_ONE_TIME 0x8
#define BH1730_REG_CONTROL_ADC_VALID 0x10

#define BH1730_GAIN_X1_MODE 0x00
#define BH1730_GAIN_X2_MODE 0x01
#define BH1730_GAIN_X64_MODE 0x02
#define BH1730_GAIN_X128_MODE 0x03

#define BH1730_RET_TIMEOUT 50

#define BH1730_ITIME 218
#define BH1730_T_INT 2.8
#define BH1730_ITIME_MS ((BH1730_T_INT/1000.0) * 964.0 * (256.0 - BH1730_ITIME))

typedef enum{
  GAIN_X1 = 1,
  GAIN_X2 = 2,
  GAIN_X64 = 64,
  GAIN_X128 = 128
} BH1730_GAIN;

void BH1730_test();
float BH1730_readLux();

#endif
