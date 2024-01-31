#include "sensor_measure.h"
#include "com_delay.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "peripherals.h"

uint8_t iic_Transmit(uint8_t slaveid, uint8_t *data, uint8_t num)
{
    status_t reVal = kStatus_Fail;

    reVal = LPI2C_MasterStart(LPI2C1_PERIPHERAL, slaveid, kLPI2C_Write);
    if (reVal != kStatus_Success)
        return kStatus_Fail;
    while (LPI2C_MasterGetStatusFlags(LPI2C1_PERIPHERAL) & kLPI2C_MasterNackDetectFlag)
        ;

    reVal = LPI2C_MasterSend(LPI2C1_PERIPHERAL, data, num);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterStop(LPI2C1_PERIPHERAL);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    return kStatus_Success;
}
uint8_t iic_Receive(uint8_t slaveid, uint8_t *wdata, uint8_t W_num, uint8_t deay_time, uint8_t *rdata, uint8_t R_num)
{
    status_t reVal = kStatus_Fail;

    reVal = LPI2C_MasterStart(LPI2C1_PERIPHERAL, slaveid, kLPI2C_Write);
    if (reVal != kStatus_Success)
        return kStatus_Fail;
    while (LPI2C_MasterGetStatusFlags(LPI2C1_PERIPHERAL) & kLPI2C_MasterNackDetectFlag)
        ;

    reVal = LPI2C_MasterSend(LPI2C1_PERIPHERAL, wdata, W_num);
    if (reVal != kStatus_Success)
        return kStatus_Fail;
    if (deay_time)
        HAL_Delay(deay_time);
    reVal = LPI2C_MasterRepeatedStart(LPI2C1_PERIPHERAL, slaveid, kLPI2C_Read);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterReceive(LPI2C1_PERIPHERAL, rdata, R_num);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterStop(LPI2C1_PERIPHERAL);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    return kStatus_Success;
}

void BH1730_test()
{
    uint8_t r_data;
    uint8_t w_data = BH1730_REG_PART_ID | BH1730_CMD;
    iic_Receive(BH1730_ADDR, &w_data, 1, 0, &r_data, 1);
    PRINTF("BH1730 r_data=%d\n", r_data);
    PRINTF("BH1730 part num=%d\n", r_data >> 4);
}

float BH1730_readLux()
{
    BH1730_GAIN gain = GAIN_X1;
    // Start one time measurement
    //   write8(BH1730_REG_CONTROL, BH1730_REG_CONTROL_POWER | BH1730_REG_CONTROL_ADC_EN | BH1730_REG_CONTROL_ONE_TIME);
    uint8_t w_data[] = { BH1730_REG_CONTROL | BH1730_CMD,BH1730_REG_CONTROL_POWER | BH1730_REG_CONTROL_ADC_EN | BH1730_REG_CONTROL_ONE_TIME };
	iic_Transmit(BH1730_ADDR, w_data, 2);
    // Wait for ADC data is valid
    uint8_t ret = 0;
    uint8_t read_temp = 0;
    uint8_t write_temp = BH1730_REG_CONTROL | BH1730_CMD;
    while (((read_temp & BH1730_REG_CONTROL_ADC_VALID) == 0) && ++ret < BH1730_RET_TIMEOUT)
    {
    	HAL_Delay(10);
        iic_Receive(BH1730_ADDR, &write_temp, 1, 0, &read_temp, 1);

    }
    if (ret == BH1730_RET_TIMEOUT)
    {
#if BH1730_DEBUG == 1
        Serial.println("Read timed out");
#endif
        return -1;
    }

    uint8_t read_temp2[2];
    // Read real light and IR light from registers

    //   float data0 = (float)read16(BH1730_REG_DATA0_LOW);
    //   float data1 = (float)read16(BH1730_REG_DATA1_LOW);
    write_temp = BH1730_REG_DATA0_LOW | BH1730_CMD;
    iic_Receive(BH1730_ADDR, &write_temp, 1, 0, &read_temp2, 2);
    float data0 = (float)(read_temp2[1] * 256 + read_temp2[0]);
    // PRINTF("0 read_temp2=%d %d,data0=%.2f\n",read_temp2[0],read_temp2[1],data0);
    write_temp = BH1730_REG_DATA1_LOW | BH1730_CMD;
    iic_Receive(BH1730_ADDR, &write_temp, 1, 0, &read_temp2, 2);
    float data1 = (float)(read_temp2[1] * 256 + read_temp2[0]);
    // PRINTF("1 read_temp2=%d %d,data1=%.2f\n",read_temp2[0],read_temp2[1],data1);
    // Calculate lux based on formula in datasheet.
    if (data0 == 0)
        return 0;

    float lx = 0;
    float div = data1 / data0;

    if (div < 0.26)
    {
        lx = ((1.29 * data0) - (2.733 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
    }
    else if (div < 0.55)
    {
        lx = ((0.795 * data0) - (0.859 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
    }
    else if (div < 1.09)
    {
        lx = ((0.51 * data0) - (0.345 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
    }
    else if (div < 2.13)
    {
        lx = ((0.276 * data0) - (0.13 * data1)) / gain * 102.6 / BH1730_ITIME_MS;
    }
    // PRINTF("lx=%.3f\n", lx);
    return lx;
}

void nsht30_set_periodic(uint8_t addr, uint16_t cmd)
{
    uint8_t data[2] = {(cmd & 0xFF00) >> 8, cmd & 0xFF};
    iic_Transmit(addr, data, 2);
}
// periodic mode
void nsht30_read_raw_periodic(uint8_t addr, uint8_t *buff)
{
    uint8_t data[2] = {0XE0, 0X00};
    iic_Receive(addr, data, 2, 10, buff, 6);
}
// Single mode
void nsht30_read_raw_single(uint8_t addr, uint16_t cmd, uint8_t *buff)
{
    uint8_t data[2] = {(cmd & 0xFF00) >> 8, cmd & 0xFF};
    iic_Receive(addr, data, 2, 10, buff, 6);
}

// CRC calculation
int nst3x_crc_check(uint8_t *data, uint8_t len, uint8_t checksum)
{
    uint8_t crc = 0xFF, bit;
    uint8_t byteCtr;
    // calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < len; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x131;
            else
                crc = (crc << 1);
        }
    }
    if (crc == checksum)
        return 1;
    else
        return 0;
}

// Reading and calculating temperature and humidity
uint8_t read_temp_rh_1ch(double *pout)
{
    uint8_t dat[6];
    uint16_t tem, hum;
    nsht30_read_raw_single(0X44, CMD_MEAS_SINGLE_L, dat); // single mode
                                                          //    PRINTF("data:%d %d %d\n  ->%d\n",dat[0],dat[1],dat[2],dat[0]*256+dat[1]);
    tem = ((uint16_t)dat[0] << 8) | dat[1];
    //    PRINTF("tem=%d\n",tem);
    hum = ((uint16_t)dat[3] << 8) | dat[4];
    if ((nst3x_crc_check(dat, 2, dat[2])) && (nst3x_crc_check(dat + 3, 2, dat[5])))
    {
        pout[0] = (175.0 * (double)tem / 65535.0 - 45.0); // T = -45 + 175 * tem / (2^16-1)
        pout[1] = (100.0 * (double)hum / 65535.0);        // RH = hum*100 / (2^16-1)
        return 0;
    }
    else
    {
        return 1;
    }
}
