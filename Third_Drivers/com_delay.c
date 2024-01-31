#include "com_delay.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "peripherals.h"

extern void SysTick_DelayTicks(uint32_t n);

void HAL_Delay(uint32_t mdelay)
{
    SysTick_DelayTicks(mdelay);
}

void HAL_DelayUs(uint32_t udelay)
{
    SDK_DelayAtLeastUs(udelay, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
}

void serial_tx(uint8_t *tx_buffer, uint16_t len)
{
    DbgConsole_SendDataReliable(tx_buffer, len);
}
/**
 * @brief
 *
 *
 * @param mled
 * @param state 0:off、1:on、2:toggle
 * @return ** void
 */
void led_state_set(uint8_t mled, uint8_t state)
{
    if (0 == mled)
    {
        switch (state)
        {
        case 0:
            GPIO_PinWrite(BOARD_U_LED_GPIO, BOARD_U_LED_GPIO_PIN, 0U);
            break;
        case 1:
            GPIO_PinWrite(BOARD_U_LED_GPIO, BOARD_U_LED_GPIO_PIN, 1U);
            break;
        case 2:
            GPIO_PortToggle(BOARD_U_LED_GPIO, BOARD_U_LED_GPIO_PIN_MASK);
            break;
        default:
            break;
        }
    }
}

static LPI2C_Type *base[] = {LPI2C1_PERIPHERAL};
//-------------------------------------------------------------------------------------------------------------------
//  @brief      写入一个字节数据到I2C设备指定寄存器地址
//  @param      iic_n       IIC模块(IIC_1,IIC_2,IIC_3,IIC_4)
//  @param      slaveid     从机地址(7位地址)
//  @param      reg         从机寄存器地址
//  @param      data        需要发送的数据
//  @return                 返回的状态值 0：成功  1：失败
//  @since      v2.0
//  Sample usage:       	iic_write_reg(IIC_2, 0x2D, 0x50,2);     //写入数据2到0x50地址，从机地址为0x2D
//-------------------------------------------------------------------------------------------------------------------
uint8_t iic_write_reg(uint8_t bus, uint8_t slaveid, uint8_t reg, uint8_t data)
{
    status_t reVal = kStatus_Fail;

    reVal = LPI2C_MasterStart(base[bus], slaveid, kLPI2C_Write);
    if (reVal != kStatus_Success)
        return kStatus_Fail;
    while (LPI2C_MasterGetStatusFlags(base[bus]) & kLPI2C_MasterNackDetectFlag)
        ;

    reVal = LPI2C_MasterSend(base[bus], &reg, 1);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterSend(base[bus], &data, 1);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterStop(base[bus]);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    return kStatus_Success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取I2C设备指定地址寄存器的数据
//  @param      iic_n       IIC模块(IIC_1,IIC_2,IIC_3,IIC_4)
//  @param      slaveid     从机地址(7位地址)
//  @param      reg         从机寄存器地址
//  @param      *data       读取回来的数据
//  @return                 返回的状态值 0：成功  1：失败
//  @since      v2.0
//  Sample usage:       	uint8_t value = iic_read_reg(IIC_2, 0x2D, 0x50);//读取0x50地址的数据，从机地址为0x2D
//-------------------------------------------------------------------------------------------------------------------
uint8_t iic_read_reg(uint8_t bus, uint8_t slaveid, uint8_t reg, uint8_t *data)
{
    status_t reVal = kStatus_Fail;

    reVal = LPI2C_MasterStart(base[bus], slaveid, kLPI2C_Write);
    if (reVal != kStatus_Success)
        return kStatus_Fail;
    while (LPI2C_MasterGetStatusFlags(base[bus]) & kLPI2C_MasterNackDetectFlag)
        ;

    reVal = LPI2C_MasterSend(base[bus], &reg, 1);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterRepeatedStart(base[bus], slaveid, kLPI2C_Read);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterReceive(base[bus], data, 1);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterStop(base[bus]);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    return kStatus_Success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取I2C设备指定地址寄存器的数据
//  @param      iic_n       IIC模块(IIC_1,IIC_2,IIC_3,IIC_4)
//  @param      slaveid     从机地址(7位地址)
//  @param      reg         从机寄存器地址
//  @param      data        读取的数据存储的地址
//  @param      num         读取字节数
//  @return                 返回的状态值 0：成功  1：失败
//  @since      v2.0
//  Sample usage:       	uint8_t value = iic_read_reg(IIC_2, 0x2D, 0x50, buf, 10);//读取0x50地址的数据，从机地址为0x2D开始的10个字节
//-------------------------------------------------------------------------------------------------------------------
uint8_t iic_read_reg_bytes(uint8_t bus, uint8_t slaveid, uint8_t reg, uint8_t *data, uint8_t num)
{
    status_t reVal = kStatus_Fail;

    reVal = LPI2C_MasterStart(base[bus], slaveid, kLPI2C_Write);
    if (reVal != kStatus_Success)
        return kStatus_Fail;
    while (LPI2C_MasterGetStatusFlags(base[bus]) & kLPI2C_MasterNackDetectFlag)
        ;

    reVal = LPI2C_MasterSend(base[bus], &reg, 1);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterRepeatedStart(base[bus], slaveid, kLPI2C_Read);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterReceive(base[bus], data, num);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    reVal = LPI2C_MasterStop(base[bus]);
    if (reVal != kStatus_Success)
        return kStatus_Fail;

    return kStatus_Success;
}
