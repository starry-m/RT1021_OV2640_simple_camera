#ifndef __COM_DELAY_H__
#define __COM_DELAY_H__

#include <stdint.h>
#include <stddef.h>
#include <math.h>

void HAL_Delay(uint32_t mdelay);
void serial_tx(uint8_t *tx_buffer, uint16_t len);
void HAL_DelayUs(uint32_t udelay);

void led_state_set(uint8_t mled, uint8_t state);

uint8_t iic_write_reg(uint8_t bus, uint8_t slaveid, uint8_t reg, uint8_t data);
uint8_t iic_read_reg(uint8_t bus, uint8_t slaveid, uint8_t reg, uint8_t *data);
uint8_t iic_read_reg_bytes(uint8_t bus, uint8_t slaveid, uint8_t reg, uint8_t *data, uint8_t num);




#endif
