#ifndef __COM_DELAY_H__
#define __COM_DELAY_H__

#include <stdint.h>
#include <stddef.h>
#include <math.h>

void HAL_Delay(uint32_t mdelay);
void serial_tx(uint8_t *tx_buffer, uint16_t len);

#endif
