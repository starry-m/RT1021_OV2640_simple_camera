#include "com_delay.h"
#include "fsl_debug_console.h"

extern void SysTick_DelayTicks(uint32_t n);

void HAL_Delay(uint32_t mdelay)
{
    SysTick_DelayTicks(mdelay);
}
void serial_tx(uint8_t *tx_buffer, uint16_t len)
{
    DbgConsole_SendDataReliable(tx_buffer,len);
}