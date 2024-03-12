/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_lpspi.h"
#include "fsl_lpi2c.h"
#include "fsl_pwm.h"
#include "fsl_adapter_gpio.h"
#include "pin_mux.h"
#include "fsl_enc.h"
#include "fsl_adc.h"
#include "fsl_pit.h"
#include "fsl_flexio_camera.h"
#include "fsl_flexio_camera_edma.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "diskio.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Used DMA device. */
#define DMA0_DMA_BASEADDR DMA0
/* Associated DMAMUX device that is used for muxing of requests. */
#define DMA0_DMAMUX_BASEADDR DMAMUX
/* BOARD_InitPeripherals defines for LPSPI4 */
/* Definition of peripheral ID */
#define LPSPI4_PERIPHERAL LPSPI4
/* Definition of clock source */
#define LPSPI4_CLOCK_FREQ 105600000UL
/* BOARD_InitPeripherals defines for LPSPI2 */
/* Definition of peripheral ID */
#define LPSPI2_PERIPHERAL LPSPI2
/* Definition of clock source */
#define LPSPI2_CLOCK_FREQ 105600000UL
/* BOARD_InitPeripherals defines for LPI2C1 */
/* Definition of peripheral ID */
#define LPI2C1_PERIPHERAL LPI2C1
/* Definition of clock source */
#define LPI2C1_CLOCK_FREQ 60000000UL
/* Definition of slave address */
#define LPI2C1_MASTER_SLAVE_ADDRESS 0
/* Definition of peripheral ID */
#define PWM2_PERIPHERAL PWM2
/* Definition of submodule 0 ID */
#define PWM2_SM0 kPWM_Module_0
/* Definition of clock source of submodule 0 frequency in Hertz */
#define PWM2_SM0_SM_CLK_SOURCE_FREQ_HZ 125000000U
/* Definition of submodule 0 counter clock source frequency in Hertz - PWM2_SM0_SM_CLK_SOURCE_FREQ_HZ divided by prescaler */
#define PWM2_SM0_COUNTER_CLK_SOURCE_FREQ_HZ 62500000U
/* Definition of submodule 0 counter (PWM) frequency in Hertz */
#define PWM2_SM0_COUNTER_FREQ_HZ 16001U
/* Definition of submodule 0 channel A ID */
#define PWM2_SM0_A kPWM_PwmA
/* Definition of submodule 0 channel B ID */
#define PWM2_SM0_B kPWM_PwmB
/* Definition of submodule 0 channel X ID */
#define PWM2_SM0_X kPWM_PwmX
/* Definition of fault Fault0 ID */
#define PWM2_F0_FAULT0 kPWM_Fault_0
/* Definition of fault Fault1 ID */
#define PWM2_F0_FAULT1 kPWM_Fault_1
/* Definition of fault Fault2 ID */
#define PWM2_F0_FAULT2 kPWM_Fault_2
/* Definition of fault Fault3 ID */
#define PWM2_F0_FAULT3 kPWM_Fault_3
/* gpio_io, 05 signal defines */
/* Definition of the pin direction */
#define BOARD_ENC_Buttion_PIN_DIRECTION kHAL_GpioDirectionIn
/* Definition of the pin level after initialization */
#define BOARD_ENC_Buttion_PIN_LEVEL 0U
/* gpio_io, 31 signal defines */
/* Definition of the pin direction */
#define BOARD_LCD_BLK_PIN_DIRECTION kHAL_GpioDirectionOut
/* Definition of the pin level after initialization */
#define BOARD_LCD_BLK_PIN_LEVEL 1U
/* Definition of peripheral ID */
#define ENC1_PERIPHERAL ENC1
/* BOARD_InitPeripherals defines for ADC1 */
/* Definition of peripheral ID */
#define ADC1_PERIPHERAL ADC1
/* Definition of special channel interconnected with ADC_ETC which takes real channel to be measured from ADC_ETC. */
#define ADC1_CHANNEL_DRIVEN_BY_ADC_ETC 16U
/* Channel 0 (IN.0) conversion control group. */
#define ADC1_CH0_CONTROL_GROUP 0U
/* BOARD_InitPeripherals defines for LPI2C3 */
/* Definition of peripheral ID */
#define LPI2C3_PERIPHERAL LPI2C3
/* Definition of clock source */
#define LPI2C3_CLOCK_FREQ 60000000UL
/* Definition of slave address */
#define LPI2C3_MASTER_SLAVE_ADDRESS 0
/* gpio_io, 05 signal defines */
/* Definition of the pin direction */
#define BOARD_USER_LED_PIN_DIRECTION kHAL_GpioDirectionOut
/* Definition of the pin level after initialization */
#define BOARD_USER_LED_PIN_LEVEL 0U
/* gpio_io, 18 signal defines */
/* Definition of the pin direction */
#define BOARD_CAM_PWDN_PIN_DIRECTION kHAL_GpioDirectionOut
/* Definition of the pin level after initialization */
#define BOARD_CAM_PWDN_PIN_LEVEL 0U
/* gpio_io, 19 signal defines */
/* Definition of the pin direction */
#define BOARD_CAM_RES_PIN_DIRECTION kHAL_GpioDirectionOut
/* Definition of the pin level after initialization */
#define BOARD_CAM_RES_PIN_LEVEL 0U
/* gpio_io, 20 signal defines */
/* Definition of the pin direction */
#define BOARD_CAM_VS_PIN_DIRECTION kHAL_GpioDirectionIn
/* Definition of the pin level after initialization */
#define BOARD_CAM_VS_PIN_LEVEL 0U
/* Definition of the pin trigger mode */
#define BOARD_CAM_VS_TRIGGER_MODE kHAL_GpioInterruptRisingEdge
/* Definition of peripheral ID */
#define PWM1_PERIPHERAL PWM1
/* Definition of submodule 0 ID */
#define PWM1_SM0 kPWM_Module_0
/* Definition of clock source of submodule 0 frequency in Hertz */
#define PWM1_SM0_SM_CLK_SOURCE_FREQ_HZ 96000000U
/* Definition of submodule 0 counter clock source frequency in Hertz - PWM1_SM0_SM_CLK_SOURCE_FREQ_HZ divided by prescaler */
#define PWM1_SM0_COUNTER_CLK_SOURCE_FREQ_HZ 96000000U
/* Definition of submodule 0 counter (PWM) frequency in Hertz */
#define PWM1_SM0_COUNTER_FREQ_HZ 24000000U
/* Definition of submodule 0 channel A ID */
#define PWM1_SM0_A kPWM_PwmA
/* Definition of submodule 0 channel B ID */
#define PWM1_SM0_B kPWM_PwmB
/* Definition of submodule 0 channel X ID */
#define PWM1_SM0_X kPWM_PwmX
/* Definition of fault Fault0 ID */
#define PWM1_F0_FAULT0 kPWM_Fault_0
/* Definition of fault Fault1 ID */
#define PWM1_F0_FAULT1 kPWM_Fault_1
/* Definition of fault Fault2 ID */
#define PWM1_F0_FAULT2 kPWM_Fault_2
/* Definition of fault Fault3 ID */
#define PWM1_F0_FAULT3 kPWM_Fault_3
/* BOARD_InitPeripherals defines for PIT */
/* Definition of peripheral ID. */
#define PIT_PERIPHERAL PIT
/* Definition of clock source frequency. */
#define PIT_CLK_FREQ 62500000UL
/* Definition of ticks count for channel 0 - deprecated. */
#define PIT_0_TICKS 312500U
/* PIT interrupt vector ID (number) - deprecated. */
#define PIT_0_IRQN PIT_IRQn
/* PIT interrupt handler identifier - deprecated. */
#define PIT_0_IRQHANDLER PIT0_IRQHandler
/* Definition of channel number for channel 0. */
#define PIT_CHANNEL_0 kPIT_Chnl_0
/* Definition of ticks count for channel 0. */
#define PIT_CHANNEL_0_TICKS 312500U
/* PIT interrupt vector ID (number). */
#define PIT_IRQN PIT_IRQn
/* PIT interrupt vector priority. */
#define PIT_IRQ_PRIORITY 0
/* PIT interrupt handler identifier. */
#define PIT_IRQHANDLER PIT_IRQHandler
/* Definition of peripheral ID */
#define FLEXIO1_PERIPHERAL FLEXIO1
/* Definition of the clock source frequency */
#define FLEXIO1_CLK_FREQ 30000000UL
/* Definition of the frame width (number of pixels in the line) */
#define FLEXIO1_FRAME_WIDTH 160
/* Definition of the frame height (number of lines) */
#define FLEXIO1_FRAME_HEIGHT 120
/* Definition of the bytes per pixel size */
#define FLEXIO1_BYTES_PER_PIXEL 2
/* Definition of the line pitch size in bytes */
#define FLEXIO1_LINE_PITCH_BYTES 320
/* Definition of number of the buffers inside allocated block */
#define FLEXIO1_FRAME_BUFFER_COUNT 2
/* Definition of camera buffer alignment */
#define FLEXIO1_BUFFER_ALIGN 32
/* FLEXIO1 eDMA source request. */
#define FLEXIO1_FLEXIO_0_DMA_REQUEST kDmaRequestMuxFlexIO1Request0Request1
/* Selected eDMA channel number. */
#define FLEXIO1_FLEXIO_0_DMA_CHANNEL 0
/* DMAMUX device that is used for muxing of the request. */
#define FLEXIO1_FLEXIO_0_DMAMUX_BASEADDR DMAMUX
/* Used DMA device. */
#define FLEXIO1_FLEXIO_0_DMA_BASEADDR DMA0
/* Debug console is initialized in the peripheral tool */
#define BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL 
/* Definition of serial peripheral instance */
#define DEBUGCONSOLE_INSTANCE 1U
/* Definition of serial peripheral type */
#define DEBUGCONSOLE_TYPE kSerialPort_Uart
/* Definition of the Baud rate */
#define DEBUGCONSOLE_BAUDRATE 115200UL
/* Definition of the clock source frequency */
#define DEBUGCONSOLE_CLK_FREQ 80000000UL

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const edma_config_t DMA0_config;
extern const lpspi_master_config_t LPSPI4_config;
extern const lpspi_master_config_t LPSPI2_config;
extern const lpi2c_master_config_t LPI2C1_masterConfig;
extern pwm_config_t PWM2_SM0_config;

extern pwm_signal_param_t PWM2_SM0_pwm_function_config[2];
extern const pwm_fault_input_filter_param_t PWM2_faultInputFilter_config;
extern const pwm_fault_param_t PWM2_Fault0_fault_config;
extern const pwm_fault_param_t PWM2_Fault1_fault_config;
extern const pwm_fault_param_t PWM2_Fault2_fault_config;
extern const pwm_fault_param_t PWM2_Fault3_fault_config;
extern GPIO_HANDLE_DEFINE(BOARD_ENC_Buttion_handle);
extern GPIO_HANDLE_DEFINE(BOARD_LCD_BLK_handle);
extern enc_config_t ENC1_config;
extern const adc_config_t ADC1_config;
extern const adc_channel_config_t ADC1_channels_config[1];
extern const lpi2c_master_config_t LPI2C3_masterConfig;
extern GPIO_HANDLE_DEFINE(BOARD_USER_LED_handle);
extern GPIO_HANDLE_DEFINE(BOARD_CAM_PWDN_handle);
extern GPIO_HANDLE_DEFINE(BOARD_CAM_RES_handle);
extern GPIO_HANDLE_DEFINE(BOARD_CAM_VS_handle);
extern pwm_config_t PWM1_SM0_config;

extern pwm_signal_param_t PWM1_SM0_pwm_function_config[1];
extern const pwm_fault_input_filter_param_t PWM1_faultInputFilter_config;
extern const pwm_fault_param_t PWM1_Fault0_fault_config;
extern const pwm_fault_param_t PWM1_Fault1_fault_config;
extern const pwm_fault_param_t PWM1_Fault2_fault_config;
extern const pwm_fault_param_t PWM1_Fault3_fault_config;
extern const pit_config_t PIT_config;
/* Frame buffer block */
extern uint16_t FLEXIO1_Camera_Buffer[FLEXIO1_FRAME_BUFFER_COUNT][FLEXIO1_FRAME_HEIGHT][FLEXIO1_FRAME_WIDTH];
extern FLEXIO_CAMERA_Type FLEXIO1_peripheralConfig;
extern flexio_camera_config_t FLEXIO1_config;
extern edma_handle_t FLEXIO1_FLEXIO_0_Handle;
extern flexio_camera_edma_handle_t FLEXIO1_Camera_eDMA_Handle;
/* FATFS System object */
extern FATFS FATFS_System_0;

/***********************************************************************************************************************
 * Global functions
 **********************************************************************************************************************/
/* Get GPIO pin configuration */
hal_gpio_pin_config_t createAdapterGpioPinConfig(GPIO_Type *port, uint8_t pin, hal_gpio_direction_t direction, uint8_t level);

/***********************************************************************************************************************
 * Callback functions
 **********************************************************************************************************************/
/* Callback function for the BOARD_CAM_VS_handle*/
extern void BOARD_CAM_VS_callback(void *param);
/* FlexIO Camera transfer Rx callback function for the FLEXIO1 component (init. function BOARD_InitPeripherals)*/
extern void CAM_DMA_COMPLETE(FLEXIO_CAMERA_Type *base, flexio_camera_edma_handle_t *handle, status_t status, void *userData);
/* Extern function for the physical layer initialization*/
extern void FATFS_DiskInit(void);

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/

void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
