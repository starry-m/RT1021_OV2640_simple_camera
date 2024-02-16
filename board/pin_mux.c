/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v15.0
processor: MIMXRT1021xxxxx
package_id: MIMXRT1021DAG5A
mcu_data: ksdk2_0
processor_version: 15.0.1
board: MIMXRT1020-EVK
external_user_signals: {}
pin_labels:
- {pin_num: '54', pin_signal: PMIC_STBY_REQ, label: SD_PWREN, identifier: U_LED}
- {pin_num: '13', pin_signal: GPIO_EMC_05, label: 'SEMC_D5/U14[10]', identifier: SEMC_D5;ENC_SW;ENC_Buttion}
- {pin_num: '125', pin_signal: GPIO_EMC_31, label: 'SEMC_DM1/U14[39]', identifier: SEMC_DM1;LCD_BLK}
- {pin_num: '123', pin_signal: GPIO_EMC_33, label: 'SEMC_D9/U14[44]', identifier: SEMC_D9;LCD_CS}
- {pin_num: '120', pin_signal: GPIO_EMC_36, label: 'SEMC_D12/U14[48]', identifier: SEMC_D12;LCD_DC}
- {pin_num: '117', pin_signal: GPIO_EMC_39, label: 'SEMC_D15/U14[53]', identifier: SEMC_D15;LCD_RES}
- {pin_num: '106', pin_signal: GPIO_AD_B0_05, label: 'JTAG_nTRST/J16[3]/USER_LED/J17[5]', identifier: USER_LED}
- {pin_num: '98', pin_signal: GPIO_AD_B0_10, label: 'ENET_RXD0/U11[16]/J19[6]', identifier: ENET_RXD0;LED1}
- {pin_num: '94', pin_signal: GPIO_AD_B0_14, label: 'ENET_TXD0/U11[24]/J17[7]', identifier: ENET_TXD0;LED2}
- {pin_num: '93', pin_signal: GPIO_AD_B0_15, label: 'ENET_TXD1/U11[25]/J19[2]', identifier: ENET_TXD1;LED3}
- {pin_num: '90', pin_signal: GPIO_AD_B1_02, label: 'SAI1_TX_SYNC/J19[10]', identifier: CAM_PWDN}
- {pin_num: '89', pin_signal: GPIO_AD_B1_03, label: 'SAI1_TXD/J19[9]', identifier: CAM_RES}
- {pin_num: '88', pin_signal: GPIO_AD_B1_04, label: AUD_INT, identifier: CAM_VS}
- {pin_num: '87', pin_signal: GPIO_AD_B1_05, label: SAI1_RXD, identifier: CAM_HS}
- {pin_num: '83', pin_signal: GPIO_AD_B1_07, label: 'SD0_VSELECT/J19[1]', identifier: CAM_XCLK}
power_domains: {NVCC_GPIO: '3.3'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_xbara.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '106', peripheral: GPIO1, signal: 'gpio_io, 05', pin_signal: GPIO_AD_B0_05, direction: OUTPUT, slew_rate: Slow, software_input_on: Disable, open_drain: Disable,
    speed: MHZ_100, drive_strength: R0_6, pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Down_100K_Ohm, hysteresis_enable: Disable}
  - {pin_num: '97', peripheral: ARM, signal: arm_trace_swo, pin_signal: GPIO_AD_B0_11, slew_rate: Slow}
  - {pin_num: '54', peripheral: GPIO5, signal: 'gpio_io, 02', pin_signal: PMIC_STBY_REQ, direction: OUTPUT}
  - {pin_num: '105', peripheral: LPUART1, signal: TX, pin_signal: GPIO_AD_B0_06}
  - {pin_num: '101', peripheral: LPUART1, signal: RX, pin_signal: GPIO_AD_B0_07}
  - {pin_num: '124', peripheral: LPSPI4, signal: SCK, pin_signal: GPIO_EMC_32}
  - {pin_num: '123', peripheral: LPSPI4, signal: PCS0, pin_signal: GPIO_EMC_33, identifier: LCD_CS, direction: OUTPUT}
  - {pin_num: '122', peripheral: LPSPI4, signal: SDO, pin_signal: GPIO_EMC_34}
  - {pin_num: '121', peripheral: LPSPI4, signal: SDI, pin_signal: GPIO_EMC_35}
  - {pin_num: '120', peripheral: GPIO3, signal: 'gpio_io, 04', pin_signal: GPIO_EMC_36, identifier: LCD_DC, direction: OUTPUT}
  - {pin_num: '119', peripheral: LPSPI4, signal: PCS2, pin_signal: GPIO_EMC_37, direction: OUTPUT}
  - {pin_num: '125', peripheral: GPIO2, signal: 'gpio_io, 31', pin_signal: GPIO_EMC_31, identifier: LCD_BLK, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '117', peripheral: GPIO3, signal: 'gpio_io, 07', pin_signal: GPIO_EMC_39, identifier: LCD_RES, direction: OUTPUT}
  - {pin_num: '7', peripheral: LPSPI2, signal: SCK, pin_signal: GPIO_EMC_10}
  - {pin_num: '4', peripheral: LPSPI2, signal: PCS0, pin_signal: GPIO_EMC_11, direction: OUTPUT}
  - {pin_num: '3', peripheral: LPSPI2, signal: SDO, pin_signal: GPIO_EMC_12}
  - {pin_num: '2', peripheral: LPSPI2, signal: SDI, pin_signal: GPIO_EMC_13}
  - {pin_num: '1', peripheral: LPSPI2, signal: PCS1, pin_signal: GPIO_EMC_14, direction: OUTPUT}
  - {pin_num: '16', peripheral: LPI2C1, signal: SCL, pin_signal: GPIO_EMC_02, software_input_on: Enable, open_drain: Enable}
  - {pin_num: '15', peripheral: LPI2C1, signal: SDA, pin_signal: GPIO_EMC_03, software_input_on: Enable, open_drain: Enable}
  - {pin_num: '98', peripheral: PWM2, signal: 'A, 2', pin_signal: GPIO_AD_B0_10, identifier: LED1, direction: OUTPUT}
  - {pin_num: '94', peripheral: PWM2, signal: 'A, 0', pin_signal: GPIO_AD_B0_14, identifier: LED2, direction: OUTPUT}
  - {pin_num: '93', peripheral: PWM2, signal: 'B, 0', pin_signal: GPIO_AD_B0_15, identifier: LED3, direction: OUTPUT}
  - {pin_num: '10', peripheral: ENC1, signal: 'PHASE, A', pin_signal: GPIO_EMC_07}
  - {pin_num: '8', peripheral: ENC1, signal: 'PHASE, B', pin_signal: GPIO_EMC_09}
  - {pin_num: '13', peripheral: GPIO2, signal: 'gpio_io, 05', pin_signal: GPIO_EMC_05, identifier: ENC_Buttion, direction: INPUT}
  - {pin_num: '96', peripheral: ADC1, signal: 'IN, 0', pin_signal: GPIO_AD_B0_12}
  - {pin_num: '100', peripheral: LPI2C3, signal: SCL, pin_signal: GPIO_AD_B0_08, software_input_on: Enable, open_drain: Enable}
  - {pin_num: '99', peripheral: LPI2C3, signal: SDA, pin_signal: GPIO_AD_B0_09, software_input_on: Enable, open_drain: Enable}
  - {pin_num: '74', peripheral: FLEXIO1, signal: 'IO, 00', pin_signal: GPIO_AD_B1_15}
  - {pin_num: '75', peripheral: FLEXIO1, signal: 'IO, 01', pin_signal: GPIO_AD_B1_14}
  - {pin_num: '76', peripheral: FLEXIO1, signal: 'IO, 02', pin_signal: GPIO_AD_B1_13}
  - {pin_num: '78', peripheral: FLEXIO1, signal: 'IO, 03', pin_signal: GPIO_AD_B1_12}
  - {pin_num: '80', peripheral: FLEXIO1, signal: 'IO, 05', pin_signal: GPIO_AD_B1_10}
  - {pin_num: '79', peripheral: FLEXIO1, signal: 'IO, 04', pin_signal: GPIO_AD_B1_11}
  - {pin_num: '81', peripheral: FLEXIO1, signal: 'IO, 06', pin_signal: GPIO_AD_B1_09}
  - {pin_num: '82', peripheral: FLEXIO1, signal: 'IO, 07', pin_signal: GPIO_AD_B1_08}
  - {pin_num: '84', peripheral: FLEXIO1, signal: 'IO, 09', pin_signal: GPIO_AD_B1_06}
  - {pin_num: '90', peripheral: GPIO1, signal: 'gpio_io, 18', pin_signal: GPIO_AD_B1_02, direction: OUTPUT}
  - {pin_num: '89', peripheral: GPIO1, signal: 'gpio_io, 19', pin_signal: GPIO_AD_B1_03, direction: OUTPUT}
  - {pin_num: '88', peripheral: GPIO1, signal: 'gpio_io, 20', pin_signal: GPIO_AD_B1_04, direction: INPUT, gpio_interrupt: kGPIO_IntFallingEdge, slew_rate: Fast}
  - {pin_num: '87', peripheral: FLEXIO1, signal: 'IO, 10', pin_signal: GPIO_AD_B1_05}
  - {pin_num: '83', peripheral: PWM1, signal: 'B, 0', pin_signal: GPIO_AD_B1_07, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           
  CLOCK_EnableClock(kCLOCK_IomuxcSnvs);       
  CLOCK_EnableClock(kCLOCK_Xbar1);            

  /* GPIO configuration of USER_LED on GPIO_AD_B0_05 (pin 106) */
  gpio_pin_config_t USER_LED_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_B0_05 (pin 106) */
  GPIO_PinInit(GPIO1, 5U, &USER_LED_config);

  /* GPIO configuration of CAM_PWDN on GPIO_AD_B1_02 (pin 90) */
  gpio_pin_config_t CAM_PWDN_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_B1_02 (pin 90) */
  GPIO_PinInit(GPIO1, 18U, &CAM_PWDN_config);

  /* GPIO configuration of CAM_RES on GPIO_AD_B1_03 (pin 89) */
  gpio_pin_config_t CAM_RES_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_B1_03 (pin 89) */
  GPIO_PinInit(GPIO1, 19U, &CAM_RES_config);

  /* GPIO configuration of CAM_VS on GPIO_AD_B1_04 (pin 88) */
  gpio_pin_config_t CAM_VS_config = {
      .direction = kGPIO_DigitalInput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_IntFallingEdge
  };
  /* Initialize GPIO functionality on GPIO_AD_B1_04 (pin 88) */
  GPIO_PinInit(GPIO1, 20U, &CAM_VS_config);
  /* Enable GPIO pin interrupt on GPIO_AD_B1_04 (pin 88) */
  GPIO_PortEnableInterrupts(GPIO1, 1U << 20U);

  /* GPIO configuration of ENC_Buttion on GPIO_EMC_05 (pin 13) */
  gpio_pin_config_t ENC_Buttion_config = {
      .direction = kGPIO_DigitalInput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_EMC_05 (pin 13) */
  GPIO_PinInit(GPIO2, 5U, &ENC_Buttion_config);

  /* GPIO configuration of LCD_BLK on GPIO_EMC_31 (pin 125) */
  gpio_pin_config_t LCD_BLK_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 1U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_EMC_31 (pin 125) */
  GPIO_PinInit(GPIO2, 31U, &LCD_BLK_config);

  /* GPIO configuration of LCD_DC on GPIO_EMC_36 (pin 120) */
  gpio_pin_config_t LCD_DC_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_EMC_36 (pin 120) */
  GPIO_PinInit(GPIO3, 4U, &LCD_DC_config);

  /* GPIO configuration of LCD_RES on GPIO_EMC_39 (pin 117) */
  gpio_pin_config_t LCD_RES_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_EMC_39 (pin 117) */
  GPIO_PinInit(GPIO3, 7U, &LCD_RES_config);

  /* GPIO configuration of U_LED on PMIC_STBY_REQ (pin 54) */
  gpio_pin_config_t U_LED_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on PMIC_STBY_REQ (pin 54) */
  GPIO_PinInit(GPIO5, 2U, &U_LED_config);

  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_LPI2C3_SCL, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_LPI2C3_SDA, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_FLEXPWM2_PWMA02, 0U); 
#if FSL_IOMUXC_DRIVER_VERSION >= MAKE_VERSION(2, 0, 3)
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_ARM_TRACE_SWO, 0U); 
#else
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_ARM_CM7_TRACE_SWO, 0U); 
#endif
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_GPIO1_IO12, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_14_FLEXPWM2_PWMA00, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_15_FLEXPWM2_PWMB00, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_GPIO1_IO18, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_GPIO1_IO19, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_GPIO1_IO20, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_FLEXIO1_FLEXIO10, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_FLEXIO1_FLEXIO09, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_FLEXPWM1_PWMB00, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_FLEXIO1_FLEXIO07, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_FLEXIO1_FLEXIO06, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_FLEXIO1_FLEXIO05, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_FLEXIO1_FLEXIO04, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_12_FLEXIO1_FLEXIO03, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_FLEXIO1_FLEXIO02, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_FLEXIO1_FLEXIO01, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_FLEXIO1_FLEXIO00, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_02_LPI2C1_SCL, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_03_LPI2C1_SDA, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_05_GPIO2_IO05, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_07_XBAR1_INOUT07, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_09_XBAR1_INOUT09, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_10_LPSPI2_SCK, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_11_LPSPI2_PCS0, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_12_LPSPI2_SDO, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_14_LPSPI2_PCS1, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_31_GPIO2_IO31, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_32_LPSPI4_SCK, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_33_LPSPI4_PCS0, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_34_LPSPI4_SDO, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_35_LPSPI4_SDI, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_36_GPIO3_IO04, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_37_LPSPI4_PCS2, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_39_GPIO3_IO07, 0U); 
  IOMUXC_GPR->GPR6 = ((IOMUXC_GPR->GPR6 &
    (~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_7_MASK | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_9_MASK))) 
      | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_7(0x00U) 
      | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_9(0x00U) 
    );
  IOMUXC_SetPinMux(IOMUXC_SNVS_PMIC_STBY_REQ_GPIO5_IO02, 0U); 
  XBARA_SetSignalsConnection(XBARA, kXBARA1_InputIomuxXbarInout07, kXBARA1_OutputEnc1PhaseAInput); 
  XBARA_SetSignalsConnection(XBARA, kXBARA1_InputIomuxXbarInout09, kXBARA1_OutputEnc1PhaseBInput); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, 0x10B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_LPI2C3_SCL, 0x18B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_LPI2C3_SDA, 0x18B0U); 
#if FSL_IOMUXC_DRIVER_VERSION >= MAKE_VERSION(2, 0, 3)
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_ARM_TRACE_SWO, 0x10B0U); 
#else
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_ARM_CM7_TRACE_SWO, 0x10B0U); 
#endif
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_04_GPIO1_IO20, 0x10B1U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_02_LPI2C1_SCL, 0x18B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_03_LPI2C1_SDA, 0x18B0U); 
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
