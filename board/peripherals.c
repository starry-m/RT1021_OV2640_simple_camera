/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v14.0
processor: MIMXRT1021xxxxx
package_id: MIMXRT1021DAG5A
mcu_data: ksdk2_0
processor_version: 15.0.1
board: MIMXRT1020-EVK
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 6525c482-a62e-461b-b29d-a618d9214dce
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
  - global_init: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - commonSetting:
    - HAL_GPIO_ISR_PRIORITY: '3'
    - HAL_GpioPreInit: 'true'
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * LPSPI4 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPSPI4'
- type: 'lpspi'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'lpspi_3b8318dca8e0034b76e041f04d445c24'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPSPI4'
- config_sets:
  - main:
    - mode: 'kLPSPI_Master'
    - clockSource: 'LpspiClock'
    - clockSourceFreq: 'ClocksTool_DefaultInit'
    - master:
      - baudRate: '50000000'
      - bitsPerFrame: '8'
      - cpol: 'kLPSPI_ClockPolarityActiveHigh'
      - cpha: 'kLPSPI_ClockPhaseFirstEdge'
      - direction: 'kLPSPI_MsbFirst'
      - pcsToSckDelayInNanoSec: '1000'
      - lastSckToPcsDelayInNanoSec: '1000'
      - betweenTransferDelayInNanoSec: '1000'
      - whichPcs: 'kLPSPI_Pcs0'
      - pcsActiveHighOrLow: 'kLPSPI_PcsActiveLow'
      - pinCfg: 'kLPSPI_SdiInSdoOut'
      - pcsFunc: 'kLPSPI_PcsAsCs'
      - dataOutConfig: 'kLpspiDataOutRetained'
      - enableInputDelay: 'false'
    - set_FifoWaterMarks: 'false'
    - fifoWaterMarks:
      - txWatermark: '0'
      - rxWatermark: '0'
    - allPcsPolarityEnable: 'false'
    - allPcsPolarity:
      - kLPSPI_Pcs1Active: 'kLPSPI_PcsActiveLow'
      - kLPSPI_Pcs2Active: 'kLPSPI_PcsActiveLow'
      - kLPSPI_Pcs3Active: 'kLPSPI_PcsActiveLow'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpspi_master_config_t LPSPI4_config = {
  .baudRate = 50000000UL,
  .bitsPerFrame = 8UL,
  .cpol = kLPSPI_ClockPolarityActiveHigh,
  .cpha = kLPSPI_ClockPhaseFirstEdge,
  .direction = kLPSPI_MsbFirst,
  .pcsToSckDelayInNanoSec = 1000UL,
  .lastSckToPcsDelayInNanoSec = 1000UL,
  .betweenTransferDelayInNanoSec = 1000UL,
  .whichPcs = kLPSPI_Pcs0,
  .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,
  .pinCfg = kLPSPI_SdiInSdoOut,
  .pcsFunc = kLPSPI_PcsAsCs,
  .dataOutConfig = kLpspiDataOutRetained,
  .enableInputDelay = false
};

static void LPSPI4_init(void) {
  LPSPI_MasterInit(LPSPI4_PERIPHERAL, &LPSPI4_config, LPSPI4_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * LPSPI2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPSPI2'
- type: 'lpspi'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'lpspi_3b8318dca8e0034b76e041f04d445c24'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPSPI2'
- config_sets:
  - main:
    - mode: 'kLPSPI_Master'
    - clockSource: 'LpspiClock'
    - clockSourceFreq: 'ClocksTool_DefaultInit'
    - master:
      - baudRate: '500000'
      - bitsPerFrame: '8'
      - cpol: 'kLPSPI_ClockPolarityActiveHigh'
      - cpha: 'kLPSPI_ClockPhaseFirstEdge'
      - direction: 'kLPSPI_MsbFirst'
      - pcsToSckDelayInNanoSec: '1000'
      - lastSckToPcsDelayInNanoSec: '1000'
      - betweenTransferDelayInNanoSec: '1000'
      - whichPcs: 'kLPSPI_Pcs0'
      - pcsActiveHighOrLow: 'kLPSPI_PcsActiveLow'
      - pinCfg: 'kLPSPI_SdiInSdoOut'
      - pcsFunc: 'kLPSPI_PcsAsCs'
      - dataOutConfig: 'kLpspiDataOutRetained'
      - enableInputDelay: 'false'
    - set_FifoWaterMarks: 'false'
    - fifoWaterMarks:
      - txWatermark: '0'
      - rxWatermark: '0'
    - allPcsPolarityEnable: 'false'
    - allPcsPolarity:
      - kLPSPI_Pcs1Active: 'kLPSPI_PcsActiveLow'
      - kLPSPI_Pcs2Active: 'kLPSPI_PcsActiveLow'
      - kLPSPI_Pcs3Active: 'kLPSPI_PcsActiveLow'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpspi_master_config_t LPSPI2_config = {
  .baudRate = 500000UL,
  .bitsPerFrame = 8UL,
  .cpol = kLPSPI_ClockPolarityActiveHigh,
  .cpha = kLPSPI_ClockPhaseFirstEdge,
  .direction = kLPSPI_MsbFirst,
  .pcsToSckDelayInNanoSec = 1000UL,
  .lastSckToPcsDelayInNanoSec = 1000UL,
  .betweenTransferDelayInNanoSec = 1000UL,
  .whichPcs = kLPSPI_Pcs0,
  .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,
  .pinCfg = kLPSPI_SdiInSdoOut,
  .pcsFunc = kLPSPI_PcsAsCs,
  .dataOutConfig = kLpspiDataOutRetained,
  .enableInputDelay = false
};

static void LPSPI2_init(void) {
  LPSPI_MasterInit(LPSPI2_PERIPHERAL, &LPSPI2_config, LPSPI2_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * LPI2C1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPI2C1'
- type: 'lpi2c'
- mode: 'master'
- custom_name_enabled: 'false'
- type_id: 'lpi2c_6b71962515c3208facfccd030afebc98'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPI2C1'
- config_sets:
  - main:
    - clockSource: 'Lpi2cClock'
    - clockSourceFreq: 'ClocksTool_DefaultInit'
  - interrupt_vector: []
  - master:
    - mode: 'polling'
    - config:
      - enableMaster: 'true'
      - enableDoze: 'true'
      - debugEnable: 'false'
      - ignoreAck: 'false'
      - pinConfig: 'kLPI2C_2PinOpenDrain'
      - baudRate_Hz: '100000'
      - busIdleTimeout_ns: '0'
      - pinLowTimeout_ns: '0'
      - sdaGlitchFilterWidth_ns: '0'
      - sclGlitchFilterWidth_ns: '0'
      - hostRequest:
        - enable: 'false'
        - source: 'kLPI2C_HostRequestExternalPin'
        - polarity: 'kLPI2C_HostRequestPinActiveHigh'
      - edmaRequestSources: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpi2c_master_config_t LPI2C1_masterConfig = {
  .enableMaster = true,
  .enableDoze = true,
  .debugEnable = false,
  .ignoreAck = false,
  .pinConfig = kLPI2C_2PinOpenDrain,
  .baudRate_Hz = 100000UL,
  .busIdleTimeout_ns = 0UL,
  .pinLowTimeout_ns = 0UL,
  .sdaGlitchFilterWidth_ns = 0U,
  .sclGlitchFilterWidth_ns = 0U,
  .hostRequest = {
    .enable = false,
    .source = kLPI2C_HostRequestExternalPin,
    .polarity = kLPI2C_HostRequestPinActiveHigh
  }
};

static void LPI2C1_init(void) {
  LPI2C_MasterInit(LPI2C1_PERIPHERAL, &LPI2C1_masterConfig, LPI2C1_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * PWM2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PWM2'
- type: 'pwm'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'pwm_3d4233561d9e621ebdcd737703a67bfd'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'PWM2'
- config_sets:
  - fsl_pwm:
    - clockSource: 'SystemClock'
    - clockSourceFreq: 'ClocksTool_DefaultInit'
    - submodules:
      - 0:
        - sm: 'kPWM_Module_0'
        - sm_id: 'SM0'
        - config:
          - clockSource: 'kPWM_BusClock'
          - prescale: 'kPWM_Prescale_Divide_2'
          - pwmFreq: '16 kHz'
          - pairOperation: 'kPWM_Independent'
          - operationMode: 'kPWM_SignedCenterAligned'
          - initializationControl: 'kPWM_Initialize_LocalSync'
          - reloadLogic: 'kPWM_ReloadImmediate'
          - reloadSelect: 'kPWM_LocalReload'
          - reloadFrequency: 'kPWM_LoadEveryOportunity'
          - forceTrigger: 'kPWM_Force_Local'
          - enableDebugMode: 'false'
          - outputTrigger_sel: ''
          - loadOK: 'false'
          - startCounter: 'false'
          - interrupt_sel: ''
          - dma_used: 'false'
          - dma:
            - pwmDMA_activate: 'false'
            - captureDMA_enable: ''
            - captureDMA_source: 'kPWM_DMARequestDisable'
            - captureDMA_watermark_control: 'kPWM_FIFOWatermarksOR'
        - channels:
          - 0:
            - channel_id: 'A'
            - functionSel: 'pwmOutput'
            - pwm:
              - dutyCyclePercent: '0'
              - level: 'kPWM_LowTrue'
              - fault_channel0:
                - dismap: 'kPWM_FaultDisable_0 kPWM_FaultDisable_1 kPWM_FaultDisable_2 kPWM_FaultDisable_3'
                - quick_selection: 'default'
              - faultState: 'kPWM_PwmFaultState0'
              - pwmchannelenable: 'true'
              - deadtime_input_by_force: 'kPWM_UsePwm'
              - clockSource: 'kPWM_BusClock'
              - deadtimeValue: '0'
              - interrupt_sel: ''
          - 1:
            - channel_id: 'B'
            - functionSel: 'pwmOutput'
            - pwm:
              - dutyCyclePercent: '0'
              - level: 'kPWM_LowTrue'
              - fault_channel0:
                - dismap: 'kPWM_FaultDisable_0 kPWM_FaultDisable_1 kPWM_FaultDisable_2 kPWM_FaultDisable_3'
                - quick_selection: 'default'
              - faultState: 'kPWM_PwmFaultState0'
              - pwmchannelenable: 'true'
              - deadtime_input_by_force: 'kPWM_UsePwm'
              - clockSource: 'kPWM_BusClock'
              - deadtimeValue: '0'
              - interrupt_sel: ''
          - 2:
            - channel_id: 'X'
            - functionSel: 'notUsed'
        - common_interruptEn: 'false'
        - common_interrupt:
          - IRQn: 'PWM2_0_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
    - faultChannels:
      - 0:
        - commonFaultSetting:
          - clockSource: 'kPWM_BusClock'
          - faultFilterPeriod: '1'
          - faultFilterCount: '3'
          - faultGlitchStretch: 'false'
        - faults:
          - 0:
            - fault_id: 'Fault0'
            - faultClearingMode: 'kPWM_Automatic'
            - faultLevelR: 'low'
            - enableCombinationalPathR: 'filtered'
            - recoverMode: 'kPWM_NoRecovery'
            - fault_int_source: 'false'
          - 1:
            - fault_id: 'Fault1'
            - faultClearingMode: 'kPWM_Automatic'
            - faultLevelR: 'low'
            - enableCombinationalPathR: 'filtered'
            - recoverMode: 'kPWM_NoRecovery'
            - fault_int_source: 'false'
          - 2:
            - fault_id: 'Fault2'
            - faultClearingMode: 'kPWM_Automatic'
            - faultLevelR: 'low'
            - enableCombinationalPathR: 'filtered'
            - recoverMode: 'kPWM_NoRecovery'
            - fault_int_source: 'false'
          - 3:
            - fault_id: 'Fault3'
            - faultClearingMode: 'kPWM_Automatic'
            - faultLevelR: 'low'
            - enableCombinationalPathR: 'filtered'
            - recoverMode: 'kPWM_NoRecovery'
            - fault_int_source: 'false'
    - fault_error_interruptEn: 'false'
    - fault_error_interrupt:
      - IRQn: 'PWM2_FAULT_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
/* PWM main configuration */
pwm_config_t PWM2_SM0_config = {
  .clockSource = kPWM_BusClock,
  .prescale = kPWM_Prescale_Divide_2,
  .pairOperation = kPWM_Independent,
  .initializationControl = kPWM_Initialize_LocalSync,
  .reloadLogic = kPWM_ReloadImmediate,
  .reloadSelect = kPWM_LocalReload,
  .reloadFrequency = kPWM_LoadEveryOportunity,
  .forceTrigger = kPWM_Force_Local,
  .enableDebugMode = false,
};

pwm_signal_param_t PWM2_SM0_pwm_function_config[2]= {
  {
    .pwmChannel = kPWM_PwmA,
    .dutyCyclePercent = 0U,
    .level = kPWM_LowTrue,
    .faultState = kPWM_PwmFaultState0,
    .pwmchannelenable = true,
    .deadtimeValue = 0U
  },
  {
    .pwmChannel = kPWM_PwmB,
    .dutyCyclePercent = 0U,
    .level = kPWM_LowTrue,
    .faultState = kPWM_PwmFaultState0,
    .pwmchannelenable = true,
    .deadtimeValue = 0U
  },
};

const pwm_fault_input_filter_param_t PWM2_faultInputFilter_config = {
  .faultFilterPeriod = 1U,
  .faultFilterCount = 3U,
  .faultGlitchStretch = false
};
const pwm_fault_param_t PWM2_Fault0_fault_config = {
  .faultClearingMode = kPWM_Automatic,
  .faultLevel = false,
  .enableCombinationalPath = true,
  .recoverMode = kPWM_NoRecovery
};
const pwm_fault_param_t PWM2_Fault1_fault_config = {
  .faultClearingMode = kPWM_Automatic,
  .faultLevel = false,
  .enableCombinationalPath = true,
  .recoverMode = kPWM_NoRecovery
};
const pwm_fault_param_t PWM2_Fault2_fault_config = {
  .faultClearingMode = kPWM_Automatic,
  .faultLevel = false,
  .enableCombinationalPath = true,
  .recoverMode = kPWM_NoRecovery
};
const pwm_fault_param_t PWM2_Fault3_fault_config = {
  .faultClearingMode = kPWM_Automatic,
  .faultLevel = false,
  .enableCombinationalPath = true,
  .recoverMode = kPWM_NoRecovery
};

static void PWM2_init(void) {
  /* Initialize PWM submodule SM0 main configuration */
  PWM_Init(PWM2_PERIPHERAL, PWM2_SM0, &PWM2_SM0_config);
  /* Initialize fault input filter configuration */
  PWM_SetupFaultInputFilter(PWM2_PERIPHERAL, &PWM2_faultInputFilter_config);
  /* Initialize fault channel 0 fault Fault0 configuration */
  PWM_SetupFaults(PWM2_PERIPHERAL, PWM2_F0_FAULT0, &PWM2_Fault0_fault_config);
  /* Initialize fault channel 0 fault Fault1 configuration */
  PWM_SetupFaults(PWM2_PERIPHERAL, PWM2_F0_FAULT1, &PWM2_Fault1_fault_config);
  /* Initialize fault channel 0 fault Fault2 configuration */
  PWM_SetupFaults(PWM2_PERIPHERAL, PWM2_F0_FAULT2, &PWM2_Fault2_fault_config);
  /* Initialize fault channel 0 fault Fault3 configuration */
  PWM_SetupFaults(PWM2_PERIPHERAL, PWM2_F0_FAULT3, &PWM2_Fault3_fault_config);
  /* Initialize submodule SM0 channel A output disable mapping to the selected faults */
  PWM_SetupFaultDisableMap(PWM2_PERIPHERAL, PWM2_SM0, PWM2_SM0_A, kPWM_faultchannel_0, (kPWM_FaultDisable_0 | kPWM_FaultDisable_1 | kPWM_FaultDisable_2 | kPWM_FaultDisable_3));
  /* Initialize submodule SM0 channel B output disable mapping to the selected faults */
  PWM_SetupFaultDisableMap(PWM2_PERIPHERAL, PWM2_SM0, PWM2_SM0_B, kPWM_faultchannel_0, (kPWM_FaultDisable_0 | kPWM_FaultDisable_1 | kPWM_FaultDisable_2 | kPWM_FaultDisable_3));
  /* Initialize deadtime logic input for the channel A */
  PWM_SetupForceSignal(PWM2_PERIPHERAL, PWM2_SM0, PWM2_SM0_A, kPWM_UsePwm);
  /* Initialize deadtime logic input for the channel B */
  PWM_SetupForceSignal(PWM2_PERIPHERAL, PWM2_SM0, PWM2_SM0_B, kPWM_UsePwm);
  /* Setup PWM output setting for submodule SM0 */
  PWM_SetupPwm(PWM2_PERIPHERAL, PWM2_SM0, PWM2_SM0_pwm_function_config, 2U, kPWM_SignedCenterAligned, PWM2_SM0_COUNTER_FREQ_HZ, PWM2_SM0_SM_CLK_SOURCE_FREQ_HZ);
}

/***********************************************************************************************************************
 * GPIO2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO2'
- type: 'igpio_adapter'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'igpio_adapter_5c0a3d4fd4d107e507335b7419af3b4f'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIO2'
- config_sets:
  - fsl_adapter_gpio:
    - signalsFilter: 'default boot'
    - gpioSignalsParameters:
      - 0: []
      - 1: []
      - 2: []
      - 3: []
      - 4: []
      - 5: []
      - 6: []
      - 7: []
      - 8: []
      - 9: []
      - 10: []
      - 11: []
      - 12: []
      - 13: []
      - 14: []
      - 15: []
      - 16: []
      - 17: []
      - 18: []
      - 19: []
      - 20: []
    - gpioPinsOverView:
      - 0: []
      - 1: []
    - gpioPinsConfig: []
    - globalCfg: []
    - differentPeripheralsAdd: []
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
/* Get GPIO pin configuration */
hal_gpio_pin_config_t createAdapterGpioPinConfig(GPIO_Type *port, uint8_t pin, hal_gpio_direction_t direction, uint8_t level){
  hal_gpio_pin_config_t temp;
  /* Array of GPIO peripheral base address. */
  static GPIO_Type *const s_gpioBases[] = GPIO_BASE_PTRS;
  uint8_t portInd;
  /* Find the port index from base address mappings. */
  for (portInd = 0U; portInd < ARRAY_SIZE(s_gpioBases); portInd++)
  {    if (s_gpioBases[portInd] == port)
    {
      break;
    }
  }
  
  assert(portInd < ARRAY_SIZE(s_gpioBases));
  
  temp.direction = direction;
  temp.level = level;
  temp.port = portInd;
  temp.pin = pin;
  
  return temp;
};
GPIO_HANDLE_DEFINE(BOARD_ENC_Buttion_handle);
GPIO_HANDLE_DEFINE(BOARD_LCD_BLK_handle);

static void GPIO2_init(void) {
  /* GPIO adapter initialization */
  static hal_gpio_pin_config_t gpioPinConfig;
  hal_gpio_status_t status;
  (void)status; // suppress warning in the run configuration
  /* gpio_io, 05 signal initialization */
  gpioPinConfig = createAdapterGpioPinConfig(BOARD_ENC_Buttion_PORT, BOARD_ENC_Buttion_PIN, BOARD_ENC_Buttion_PIN_DIRECTION, BOARD_ENC_Buttion_PIN_LEVEL);
  status = HAL_GpioInit(BOARD_ENC_Buttion_handle, &gpioPinConfig);
  assert(status == kStatus_HAL_GpioSuccess);
  /* gpio_io, 31 signal initialization */
  gpioPinConfig = createAdapterGpioPinConfig(BOARD_LCD_BLK_PORT, BOARD_LCD_BLK_PIN, BOARD_LCD_BLK_PIN_DIRECTION, BOARD_LCD_BLK_PIN_LEVEL);
  status = HAL_GpioInit(BOARD_LCD_BLK_handle, &gpioPinConfig);
  assert(status == kStatus_HAL_GpioSuccess);
}

/***********************************************************************************************************************
 * ENC1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ENC1'
- type: 'enc'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'enc_48d106d37c3ae205724d82e4ee6d2b7b'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'ENC1'
- config_sets:
  - fsl_enc:
    - config:
      - clockConfig_t:
        - clockSource: 'BusInterfaceClock'
        - clockSourceFreq: 'ClocksTool_DefaultInit'
      - decoderWorkMode: 'kENC_DecoderWorkAsNormalMode'
      - enableReverseDirection: 'false'
      - posInitialization:
        - HOMETriggerMode: 'kENC_HOMETriggerDisabled'
        - INDEXTriggerMode: 'kENC_INDEXTriggerDisabled'
        - positionInitialValue: '0'
      - triggerAction: ''
      - positionMatchConfig:
        - positionMatchMode: 'kENC_POSMATCHOnPositionCounterEqualToComapreValue'
        - positionCompareValue: '0xFFFFFFFF'
      - modulusConfig:
        - revolutionCountCondition: 'kENC_RevolutionCountOnINDEXPulse'
        - enableModuloCountMode: 'false'
      - wdogConfig:
        - enableWatchdog: 'false'
      - filter_config_t:
        - enableFilter: 'false'
    - interruptsCfg:
      - interruptSources: ''
      - isInterruptEnabled: 'false'
      - interrupt:
        - IRQn: 'ENC1_IRQn'
        - enable_interrrupt: 'enabled'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
/* ENC configuration */
enc_config_t ENC1_config = {
  .decoderWorkMode = kENC_DecoderWorkAsNormalMode,
  .enableReverseDirection = false,
  .HOMETriggerMode = kENC_HOMETriggerDisabled,
  .INDEXTriggerMode = kENC_INDEXTriggerDisabled,
  .positionInitialValue = 0,
  .enableTRIGGERClearPositionCounter = false,
  .enableTRIGGERClearHoldPositionCounter = false,
  .positionMatchMode = kENC_POSMATCHOnPositionCounterEqualToComapreValue,
  .positionCompareValue = 4294967295,
  .revolutionCountCondition = kENC_RevolutionCountOnINDEXPulse,
  .enableModuloCountMode = false,
  .positionModulusValue = 0,
  .enableWatchdog = false,
  .watchdogTimeoutValue = 0,
  .filterPrescaler = kENC_FilterPrescalerDiv1,
  .filterCount = 0,
  .filterSamplePeriod = 0,
};

static void ENC1_init(void) {
  /* ENC initialization */
  ENC_Init(ENC1_PERIPHERAL, &ENC1_config);
}

/***********************************************************************************************************************
 * ADC1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC1'
- type: 'adc_12b1msps_sar'
- mode: 'ADC_GENERAL'
- custom_name_enabled: 'false'
- type_id: 'adc_12b1msps_sar_6a490e886349a7b2b07bed10ce7b299b'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'ADC1'
- config_sets:
  - fsl_adc:
    - clockConfig:
      - clockSource: 'kADC_ClockSourceAD'
      - clockSourceFreq: 'custom:10 MHz'
      - clockDriver: 'kADC_ClockDriver2'
      - samplePeriodMode: 'kADC_SamplePeriodShort2Clocks'
      - enableAsynchronousClockOutput: 'true'
    - conversionConfig:
      - resolution: 'kADC_Resolution12Bit'
      - hardwareAverageMode: 'kADC_HardwareAverageDisable'
      - enableHardwareTrigger: 'software'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
      - enableOverWrite: 'false'
      - enableDma: 'false'
    - resultingTime: []
    - resultCorrection:
      - doAutoCalibration: 'false'
      - offset: '0'
    - hardwareCompareConfiguration:
      - hardwareCompareMode: 'disabled'
      - value1: '0'
      - value2: '0'
    - enableInterrupt: 'false'
    - adc_interrupt:
      - IRQn: 'ADC1_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - adc_channels_config:
      - 0:
        - channelNumber: 'IN.0'
        - channelName: ''
        - channelGroup: '0'
        - initializeChannel: 'true'
        - enableInterruptOnConversionCompleted: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const adc_config_t ADC1_config = {
  .enableOverWrite = false,
  .enableContinuousConversion = false,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableLongSample = false,
  .enableAsynchronousClockOutput = true,
  .referenceVoltageSource = kADC_ReferenceVoltageSourceAlt0,
  .samplePeriodMode = kADC_SamplePeriodShort2Clocks,
  .clockSource = kADC_ClockSourceAD,
  .clockDriver = kADC_ClockDriver2,
  .resolution = kADC_Resolution12Bit
};
const adc_channel_config_t ADC1_channels_config[1] = {
  {
    .channelNumber = 0U,
    .enableInterruptOnConversionCompleted = false
  }
};
static void ADC1_init(void) {
  /* Initialize ADC1 peripheral. */
  ADC_Init(ADC1_PERIPHERAL, &ADC1_config);
  /* Initialize ADC1 channel 0. */
  ADC_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP, &ADC1_channels_config[0]);
}

/***********************************************************************************************************************
 * DebugConsole initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'DebugConsole'
- type: 'debug_console'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'debug_console_51864e4f3ac859dae7b603e07bc4ae33'
- functional_group: 'BOARD_InitPeripherals'
- config_sets:
  - fsl_debug_console:
    - config:
      - SDK_DEBUGCONSOLE: 'DEBUGCONSOLE_REDIRECT_TO_SDK'
      - SDK_DEBUGCONSOLE_UART: 'semihost'
      - DEBUG_CONSOLE_RX_ENABLE: 'true'
      - DEBUG_CONSOLE_PRINTF_MAX_LOG_LEN: '128'
      - DEBUG_CONSOLE_SCANF_MAX_LOG_LEN: '20'
      - DEBUG_CONSOLE_ENABLE_ECHO: 'false'
      - PRINTF_FLOAT_ENABLE: 'false'
      - SCANF_FLOAT_ENABLE: 'false'
      - PRINTF_ADVANCED_ENABLE: 'false'
      - SCANF_ADVANCED_ENABLE: 'false'
      - DEBUG_CONSOLE_TRANSFER_NON_BLOCKING: 'false'
      - DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN: '512'
      - DEBUG_CONSOLE_RECEIVE_BUFFER_LEN: '1024'
      - DEBUG_CONSOLE_TX_RELIABLE_ENABLE: 'true'
      - DEBUG_CONSOLE_DISABLE_RTOS_SYNCHRONIZATION: 'false'
    - peripheral_config:
      - serial_port_type: 'kSerialPort_Uart'
      - uart_config:
        - peripheralUART: 'LPUART1'
        - clockSource: 'genericUartClockSource'
        - clockSourceFreq: 'ClocksTool_DefaultInit'
        - baudRate_Bps: '115200'
    - debug_console_codegenerator: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void DebugConsole_init(void) {
  /* Debug console initialization */
  DbgConsole_Init(DEBUGCONSOLE_INSTANCE, DEBUGCONSOLE_BAUDRATE, DEBUGCONSOLE_TYPE, DEBUGCONSOLE_CLK_FREQ);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Global initialization */
  /* GPIO adapter pre-initialization */
  HAL_GpioPreInit();

  /* Initialize components */
  LPSPI4_init();
  LPSPI2_init();
  LPI2C1_init();
  PWM2_init();
  GPIO2_init();
  ENC1_init();
  ADC1_init();
  DebugConsole_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
