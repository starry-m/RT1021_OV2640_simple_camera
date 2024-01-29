/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* GPIO_AD_B0_05 (number 106), JTAG_nTRST/J16[3]/USER_LED/J17[5] */
/* Routed pin properties */
#define BOARD_USER_LED_PERIPHERAL                                          GPIO1   /*!< Peripheral name */
#define BOARD_USER_LED_SIGNAL                                            gpio_io   /*!< Signal name */
#define BOARD_USER_LED_CHANNEL                                                5U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_USER_LED_GPIO                                                GPIO1   /*!< GPIO peripheral base pointer */
#define BOARD_USER_LED_GPIO_PIN                                               5U   /*!< GPIO pin number */
#define BOARD_USER_LED_GPIO_PIN_MASK                                  (1U << 5U)   /*!< GPIO pin mask */
#define BOARD_USER_LED_PORT                                                GPIO1   /*!< PORT peripheral base pointer */
#define BOARD_USER_LED_PIN                                                    5U   /*!< PORT pin number */
#define BOARD_USER_LED_PIN_MASK                                       (1U << 5U)   /*!< PORT pin mask */

/* GPIO_AD_B0_11 (number 97), ENET_CRS_DV/U11[18]/J19[3] */
/* Routed pin properties */
#define BOARD_ENET_CRS_DV_PERIPHERAL                                         ARM   /*!< Peripheral name */
#define BOARD_ENET_CRS_DV_SIGNAL                                   arm_trace_swo   /*!< Signal name */

/* PMIC_STBY_REQ (number 54), SD_PWREN */
/* Routed pin properties */
#define BOARD_U_LED_PERIPHERAL                                             GPIO5   /*!< Peripheral name */
#define BOARD_U_LED_SIGNAL                                               gpio_io   /*!< Signal name */
#define BOARD_U_LED_CHANNEL                                                   2U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_U_LED_GPIO                                                   GPIO5   /*!< GPIO peripheral base pointer */
#define BOARD_U_LED_GPIO_PIN                                                  2U   /*!< GPIO pin number */
#define BOARD_U_LED_GPIO_PIN_MASK                                     (1U << 2U)   /*!< GPIO pin mask */
#define BOARD_U_LED_PORT                                                   GPIO5   /*!< PORT peripheral base pointer */
#define BOARD_U_LED_PIN                                                       2U   /*!< PORT pin number */
#define BOARD_U_LED_PIN_MASK                                          (1U << 2U)   /*!< PORT pin mask */

/* GPIO_AD_B0_06 (number 105), UART1_TXD/J17[6] */
/* Routed pin properties */
#define BOARD_UART1_TXD_PERIPHERAL                                       LPUART1   /*!< Peripheral name */
#define BOARD_UART1_TXD_SIGNAL                                                TX   /*!< Signal name */

/* GPIO_AD_B0_07 (number 101), UART1_RXD/J17[4] */
/* Routed pin properties */
#define BOARD_UART1_RXD_PERIPHERAL                                       LPUART1   /*!< Peripheral name */
#define BOARD_UART1_RXD_SIGNAL                                                RX   /*!< Signal name */

/* GPIO_EMC_32 (number 124), SEMC_D8/U14[42] */
/* Routed pin properties */
#define BOARD_SEMC_D8_PERIPHERAL                                          LPSPI4   /*!< Peripheral name */
#define BOARD_SEMC_D8_SIGNAL                                                 SCK   /*!< Signal name */

/* GPIO_EMC_33 (number 123), SEMC_D9/U14[44] */
/* Routed pin properties */
#define BOARD_LCD_CS_PERIPHERAL                                           LPSPI4   /*!< Peripheral name */
#define BOARD_LCD_CS_SIGNAL                                                 PCS0   /*!< Signal name */

/* GPIO_EMC_34 (number 122), SEMC_D10/U14[45] */
/* Routed pin properties */
#define BOARD_SEMC_D10_PERIPHERAL                                         LPSPI4   /*!< Peripheral name */
#define BOARD_SEMC_D10_SIGNAL                                                SDO   /*!< Signal name */

/* GPIO_EMC_35 (number 121), SEMC_D11/U14[47] */
/* Routed pin properties */
#define BOARD_SEMC_D11_PERIPHERAL                                         LPSPI4   /*!< Peripheral name */
#define BOARD_SEMC_D11_SIGNAL                                                SDI   /*!< Signal name */

/* GPIO_EMC_36 (number 120), SEMC_D12/U14[48] */
/* Routed pin properties */
#define BOARD_LCD_DC_PERIPHERAL                                            GPIO3   /*!< Peripheral name */
#define BOARD_LCD_DC_SIGNAL                                              gpio_io   /*!< Signal name */
#define BOARD_LCD_DC_CHANNEL                                                  4U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_LCD_DC_GPIO                                                  GPIO3   /*!< GPIO peripheral base pointer */
#define BOARD_LCD_DC_GPIO_PIN                                                 4U   /*!< GPIO pin number */
#define BOARD_LCD_DC_GPIO_PIN_MASK                                    (1U << 4U)   /*!< GPIO pin mask */
#define BOARD_LCD_DC_PORT                                                  GPIO3   /*!< PORT peripheral base pointer */
#define BOARD_LCD_DC_PIN                                                      4U   /*!< PORT pin number */
#define BOARD_LCD_DC_PIN_MASK                                         (1U << 4U)   /*!< PORT pin mask */

/* GPIO_EMC_37 (number 119), SEMC_D13/U14[50] */
/* Routed pin properties */
#define BOARD_SEMC_D13_PERIPHERAL                                         LPSPI4   /*!< Peripheral name */
#define BOARD_SEMC_D13_SIGNAL                                               PCS2   /*!< Signal name */

/* GPIO_EMC_31 (number 125), SEMC_DM1/U14[39] */
/* Routed pin properties */
#define BOARD_LCD_BLK_PERIPHERAL                                           GPIO2   /*!< Peripheral name */
#define BOARD_LCD_BLK_SIGNAL                                             gpio_io   /*!< Signal name */
#define BOARD_LCD_BLK_CHANNEL                                                31U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_LCD_BLK_GPIO                                                 GPIO2   /*!< GPIO peripheral base pointer */
#define BOARD_LCD_BLK_GPIO_PIN                                               31U   /*!< GPIO pin number */
#define BOARD_LCD_BLK_GPIO_PIN_MASK                                  (1U << 31U)   /*!< GPIO pin mask */
#define BOARD_LCD_BLK_PORT                                                 GPIO2   /*!< PORT peripheral base pointer */
#define BOARD_LCD_BLK_PIN                                                    31U   /*!< PORT pin number */
#define BOARD_LCD_BLK_PIN_MASK                                       (1U << 31U)   /*!< PORT pin mask */

/* GPIO_EMC_39 (number 117), SEMC_D15/U14[53] */
/* Routed pin properties */
#define BOARD_LCD_RES_PERIPHERAL                                           GPIO3   /*!< Peripheral name */
#define BOARD_LCD_RES_SIGNAL                                             gpio_io   /*!< Signal name */
#define BOARD_LCD_RES_CHANNEL                                                 7U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_LCD_RES_GPIO                                                 GPIO3   /*!< GPIO peripheral base pointer */
#define BOARD_LCD_RES_GPIO_PIN                                                7U   /*!< GPIO pin number */
#define BOARD_LCD_RES_GPIO_PIN_MASK                                   (1U << 7U)   /*!< GPIO pin mask */
#define BOARD_LCD_RES_PORT                                                 GPIO3   /*!< PORT peripheral base pointer */
#define BOARD_LCD_RES_PIN                                                     7U   /*!< PORT pin number */
#define BOARD_LCD_RES_PIN_MASK                                        (1U << 7U)   /*!< PORT pin mask */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
