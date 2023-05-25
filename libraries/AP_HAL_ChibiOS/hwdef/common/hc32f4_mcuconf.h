/*
    ChibiOS - Copyright (C) 2022 Radiolink
    Driver by Radiolink Wu, March 2022
*/
/*
  this provides the default mcuconf.h for each board. Override values in hwdef.dat
 */
#pragma once

/*
 * HAL driver system settings.
 */
#define HC32_PLLSRC                         HC32_PLLSRC_XTAL
#define HC32_PLLH_M_VALUE                   1
#define HC32_PLLH_N_VALUE                   50
#define HC32_PLLH_P_VALUE                   5
#define HC32_PLLH_Q_VALUE                   5
#define HC32_PLLH_R_VALUE                   5
#define HC32_HCLK                           240000000UL
#define HC32_PCLK1							120000000UL

/*
 * ST driver system settings.
 */
#define HC32_ST_USE_TIMER                   1

#ifndef HC32_GPT_USE_TIM6_2
#define HC32_GPT_USE_TIM6_2                 FALSE
#endif

#ifndef HC32_GPT_USE_TIM6_3
#define HC32_GPT_USE_TIM6_3                 FALSE
#endif

#ifndef HC32_GPT_USE_TIM6_4
#define HC32_GPT_USE_TIM6_4                 FALSE
#endif

#ifndef HC32_GPT_USE_TIM6_5
#define HC32_GPT_USE_TIM6_5                 FALSE
#endif

#ifndef HC32_GPT_USE_TIM6_6
#define HC32_GPT_USE_TIM6_6                 FALSE
#endif

#ifndef HC32_GPT_USE_TIM6_7
#define HC32_GPT_USE_TIM6_7                 FALSE
#endif

#ifndef HC32_GPT_USE_TIM6_8
#define HC32_GPT_USE_TIM6_8                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_1
#define HC32_GPT_USE_TIMA_1                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_2
#define HC32_GPT_USE_TIMA_2                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_3
#define HC32_GPT_USE_TIMA_3                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_4
#define HC32_GPT_USE_TIMA_4                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_5
#define HC32_GPT_USE_TIMA_5                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_6
#define HC32_GPT_USE_TIMA_6                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_7
#define HC32_GPT_USE_TIMA_7                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_8
#define HC32_GPT_USE_TIMA_8                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_9
#define HC32_GPT_USE_TIMA_9                 FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_10
#define HC32_GPT_USE_TIMA_10                FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_11
#define HC32_GPT_USE_TIMA_11                FALSE
#endif

#ifndef HC32_GPT_USE_TIMA_12
#define HC32_GPT_USE_TIMA_12                FALSE
#endif

#ifndef HC32_ICU_USE_TIM6_2
#define HC32_ICU_USE_TIM6_2                 FALSE
#endif

#ifndef HC32_ICU_USE_TIM6_3
#define HC32_ICU_USE_TIM6_3                 FALSE
#endif

#ifndef HC32_ICU_USE_TIM6_4
#define HC32_ICU_USE_TIM6_4                 FALSE
#endif

#ifndef HC32_ICU_USE_TIM6_5
#define HC32_ICU_USE_TIM6_5                 FALSE
#endif

#ifndef HC32_ICU_USE_TIM6_6
#define HC32_ICU_USE_TIM6_6                 FALSE
#endif

#ifndef HC32_ICU_USE_TIM6_7
#define HC32_ICU_USE_TIM6_7                 FALSE
#endif

#ifndef HC32_ICU_USE_TIM6_8
#define HC32_ICU_USE_TIM6_8                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_1
#define HC32_ICU_USE_TIMA_1                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_2
#define HC32_ICU_USE_TIMA_2                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_3
#define HC32_ICU_USE_TIMA_3                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_4
#define HC32_ICU_USE_TIMA_4                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_5
#define HC32_ICU_USE_TIMA_5                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_6
#define HC32_ICU_USE_TIMA_6                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_7
#define HC32_ICU_USE_TIMA_7                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_8
#define HC32_ICU_USE_TIMA_8                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_9
#define HC32_ICU_USE_TIMA_9                 FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_10
#define HC32_ICU_USE_TIMA_10                FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_11
#define HC32_ICU_USE_TIMA_11                FALSE
#endif

#ifndef HC32_ICU_USE_TIMA_12
#define HC32_ICU_USE_TIMA_12                FALSE
#endif

#ifndef HC32_PWM_USE_TIM6_2
#define HC32_PWM_USE_TIM6_2                 FALSE
#endif

#ifndef HC32_PWM_USE_TIM6_3
#define HC32_PWM_USE_TIM6_3                 FALSE
#endif

#ifndef HC32_PWM_USE_TIM6_4
#define HC32_PWM_USE_TIM6_4                 FALSE
#endif

#ifndef HC32_PWM_USE_TIM6_5
#define HC32_PWM_USE_TIM6_5                 FALSE
#endif

#ifndef HC32_PWM_USE_TIM6_6
#define HC32_PWM_USE_TIM6_6                 FALSE
#endif

#ifndef HC32_PWM_USE_TIM6_7
#define HC32_PWM_USE_TIM6_7                 FALSE
#endif

#ifndef HC32_PWM_USE_TIM6_8
#define HC32_PWM_USE_TIM6_8                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_1
#define HC32_PWM_USE_TIMA_1                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_2
#define HC32_PWM_USE_TIMA_2                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_3
#define HC32_PWM_USE_TIMA_3                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_4
#define HC32_PWM_USE_TIMA_4                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_5
#define HC32_PWM_USE_TIMA_5                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_6
#define HC32_PWM_USE_TIMA_6                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_7
#define HC32_PWM_USE_TIMA_7                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_8
#define HC32_PWM_USE_TIMA_8                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_9
#define HC32_PWM_USE_TIMA_9                 FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_10
#define HC32_PWM_USE_TIMA_10                FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_11
#define HC32_PWM_USE_TIMA_11                FALSE
#endif

#ifndef HC32_PWM_USE_TIMA_12
#define HC32_PWM_USE_TIMA_12                FALSE
#endif

/*
 * SPI driver system settings.
 */
#ifndef HC32_SPI_USE_SPI1
#define HC32_SPI_USE_SPI1                   FALSE
#endif

#ifndef HC32_SPI_USE_SPI2
#define HC32_SPI_USE_SPI2                   FALSE
#endif

#ifndef HC32_SPI_USE_SPI3
#define HC32_SPI_USE_SPI3                   FALSE
#endif

#ifndef HC32_SPI_USE_SPI4
#define HC32_SPI_USE_SPI4                   FALSE
#endif

#ifndef HC32_SPI_USE_SPI5
#define HC32_SPI_USE_SPI5                   FALSE
#endif

#ifndef HC32_SPI_USE_SPI6
#define HC32_SPI_USE_SPI6                   FALSE
#endif

#ifndef HC32_SPI_SPI1_RX_DMA_CHANNEL
#define HC32_SPI_SPI1_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_SPI_SPI1_TX_DMA_CHANNEL
#define HC32_SPI_SPI1_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 4)
#endif

#ifndef HC32_SPI_SPI2_RX_DMA_CHANNEL
#define HC32_SPI_SPI2_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_SPI_SPI2_TX_DMA_CHANNEL
#define HC32_SPI_SPI2_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 4)
#endif

#ifndef HC32_SPI_SPI3_RX_DMA_CHANNEL
#define HC32_SPI_SPI3_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_SPI_SPI3_TX_DMA_CHANNEL
#define HC32_SPI_SPI3_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 4)
#endif

#ifndef HC32_SPI_SPI4_RX_DMA_CHANNEL
#define HC32_SPI_SPI4_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_SPI_SPI4_TX_DMA_CHANNEL
#define HC32_SPI_SPI4_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 4)
#endif

#ifndef HC32_SPI_SPI5_RX_DMA_CHANNEL
#define HC32_SPI_SPI5_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_SPI_SPI5_TX_DMA_CHANNEL
#define HC32_SPI_SPI5_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 4)
#endif

#ifndef HC32_SPI_SPI6_RX_DMA_CHANNEL
#define HC32_SPI_SPI6_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_SPI_SPI6_TX_DMA_CHANNEL
#define HC32_SPI_SPI6_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 0)
#endif

#define HC32_SPI_SPI1_IRQ_PRIORITY          15
#define HC32_SPI_SPI2_IRQ_PRIORITY          15
#define HC32_SPI_SPI3_IRQ_PRIORITY          15
#define HC32_SPI_SPI4_IRQ_PRIORITY          15
#define HC32_SPI_SPI5_IRQ_PRIORITY          15
#define HC32_SPI_SPI6_IRQ_PRIORITY          15
#define HC32_SPI_DMA_ERROR_HOOK(spip)       osalSysHalt("DMA failure")

/*
 * USB driver system settings.
 */
#ifndef HC32_USB_USE_OTG1
#define HC32_USB_USE_OTG1                   TRUE
#endif

#define HC32_USB_OTG1_IRQ_PRIORITY          15
#define HC32_USB_OTG1_RX_FIFO_SIZE          512
#define HC32_USB_HOST_WAKEUP_DURATION       5

#define HC32_ICU2_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU3_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU4_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU5_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU6_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU7_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU8_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU9_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_ICU10_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU11_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU12_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU13_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU14_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU15_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU16_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU17_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU18_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU19_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_ICU20_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15

#define HC32_PWM2_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM3_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM4_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM5_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM6_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM7_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM8_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM9_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15
#define HC32_PWM10_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM11_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM12_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM13_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM14_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM15_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM16_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM17_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM18_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM19_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15
#define HC32_PWM20_IRQ_PRIORITY             DDL_IRQ_PRIORITY_15

/*
 * USART drive system setting
 */
#define HC32_UART_USART1_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART2_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART3_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART4_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART5_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART6_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART7_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART8_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART9_IRQ_PRIORITY               DDL_IRQ_PRIORITY_15
#define HC32_UART_USART10_IRQ_PRIORITY              DDL_IRQ_PRIORITY_15

#ifndef HC32_UART_USART1_RX_DMA_CHANNEL
#define HC32_UART_USART1_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 0)
#endif

#ifndef HC32_UART_USART1_TX_DMA_CHANNEL
#define HC32_UART_USART1_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_UART_USART2_RX_DMA_CHANNEL
#define HC32_UART_USART2_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 2)
#endif

#ifndef HC32_UART_USART2_TX_DMA_CHANNEL
#define HC32_UART_USART2_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 3)
#endif

#ifndef HC32_UART_USART3_RX_DMA_CHANNEL
#define HC32_UART_USART3_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 4)
#endif

#ifndef HC32_UART_USART3_TX_DMA_CHANNEL
#define HC32_UART_USART3_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 5)
#endif

#ifndef HC32_UART_USART4_RX_DMA_CHANNEL
#define HC32_UART_USART4_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 6)
#endif

#ifndef HC32_UART_USART4_TX_DMA_CHANNEL
#define HC32_UART_USART4_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(1, 7)
#endif

#ifndef HC32_UART_USART5_RX_DMA_CHANNEL
#define HC32_UART_USART5_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 0)
#endif

#ifndef HC32_UART_USART5_TX_DMA_CHANNEL
#define HC32_UART_USART5_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 1)
#endif

#ifndef HC32_UART_USART6_RX_DMA_CHANNEL
#define HC32_UART_USART6_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 2)
#endif

#ifndef HC32_UART_USART6_TX_DMA_CHANNEL
#define HC32_UART_USART6_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 3)
#endif

#ifndef HC32_UART_USART7_RX_DMA_CHANNEL
#define HC32_UART_USART7_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 4)
#endif

#ifndef HC32_UART_USART7_TX_DMA_CHANNEL
#define HC32_UART_USART7_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 5)
#endif

#ifndef HC32_UART_USART8_RX_DMA_CHANNEL
#define HC32_UART_USART8_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 6)
#endif

#ifndef HC32_UART_USART8_TX_DMA_CHANNEL
#define HC32_UART_USART8_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 7)
#endif

#ifndef HC32_UART_USART9_RX_DMA_CHANNEL
#define HC32_UART_USART9_RX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 0)
#endif

#ifndef HC32_UART_USART9_TX_DMA_CHANNEL
#define HC32_UART_USART9_TX_DMA_CHANNEL     HC32_DMA_CHANNEL_ID(2, 1)
#endif

#ifndef HC32_UART_USART10_RX_DMA_CHANNEL
#define HC32_UART_USART10_RX_DMA_CHANNEL    HC32_DMA_CHANNEL_ID(2, 2)
#endif

#ifndef HC32_UART_USART10_TX_DMA_CHANNEL
#define HC32_UART_USART10_TX_DMA_CHANNEL    HC32_DMA_CHANNEL_ID(2, 3)
#endif

#ifndef HC32_UART_USE_USART1
#define HC32_UART_USE_USART1                          FALSE
#endif

#ifndef HC32_UART_USE_USART2
#define HC32_UART_USE_USART2                          FALSE
#endif

#ifndef HC32_UART_USE_USART3
#define HC32_UART_USE_USART3                          FALSE
#endif

#ifndef HC32_UART_USE_USART4
#define HC32_UART_USE_USART4                          FALSE
#endif

#ifndef HC32_UART_USE_USART5
#define HC32_UART_USE_USART5                          FALSE
#endif

#ifndef HC32_UART_USE_USART6
#define HC32_UART_USE_USART6                          FALSE
#endif

#ifndef HC32_UART_USE_USART7
#define HC32_UART_USE_USART7                          FALSE
#endif

#ifndef HC32_UART_USE_USART8
#define HC32_UART_USE_USART8                          FALSE
#endif

#ifndef HC32_UART_USE_USART9
#define HC32_UART_USE_USART9                          FALSE
#endif

#ifndef HC32_UART_USE_USART10
#define HC32_UART_USE_USART10                         FALSE
#endif

#ifndef HC32_SERIAL_USE_USART1
#define HC32_SERIAL_USE_USART1                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART2
#define HC32_SERIAL_USE_USART2                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART3
#define HC32_SERIAL_USE_USART3                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART4
#define HC32_SERIAL_USE_USART4                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART5
#define HC32_SERIAL_USE_USART5                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART6
#define HC32_SERIAL_USE_USART6                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART7
#define HC32_SERIAL_USE_USART7                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART8
#define HC32_SERIAL_USE_USART8                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART9
#define HC32_SERIAL_USE_USART9                        FALSE
#endif

#ifndef HC32_SERIAL_USE_USART10
#define HC32_SERIAL_USE_USART10                       FALSE
#endif
/*
 * I2C Drive system setting
 */
#define HC32_I2C_BUSY_TIMEOUT                   25

#ifndef HC32_I2C_USE_I2C1
#define HC32_I2C_USE_I2C1                       FALSE
#endif

#ifndef HC32_I2C_USE_I2C2
#define HC32_I2C_USE_I2C2                       FALSE
#endif

#ifndef HC32_I2C_USE_I2C3
#define HC32_I2C_USE_I2C3                       FALSE
#endif

#ifndef HC32_I2C_USE_I2C4
#define HC32_I2C_USE_I2C4                       FALSE
#endif

#ifndef HC32_I2C_USE_I2C5
#define HC32_I2C_USE_I2C5                       FALSE
#endif

#ifndef HC32_I2C_USE_I2C6
#define HC32_I2C_USE_I2C6                       FALSE
#endif

#ifndef HC32_I2C_I2C1_RX_DMA_CHANNEL
#define HC32_I2C_I2C1_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 0)
#endif

#ifndef HC32_I2C_I2C1_TX_DMA_CHANNEL
#define HC32_I2C_I2C1_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 1)
#endif

#ifndef HC32_I2C_I2C2_RX_DMA_CHANNEL
#define HC32_I2C_I2C2_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 2)
#endif

#ifndef HC32_I2C_I2C2_TX_DMA_CHANNEL
#define HC32_I2C_I2C2_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 3)
#endif

#ifndef HC32_I2C_I2C3_RX_DMA_CHANNEL
#define HC32_I2C_I2C3_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 4)
#endif

#ifndef HC32_I2C_I2C3_TX_DMA_CHANNEL
#define HC32_I2C_I2C3_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 5)
#endif

#ifndef HC32_I2C_I2C4_RX_DMA_CHANNEL
#define HC32_I2C_I2C4_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 6)
#endif

#ifndef HC32_I2C_I2C4_TX_DMA_CHANNEL
#define HC32_I2C_I2C4_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(1, 7)
#endif

#ifndef HC32_I2C_I2C5_RX_DMA_CHANNEL
#define HC32_I2C_I2C5_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(2, 1)
#endif

#ifndef HC32_I2C_I2C5_TX_DMA_CHANNEL
#define HC32_I2C_I2C5_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(2, 2)
#endif

#ifndef HC32_I2C_I2C6_RX_DMA_CHANNEL
#define HC32_I2C_I2C6_RX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(2, 3)
#endif

#ifndef HC32_I2C_I2C6_TX_DMA_CHANNEL
#define HC32_I2C_I2C6_TX_DMA_CHANNEL        HC32_DMA_CHANNEL_ID(2, 4)
#endif

#define HC32_I2C1_IRQ_PRIORITY                  DDL_IRQ_PRIORITY_15
#define HC32_I2C2_IRQ_PRIORITY                  DDL_IRQ_PRIORITY_15
#define HC32_I2C3_IRQ_PRIORITY                  DDL_IRQ_PRIORITY_15
#define HC32_I2C4_IRQ_PRIORITY                  DDL_IRQ_PRIORITY_15
#define HC32_I2C5_IRQ_PRIORITY                  DDL_IRQ_PRIORITY_15
#define HC32_I2C6_IRQ_PRIORITY                  DDL_IRQ_PRIORITY_15

#define HC32_I2C_DMA_ERROR_HOOK(i2cp)           osalSysHalt("DMA failure")

/*
 * ADC driver system settings.
 */
#ifndef HC32_ADC_USE_ADC1
#define HC32_ADC_USE_ADC1                  TRUE
#endif

#ifndef HC32_ADC_ADC1_DMA_CHANNEL
#define HC32_ADC_ADC1_DMA_CHANNEL          HC32_DMA_CHANNEL_ID(2, 8)
#endif

#define HC32_ADC_ADC1_DMA_IRQ_PRIORITY     3

/*
 * SDC driver system settings.
 */
#define HC32_SDC_SDIO_DMA_PRIORITY         DDL_IRQ_PRIORITY_03
#define HC32_SDC_SDIO_IRQ_PRIORITY         DDL_IRQ_PRIORITY_09
#define HC32_SDC_WRITE_TIMEOUT_MS          1000
#define HC32_SDC_READ_TIMEOUT_MS           1000
#define HC32_SDC_CLOCK_ACTIVATION_DELAY    10
#define HC32_SDC_SDIO_UNALIGNED_SUPPORT    FALSE
#define SDIO_USE_BOUNCEBUFFER			   TRUE
#ifndef HC32_SDC_SDIO_DMA_CHANNEL
#define HC32_SDC_SDIO_DMA_CHANNEL          HC32_DMA_CHANNEL_ID(2, 3)
#endif
