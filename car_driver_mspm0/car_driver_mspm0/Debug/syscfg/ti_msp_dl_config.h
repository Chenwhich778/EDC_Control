/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_G0 */
#define PWM_G0_INST                                                        TIMG0
#define PWM_G0_INST_IRQHandler                                  TIMG0_IRQHandler
#define PWM_G0_INST_INT_IRQN                                    (TIMG0_INT_IRQn)
#define PWM_G0_INST_CLK_FREQ                                              400000
/* GPIO defines for channel 0 */
#define GPIO_PWM_G0_C0_PORT                                                GPIOA
#define GPIO_PWM_G0_C0_PIN                                        DL_GPIO_PIN_23
#define GPIO_PWM_G0_C0_IOMUX                                     (IOMUX_PINCM53)
#define GPIO_PWM_G0_C0_IOMUX_FUNC                    IOMUX_PINCM53_PF_TIMG0_CCP0
#define GPIO_PWM_G0_C0_IDX                                   DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_G0_C1_PORT                                                GPIOA
#define GPIO_PWM_G0_C1_PIN                                        DL_GPIO_PIN_13
#define GPIO_PWM_G0_C1_IOMUX                                     (IOMUX_PINCM35)
#define GPIO_PWM_G0_C1_IOMUX_FUNC                    IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_G0_C1_IDX                                   DL_TIMER_CC_1_INDEX

/* Defines for PWM_MOTOR_1_2 */
#define PWM_MOTOR_1_2_INST                                                 TIMG7
#define PWM_MOTOR_1_2_INST_IRQHandler                           TIMG7_IRQHandler
#define PWM_MOTOR_1_2_INST_INT_IRQN                             (TIMG7_INT_IRQn)
#define PWM_MOTOR_1_2_INST_CLK_FREQ                                     32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_1_2_C0_PORT                                         GPIOA
#define GPIO_PWM_MOTOR_1_2_C0_PIN                                 DL_GPIO_PIN_26
#define GPIO_PWM_MOTOR_1_2_C0_IOMUX                              (IOMUX_PINCM59)
#define GPIO_PWM_MOTOR_1_2_C0_IOMUX_FUNC             IOMUX_PINCM59_PF_TIMG7_CCP0
#define GPIO_PWM_MOTOR_1_2_C0_IDX                            DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_1_2_C1_PORT                                         GPIOA
#define GPIO_PWM_MOTOR_1_2_C1_PIN                                  DL_GPIO_PIN_4
#define GPIO_PWM_MOTOR_1_2_C1_IOMUX                               (IOMUX_PINCM9)
#define GPIO_PWM_MOTOR_1_2_C1_IOMUX_FUNC              IOMUX_PINCM9_PF_TIMG7_CCP1
#define GPIO_PWM_MOTOR_1_2_C1_IDX                            DL_TIMER_CC_1_INDEX

/* Defines for PWM_MOTOR_3_4 */
#define PWM_MOTOR_3_4_INST                                                 TIMG6
#define PWM_MOTOR_3_4_INST_IRQHandler                           TIMG6_IRQHandler
#define PWM_MOTOR_3_4_INST_INT_IRQN                             (TIMG6_INT_IRQn)
#define PWM_MOTOR_3_4_INST_CLK_FREQ                                     32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_3_4_C0_PORT                                         GPIOA
#define GPIO_PWM_MOTOR_3_4_C0_PIN                                 DL_GPIO_PIN_29
#define GPIO_PWM_MOTOR_3_4_C0_IOMUX                               (IOMUX_PINCM4)
#define GPIO_PWM_MOTOR_3_4_C0_IOMUX_FUNC              IOMUX_PINCM4_PF_TIMG6_CCP0
#define GPIO_PWM_MOTOR_3_4_C0_IDX                            DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_3_4_C1_PORT                                         GPIOA
#define GPIO_PWM_MOTOR_3_4_C1_PIN                                 DL_GPIO_PIN_30
#define GPIO_PWM_MOTOR_3_4_C1_IOMUX                               (IOMUX_PINCM5)
#define GPIO_PWM_MOTOR_3_4_C1_IOMUX_FUNC              IOMUX_PINCM5_PF_TIMG6_CCP1
#define GPIO_PWM_MOTOR_3_4_C1_IDX                            DL_TIMER_CC_1_INDEX



/* Defines for TIMER_M4 */
#define TIMER_M4_INST                                                    (TIMA1)
#define TIMER_M4_INST_IRQHandler                                TIMA1_IRQHandler
#define TIMER_M4_INST_INT_IRQN                                  (TIMA1_INT_IRQn)
/* GPIO defines for channel 1 */
#define GPIO_TIMER_M4_C1_PORT                                              GPIOB
#define GPIO_TIMER_M4_C1_PIN                                      DL_GPIO_PIN_18
#define GPIO_TIMER_M4_C1_IOMUX                                   (IOMUX_PINCM44)
#define GPIO_TIMER_M4_C1_IOMUX_FUNC                  IOMUX_PINCM44_PF_TIMA1_CCP1

/* Defines for TIMER_M3 */
#define TIMER_M3_INST                                                    (TIMG8)
#define TIMER_M3_INST_IRQHandler                                TIMG8_IRQHandler
#define TIMER_M3_INST_INT_IRQN                                  (TIMG8_INT_IRQn)
/* GPIO defines for channel 0 */
#define GPIO_TIMER_M3_C0_PORT                                              GPIOB
#define GPIO_TIMER_M3_C0_PIN                                      DL_GPIO_PIN_21
#define GPIO_TIMER_M3_C0_IOMUX                                   (IOMUX_PINCM49)
#define GPIO_TIMER_M3_C0_IOMUX_FUNC                  IOMUX_PINCM49_PF_TIMG8_CCP0




/* Defines for TIMER_A0_100us */
#define TIMER_A0_100us_INST                                              (TIMA0)
#define TIMER_A0_100us_INST_IRQHandler                          TIMA0_IRQHandler
#define TIMER_A0_100us_INST_INT_IRQN                            (TIMA0_INT_IRQn)
#define TIMER_A0_100us_INST_LOAD_VALUE                                    (159U)
/* Defines for TIMER_G12_1ms */
#define TIMER_G12_1ms_INST                                              (TIMG12)
#define TIMER_G12_1ms_INST_IRQHandler                          TIMG12_IRQHandler
#define TIMER_G12_1ms_INST_INT_IRQN                            (TIMG12_INT_IRQn)
#define TIMER_G12_1ms_INST_LOAD_VALUE                                    (3199U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_28
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM3)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_31
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM6)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM6_PF_I2C0_SCL


/* Defines for USB_UART_0 */
#define USB_UART_0_INST                                                    UART0
#define USB_UART_0_INST_FREQUENCY                                       32000000
#define USB_UART_0_INST_IRQHandler                              UART0_IRQHandler
#define USB_UART_0_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_USB_UART_0_RX_PORT                                            GPIOA
#define GPIO_USB_UART_0_TX_PORT                                            GPIOA
#define GPIO_USB_UART_0_RX_PIN                                     DL_GPIO_PIN_1
#define GPIO_USB_UART_0_TX_PIN                                     DL_GPIO_PIN_0
#define GPIO_USB_UART_0_IOMUX_RX                                  (IOMUX_PINCM2)
#define GPIO_USB_UART_0_IOMUX_TX                                  (IOMUX_PINCM1)
#define GPIO_USB_UART_0_IOMUX_RX_FUNC                   IOMUX_PINCM2_PF_UART0_RX
#define GPIO_USB_UART_0_IOMUX_TX_FUNC                   IOMUX_PINCM1_PF_UART0_TX
#define USB_UART_0_BAUD_RATE                                            (115200)
#define USB_UART_0_IBRD_32_MHZ_115200_BAUD                                  (17)
#define USB_UART_0_FBRD_32_MHZ_115200_BAUD                                  (23)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_17
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM39)
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM39_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_1_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for UART_0 */
#define UART_0_INST                                                        UART2
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART2_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_22
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_21
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM47)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM46)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM46_PF_UART2_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_0_FBRD_32_MHZ_9600_BAUD                                        (21)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI0
#define SPI_0_INST_IRQHandler                                   SPI0_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI0_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOA
#define GPIO_SPI_0_PICO_PIN                                      DL_GPIO_PIN_14
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM36)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_SPI_0_POCI_PORT                                              GPIOA
#define GPIO_SPI_0_POCI_PIN                                      DL_GPIO_PIN_10
#define GPIO_SPI_0_IOMUX_POCI                                   (IOMUX_PINCM21)
#define GPIO_SPI_0_IOMUX_POCI_FUNC                   IOMUX_PINCM21_PF_SPI0_POCI
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOA
#define GPIO_SPI_0_SCLK_PIN                                      DL_GPIO_PIN_11
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM22)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM22_PF_SPI0_SCLK



/* Defines for VCC_ADC */
#define VCC_ADC_INST                                                        ADC1
#define VCC_ADC_INST_IRQHandler                                  ADC1_IRQHandler
#define VCC_ADC_INST_INT_IRQN                                    (ADC1_INT_IRQn)
#define VCC_ADC_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define VCC_ADC_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define VCC_ADC_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_VCC_ADC_C0_PORT                                               GPIOA
#define GPIO_VCC_ADC_C0_PIN                                       DL_GPIO_PIN_15



/* Port definition for Pin Group KEY1 */
#define KEY1_PORT                                                        (GPIOB)

/* Defines for PB5: GPIOB.5 with pinCMx 18 on package pin 53 */
// groups represented: ["MOTOR1","MOTOR2","MOTOR4","KEY1"]
// pins affected: ["ENCA1","ENCA2","ENCA4","PB5"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define KEY1_PB5_IIDX                                        (DL_GPIO_IIDX_DIO5)
#define KEY1_PB5_PIN                                             (DL_GPIO_PIN_5)
#define KEY1_PB5_IOMUX                                           (IOMUX_PINCM18)
/* Port definition for Pin Group SPI0 */
#define SPI0_PORT                                                        (GPIOA)

/* Defines for CS0: GPIOA.8 with pinCMx 19 on package pin 54 */
#define SPI0_CS0_PIN                                             (DL_GPIO_PIN_8)
#define SPI0_CS0_IOMUX                                           (IOMUX_PINCM19)
/* Defines for RESET: GPIOB.9 with pinCMx 26 on package pin 61 */
#define OLED_RESET_PORT                                                  (GPIOB)
#define OLED_RESET_PIN                                           (DL_GPIO_PIN_9)
#define OLED_RESET_IOMUX                                         (IOMUX_PINCM26)
/* Defines for DC: GPIOB.10 with pinCMx 27 on package pin 62 */
#define OLED_DC_PORT                                                     (GPIOB)
#define OLED_DC_PIN                                             (DL_GPIO_PIN_10)
#define OLED_DC_IOMUX                                            (IOMUX_PINCM27)
/* Defines for CS: GPIOA.3 with pinCMx 8 on package pin 43 */
#define OLED_CS_PORT                                                     (GPIOA)
#define OLED_CS_PIN                                              (DL_GPIO_PIN_3)
#define OLED_CS_IOMUX                                             (IOMUX_PINCM8)
/* Port definition for Pin Group MOTOR1 */
#define MOTOR1_PORT                                                      (GPIOB)

/* Defines for CTRL1: GPIOB.13 with pinCMx 30 on package pin 1 */
#define MOTOR1_CTRL1_PIN                                        (DL_GPIO_PIN_13)
#define MOTOR1_CTRL1_IOMUX                                       (IOMUX_PINCM30)
/* Defines for CTRL2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define MOTOR1_CTRL2_PIN                                        (DL_GPIO_PIN_17)
#define MOTOR1_CTRL2_IOMUX                                       (IOMUX_PINCM43)
/* Defines for ENCA1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define MOTOR1_ENCA1_IIDX                                   (DL_GPIO_IIDX_DIO14)
#define MOTOR1_ENCA1_PIN                                        (DL_GPIO_PIN_14)
#define MOTOR1_ENCA1_IOMUX                                       (IOMUX_PINCM31)
/* Defines for ENCB1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define MOTOR1_ENCB1_PIN                                        (DL_GPIO_PIN_15)
#define MOTOR1_ENCB1_IOMUX                                       (IOMUX_PINCM32)
/* Defines for CTRL3: GPIOA.25 with pinCMx 55 on package pin 26 */
#define MOTOR2_CTRL3_PORT                                                (GPIOA)
#define MOTOR2_CTRL3_PIN                                        (DL_GPIO_PIN_25)
#define MOTOR2_CTRL3_IOMUX                                       (IOMUX_PINCM55)
/* Defines for CTRL4: GPIOB.26 with pinCMx 57 on package pin 28 */
#define MOTOR2_CTRL4_PORT                                                (GPIOB)
#define MOTOR2_CTRL4_PIN                                        (DL_GPIO_PIN_26)
#define MOTOR2_CTRL4_IOMUX                                       (IOMUX_PINCM57)
/* Defines for ENCA2: GPIOB.23 with pinCMx 51 on package pin 22 */
#define MOTOR2_ENCA2_PORT                                                (GPIOB)
#define MOTOR2_ENCA2_IIDX                                   (DL_GPIO_IIDX_DIO23)
#define MOTOR2_ENCA2_PIN                                        (DL_GPIO_PIN_23)
#define MOTOR2_ENCA2_IOMUX                                       (IOMUX_PINCM51)
/* Defines for ENCB2: GPIOB.22 with pinCMx 50 on package pin 21 */
#define MOTOR2_ENCB2_PORT                                                (GPIOB)
#define MOTOR2_ENCB2_PIN                                        (DL_GPIO_PIN_22)
#define MOTOR2_ENCB2_IOMUX                                       (IOMUX_PINCM50)
/* Defines for CTRL5: GPIOB.27 with pinCMx 58 on package pin 29 */
#define MOTOR3_CTRL5_PORT                                                (GPIOB)
#define MOTOR3_CTRL5_PIN                                        (DL_GPIO_PIN_27)
#define MOTOR3_CTRL5_IOMUX                                       (IOMUX_PINCM58)
/* Defines for CTRL6: GPIOA.5 with pinCMx 10 on package pin 45 */
#define MOTOR3_CTRL6_PORT                                                (GPIOA)
#define MOTOR3_CTRL6_PIN                                         (DL_GPIO_PIN_5)
#define MOTOR3_CTRL6_IOMUX                                       (IOMUX_PINCM10)
/* Defines for ENCB3: GPIOB.20 with pinCMx 48 on package pin 19 */
#define MOTOR3_ENCB3_PORT                                                (GPIOB)
#define MOTOR3_ENCB3_PIN                                        (DL_GPIO_PIN_20)
#define MOTOR3_ENCB3_IOMUX                                       (IOMUX_PINCM48)
/* Defines for CTRL7: GPIOA.6 with pinCMx 11 on package pin 46 */
#define MOTOR4_CTRL7_PORT                                                (GPIOA)
#define MOTOR4_CTRL7_PIN                                         (DL_GPIO_PIN_6)
#define MOTOR4_CTRL7_IOMUX                                       (IOMUX_PINCM11)
/* Defines for CTRL8: GPIOB.1 with pinCMx 13 on package pin 48 */
#define MOTOR4_CTRL8_PORT                                                (GPIOB)
#define MOTOR4_CTRL8_PIN                                         (DL_GPIO_PIN_1)
#define MOTOR4_CTRL8_IOMUX                                       (IOMUX_PINCM13)
/* Defines for ENCA4: GPIOB.19 with pinCMx 45 on package pin 16 */
#define MOTOR4_ENCA4_PORT                                                (GPIOB)
#define MOTOR4_ENCA4_IIDX                                   (DL_GPIO_IIDX_DIO19)
#define MOTOR4_ENCA4_PIN                                        (DL_GPIO_PIN_19)
#define MOTOR4_ENCA4_IOMUX                                       (IOMUX_PINCM45)
/* Defines for G: GPIOA.24 with pinCMx 54 on package pin 25 */
#define RGB_GB_G_PORT                                                    (GPIOA)
#define RGB_GB_G_PIN                                            (DL_GPIO_PIN_24)
#define RGB_GB_G_IOMUX                                           (IOMUX_PINCM54)
/* Defines for B: GPIOB.0 with pinCMx 12 on package pin 47 */
#define RGB_GB_B_PORT                                                    (GPIOB)
#define RGB_GB_B_PIN                                             (DL_GPIO_PIN_0)
#define RGB_GB_B_IOMUX                                           (IOMUX_PINCM12)
/* Defines for PIN_0: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_SENSOR_PIN_0_PORT                                           (GPIOB)
#define GPIO_SENSOR_PIN_0_PIN                                    (DL_GPIO_PIN_6)
#define GPIO_SENSOR_PIN_0_IOMUX                                  (IOMUX_PINCM23)
/* Defines for PIN_1: GPIOA.9 with pinCMx 20 on package pin 55 */
#define GPIO_SENSOR_PIN_1_PORT                                           (GPIOA)
#define GPIO_SENSOR_PIN_1_PIN                                    (DL_GPIO_PIN_9)
#define GPIO_SENSOR_PIN_1_IOMUX                                  (IOMUX_PINCM20)
/* Defines for PIN_2: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_SENSOR_PIN_2_PORT                                           (GPIOB)
#define GPIO_SENSOR_PIN_2_PIN                                   (DL_GPIO_PIN_11)
#define GPIO_SENSOR_PIN_2_IOMUX                                  (IOMUX_PINCM28)
/* Defines for PIN_3: GPIOB.3 with pinCMx 16 on package pin 51 */
#define GPIO_SENSOR_PIN_3_PORT                                           (GPIOB)
#define GPIO_SENSOR_PIN_3_PIN                                    (DL_GPIO_PIN_3)
#define GPIO_SENSOR_PIN_3_IOMUX                                  (IOMUX_PINCM16)
/* Defines for PIN_4: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_SENSOR_PIN_4_PORT                                           (GPIOA)
#define GPIO_SENSOR_PIN_4_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_SENSOR_PIN_4_IOMUX                                  (IOMUX_PINCM38)
/* Defines for PIN_5: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_SENSOR_PIN_5_PORT                                           (GPIOB)
#define GPIO_SENSOR_PIN_5_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_SENSOR_PIN_5_IOMUX                                  (IOMUX_PINCM33)
/* Defines for PIN_6: GPIOA.2 with pinCMx 7 on package pin 42 */
#define GPIO_SENSOR_PIN_6_PORT                                           (GPIOA)
#define GPIO_SENSOR_PIN_6_PIN                                    (DL_GPIO_PIN_2)
#define GPIO_SENSOR_PIN_6_IOMUX                                   (IOMUX_PINCM7)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_G0_init(void);
void SYSCFG_DL_PWM_MOTOR_1_2_init(void);
void SYSCFG_DL_PWM_MOTOR_3_4_init(void);
void SYSCFG_DL_TIMER_M4_init(void);
void SYSCFG_DL_TIMER_M3_init(void);
void SYSCFG_DL_TIMER_A0_100us_init(void);
void SYSCFG_DL_TIMER_G12_1ms_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_USB_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_0_init(void);
void SYSCFG_DL_VCC_ADC_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
