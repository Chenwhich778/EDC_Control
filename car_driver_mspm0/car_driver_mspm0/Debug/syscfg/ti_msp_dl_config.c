/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerG_backupConfig gPWM_MOTOR_1_2Backup;
DL_TimerG_backupConfig gPWM_MOTOR_3_4Backup;
DL_TimerA_backupConfig gTIMER_M4Backup;
DL_TimerA_backupConfig gTIMER_A0_100usBackup;
DL_SPI_backupConfig gSPI_0Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_G0_init();
    SYSCFG_DL_PWM_MOTOR_1_2_init();
    SYSCFG_DL_PWM_MOTOR_3_4_init();
    SYSCFG_DL_TIMER_M4_init();
    SYSCFG_DL_TIMER_M3_init();
    SYSCFG_DL_TIMER_A0_100us_init();
    SYSCFG_DL_TIMER_G12_1ms_init();
    SYSCFG_DL_I2C_0_init();
    SYSCFG_DL_USB_UART_0_init();
    SYSCFG_DL_UART_1_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_SPI_0_init();
    SYSCFG_DL_VCC_ADC_init();
    SYSCFG_DL_ADC12_0_init();
    /* Ensure backup structures have no valid state */
	gPWM_MOTOR_1_2Backup.backupRdy 	= false;
	gPWM_MOTOR_3_4Backup.backupRdy 	= false;
	gTIMER_M4Backup.backupRdy 	= false;
	gTIMER_A0_100usBackup.backupRdy 	= false;

	gSPI_0Backup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(PWM_MOTOR_1_2_INST, &gPWM_MOTOR_1_2Backup);
	retStatus &= DL_TimerG_saveConfiguration(PWM_MOTOR_3_4_INST, &gPWM_MOTOR_3_4Backup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_M4_INST, &gTIMER_M4Backup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_A0_100us_INST, &gTIMER_A0_100usBackup);
	retStatus &= DL_SPI_saveConfiguration(SPI_0_INST, &gSPI_0Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(PWM_MOTOR_1_2_INST, &gPWM_MOTOR_1_2Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(PWM_MOTOR_3_4_INST, &gPWM_MOTOR_3_4Backup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_M4_INST, &gTIMER_M4Backup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_A0_100us_INST, &gTIMER_A0_100usBackup, false);
	retStatus &= DL_SPI_restoreConfiguration(SPI_0_INST, &gSPI_0Backup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(PWM_G0_INST);
    DL_TimerG_reset(PWM_MOTOR_1_2_INST);
    DL_TimerG_reset(PWM_MOTOR_3_4_INST);
    DL_TimerA_reset(TIMER_M4_INST);
    DL_TimerG_reset(TIMER_M3_INST);
    DL_TimerA_reset(TIMER_A0_100us_INST);
    DL_TimerG_reset(TIMER_G12_1ms_INST);
    DL_I2C_reset(I2C_0_INST);
    DL_UART_Main_reset(USB_UART_0_INST);
    DL_UART_Main_reset(UART_1_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_SPI_reset(SPI_0_INST);
    DL_ADC12_reset(VCC_ADC_INST);
    DL_ADC12_reset(ADC12_0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(PWM_G0_INST);
    DL_TimerG_enablePower(PWM_MOTOR_1_2_INST);
    DL_TimerG_enablePower(PWM_MOTOR_3_4_INST);
    DL_TimerA_enablePower(TIMER_M4_INST);
    DL_TimerG_enablePower(TIMER_M3_INST);
    DL_TimerA_enablePower(TIMER_A0_100us_INST);
    DL_TimerG_enablePower(TIMER_G12_1ms_INST);
    DL_I2C_enablePower(I2C_0_INST);
    DL_UART_Main_enablePower(USB_UART_0_INST);
    DL_UART_Main_enablePower(UART_1_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_SPI_enablePower(SPI_0_INST);
    DL_ADC12_enablePower(VCC_ADC_INST);
    DL_ADC12_enablePower(ADC12_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_G0_C0_IOMUX,GPIO_PWM_G0_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_G0_C0_PORT, GPIO_PWM_G0_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_G0_C1_IOMUX,GPIO_PWM_G0_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_G0_C1_PORT, GPIO_PWM_G0_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_1_2_C0_IOMUX,GPIO_PWM_MOTOR_1_2_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_1_2_C0_PORT, GPIO_PWM_MOTOR_1_2_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_1_2_C1_IOMUX,GPIO_PWM_MOTOR_1_2_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_1_2_C1_PORT, GPIO_PWM_MOTOR_1_2_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_3_4_C0_IOMUX,GPIO_PWM_MOTOR_3_4_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_3_4_C0_PORT, GPIO_PWM_MOTOR_3_4_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_3_4_C1_IOMUX,GPIO_PWM_MOTOR_3_4_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_3_4_C1_PORT, GPIO_PWM_MOTOR_3_4_C1_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_TIMER_M4_C1_IOMUX,GPIO_TIMER_M4_C1_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_TIMER_M3_C0_IOMUX,GPIO_TIMER_M3_C0_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_USB_UART_0_IOMUX_TX, GPIO_USB_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_USB_UART_0_IOMUX_RX, GPIO_USB_UART_0_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_1_IOMUX_TX, GPIO_UART_1_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_0_IOMUX_SCLK, GPIO_SPI_0_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_0_IOMUX_PICO, GPIO_SPI_0_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_0_IOMUX_POCI, GPIO_SPI_0_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalInputFeatures(KEY1_PB5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(SPI0_CS0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutput(OLED_RESET_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_DC_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_CS_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR1_CTRL1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR1_CTRL2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(MOTOR1_ENCA1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(MOTOR1_ENCB1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MOTOR2_CTRL3_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR2_CTRL4_IOMUX);

    DL_GPIO_initDigitalInputFeatures(MOTOR2_ENCA2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(MOTOR2_ENCB2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MOTOR3_CTRL5_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR3_CTRL6_IOMUX);

    DL_GPIO_initDigitalInputFeatures(MOTOR3_ENCB3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MOTOR4_CTRL7_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR4_CTRL8_IOMUX);

    DL_GPIO_initDigitalInputFeatures(MOTOR4_ENCA4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(RGB_GB_G_IOMUX);

    DL_GPIO_initDigitalOutput(RGB_GB_B_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_SENSOR_PIN_0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_SENSOR_PIN_1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_SENSOR_PIN_2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_SENSOR_PIN_3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_SENSOR_PIN_4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_SENSOR_PIN_5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_SENSOR_PIN_6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, SPI0_CS0_PIN |
		OLED_CS_PIN |
		MOTOR2_CTRL3_PIN |
		MOTOR3_CTRL6_PIN |
		MOTOR4_CTRL7_PIN |
		RGB_GB_G_PIN);
    DL_GPIO_enableOutput(GPIOA, SPI0_CS0_PIN |
		OLED_CS_PIN |
		MOTOR2_CTRL3_PIN |
		MOTOR3_CTRL6_PIN |
		MOTOR4_CTRL7_PIN |
		RGB_GB_G_PIN);
    DL_GPIO_clearPins(GPIOB, OLED_RESET_PIN |
		OLED_DC_PIN |
		MOTOR1_CTRL1_PIN |
		MOTOR1_CTRL2_PIN |
		MOTOR2_CTRL4_PIN |
		MOTOR3_CTRL5_PIN |
		MOTOR4_CTRL8_PIN |
		RGB_GB_B_PIN);
    DL_GPIO_enableOutput(GPIOB, OLED_RESET_PIN |
		OLED_DC_PIN |
		MOTOR1_CTRL1_PIN |
		MOTOR1_CTRL2_PIN |
		MOTOR2_CTRL4_PIN |
		MOTOR3_CTRL5_PIN |
		MOTOR4_CTRL8_PIN |
		RGB_GB_B_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_5_EDGE_FALL |
		DL_GPIO_PIN_14_EDGE_FALL |
		DL_GPIO_PIN_15_EDGE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_23_EDGE_FALL |
		DL_GPIO_PIN_19_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, KEY1_PB5_PIN |
		MOTOR1_ENCA1_PIN |
		MOTOR2_ENCA2_PIN |
		MOTOR4_ENCA4_PIN);
    DL_GPIO_enableInterrupt(GPIOB, KEY1_PB5_PIN |
		MOTOR1_ENCA1_PIN |
		MOTOR2_ENCA2_PIN |
		MOTOR4_ENCA4_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOB_INT_IRQn, 1);

}


/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   400000 Hz = 4000000 Hz / (8 * (9 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_G0ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 9U
};

static const DL_TimerG_PWMConfig gPWM_G0Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_G0_init(void) {

    DL_TimerG_setClockConfig(
        PWM_G0_INST, (DL_TimerG_ClockConfig *) &gPWM_G0ClockConfig);

    DL_TimerG_initPWMMode(
        PWM_G0_INST, (DL_TimerG_PWMConfig *) &gPWM_G0Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_G0_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_G0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_G0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_G0_INST, 1000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(PWM_G0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_G0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_G0_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_G0_INST);


    
    DL_TimerG_setCCPDirection(PWM_G0_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_MOTOR_1_2ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_MOTOR_1_2Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_MOTOR_1_2_init(void) {

    DL_TimerG_setClockConfig(
        PWM_MOTOR_1_2_INST, (DL_TimerG_ClockConfig *) &gPWM_MOTOR_1_2ClockConfig);

    DL_TimerG_initPWMMode(
        PWM_MOTOR_1_2_INST, (DL_TimerG_PWMConfig *) &gPWM_MOTOR_1_2Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_MOTOR_1_2_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_MOTOR_1_2_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_MOTOR_1_2_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST, 1000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(PWM_MOTOR_1_2_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_MOTOR_1_2_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_MOTOR_1_2_INST);


    
    DL_TimerG_setCCPDirection(PWM_MOTOR_1_2_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_MOTOR_3_4ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_MOTOR_3_4Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_MOTOR_3_4_init(void) {

    DL_TimerG_setClockConfig(
        PWM_MOTOR_3_4_INST, (DL_TimerG_ClockConfig *) &gPWM_MOTOR_3_4ClockConfig);

    DL_TimerG_initPWMMode(
        PWM_MOTOR_3_4_INST, (DL_TimerG_PWMConfig *) &gPWM_MOTOR_3_4Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_MOTOR_3_4_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_MOTOR_3_4_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_MOTOR_3_4_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST, 1000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(PWM_MOTOR_3_4_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_MOTOR_3_4_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_MOTOR_3_4_INST);


    
    DL_TimerG_setCCPDirection(PWM_MOTOR_3_4_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   16000000 Hz = 32000000 Hz / (1 * (1 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_M4ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 1U
};

static const DL_TimerA_CompareConfig gTIMER_M4CompareConfig = {
    .compareMode    = DL_TIMER_COMPARE_MODE_EDGE_COUNT,
    .count          = 0,
    .startTimer     = DL_TIMER_STOP,
    .edgeDetectMode = DL_TIMER_COMPARE_EDGE_DETECTION_MODE_RISING,
    .inputChan      = DL_TIMER_INPUT_CHAN_1,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_M4_init(void) {

    DL_TimerA_setClockConfig(TIMER_M4_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_M4ClockConfig);
    DL_TimerA_initCompareMode(TIMER_M4_INST,
        (DL_TimerA_CompareConfig *) &gTIMER_M4CompareConfig);
    DL_TimerA_enableInterrupt(TIMER_M4_INST , DL_TIMERA_INTERRUPT_LOAD_EVENT);



    DL_TimerA_enableClock(TIMER_M4_INST);




}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   16000000 Hz = 32000000 Hz / (1 * (1 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_M3ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 1U
};

static const DL_TimerG_CompareConfig gTIMER_M3CompareConfig = {
    .compareMode    = DL_TIMER_COMPARE_MODE_EDGE_COUNT,
    .count          = 0,
    .startTimer     = DL_TIMER_STOP,
    .edgeDetectMode = DL_TIMER_COMPARE_EDGE_DETECTION_MODE_RISING,
    .inputChan      = DL_TIMER_INPUT_CHAN_0,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_M3_init(void) {

    DL_TimerG_setClockConfig(TIMER_M3_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_M3ClockConfig);
    DL_TimerG_initCompareMode(TIMER_M3_INST,
        (DL_TimerG_CompareConfig *) &gTIMER_M3CompareConfig);
    DL_TimerG_enableInterrupt(TIMER_M3_INST , DL_TIMERG_INTERRUPT_LOAD_EVENT);



    DL_TimerG_enableClock(TIMER_M3_INST);




}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1600000 Hz = 32000000 Hz / (1 * (19 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_A0_100usClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 19U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_A0_100us_INST_LOAD_VALUE = (100 us * 1600000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMER_A0_100usTimerConfig = {
    .period     = TIMER_A0_100us_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_A0_100us_init(void) {

    DL_TimerA_setClockConfig(TIMER_A0_100us_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_A0_100usClockConfig);

    DL_TimerA_initTimerMode(TIMER_A0_100us_INST,
        (DL_TimerA_TimerConfig *) &gTIMER_A0_100usTimerConfig);
    DL_TimerA_enableInterrupt(TIMER_A0_100us_INST , DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(TIMER_A0_100us_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_G12_1msClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_G12_1ms_INST_LOAD_VALUE = (100 us * 32000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_G12_1msTimerConfig = {
    .period     = TIMER_G12_1ms_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_G12_1ms_init(void) {

    DL_TimerG_setClockConfig(TIMER_G12_1ms_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_G12_1msClockConfig);

    DL_TimerG_initTimerMode(TIMER_G12_1ms_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_G12_1msTimerConfig);
    DL_TimerG_enableInterrupt(TIMER_G12_1ms_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMER_G12_1ms_INST);





}


static const DL_I2C_ClockConfig gI2C_0ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_0_init(void) {

    DL_I2C_setClockConfig(I2C_0_INST,
        (DL_I2C_ClockConfig *) &gI2C_0ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_0_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_0_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_0_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_7);
    DL_I2C_setControllerRXFIFOThreshold(I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_8);
    DL_I2C_enableControllerClockStretching(I2C_0_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_0_INST);


}

static const DL_UART_Main_ClockConfig gUSB_UART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUSB_UART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_USB_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(USB_UART_0_INST, (DL_UART_Main_ClockConfig *) &gUSB_UART_0ClockConfig);

    DL_UART_Main_init(USB_UART_0_INST, (DL_UART_Main_Config *) &gUSB_UART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(USB_UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(USB_UART_0_INST, USB_UART_0_IBRD_32_MHZ_115200_BAUD, USB_UART_0_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(USB_UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(USB_UART_0_INST_INT_IRQN, 0);


    DL_UART_Main_enable(USB_UART_0_INST);
}
static const DL_UART_Main_ClockConfig gUART_1ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_1Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_1_init(void)
{
    DL_UART_Main_setClockConfig(UART_1_INST, (DL_UART_Main_ClockConfig *) &gUART_1ClockConfig);

    DL_UART_Main_init(UART_1_INST, (DL_UART_Main_Config *) &gUART_1Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_1_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST, UART_1_IBRD_32_MHZ_115200_BAUD, UART_1_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_1_INST);
}
static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.24
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_32_MHZ_9600_BAUD, UART_0_FBRD_32_MHZ_9600_BAUD);



    DL_UART_Main_enable(UART_0_INST);
}

static const DL_SPI_Config gSPI_0_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_0_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_0_init(void) {
    DL_SPI_setClockConfig(SPI_0_INST, (DL_SPI_ClockConfig *) &gSPI_0_clockConfig);

    DL_SPI_init(SPI_0_INST, (DL_SPI_Config *) &gSPI_0_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     16000000 = (32000000)/((1 + 0) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(SPI_0_INST, 0);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_0_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(SPI_0_INST);
}

/* VCC_ADC Initialization */
static const DL_ADC12_ClockConfig gVCC_ADCClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_VCC_ADC_init(void)
{
    DL_ADC12_setClockConfig(VCC_ADC_INST, (DL_ADC12_ClockConfig *) &gVCC_ADCClockConfig);

    DL_ADC12_initSeqSample(VCC_ADC_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_00, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(VCC_ADC_INST, VCC_ADC_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(VCC_ADC_INST,400);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(VCC_ADC_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_enableInterrupt(VCC_ADC_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_enableConversions(VCC_ADC_INST);
}
/* ADC12_0 Initialization */
static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_0_init(void)
{
    DL_ADC12_setClockConfig(ADC12_0_INST, (DL_ADC12_ClockConfig *) &gADC12_0ClockConfig);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_enableConversions(ADC12_0_INST);
}

