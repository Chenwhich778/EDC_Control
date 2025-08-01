################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/CCS_20.2/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS

GEN_OPTS__FLAG := @"syscfg/device.opt" 
GEN_CMDS__FLAG := -Wl,-l"syscfg/device_linker.cmd" 

ORDERED_OBJS += \
"./syscfg/ti_msp_dl_config.o" \
"./syscfg/boot_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./main.o" \
"./car_bsp/ADC.o" \
"./car_bsp/Buzzer.o" \
"./car_bsp/No_Mcu_Ganv_Grayscale_Sensor.o" \
"./car_bsp/PID.o" \
"./car_bsp/draw.o" \
"./car_bsp/key.o" \
"./car_bsp/key1.o" \
"./car_bsp/laser_tracker.o" \
"./car_bsp/motor.o" \
"./car_bsp/rgb.o" \
"./car_bsp/sensor.o" \
"./car_bsp/servo_protocol.o" \
"./car_bsp/usart.o" \
"./car_bsp/usb_uart0.o" \
"./car_bsp/vcc_adc.o" \
"./car_bsp/Grayscale_Sensor/IIC.o" \
"./car_bsp/Grayscale_Sensor/hardware_iic.o" \
"./car_bsp/ICM45686/ICM45686/inv_imu_driver.o" \
"./car_bsp/ICM45686/ICM45686/inv_imu_transport.o" \
"./car_bsp/ICM45686/IMU/IMU.o" \
"./car_bsp/ICM45686/basic_read_reg.o" \
"./car_bsp/ICM45686/timer_100us.o" \
"./car_bsp/OLED/oled.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include car_bsp/subdir_vars.mk
-include car_bsp/Grayscale_Sensor/subdir_vars.mk
-include car_bsp/ICM45686/ICM45686/subdir_vars.mk
-include car_bsp/ICM45686/IMU/subdir_vars.mk
-include car_bsp/ICM45686/subdir_vars.mk
-include car_bsp/OLED/subdir_vars.mk
-include subdir_rules.mk
-include car_bsp/subdir_rules.mk
-include car_bsp/Grayscale_Sensor/subdir_rules.mk
-include car_bsp/ICM45686/ICM45686/subdir_rules.mk
-include car_bsp/ICM45686/IMU/subdir_rules.mk
-include car_bsp/ICM45686/subdir_rules.mk
-include car_bsp/OLED/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
car_driver_mspm0.out 

EXE_OUTPUTS__QUOTED += \
"car_driver_mspm0.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "car_driver_mspm0.out"

# Tool invocations
car_driver_mspm0.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"D:/ti/CCS_20.2/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wl,-m"car_driver_mspm0.map" -Wl,-i"D:/ti/CCS_20.2/mspm0_sdk_2_05_01_00/source" -Wl,-i"C:/Users/28248/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug/syscfg" -Wl,-i"D:/ti/CCS_20.2/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="car_driver_mspm0_linkInfo.xml" -Wl,--rom_model -o "car_driver_mspm0.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "syscfg\ti_msp_dl_config.o" "syscfg\boot_config.o" "startup_mspm0g350x_ticlang.o" "main.o" "car_bsp\ADC.o" "car_bsp\Buzzer.o" "car_bsp\No_Mcu_Ganv_Grayscale_Sensor.o" "car_bsp\PID.o" "car_bsp\draw.o" "car_bsp\key.o" "car_bsp\key1.o" "car_bsp\laser_tracker.o" "car_bsp\motor.o" "car_bsp\rgb.o" "car_bsp\sensor.o" "car_bsp\servo_protocol.o" "car_bsp\usart.o" "car_bsp\usb_uart0.o" "car_bsp\vcc_adc.o" "car_bsp\Grayscale_Sensor\IIC.o" "car_bsp\Grayscale_Sensor\hardware_iic.o" "car_bsp\ICM45686\ICM45686\inv_imu_driver.o" "car_bsp\ICM45686\ICM45686\inv_imu_transport.o" "car_bsp\ICM45686\IMU\IMU.o" "car_bsp\ICM45686\basic_read_reg.o" "car_bsp\ICM45686\timer_100us.o" "car_bsp\OLED\oled.o" 
	-$(RM) "syscfg\ti_msp_dl_config.d" "syscfg\boot_config.d" "startup_mspm0g350x_ticlang.d" "main.d" "car_bsp\ADC.d" "car_bsp\Buzzer.d" "car_bsp\No_Mcu_Ganv_Grayscale_Sensor.d" "car_bsp\PID.d" "car_bsp\draw.d" "car_bsp\key.d" "car_bsp\key1.d" "car_bsp\laser_tracker.d" "car_bsp\motor.d" "car_bsp\rgb.d" "car_bsp\sensor.d" "car_bsp\servo_protocol.d" "car_bsp\usart.d" "car_bsp\usb_uart0.d" "car_bsp\vcc_adc.d" "car_bsp\Grayscale_Sensor\IIC.d" "car_bsp\Grayscale_Sensor\hardware_iic.d" "car_bsp\ICM45686\ICM45686\inv_imu_driver.d" "car_bsp\ICM45686\ICM45686\inv_imu_transport.d" "car_bsp\ICM45686\IMU\IMU.d" "car_bsp\ICM45686\basic_read_reg.d" "car_bsp\ICM45686\timer_100us.d" "car_bsp\OLED\oled.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

