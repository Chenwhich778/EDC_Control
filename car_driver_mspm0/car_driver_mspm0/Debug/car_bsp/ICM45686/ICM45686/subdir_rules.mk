################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
car_bsp/ICM45686/ICM45686/%.o: ../car_bsp/ICM45686/ICM45686/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/CCS_20.2/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/28248/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0" -I"C:/Users/28248/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug" -I"D:/ti/CCS_20.2/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"D:/ti/CCS_20.2/mspm0_sdk_2_05_01_00/source" -I"C:/Users/28248/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/car_bsp" -gdwarf-3 -MMD -MP -MF"car_bsp/ICM45686/ICM45686/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/28248/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


