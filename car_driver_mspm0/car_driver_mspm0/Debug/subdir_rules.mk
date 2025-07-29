################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-631825103: ../empty_mspm0g3507\ copy.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"E:/M0CCS/CCS/ccs/utils/sysconfig_1.23.0/sysconfig_cli.bat" --script "C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/empty_mspm0g3507 copy.syscfg" -o "syscfg" -s "E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/device_linker.cmd: build-631825103 ../empty_mspm0g3507\ copy.syscfg
syscfg/device.opt: build-631825103
syscfg/device.cmd.genlibs: build-631825103
syscfg/ti_msp_dl_config.c: build-631825103
syscfg/ti_msp_dl_config.h: build-631825103
syscfg/Event.dot: build-631825103
syscfg/boot_config.c: build-631825103
syscfg/boot_config.h: build-631825103
syscfg: build-631825103

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"E:/M0CCS/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.2.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug" -I"E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/source" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/car_bsp" -gdwarf-3 -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug/syscfg"  @"syscfg/device.opt" -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"E:/M0CCS/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.2.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug" -I"E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/source" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/car_bsp" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug/syscfg"  @"syscfg/device.opt" -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-670250977: ../empty_mspm0g3507.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"E:/M0CCS/CCS/ccs/utils/sysconfig_1.23.0/sysconfig_cli.bat" --script "C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/empty_mspm0g3507.syscfg" -o "syscfg" -s "E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/device_linker.cmd: build-670250977 ../empty_mspm0g3507.syscfg
syscfg/device.opt: build-670250977
syscfg/device.cmd.genlibs: build-670250977
syscfg/ti_msp_dl_config.c: build-670250977
syscfg/ti_msp_dl_config.h: build-670250977
syscfg/Event.dot: build-670250977
syscfg/boot_config.c: build-670250977
syscfg/boot_config.h: build-670250977
syscfg: build-670250977

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"E:/M0CCS/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.2.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug" -I"E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"E:/M0CCS/CCS/mspm0_sdk_2_05_01_00/source" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/car_bsp" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/HuYiFei/Desktop/cocode/EDC_Control/car_driver_mspm0/car_driver_mspm0/Debug/syscfg"  @"syscfg/device.opt" -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


