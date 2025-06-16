cmake_minimum_required(VERSION 3.8)

project(firmware)

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m0plus -march=armv6-m -mfloat-abi=soft")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/mspm0g3507.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_executable(${CMAKE_PROJECT_NAME}
    Core/Startup/startup_mspm0g350x_uvision.s
    Core/Src/main.c
    Core/Src/cRingbuf.c
    Core/Src/lsm6dso_reg.c
    Core/Src/lsm6dso.c
    Core/Src/lsm6dsv_reg.c
    Core/Src/main.c
    Core/Src/mspm0g3507_it.c
    Core/Src/oled_data.c
    Core/Src/oled.c
    Core/Src/tb6612.c
    Core/Src/ti_msp_dl_config.c
    Core/Src/uart.c
    Core/Src/syscalls.c
    Drivers/ti/driverlib/m0p/sysctl/dl_sysctl_mspm0g1x0x_g3x0x.c
    Drivers/ti/driverlib/m0p/dl_factoryregion.c
    Drivers/ti/driverlib/m0p/dl_interrupt.c
    Drivers/ti/driverlib/dl_adc12.c
    Drivers/ti/driverlib/dl_aes.c
    Drivers/ti/driverlib/dl_aesadv.c
    Drivers/ti/driverlib/dl_common.c
    Drivers/ti/driverlib/dl_crc.c
    Drivers/ti/driverlib/dl_crcp.c
    Drivers/ti/driverlib/dl_dac12.c
    Drivers/ti/driverlib/dl_dma.c
    Drivers/ti/driverlib/dl_flashctl.c
    Drivers/ti/driverlib/dl_i2c.c
    Drivers/ti/driverlib/dl_keystorectl.c
    Drivers/ti/driverlib/dl_lcd.c
    Drivers/ti/driverlib/dl_lfss.c
    Drivers/ti/driverlib/dl_mathacl.c
    Drivers/ti/driverlib/dl_mcan.c
    Drivers/ti/driverlib/dl_opa.c
    Drivers/ti/driverlib/dl_rtc_common.c
    Drivers/ti/driverlib/dl_spi.c
    Drivers/ti/driverlib/dl_timer.c
    Drivers/ti/driverlib/dl_trng.c
    Drivers/ti/driverlib/dl_uart.c
    Drivers/ti/driverlib/dl_vref.c
    Middlewares/FreeRTOS/portable/GCC/ARM_CM0/port.c
    Middlewares/FreeRTOS/portable/MemMang/heap_4.c
    Middlewares/FreeRTOS/croutine.c
    Middlewares/FreeRTOS/event_groups.c
    Middlewares/FreeRTOS/list.c
    Middlewares/FreeRTOS/queue.c
    Middlewares/FreeRTOS/tasks.c
    Middlewares/FreeRTOS/stream_buffer.c
    Middlewares/FreeRTOS/timers.c
    # Middlewares/Micro-CDR/src/c/types/array.c
    # Middlewares/Micro-CDR/src/c/types/basic.c
    # Middlewares/Micro-CDR/src/c/types/sequence.c
    # Middlewares/Micro-CDR/src/c/types/string.c
    # Middlewares/Micro-CDR/src/c/common.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/log/log.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/serialization/xrce_header.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/serialization/xrce_subheader.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/serialization/xrce_types.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/stream/input_best_effort_stream.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/stream/input_reliable_stream.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/stream/output_best_effort_stream.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/stream/output_reliable_stream.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/stream/seq_num.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/stream/stream_id.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/stream/stream_storage.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/common_create_entities.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/create_entities_bin.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/create_entities_ref.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/create_entities_xml.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/object_id.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/read_access.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/session_info.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/session.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/write_access.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/core/session/submessage.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/profile/transport/stream_framing/stream_framing_protocol.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/profile/discovery/discovery.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/profile/matching/matching.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/util/ping.c
    # Middlewares/Micro-XRCE-DDS-Client/src/c/util/time.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers
    Drivers/CMSIS/Core/Include
    Middlewares/FreeRTOS/include
    Middlewares/FreeRTOS/portable/GCC/ARM_CM0
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    -D__MSPM0G3507__
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    m
    c
)