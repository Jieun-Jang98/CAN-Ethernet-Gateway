cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_GCC arm-none-eabi-gcc REQUIRED)
find_program(ARM_GXX arm-none-eabi-g++ REQUIRED)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)
set(CMAKE_C_COMPILER ${ARM_GCC})
set(CMAKE_CXX_COMPILER ${ARM_GXX})
set(CMAKE_ASM_COMPILER ${ARM_GCC})

project(KUSE_Straight_CAN_Log LANGUAGES C CXX ASM)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(BOARD ${CMAKE_CURRENT_LIST_DIR}/board)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS ${BOARD}/Drivers/STM32F4xx_HAL_Driver/Src/*.c)

add_executable(KUSE_Straight_CAN_Log.elf
    ${BOARD}/Core/Startup/startup_stm32f446retx.s
    ${BOARD}/Core/Src/main.c
    ${BOARD}/Core/Src/stm32f4xx_hal_msp.c
    ${BOARD}/Core/Src/stm32f4xx_it.c
    ${BOARD}/Core/Src/system_stm32f4xx.c
    ${BOARD}/Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c
    ${HAL_SOURCES}
    ${ROOT}/common/src/can_codec.cpp
    ${ROOT}/common/src/checks.cpp
    ${ROOT}/common/src/protocol.cpp
    ${BOARD}/Core/Src/straight_node.cpp
    ${BOARD}/Core/Src/app_main.cpp
)

target_include_directories(KUSE_Straight_CAN_Log.elf PRIVATE
    ${BOARD}/Core/Inc
    ${BOARD}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${BOARD}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${BOARD}/Drivers/CMSIS/Include
    ${BOARD}/Drivers/BSP/STM32F4xx-Nucleo
    ${ROOT}/common/include
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(KUSE_Straight_CAN_Log.elf PRIVATE USE_HAL_DRIVER STM32F446xx)
target_compile_options(KUSE_Straight_CAN_Log.elf PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections -Wall -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++17 -fno-exceptions -fno-rtti -fno-use-cxa-atexit>
)
target_link_options(KUSE_Straight_CAN_Log.elf PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -T${BOARD}/STM32F446RETX_FLASH.ld
    -Wl,--gc-sections -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/KUSE_Straight_CAN_Log.map
    --specs=nano.specs --specs=nosys.specs
)

add_custom_command(TARGET KUSE_Straight_CAN_Log.elf POST_BUILD
    COMMAND ${ARM_OBJCOPY} -O ihex $<TARGET_FILE:KUSE_Straight_CAN_Log.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/KUSE_Straight_CAN_Log.hex
)
