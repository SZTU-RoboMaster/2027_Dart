#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 4.0)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(2027_Dart C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core/Inc Drivers/STM32F4xx_HAL_Driver/Inc Drivers/STM32F4xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32F4xx/Include Drivers/CMSIS/Include USB_DEVICE/App USB_DEVICE/Target Middlewares/ST/STM32_USB_Device_Library/Core/Inc Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F bsp application/A_Dart application/Chassis application/Communication application/Gimbal application/Operate application/Referee_system component/algorithm component/comunication component/controller component/support component/devices)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32F427xx)

file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "Drivers/*.*" "USB_DEVICE/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F427IIHX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT}
        bsp/bsp_can.c
        bsp/bsp_can.h
        bsp/bsp_crc32.h
        bsp/bsp_crc32.c
        bsp/bsp_delay.h
        bsp/bsp_delay.c
        bsp/bsp_flash.h
        bsp/bsp_flash.c
        bsp/bsp_adc.h
        bsp/bsp_buzzer.c
        bsp/bsp_buzzer.h
        bsp/bsp_adc.c
        bsp/dm_8009.h
        bsp/dm_8009.c
        bsp/DM_MOTOR.h
        bsp/DM_MOTOR.c
        bsp/bsp_cap.h
        bsp/bsp_cap.c
        application/A_Dart/dart.h
        application/A_Dart/dart.c
        application/Communication/can_receive.c
        application/Communication/can_receive.h
        application/Chassis/Cap.c
        application/Chassis/Cap.h
        application/Operate/key_board.c
        application/Operate/key_board.h
        application/Referee_system/Referee.c
        application/Referee_system/Referee.h
        application/Communication/usb_task.c
        application/Communication/usb_task.h
        application/A_Dart/Adjust_Board.c
        application/A_Dart/Adjust_Board.h
        application/A_Dart/struct_typedef.h
        application/A_Dart/protocol_shaob.h
        component/algorithm/AHRS.h
        component/algorithm/AHRS.c
        component/algorithm/AHRS_MiddleWare.c
        component/algorithm/AHRS_MiddleWare.h
        component/algorithm/arm_math.h
        component/algorithm/arm_common_tables.h
        component/algorithm/arm_const_structs.h
        component/algorithm/core_sc000.h
        component/algorithm/core_sc300.h
        component/algorithm/filter.h
        component/algorithm/filter.c
        component/algorithm/ramp.h
        component/algorithm/ramp.c
        component/algorithm/STL.h
        component/algorithm/STL.c
        component/algorithm/user_lib.h
        component/algorithm/user_lib.c
        component/comunication/decode.h
        component/comunication/decode.c
        component/comunication/packet.h
        component/comunication/packet.c
        component/controller/PID.h
        component/controller/PID.c
        component/support/CRC8_CRC16.h
        component/support/CRC8_CRC16.c
        component/support/fifo.h
        component/support/fifo.c
        component/support/linux_list.h
        component/support/macro_mutex.h
        component/support/mem_mang.h
        component/support/mem_mang4.c
        component/devices/zp10s_servo.c
        component/devices/zp10s_servo.h
        application/Operate/remote.c
        application/Operate/remote.h
        application/Referee_system/Detection.c
        application/Referee_system/Detection.h
        application/Referee_system/Calibrate.h
        application/Referee_system/Calibrate.c
        application/A_Dart/Send_to_Screen.c
        application/A_Dart/Send_to_Screen.h
        application/Chassis/Chassis.c
        application/Chassis/Chassis.h
        application/Gimbal/Gimbal.c
        application/Gimbal/Gimbal.h
        application/Gimbal/launcher.c
        application/Gimbal/launcher.h
        application/Communication/Atti.c
        application/Communication/Atti.h
        bsp/bsp_rc.c
        bsp/bsp_rc.h

)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
