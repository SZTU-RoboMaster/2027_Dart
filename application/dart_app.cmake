# 飞镖应用独立维护的源码清单。该清单不依赖自动生成目录，重新生成底层工程时不会把已经
# 淘汰的底盘、旧云台、旧发射器或界面模块悄悄加入固件。
set(DART_APPLICATION_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/bsp_can.c
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/bsp_crc32.c
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/bsp_flash.c
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/DM_MOTOR.c
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/bsp_rc.c
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/bsp_usart.c
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/dart.c
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/dart_runtime.c
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/dart_sm.c
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/dart_parameters.c
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/dart_platform_stm32.c
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/Adjust_Board.c
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/loader/detached_reload_strategy.c
    ${CMAKE_CURRENT_LIST_DIR}/framework/app_tasks.c
    ${CMAKE_CURRENT_LIST_DIR}/framework/topic_bus.c
    ${CMAKE_CURRENT_LIST_DIR}/framework/operator_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/Communication/can_receive.c
    ${CMAKE_CURRENT_LIST_DIR}/Communication/usb_task.c
    ${CMAKE_CURRENT_LIST_DIR}/Operate/remote.c
    ${CMAKE_CURRENT_LIST_DIR}/Referee_system/Referee.c
    ${CMAKE_CURRENT_LIST_DIR}/../component/algorithm/user_lib.c
    ${CMAKE_CURRENT_LIST_DIR}/../component/comunication/decode.c
    ${CMAKE_CURRENT_LIST_DIR}/../component/comunication/packet.c
    ${CMAKE_CURRENT_LIST_DIR}/../component/controller/PID.c
    ${CMAKE_CURRENT_LIST_DIR}/../component/support/CRC8_CRC16.c
    ${CMAKE_CURRENT_LIST_DIR}/../component/support/fifo.c
    ${CMAKE_CURRENT_LIST_DIR}/../component/support/mem_mang4.c
)

# 原转盘升降机构作为可选板级功能保留。硬件拆除期间默认固件不编译其实现，避免任何意外
# 调用路径向不存在的升降轴、转盘或舵机门发送输出。
if(DART_ENABLE_CAROUSEL_LOADER)
    list(APPEND DART_APPLICATION_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/A_Dart/loader/carousel_lift_loader.c
        ${CMAKE_CURRENT_LIST_DIR}/A_Dart/loader/dart_reload_strategy.c
        ${CMAKE_CURRENT_LIST_DIR}/../component/devices/zp10s_servo.c
    )
endif()

set(DART_APPLICATION_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart
    ${CMAKE_CURRENT_LIST_DIR}/A_Dart/loader
    ${CMAKE_CURRENT_LIST_DIR}/framework
    ${CMAKE_CURRENT_LIST_DIR}/Communication
    ${CMAKE_CURRENT_LIST_DIR}/Operate
    ${CMAKE_CURRENT_LIST_DIR}/Referee_system
    ${CMAKE_CURRENT_LIST_DIR}/../bsp
    ${CMAKE_CURRENT_LIST_DIR}/../component/algorithm
    ${CMAKE_CURRENT_LIST_DIR}/../component/comunication
    ${CMAKE_CURRENT_LIST_DIR}/../component/controller
    ${CMAKE_CURRENT_LIST_DIR}/../component/support
    ${CMAKE_CURRENT_LIST_DIR}/../component/devices
)
