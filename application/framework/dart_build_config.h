#ifndef DART_BUILD_CONFIG_H
#define DART_BUILD_CONFIG_H

/*
 * 飞镖固件功能开关
 * ----------------
 * 当前 DM 转盘、左右升降轴和四个舵机门已经从整机拆除，所以默认关闭原换弹机构。关闭后
 * 固件不会等待这些设备的 CAN 反馈，也不会向它们发送使能、电流、位置或舵机命令。
 *
 * 重新安装原机构后，可在 CMake 中设置 DART_ENABLE_CAROUSEL_LOADER=ON；其他构建系统
 * 则定义 DART_ENABLE_CAROUSEL_LOADER=1。此处保留默认值，可以保证非 CMake 构建在
 * 没有额外配置时同样采用安全的拆机模式。
 */
#ifndef DART_ENABLE_CAROUSEL_LOADER
#define DART_ENABLE_CAROUSEL_LOADER 0
#endif

#endif /* DART_BUILD_CONFIG_H */
