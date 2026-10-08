# Dart 控制固件

当前固件只创建四个业务任务：`DartTask`、`DecodeTask`、`UsbTask` 和
`AdjustTask`。任务、消息、业务状态机、换弹机构以及 STM32 驱动已经分层，
详细结构和移植方法见 [架构说明](application/ARCHITECTURE.md)。

## 构建

```powershell
cmake --build cmake-build-debug -- -j4
```

参数 Flash 使用 STM32F427 Bank 1 的 Sector 11（`0x080E0000`），链接脚本已将
该扇区从程序区中独立保留。

## 当前换弹机构状态

当前 DM6006 转盘、左右升降轴和换弹舵机门已经拆除，因此默认构建使用
`DART_ENABLE_CAROUSEL_LOADER=OFF`。该配置不会等待这些设备上线，也不会向它们发送
使能、位置、电流或舵机命令。若参数声明首发已预装，首发流程仍可运行；请求第二发
换弹时会明确进入 `DART_FAULT_LOADER_UNAVAILABLE`，不会伪装成换弹成功。

装回原机构后使用以下配置恢复旧机构实现：

```powershell
cmake -S . -B cmake-build-loader -DDART_ENABLE_CAROUSEL_LOADER=ON
cmake --build cmake-build-loader --target 2027_Dart.elf -j 4
```
