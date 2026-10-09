# 串口屏通信协议

本文档根据以下代码整理：

- `application/A_Dart/Adjust_Board.c`
- `application/A_Dart/Send_to_Screen.c`
- `application/A_Dart/dart.c`
- `Core/Src/usart.c`

## 1. 串口配置

串口屏和调参板共用 `UART7`：

| 项目 | 配置 |
|---|---|
| 串口 | UART7 |
| TX 引脚 | PE8 |
| RX 引脚 | PE7 |
| 波特率 | 115200 |
| 数据位 | 8 bit |
| 校验位 | 无校验 |
| 停止位 | 1 bit |
| 硬件流控 | 无 |
| 工作模式 | 全双工 |

MCU 接收使用 `HAL_UARTEx_ReceiveToIdle_IT`，发送使用 UART7 DMA。

## 2. 串口屏到 MCU 的调参协议

### 2.1 帧格式

一帧固定包含 4 个参数：

```text
55 CMD ITEM_1 VALUE_1 FF FF
      ITEM_2 VALUE_2 FF FF
      ITEM_3 VALUE_3 FF FF
      ITEM_4 VALUE_4 FF FF
```

字段定义如下：

| 字段 | 长度 | 格式 | 说明 |
|---|---:|---|---|
| `SOF` | 1 字节 | `0x55` | 固定帧头 |
| `CMD` | 1 字节 | `0x01` ~ `0x04` | 参数类型和目标站点 |
| `ITEM` | 1 字节 | `0x01` ~ `0x04` | 参数编号，必须按顺序发送 |
| `VALUE` | 不定长 | ASCII 数字 | 支持负号和小数点 |
| `VALUE_END` | 2 字节 | `0xFF 0xFF` | 当前参数结束标志 |

数值格式为：

```text
-?[0-9]+(\.[0-9]*)?
```

支持的示例：

```text
57
-57.6753
0.5
-0.5
```

不支持正号、科学计数法和空格。负号只能出现在数值最前面。

### 2.2 CMD 对应参数

| `CMD` | 写入数组 | 含义 |
|---|---|---|
| `0x01` | `dart_goal_set.trigger_distance_set[GOAL_FRONT_STATION][0..3]` | 前哨站 4 个触发位置 |
| `0x02` | `dart_goal_set.trigger_distance_set[GOAL_BASE_STATION][0..3]` | 基地 4 个触发位置 |
| `0x03` | `dart_goal_set.yaw_angle_offset[GOAL_FRONT_STATION][0..3]` | 前哨站 4 个 Yaw 偏移 |
| `0x04` | `dart_goal_set.yaw_angle_offset[GOAL_BASE_STATION][0..3]` | 基地 4 个 Yaw 偏移 |

`ITEM=0x01` 对应数组下标 `0`，`ITEM=0x02` 对应下标 `1`，依次类推。`ITEM` 必须严格按 `1、2、3、4` 顺序出现。

### 2.3 参数范围

#### 触发位置：`CMD=0x01` 或 `0x02`

```text
-95.0 <= value <= -15.0
```

对应代码中的：

```c
MIN_TRIGGER_POS <= value <= MAX_TRIGGER_POS
```

#### Yaw 偏移：`CMD=0x03` 或 `0x04`

偏移值必须保证绝对角度处于以下范围：

```text
15.0 <= yaw_angle_set + offset <= 142.0
```

也就是：

```text
MIN_YAW_POS - yaw_angle_set
<= offset <=
MAX_YAW_POS - yaw_angle_set
```

只有 4 个参数全部解析成功且全部通过范围检查时，才会一次性写入目标数组。任意一个参数错误，整帧失败，原数组保持不变。

### 2.4 接收示例

设置前哨站 4 个触发位置：

```text
55 01 01 -57.6753 FF FF
   02 -60.2225 FF FF
   03 -60.9300 FF FF
   04 -61.8863 FF FF
```

实际发送时，数值是 ASCII 字节，不发送空格和引号。例如第一项为：

```text
55 01 01 2D 35 37 2E 36 37 35 33 FF FF
```

完整连续字节流为：

```text
55 01 01 2D 35 37 2E 36 37 35 33 FF FF
02 2D 36 30 2E 32 32 32 35 FF FF
03 2D 36 30 2E 39 33 30 30 FF FF
04 2D 36 31 2E 38 38 36 33 FF FF
```

协议没有长度字段、序号、CRC 或额外的整帧结束符。收到第 4 个参数的第二个 `0xFF` 后，MCU 即认为一帧结束。

## 3. MCU 到串口屏的命令协议

### 3.1 通用命令

调用接口：

```c
void Send_to_Screen(const char *msg);
```

函数实际发送：

```text
ASCII 命令 + 0xFF 0xFF 0xFF
```

例如：

```c
Send_to_Screen("page main");
```

实际字节为：

```text
70 61 67 65 20 6D 61 69 6E FF FF FF
```

### 3.2 调参结果应答

调参帧处理完成后，MCU 向屏幕发送：

成功：

```text
adjust_ack.val=1 FF FF FF
```

失败：

```text
adjust_ack.val=0 FF FF FF
```

串口屏页面需要存在名为 `adjust_ack` 的数值控件。

以下情况会返回失败：

- `CMD` 不在 `0x01` ~ `0x04` 范围内
- `ITEM` 缺失、重复或顺序错误
- 数值格式错误
- 缺少 `0xFF 0xFF`
- 参数超出允许范围
- 一帧没有完整收到 4 个参数

解析失败后，MCU 会重新寻找下一个 `0x55` 帧头。

### 3.3 参数数据显示

调用接口：

```c
void Send_to_Screen_Data(const float *outpost, const float *base);
```

每个数组元素会生成一条 Nextion 风格命令，数值保留 4 位小数：

```text
outpost.t3.txt="%.4f" FF FF FF
base.t3.txt="%.4f" FF FF FF
```

发送顺序为：

```text
outpost.t3
base.t3
outpost.t4
base.t4
outpost.t5
base.t5
outpost.t6
base.t6
```

例如：

```text
outpost.t3.txt="-57.6753" FF FF FF
```

当前 `application/A_Dart/dart.c` 中对 `Send_to_Screen_Data(...)` 的调用被注释掉，因此默认运行时主要发送调参成功或失败应答。

## 4. 发送队列

屏幕发送由 `Send_to_Screen.c` 管理：

- 单条发送缓冲区最大 512 字节
- DMA 队列深度为 32 条
- 多条命令按入队顺序发送
- 队列满时，新命令会被丢弃
- DMA 发送完成后自动发送下一条队列数据

## 5. 参数保存说明

接收到的调参值只写入 RAM 中的：

```c
dart_goal_set.trigger_distance_set
dart_goal_set.yaw_angle_offset
```

当前代码没有将这些参数写入 Flash。因此 MCU 复位或断电后，参数会恢复为程序中的默认值。

## 6. 上位机或串口屏发送规则

1. 串口配置使用 `115200 8N1`。
2. 每帧以单字节 `0x55` 开始。
3. `CMD` 后必须发送完整的 4 组参数。
4. `ITEM` 必须按 `0x01、0x02、0x03、0x04` 顺序发送。
5. 数值使用 ASCII 十进制格式，不要发送引号、空格或 `+` 号。
6. 每个数值后发送二进制字节 `0xFF 0xFF`。
7. 不要把字符串 `"FF"` 或 `"\\xFF"` 当作结束符发送。

