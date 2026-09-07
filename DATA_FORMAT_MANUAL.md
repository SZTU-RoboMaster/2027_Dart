# 飞镖车调参板与屏幕数据格式手册

本文档根据以下实现整理：

- `application/A_Dart/Adjust_Board.c`
- `application/A_Dart/Send_to_Screen.c`
- `application/A_Dart/dart.h`
- `Core/Src/usart.c`

## 1. 接口总览

| 用途 | 固件接口 | 物理引脚 | 串口参数 | 数据方向 |
|---|---|---|---|---|
| 调参板/屏幕 | UART7 | PE7 RX、PE8 TX | 115200 baud，8N1，无硬件流控 | 调参板 -> MCU；MCU -> 屏幕 |

调参接收缓存为 128 字节，使用 UART7 的 HAL Receive-to-Idle 中断接收方式。接收回调会逐字节交给持久化解析器，因此一次回调可以包含多帧，也可以只包含半帧；未完成的帧会保留到下一次回调。屏幕命令使用 Nextion 风格的 ASCII 指令，命令末尾固定追加三个 `0xFF`。

## 2. 调参板 -> MCU 协议

### 2.1 帧结构

一帧由 2 字节帧头和 4 个按顺序排列的参数项组成：

```text
55 CMD ITEM_1 VALUE_1 FF FF ITEM_2 VALUE_2 FF FF
              ...
              ITEM_4 VALUE_4 FF FF
```

| 字段 | 长度 | 取值/格式 | 说明 |
|---|---:|---|---|
| SOF | 1 | `0x55` | 固定帧头 |
| CMD | 1 | `0x01`、`0x02`、`0x03` 或 `0x04` | 参数类型和目标站点，见下表 |
| ITEM_n | 1 | `0x01`、`0x02`、`0x03`、`0x04` | 参数编号，必须按 1 到 4 顺序出现 |
| VALUE_n | 变长 | ASCII 数字，可带前导 `-`，可含一个 `.` | 参数值，不是 IEEE-754 二进制浮点 |
| VALUE_END | 2 | `0xFF 0xFF` | 单个数值结束符 |

当前代码没有长度字段、序号、CRC 或额外的整帧结束符。一个 CMD 后完整收到 4 个 `ITEM + VALUE + FF FF` 后，即判定一帧结束；下一字节可以立即是下一帧的 `0x55`。Receive-to-Idle 每次最多提供 128 字节，但帧可以跨越多个接收回调。

### 2.2 CMD 与数组映射

| CMD | 目标 | 固件数组 | 参数含义 |
|---|---|---|---|
| `0x01` | 前哨站（outpost） | `dart_goal_set.trigger_distance_set[1][0..3]` | 4 个发射触发位置 |
| `0x02` | 基地（base） | `dart_goal_set.trigger_distance_set[2][0..3]` | 4 个发射触发位置 |
| `0x03` | 前哨站（outpost） | `dart_goal_set.yaw_angle_offset[1][0..3]` | 4 个 yaw 角度偏移 |
| `0x04` | 基地（base） | `dart_goal_set.yaw_angle_offset[2][0..3]` | 4 个 yaw 角度偏移 |

`ITEM=0x01` 写入数组下标 `0`，`ITEM=0x02` 写入下标 `1`，依此类推。代码只接受严格的 `1,2,3,4` 顺序；编号缺失或乱序时不会跳过错误项。

### 2.3 数值编码

解析函数 `Str_to_float` 的实际支持范围是：

```text
-?[0-9]+(\.[0-9]*)?
```

示例：`57`、`-57.6753`、`0.5`、`-0.5`。

负号只能作为数值的第一个字符，编码为 ASCII `'-'`（`0x2D`）。当前解析器不支持前导正号 `+`、指数表示法或空格。

### 2.4 示例帧

以下帧把前哨站四个参数设置为 `-57.6753`、`-60.2225`、`-60.9300`、`-61.8863`：

```text
55 01 01 -57.6753 FF FF 02 -60.2225 FF FF
   03 -60.9300 FF FF 04 -61.8863 FF FF
```

其中数值字符是 ASCII 字节，例如 `-` 为 `0x2D`，`5` 为 `0x35`，`.` 为 `0x2E`，`0xFF 0xFF` 为二进制结束符。对应的连续字节序列可写为：

```text
55 01 01 2D 35 37 2E 36 37 35 33 FF FF
02 2D 36 30 2E 32 32 32 35 FF FF
03 2D 36 30 2E 39 33 30 30 FF FF
04 2D 36 31 2E 38 38 36 33 FF FF
```

例如，前哨站四个 yaw 偏移使用 `CMD=0x03`，基地四个 yaw 偏移使用 `CMD=0x04`；帧内 4 个参数的格式与触发位置完全相同：

```text
55 03 01 -0.5000 FF FF 02 0.2000 FF FF
   03 -0.3000 FF FF 04 0.1000 FF FF
```

当四项参数均合法并完整接收后，固件通过 UART7 DMA 发送一次屏幕确认命令：

```text
adjust_ack.val=1 FF FF FF
```

如果帧格式错误、CMD 不支持，或者任意一个参数超出对应机构的允许范围，四个参数均不会写入，原数组保持不变，并返回：

```text
adjust_ack.val=0 FF FF FF
```

屏幕工程需要存在名为 `adjust_ack` 的数值控件；如控件名称不同，应同步修改 `Adjust_Board.c` 中的 ACK 命令宏。

### 2.5 参数范围检查

| 参数类型 | 检查规则 | 当前有效范围 |
|---|---|---|
| trigger 位置（CMD `0x01`、`0x02`） | 每个接收值直接按 trigger 电机绝对位置检查 | `MIN_TRIGGER_POS <= value <= MAX_TRIGGER_POS`，当前为 `-95.0` 到 `-15.0` |
| yaw 偏移（CMD `0x03`、`0x04`） | 检查 `yaw_angle_set[目标] + offset` 是否处于 yaw 绝对位置限位内 | `MIN_YAW_POS <= yaw_angle_set + offset <= MAX_YAW_POS`，当前绝对位置限位为 `15.0` 到 `142.0` |

四项数据使用“全部成功才写入”的方式处理。只要其中一项越界，整帧失败，不会只更新其中一部分。

### 2.6 参数生命周期

调参帧解析成功后，参数只更新到 RAM 中的 `dart_goal_set.trigger_distance_set` 或 `dart_goal_set.yaw_angle_offset`，不写入 A 板 Flash。当前代码只负责接收和存储 yaw 偏移，尚未把它叠加到云台控制目标。MCU 复位或断电后，本次调参值会丢失，并重新使用固件初始化时设置的默认值（`yaw_angle_offset` 未显式初始化时为 0）。

## 3. MCU -> 屏幕协议

### 3.1 通用命令 `Send_to_Screen`

调用接口：

```c
void Send_to_Screen(const char *msg);
```

调用方传入不带结束符的 ASCII 命令，函数实际发送：

```text
msg + FF FF FF
```

例如：

```c
Send_to_Screen("page main");
```

线上字节为：

```text
70 61 67 65 20 6D 61 69 6E FF FF FF
```

函数通过 UART7 DMA 发送，内部活动/待发缓冲区各为 512 字节。

### 3.2 参数显示 `Send_to_Screen_Data`

调用接口：

```c
void Send_to_Screen_Data(const float *outpost, const float *base);
```

`outpost` 和 `base` 均必须至少包含 4 个 `float`。函数将每一组数据写入屏幕控件 `t3` 到 `t6`，每个控件分别发送一条独立命令：

| 数组下标 | 屏幕控件 | 命令模板 |
|---:|---|---|
| 0 | `outpost.t3` | `outpost.t3.txt="%.4f"` |
| 0 | `base.t3` | `base.t3.txt="%.4f"` |
| 1 | `outpost.t4` | `outpost.t4.txt="%.4f"` |
| 1 | `base.t4` | `base.t4.txt="%.4f"` |
| 2 | `outpost.t5` | `outpost.t5.txt="%.4f"` |
| 2 | `base.t5` | `base.t5.txt="%.4f"` |
| 3 | `outpost.t6` | `outpost.t6.txt="%.4f"` |
| 3 | `base.t6` | `base.t6.txt="%.4f"` |

每条命令末尾均为 `0xFF 0xFF 0xFF`，数值固定输出 4 位小数。例如 `outpost[0] = 57.6753` 时发送：

```text
outpost.t3.txt="57.6753" FF FF FF
```

函数发送顺序为：

```text
outpost.t3 -> base.t3 -> outpost.t4 -> base.t4
-> outpost.t5 -> base.t5 -> outpost.t6 -> base.t6
```

该函数将 8 条命令组装后通过 UART7 DMA 一次发送，不阻塞调用任务。所有屏幕发送命令共享一个 32 槽环形 DMA 队列，发送完成后自动发送下一槽。

## 4. 当前实现必须注意的问题

1. **UART7 共用注意事项**：调参接收和屏幕发送共用 UART7 的 RX/TX 引脚，硬件必须保证 PE7/PE8 上的设备连接方式正确；UART7 是全双工接口，收发可同时进行。
2. **发送缓存和并发**：屏幕发送使用 32 个静态 DMA 缓冲槽，发送中的槽不会被覆盖；多个 ACK 会按产生顺序排队发送。队列满时，新命令会被丢弃，因此发送速率长期低于接收速率时仍需控制发送频率。
3. **连续帧接收**：一次接收回调中可以放置任意多个完整帧；半帧会跨回调保留。每解析完成一帧就独立校验、更新并返回一个 ACK。
4. **异常帧保护**：参数编号错误、缺少 `FF FF`、数值格式非法、参数越界或帧内容中断时，当前帧不会更新目标数组，并返回 `adjust_ack.val=0`；解析器随后重新寻找下一个 `0x55` 帧头。
5. **调参值不持久化**：参数仅保存在 RAM 中，复位或断电后会恢复为固件默认值。
6. **ACK 控件名称**：成功标志默认发送到 `adjust_ack.val`；屏幕页面必须使用相同控件名，或修改 ACK 宏。

## 5. 推荐的上位机实现规则

- 发送前将参数格式化为 ASCII 十进制，使用 `.` 作为小数点；负数使用一个前导 `-`，不要添加空格或 `+`。
- 每帧固定包含一个 CMD 和完整的 4 项参数，ITEM 严格为 `01 02 03 04`。
- 每个参数后追加二进制 `FF FF`，不要把字符 `"FF"` 或 `"\\xFF"` 当作文本发送。
- 接收缓存单次最多处理 128 字节；帧可以跨越多个接收回调，多个帧也可以连续发送。
- 若需要支持指数格式、校验或断电保存，应先扩展固件协议，再同步更新本手册。
