#ifndef _REMOTE_H_
#define _REMOTE_H_


#include "../A_Dart/struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* DBUS 三段拨杆编码；注意数组下标与物理左右拨杆映射。 */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
#define RC_s_R 0
#define RC_s_L 1
/* 键盘按键位定义：每一位对应遥控器透传键盘数据中的一个按键。 */
#define KEY_W            ((uint16_t)1 << 0)
#define KEY_S            ((uint16_t)1 << 1)
#define KEY_A            ((uint16_t)1 << 2)
#define KEY_D            ((uint16_t)1 << 3)
#define KEY_SHIFT        ((uint16_t)1 << 4)
#define KEY_CTRL         ((uint16_t)1 << 5)
#define KEY_Q            ((uint16_t)1 << 6)
#define KEY_E            ((uint16_t)1 << 7)
#define KEY_R            ((uint16_t)1 << 8)
#define KEY_F            ((uint16_t)1 << 9)
#define KEY_G            ((uint16_t)1 << 10)
#define KEY_Z            ((uint16_t)1 << 11)
#define KEY_X            ((uint16_t)1 << 12)
#define KEY_C            ((uint16_t)1 << 13)
#define KEY_V            ((uint16_t)1 << 14)
#define KEY_B            ((uint16_t)1 << 15)
/* 鼠标按键状态定义。 */
#define MOUSE_YES 1
#define MOUSE_NO 0
/*
 * 完整 18 字节 DBUS 解码结果。
 * __packed 保留历史布局；DartTask 只读取 rc.ch、rc.s 和拨轮 ch[4]。
 */
typedef  struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
        char last_s[2];
        int16_t last_ch[5];
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
    } key;

}__packed RC_ctrl_t;

/* 启动接收；由 main 在调度器启动前调用。 */
void remote_control_init(void);
/* 校验并在错误时把 rc_ctrl 置为安全值，0=有效，1=错误。 */
uint8_t RC_data_is_error(void);
/* 最近一帧有效 DBUS 数据的本地毫秒时间，用于 200 ms 离线判断。 */
extern volatile uint32_t remote_last_update_ms;
#endif
