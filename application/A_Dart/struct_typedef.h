#ifndef _STRUCT_TYPEDEF_H
#define _STRUCT_TYPEDEF_H

#include "main.h"

/*
 * 历史模块使用的基础数值别名。
 * 新代码优先使用 stdbool.h 的 bool 与 stdint.h 的定宽整数；保留这些名称是为了兼容
 * 裁判、PID 和旧通信结构，避免一次性修改第三方协议代码的二进制布局。
 */
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


#endif



