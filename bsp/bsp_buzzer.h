#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include <stdint.h>

#include "../application/A_Dart/struct_typedef.h"
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#endif
