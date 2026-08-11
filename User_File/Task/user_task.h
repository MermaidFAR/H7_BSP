#ifndef __USER_TASK_H
#define __USER_TASK_H
#include "bsp_bmi088.h"
#include "bsp_key.h"
#include "bsp_usb.h" // USB_Init 声明
#include "bsp_ws2812.h"
#include "cmsis_os2.h"
#include "dvc_erictool.h"
#include <cstdint>
#include <sys/types.h>
#include "sys_debug.h"
#include "alg_pulse.h"

#ifdef __cplusplus
extern "C" {
#endif


void Ins_Task(void *argument);
void Transport_Task(void *argument);
void can_tx_task(void *argument);
void Status_Task(void *argument);


#ifdef __cplusplus
}
#endif

#endif /* __USER_TASK_H */