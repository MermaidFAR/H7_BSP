#ifndef USER_TASK_H
#define USER_TASK_H
#include "alg_pulse.h"
#include "bsp_bmi088.h"
#include "bsp_key.h"
#include "bsp_usb.h" // USB_Init 声明
#include "bsp_ws2812.h"
#include "cmsis_os2.h"
#include "erictool.h"
#include <cstdint>
#include <sys/types.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void Ins_Task(void *argument);
    void Transport_Task(void *argument);
    void can_tx_task(void *argument);
    void Status_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* USER_TASK_H */