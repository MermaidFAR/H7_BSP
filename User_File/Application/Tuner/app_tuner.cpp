/**
 * @file app_tuner.cpp
 * @author zzm
 * @brief PID 在线调参模块实现 (appTuner)
 * @version 2.0
 * @date 2026-06-06 2.0 多实例 / 独立 Flash 任务 / Erase 前 Set_Write_Enable
 *
 * @details
 * EricTool 下行指令字典 (11 项):
 *   P / I / D / F / OutMax / IOutMax / Aspeed / Bspeed / SepThr / DFirst / Target
 *
 * Now 不在字典中, 由电机等外部代码直接写 PID_Tuner.instance[i].Now。
 * D_T 可变不可调, 不在字典中。
 * Out / I_Out 仅 TX 遥测, 不接收下行。
 *
 * Flash 写入由 Flash_Task 独立执行, EricTool RX 只操作 RAM 并标记 dirty。
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#include "app_tuner.h"

#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 全局 PID Tuner 管理器 */
PID_Tuner_Manager_t PID_Tuner;

/** @brief EricTool 下行指令数量 */
static const uint8_t RX_DICT_NUM = 11;

/** @brief USB CDC 接收完成标记 (ISR 置位, App_Tuner_RX 清零) */
static volatile bool eric_rx_pending = false;

/**
 * @brief EricTool 下行指令字典
 *
 * @note  索引 0~8:  Flash 持久字段
 * @note  索引 9:    D_First (float 0/1 → DISABLE/ENABLE)
 * @note  索引 10:   Target (只写 RAM, 不标 dirty)
 * @note  Now 不在字典中, 由外部写入
 */
static char EricTool_RX_Dict[RX_DICT_NUM][ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    "P",        ///< 0  → K_P
    "I",        ///< 1  → K_I
    "D",        ///< 2  → K_D
    "F",        ///< 3  → K_F
    "OutMax",   ///< 4  → Out_Max
    "IOutMax",  ///< 5  → I_Out_Max
    "Aspeed",   ///< 6  → I_Variable_Speed_A
    "Bspeed",   ///< 7  → I_Variable_Speed_B
    "SepThr",   ///< 8  → I_Separate_Threshold
    "DFirst",   ///< 9  → D_First
    "Target"    ///< 10 → Target
};

/**
 * @brief 将 Flash 参数镜像推入指定实例的 PID 对象
 *
 * @param inst 目标实例指针
 */
static void Push_Flash_Params_To_PID(PID_Tuner_Instance_t *inst) {
    if (!inst->pid) return;
    inst->pid->Set_K_P(inst->flash.K_P);
    inst->pid->Set_K_I(inst->flash.K_I);
    inst->pid->Set_K_D(inst->flash.K_D);
    inst->pid->Set_K_F(inst->flash.K_F);
    inst->pid->Set_Out_Max(inst->flash.Out_Max);
    inst->pid->Set_I_Out_Max(inst->flash.I_Out_Max);
    inst->pid->Set_I_Variable_Speed_A(inst->flash.I_Variable_Speed_A);
    inst->pid->Set_I_Variable_Speed_B(inst->flash.I_Variable_Speed_B);
    inst->pid->Set_I_Separate_Threshold(inst->flash.I_Separate_Threshold);
}

/**
 * @brief 写入默认 PID 参数到 Flash (首次上电)
 *
 * @param inst 目标实例指针
 * @note  流程: Set_Write_Enable → Erase Sector → Write → 更新 RAM 镜像
 */
static void Write_Flash_Defaults(PID_Tuner_Instance_t *inst) {
    Flash_PID_Params_t defaults = {
        .K_P = 1.0f,
        .K_I = 0.0f,
        .K_D = 0.0f,
        .K_F = 0.0f,
        .I_Out_Max = 0.0f,
        .Out_Max = 0.0f,
        .I_Variable_Speed_A = 0.0f,
        .I_Variable_Speed_B = 0.0f,
        .I_Separate_Threshold = 0.0f,
        .D_First = PID_D_First_DISABLE,
        .D_T = 0.001f
    };

    BSP_W25Q64JV.Set_Write_Enable();
    while (!BSP_W25Q64JV.Is_Ready()) {
        osDelay(1);
    }

    BSP_W25Q64JV.Set_Sector_Erased(inst->flash_addr);
    while (!BSP_W25Q64JV.Is_Ready()) {
        osDelay(1);
    }

    BSP_W25Q64JV.Write_Data(&defaults, inst->flash_addr, sizeof(Flash_PID_Params_t));
    while (!BSP_W25Q64JV.Is_Ready()) {
        osDelay(1);
    }

    inst->flash = defaults;
}

/**
 * @brief 提交单个实例的 dirty 数据到 Flash
 *
 * @param inst 目标实例指针
 * @note  需在独立 Flash 任务中调用, 不在 ISR 或 1ms 周期内调用
 */
static void Commit_Flash(PID_Tuner_Instance_t *inst) {
    if (!inst->dirty) return;
    if (!inst->pid) return;
    if (!BSP_W25Q64JV.Is_Ready()) return;

    BSP_W25Q64JV.Set_Write_Enable();
    while (!BSP_W25Q64JV.Is_Ready()) {
        osDelay(1);
    }

    BSP_W25Q64JV.Set_Sector_Erased(inst->flash_addr);
    while (!BSP_W25Q64JV.Is_Ready()) {
        osDelay(1);
    }

    BSP_W25Q64JV.Write_Data(&inst->flash, inst->flash_addr, sizeof(Flash_PID_Params_t));
    while (!BSP_W25Q64JV.Is_Ready()) {
        osDelay(1);
    }

    inst->dirty = false;
}

void App_Tuner_Register_PID(uint8_t index, Class_PID *pid, uint32_t flash_addr) {
    if (index >= PID_TUNER_MAX) return;

    PID_Tuner.instance[index].pid = pid;
    PID_Tuner.instance[index].flash_addr = flash_addr;

    if (index >= PID_Tuner.count) {
        PID_Tuner.count = index + 1;
    }
}

void App_Tuner_Init(void) {
    USB_Init([](uint8_t *Buffer, uint16_t Length) {
        EricTool_USB.USB_RxCallback(Buffer, Length);
        eric_rx_pending = true;
    });
    EricTool_USB.Init(RX_DICT_NUM, (const char **)EricTool_RX_Dict);

    for (uint8_t i = 0; i < PID_Tuner.count; i++) {
        PID_Tuner_Instance_t *inst = &PID_Tuner.instance[i];

        if (!inst->pid) continue;

        BSP_W25Q64JV.Read_Data(&inst->flash, inst->flash_addr, sizeof(Flash_PID_Params_t));

        if (std::isnan(inst->flash.K_P) || std::isnan(inst->flash.D_T)) {
            Write_Flash_Defaults(inst);
        }

        Push_Flash_Params_To_PID(inst);
    }
}

void App_Tuner_TX(void) {
    PID_Tuner_Instance_t *a = &PID_Tuner.instance[PID_Tuner.active_index];

    EricTool_USB.Set_Data(13,
        (int)&a->flash.K_P,
        (int)&a->flash.K_I,
        (int)&a->flash.K_D,
        (int)&a->flash.K_F,
        (int)&a->flash.I_Out_Max,
        (int)&a->flash.Out_Max,
        (int)&a->flash.I_Variable_Speed_A,
        (int)&a->flash.I_Variable_Speed_B,
        (int)&a->flash.I_Separate_Threshold,
        (int)&a->Target,
        (int)&a->Now,
        (int)&a->Out,
        (int)&a->I_Out
    );
}

void App_Tuner_RX(void) {
    if (!eric_rx_pending) return;
    eric_rx_pending = false;

    int32_t idx = EricTool_USB.Get_Variable_Index();
    if (idx < 0) return;

    float val = EricTool_USB.Get_Variable_Value();
    PID_Tuner_Instance_t *a = &PID_Tuner.instance[PID_Tuner.active_index];

    switch (idx) {
        case 0:     ///< P
            a->flash.K_P = val;
            if (a->pid) a->pid->Set_K_P(val);
            a->dirty = true;
            break;
        case 1:     ///< I
            a->flash.K_I = val;
            if (a->pid) a->pid->Set_K_I(val);
            a->dirty = true;
            break;
        case 2:     ///< D
            a->flash.K_D = val;
            if (a->pid) a->pid->Set_K_D(val);
            a->dirty = true;
            break;
        case 3:     ///< F
            a->flash.K_F = val;
            if (a->pid) a->pid->Set_K_F(val);
            a->dirty = true;
            break;
        case 4:     ///< OutMax
            a->flash.Out_Max = val;
            if (a->pid) a->pid->Set_Out_Max(val);
            a->dirty = true;
            break;
        case 5:     ///< IOutMax
            a->flash.I_Out_Max = val;
            if (a->pid) a->pid->Set_I_Out_Max(val);
            a->dirty = true;
            break;
        case 6:     ///< Aspeed
            a->flash.I_Variable_Speed_A = val;
            if (a->pid) a->pid->Set_I_Variable_Speed_A(val);
            a->dirty = true;
            break;
        case 7:     ///< Bspeed
            a->flash.I_Variable_Speed_B = val;
            if (a->pid) a->pid->Set_I_Variable_Speed_B(val);
            a->dirty = true;
            break;
        case 8:     ///< SepThr
            a->flash.I_Separate_Threshold = val;
            if (a->pid) a->pid->Set_I_Separate_Threshold(val);
            a->dirty = true;
            break;
        case 9:     ///< DFirst (float 0/1 → enum)
            a->flash.D_First = (val != 0.0f) ? PID_D_First_ENABLE : PID_D_First_DISABLE;
            a->dirty = true;
            break;
        case 10:    ///< Target (RAM only, 不标 dirty)
            a->Target = val;
            if (a->pid) a->pid->Set_Target(val);
            break;
        default:
            break;
    }
}

void App_Tuner_Flash_Task(void) {
    for (;;) {
        for (uint8_t i = 0; i < PID_Tuner.count; i++) {
            Commit_Flash(&PID_Tuner.instance[i]);
        }
        osDelay(100);
    }
}

#ifdef __cplusplus
}
#endif
