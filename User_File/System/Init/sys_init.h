

#ifndef SYS_INIT_H
#define SYS_INIT_H

#ifdef __cplusplus
extern "C"
{
#endif

    enum Enum_System_Init_Error
    {
        SYSTEM_INIT_ERROR_NONE = 0,
        SYSTEM_INIT_ERROR_BMI088 = 1 << 0,
        SYSTEM_INIT_ERROR_FLASH = 1 << 1,
        SYSTEM_INIT_ERROR_ADC1 = 1 << 2,
    };

    void System_Init(void);
    // 初始化流程结束不等于所有设备正常；位掩码用于上层诊断。
    unsigned int System_Get_Init_Errors(void);

#ifdef __cplusplus
}
#endif

#endif /* SYS_INIT_H */
