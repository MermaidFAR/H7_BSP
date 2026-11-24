

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

    /** @brief 在 RTOS 内核初始化前配置板载驱动并放行应用回调。 @author zzm */
    void System_Init(void);
    /** @brief 返回板载设备初始化失败的位掩码，供健康任务诊断。 @author zzm */
    unsigned int System_Get_Init_Errors(void);

#ifdef __cplusplus
}
#endif

#endif /* SYS_INIT_H */
