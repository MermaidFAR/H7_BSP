/**
 * @file heap_regions_patched.c
 * @author zzm
 * @brief FreeRTOS heap_5 memory-region definition.
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"

/* Private macros ------------------------------------------------------------*/
#ifndef H7_FREERTOS_DTCM_HEAP_SIZE
#define H7_FREERTOS_DTCM_HEAP_SIZE 49152U
#endif

#define FREERTOS_RAM_D1_HEAP_SIZE \
    (configTOTAL_HEAP_SIZE - H7_FREERTOS_DTCM_HEAP_SIZE)

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
_Static_assert(H7_FREERTOS_DTCM_HEAP_SIZE > 0U,
               "The DTCMRAM FreeRTOS heap region must not be empty");
_Static_assert(H7_FREERTOS_DTCM_HEAP_SIZE < configTOTAL_HEAP_SIZE,
               "The RAM_D1 FreeRTOS heap region must not be empty");
_Static_assert((H7_FREERTOS_DTCM_HEAP_SIZE % portBYTE_ALIGNMENT) == 0U,
               "The DTCMRAM FreeRTOS heap region must be port-aligned");
_Static_assert((FREERTOS_RAM_D1_HEAP_SIZE % portBYTE_ALIGNMENT) == 0U,
               "The RAM_D1 FreeRTOS heap region must be port-aligned");

static uint8_t FreeRTOS_Heap_DTCM[H7_FREERTOS_DTCM_HEAP_SIZE]
    __attribute__((aligned(portBYTE_ALIGNMENT)));

static uint8_t FreeRTOS_Heap_RAM_D1[FREERTOS_RAM_D1_HEAP_SIZE]
    __attribute__((section(".ram_d1_data.freertos_heap"),
                   aligned(portBYTE_ALIGNMENT)));

HeapRegion_t SYS_FreeRTOS_Heap_Regions[] = {
    {FreeRTOS_Heap_DTCM, sizeof(FreeRTOS_Heap_DTCM)},
    {FreeRTOS_Heap_RAM_D1, sizeof(FreeRTOS_Heap_RAM_D1)},
    {NULL, 0U},
};

/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
