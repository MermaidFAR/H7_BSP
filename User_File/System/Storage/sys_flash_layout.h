/**
 * @file sys_flash_layout.h
 * @author zzm
 * @brief Central W25Q64 address allocation.
 */

#ifndef __SYS_FLASH_LAYOUT_H
#define __SYS_FLASH_LAYOUT_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

namespace Namespace_SYS_Flash_Layout
{
constexpr uint32_t W25Q64_SIZE = 0x00800000U;
constexpr uint32_t W25Q64_SECTOR_SIZE = 0x00001000U;

// The last two sectors are exclusively owned by the IMU bias A/B journal.
constexpr uint32_t IMU_BIAS_SLOT_A_ADDRESS = 0x007fe000U;
constexpr uint32_t IMU_BIAS_SLOT_B_ADDRESS = 0x007ff000U;

// Other persistent users must stay below this exclusive region.
constexpr uint32_t GENERAL_STORAGE_END_ADDRESS = IMU_BIAS_SLOT_A_ADDRESS;

static_assert((IMU_BIAS_SLOT_A_ADDRESS % W25Q64_SECTOR_SIZE) == 0U,
              "IMU bias slot A must be sector aligned");
static_assert((IMU_BIAS_SLOT_B_ADDRESS % W25Q64_SECTOR_SIZE) == 0U,
              "IMU bias slot B must be sector aligned");
static_assert((IMU_BIAS_SLOT_B_ADDRESS + W25Q64_SECTOR_SIZE) == W25Q64_SIZE,
              "IMU bias slots must end at the flash boundary");
}

#endif /* __SYS_FLASH_LAYOUT_H */
