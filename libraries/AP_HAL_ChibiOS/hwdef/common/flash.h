

/*
 * Copyright (C) Siddharth Bharat Purohit 2017
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup EFM_Flag_definition EFM Flag definition
 * @{
 */
#define EFM_FLAG_OTPWERR0           (EFM_FSR_OTPWERR0)       /*!< EFM Flash0 otp Programming/erase error flag.       */
#define EFM_FLAG_PRTWERR0           (EFM_FSR_PRTWERR0)       /*!< EFM Flash0 write protect address error flag.       */
#define EFM_FLAG_PGSZERR0           (EFM_FSR_PGSZERR0)       /*!< EFM Flash0 programming size error flag.            */
#define EFM_FLAG_MISMTCH0           (EFM_FSR_MISMTCH0)       /*!< EFM Flash0 programming missing match error flag.   */
#define EFM_FLAG_OPTEND0            (EFM_FSR_OPTEND0)        /*!< EFM Flash0 end of operation flag.                  */
#define EFM_FLAG_CLOLERR0           (EFM_FSR_COLERR0)        /*!< EFM Flash0 read collide error flag.                */
#define EFM_FLAG_RDY0               (EFM_FSR_RDY0)           /*!< EFM Flash0 ready flag.                             */
#define EFM_FLAG_PRTWERR1           (EFM_FSR_PRTWERR1)       /*!< EFM Flash1 write protect address error flag.       */
#define EFM_FLAG_PGSZERR1           (EFM_FSR_PGSZERR1)       /*!< EFM Flash1 programming size error flag.            */
#define EFM_FLAG_MISMTCH1           (EFM_FSR_MISMTCH1)       /*!< EFM Flash1 programming missing match error flag.   */
#define EFM_FLAG_OPTEND1            (EFM_FSR_OPTEND1)        /*!< EFM Flash1 end of operation flag.                  */
#define EFM_FLAG_CLOLERR1           (EFM_FSR_COLERR1)        /*!< EFM Flash1 read collide error flag.                */
#define EFM_FLAG_RDY1               (EFM_FSR_RDY1)           /*!< EFM Flash1 ready flag.                             */

#define EFM_FLAG_CLR_OTPWERR0       (EFM_FSCLR_OTPWERRCLR0)  /*!< EFM Clear Flash0 otp Programming/erase error flag. */
#define EFM_FLAG_CLR_PRTWERR0       (EFM_FSCLR_PRTWERRCLR0)  /*!< EFM Clear Flash0 write protect address error flag. */
#define EFM_FLAG_CLR_PGSZERR0       (EFM_FSCLR_PGSZERRCLR0)  /*!< EFM Clear Flash0 programming size error flag.      */
#define EFM_FLAG_CLR_MISMTCH0       (EFM_FSCLR_MISMTCHCLR0)  /*!< EFM Clear Flash0 programming miss match error flag.*/
#define EFM_FLAG_CLR_OPTEND0        (EFM_FSCLR_OPTENDCLR0)   /*!< EFM Clear Flash0 end of operation flag.            */
#define EFM_FLAG_CLR_CLOLERR0       (EFM_FSCLR_COLERRCLR0)   /*!< EFM Clear Flash0 read collide error flag.          */
#define EFM_FLAG_CLR_PRTWERR1       (EFM_FSCLR_PRTWERRCLR1)  /*!< EFM Clear Flash1 write protect address error flag. */
#define EFM_FLAG_CLR_PGSZERR1       (EFM_FSCLR_PGSZERRCLR1)  /*!< EFM Clear Flash1 programming size error flag.      */
#define EFM_FLAG_CLR_MISMTCH1       (EFM_FSCLR_MISMTCHCLR1)  /*!< EFM Clear Flash1 programming miss match error flag.*/
#define EFM_FLAG_CLR_OPTEND1        (EFM_FSCLR_OPTENDCLR1)   /*!< EFM Clear Flash1 end of operation flag.            */
#define EFM_FLAG_CLR_CLOLERR1       (EFM_FSCLR_COLERRCLR1)   /*!< EFM Clear Flash1 read collide error flag.          */

/**
 * @defgroup EFM_OperateMode_Definition EFM program or erase mode definition
 * @{
 */
#define EFM_MODE_PROGRAMSINGLE      (0x1UL)                  /*!< Program single mode          */
#define EFM_MODE_PROGRAMREADBACK    (0x2UL)                  /*!< Program and read back mode   */
#define EFM_MODE_PROGRAMSEQUENCE    (0x3UL)                  /*!< Program sequence mode        */
#define EFM_MODE_ERASESECTOR        (0x4UL)                  /*!< Sector erase mode            */
#define EFM_MODE_ERASECHIP1         (0x5UL)                  /*!< A flash Chip erase mode      */
#define EFM_MODE_ERASEFULL          (0x6UL)                  /*!< Full erase mode    */
#define EFM_MODE_READONLY           (0x0UL)                  /*!< Read only mode               */

/**
 * @defgroup EFM_Wait_Cycle EFM Wait Cycle
 * @{
 */
#define EFM_WAIT_CYCLE_0                   (0U)      /*!< Don't insert read wait cycle */
#define EFM_WAIT_CYCLE_1                   (1U)      /*!< Insert 1 read wait cycle     */
#define EFM_WAIT_CYCLE_2                   (2U)      /*!< Insert 2 read wait cycles    */
#define EFM_WAIT_CYCLE_3                   (3U)      /*!< Insert 3 read wait cycles    */
#define EFM_WAIT_CYCLE_4                   (4U)      /*!< Insert 4 read wait cycles    */
#define EFM_WAIT_CYCLE_5                   (5U)      /*!< Insert 5 read wait cycles    */
#define EFM_WAIT_CYCLE_6                   (6U)      /*!< Insert 6 read wait cycles    */
#define EFM_WAIT_CYCLE_7                   (7U)      /*!< Insert 7 read wait cycles    */
#define EFM_WAIT_CYCLE_8                   (8U)      /*!< Insert 8 read wait cycles    */
#define EFM_WAIT_CYCLE_9                   (9U)      /*!< Insert 9 read wait cycles    */
#define EFM_WAIT_CYCLE_10                  (10U)     /*!< Insert 10 read wait cycles   */
#define EFM_WAIT_CYCLE_11                  (11U)     /*!< Insert 11 read wait cycles   */
#define EFM_WAIT_CYCLE_12                  (12U)     /*!< Insert 12 read wait cycles   */
#define EFM_WAIT_CYCLE_13                  (13U)     /*!< Insert 13 read wait cycles   */
#define EFM_WAIT_CYCLE_14                  (14U)     /*!< Insert 14 read wait cycles   */
#define EFM_WAIT_CYCLE_15                  (15U)     /*!< Insert 15 read wait cycles   */
/**
 * @}
 */

#define EFM_INSCACHE_ON             (EFM_FRMC_ICACHE)       /*!< Enable instruction cache function */
#define EFM_INSCACHE_OFF            (0x0UL)                 /*!< Disable instruction cache function */

#define EFM_DATACACHE_ON            (EFM_FRMC_DCACHE)       /*!< Enable data cache function */
#define EFM_DATACACHE_OFF           (0x0UL)                 /*!< Disable data cache function */

#define EFM_PREFETCH_ON             (EFM_FRMC_PREFE)        /*!< Enable prefetch function */
#define EFM_PREFETCH_OFF            (0x0UL)                 /*!< Disable prefetch function */

#define EFM_CACHERST_ON             (EFM_FRMC_CRST)         /*!< Enable data cache reset function */
#define EFM_CACHERST_OFF            (0x0UL)                 /*!< Disable data cache reset function */

#define EFM_LOWVOLREAD_ON           (EFM_FRMC_LVM)          /*!< Read of low-voltage mode */
#define EFM_LOWVOLREAD_OFF          (0x0UL)

#define EFM_BUS_BUSY                (0x0UL)                  /*!< Bus busy while flash program or erase */
#define EFM_BUS_RELEASE             (EFM_FWMC_BUSHLDCTL)     /*!< Bus release while flash program or erase */

#define EFM_FLASH0_ACT_FLASH1_ACT      (0x00000000UL)  /*!< Flash 0 and 1 activity */
#define EFM_FLASH0_STP_FLASH1_ACT      (0x00000001UL)  /*!< Flash 0 stop,Flash 1 activity */
#define EFM_FLASH0_ACT_FLASH1_STP      (0x00000002UL)  /*!< Flash 0 activity,Flash 1 stop */
#define EFM_FLASH0_STP_FLASH1_STP      (0x00000003UL)  /*!< Flash 0 and 1 stop */

#define SECTOR_SIZE                  (0x2000UL)
#define REG_LENGTH                   (32U)
#define EFM_ADDR_SECTOR0            (0x00000000UL)   /*!< Sector 0 */

#define CHECKADDR  	0x2c000
#define CHECKWORD 	0x88361717

uint32_t hc32_getUID0(void);
uint32_t hc32_getUID1(void);
uint32_t hc32_getUID2(void);
uint32_t hc32_flash_read(uint32_t addr);
uint32_t hc32_flash_getpageaddr(uint32_t page);
uint32_t hc32_flash_getpagesize(uint32_t page);
uint32_t hc32_flash_getnumpages(void);
bool hc32_flash_erasepage(uint32_t page);
bool hc32_flash_write(uint32_t addr, const void *buf, uint32_t count);
void hc32_flash_keep_unlocked(bool set);
bool hc32_flash_ispageerased(uint32_t page);
void hc32_flash_config(void);
void hc32_flash_sectors_unlock(uint32_t StartAddr, uint16_t SectorCnt, bool lock);
#ifndef HAL_BOOTLOADER_BUILD
bool hc32_flash_recent_erase(void);
#endif
#ifdef __cplusplus
}
#endif
