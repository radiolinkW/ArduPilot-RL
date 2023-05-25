/*
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
#pragma once

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

//void hc32_timer_set_input_filter(hc32_tim_t *tim, uint8_t channel, uint8_t filter_mode);
//void hc32_timer_set_channel_input(hc32_tim_t *tim, uint8_t channel, uint8_t input_source);

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
// print stack usage
void show_stack_usage(void);
#endif

// allocation functions in malloc.c    
size_t mem_available(void);
void *malloc_dma(size_t size);
void *malloc_sdcard_dma(size_t size);
void *malloc_axi_sram(size_t size);
void *malloc_fastmem(size_t size);
thread_t *thread_create_alloc(size_t size, const char *name, tprio_t prio, tfunc_t pf, void *arg);

struct memory_region {
    void *address;
    uint32_t size;
    uint32_t flags;
};
#if CH_CFG_USE_HEAP == TRUE
uint8_t malloc_get_heaps(memory_heap_t **_heaps, const struct memory_region **regions);
#endif

// flush all dcache
void memory_flush_all(void);
    
// UTC system clock handling    
void hc32_set_utc_usec(uint64_t time_utc_usec);
uint64_t hc32_get_utc_usec(void);

// hook for FAT timestamps    
uint32_t get_fattime(void);

// one-time programmable area
#if defined(FLASH_OTP_BASE)
#define OTP_BASE 0x03000000
#define OTP_SIZE 6876B
#endif

enum rtc_boot_magic {
    RTC_BOOT_OFF  = 0,
    RTC_BOOT_HOLD = 0xb0070001,
    RTC_BOOT_FAST = 0xb0070002,
    RTC_BOOT_CANBL = 0xb0080000, // ORd with 8 bit local node ID
    RTC_BOOT_FWOK = 0xb0093a26 // indicates FW ran for 30s
};
    
// see if RTC registers is setup for a fast reboot
enum rtc_boot_magic check_fast_reboot(void);

// set RTC register for a fast reboot
void set_fast_reboot(enum rtc_boot_magic v);

// enable peripheral power if needed
void peripheral_power_enable(void);

// initialise allocation subsystem
void malloc_init(void);

/*
  read mode of a pin. This allows a pin config to be read, changed and
  then written back
 */
iomode_t palReadLineMode(ioline_t line);
void palDisableLineFunc(ioline_t line);
void palEnableLineFunc(ioline_t line,uint16_t func);

enum PalPushPull {
    PAL_PUSHPULL_PULLUP=0,
    PAL_PUSHPULL_PULLDOWN=1,
};

void palLineSetPushPull(ioline_t line, enum PalPushPull pp);


// set n RTC backup registers starting at given idx
void set_rtc_backup(uint8_t idx, const uint32_t *v, uint8_t n);

// get RTC backup registers starting at given idx
void get_rtc_backup(uint8_t idx, uint32_t *v, uint8_t n);

void hc32_cacheBufferInvalidate(const void *p, size_t size);
void hc32_cacheBufferFlush(const void *p, size_t size);

#ifdef HAL_GPIO_PIN_FAULT
// printf for fault handlers
void fault_printf(const char *fmt, ...);
#endif

// halt hook for printing panic message
void system_halt_hook(void);

// hook for stack overflow
void stack_overflow(thread_t *tp);

/*
  check how much stack is free given a stack base. Assumes the fill
  byte is 0x55
 */
uint32_t stack_free(void *stack_base);

// return the start of memory region that contains the address
void* get_addr_mem_region_start_addr(void *addr);
// return the end of memory region that contains the address
void* get_addr_mem_region_end_addr(void *addr);

// return the size of crash dump
uint32_t hc32_crash_dump_size(void);
uint32_t hc32_crash_dump_addr(void);
uint32_t hc32_crash_dump_max_size(void);

typedef enum  {
    RESET = 1,
    NMI = 2,
    HardFault = 3,
    MemManage = 4,
    BusFault = 5,
    UsageFault = 6,
} FaultType;

// Record information about a fault
void save_fault_watchdog(uint16_t line, FaultType fault_type, uint32_t fault_addr, uint32_t lr);

void hc32_flash_protect_flash(bool bootloader, bool protect);
void hc32_flash_unprotect_flash(void);

// allow stack view code to show free ISR stack
extern uint32_t __main_stack_base__;
extern uint32_t __main_stack_end__;
extern uint32_t __main_thread_stack_base__;
extern uint32_t __main_thread_stack_end__;

#ifdef __cplusplus
}
#endif

