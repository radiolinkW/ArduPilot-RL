#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/*
  setup the watchdog
 */
void hc32_watchdog_init(void);

/*
  pat the dog, to prevent a reset. If not called for 1s
  after hc32_watchdog_init() then MCU will reset
 */
void hc32_watchdog_pat(void);

/*
  return true if reboot was from a watchdog reset
 */
bool hc32_was_watchdog_reset(void);

/*
  save the reset reason code
 */
void hc32_watchdog_save_reason(void);

/*
  clear reset reason code
 */
void hc32_watchdog_clear_reason(void);

/*
  save persistent watchdog data
 */
void hc32_watchdog_save(const uint32_t *data, uint32_t nwords);

/*
  load persistent watchdog data
 */
void hc32_watchdog_load(uint32_t *data, uint32_t nwords);
    
#ifdef __cplusplus
}
#endif
    
