#pragma once

#define LED_ACTIVITY	1
#define LED_BOOTLOADER	2

#define PWC_FCG0_MASK                                                           \
                 (PWC_FCG0_DCU8 | PWC_FCG0_DCU7 | PWC_FCG0_DCU6    |             \
                 PWC_FCG0_DCU5  | PWC_FCG0_DCU4 | PWC_FCG0_DCU3    |             \
                 PWC_FCG0_DCU2  | PWC_FCG0_DCU1 | PWC_FCG0_CRC     |             \
                 PWC_FCG0_TRNG  | PWC_FCG0_HASH | PWC_FCG0_AES     |             \
                 PWC_FCG0_MAU   | PWC_FCG0_CTC  | PWC_FCG0_AOS     |             \
                 PWC_FCG0_FCM   | PWC_FCG0_DMA2 | PWC_FCG0_DMA1    |             \
                 PWC_FCG0_KEY   | PWC_FCG0_SRAMB| PWC_FCG0_SRAM4   |             \
                 PWC_FCG0_SRAM3 | PWC_FCG0_SRAM2| PWC_FCG0_SRAM1   |             \
                 PWC_FCG0_SRAMH)

 #define PWC_FCG1_MASK                                                           \
                 (PWC_FCG1_CAN1 | PWC_FCG1_CAN2   | PWC_FCG1_ETHMAC  |           \
                 PWC_FCG1_QSPI  | PWC_FCG1_I2C1   | PWC_FCG1_I2C2    |           \
                 PWC_FCG1_I2C3  | PWC_FCG1_I2C4   | PWC_FCG1_I2C5    |           \
                 PWC_FCG1_I2C6  | PWC_FCG1_SDIOC1 | PWC_FCG1_SDIOC2  |           \
                 PWC_FCG1_I2S1  | PWC_FCG1_I2S2   | PWC_FCG1_I2S3    |           \
                 PWC_FCG1_I2S4  | PWC_FCG1_SPI1   | PWC_FCG1_SPI2    |           \
                 PWC_FCG1_SPI3  | PWC_FCG1_SPI4   | PWC_FCG1_SPI5    |           \
                 PWC_FCG1_SPI6  | PWC_FCG1_USBFS  | PWC_FCG1_USBHS   |           \
                 PWC_FCG1_FMAC1 | PWC_FCG1_FMAC2  | PWC_FCG1_FMAC3   |           \
                 PWC_FCG1_FMAC4)
 #define PWC_FCG2_MASK                                                           \
                 (PWC_FCG2_TMR6_1| PWC_FCG2_TMR6_2 | PWC_FCG2_TMR6_3 |           \
                 PWC_FCG2_TMR6_4 | PWC_FCG2_TMR6_5 | PWC_FCG2_TMR6_6 |           \
                 PWC_FCG2_TMR6_7 | PWC_FCG2_TMR6_8 | PWC_FCG2_TMR4_1 |           \
                 PWC_FCG2_TMR4_2 | PWC_FCG2_TMR4_3 | PWC_FCG2_HRPWM  |           \
                 PWC_FCG2_TMR0_1 | PWC_FCG2_TMR0_2 | PWC_FCG2_EMB    |           \
                 PWC_FCG2_TMR2_1 | PWC_FCG2_TMR2_2 | PWC_FCG2_TMR2_3 |           \
                 PWC_FCG2_TMR2_4 | PWC_FCG2_TMRA_1 | PWC_FCG2_TMRA_2 |           \
                 PWC_FCG2_TMRA_3 | PWC_FCG2_TMRA_4 | PWC_FCG2_TMRA_5 |           \
                 PWC_FCG2_TMRA_6 | PWC_FCG2_TMRA_7 | PWC_FCG2_TMRA_8 |           \
                 PWC_FCG2_TMRA_9 | PWC_FCG2_TMRA_10| PWC_FCG2_TMRA_11|           \
                 PWC_FCG2_TMRA_12)
 #define PWC_FCG3_MASK                                                           \
                 (PWC_FCG3_ADC1  | PWC_FCG3_ADC2   | PWC_FCG3_ADC3   |           \
                 PWC_FCG3_DAC1   | PWC_FCG3_DAC2   | PWC_FCG3_CMP1   |           \
                 PWC_FCG3_CMP2   | PWC_FCG3_OTS    | PWC_FCG3_DVP    |           \
                 PWC_FCG3_SMC    | PWC_FCG3_DMC    | PWC_FCG3_NFC    |           \
                 PWC_FCG3_USART1 | PWC_FCG3_USART2 | PWC_FCG3_USART3 |           \
                 PWC_FCG3_USART4 | PWC_FCG3_USART5 | PWC_FCG3_USART6 |           \
                 PWC_FCG3_USART7 | PWC_FCG3_USART8 | PWC_FCG3_USART9 |           \
                 PWC_FCG3_USART10| PWC_FCG3_CMBIAS)

/* board info forwarded from board-specific code to booloader */
struct boardinfo {
    uint32_t	board_type;
    uint32_t	board_rev;
    uint32_t	fw_size;
    uint32_t    extf_size;
} __attribute__((packed));

extern struct boardinfo board_info;

void init_uarts(void);
int16_t cin(unsigned timeout_ms);
int cin_word(uint32_t *wp, unsigned timeout_ms);
void cout(uint8_t *data, uint32_t len);
void port_setbaud(uint32_t baudrate);

void flash_init();

uint32_t flash_func_read_word(uint32_t offset);
bool flash_func_write_word(uint32_t offset, uint32_t v);
bool flash_func_write_words(uint32_t offset, uint32_t *v, uint8_t n);
uint32_t flash_func_sector_size(uint32_t sector);
bool flash_func_erase_sector(uint32_t sector);
uint32_t flash_func_read_otp(uint32_t idx);
uint32_t flash_func_read_sn(uint32_t idx);
void flash_set_keep_unlocked(bool);
void lock_bl_port(void);

bool flash_write_flush(void);
bool flash_write_buffer(uint32_t address, const uint32_t *v, uint8_t nwords);

uint32_t get_mcu_id(void);
uint32_t get_mcu_desc(uint32_t len, uint8_t *buf);
bool check_limit_flash_1M(void);

uint32_t board_get_rtc_signature(void);
void board_set_rtc_signature(uint32_t sig);

void led_on(unsigned led);
void led_off(unsigned led);
void led_toggle(unsigned led);

// printf to debug uart (or USB)
extern "C" {
void uprintf(const char *fmt, ...) FMT_PRINTF(1,2);
}

// generate a LED sequence forever
void led_pulses(uint8_t npulses);

typedef struct mcu_des_t {
    uint32_t mcuid;
    const char *desc;
    char  rev;
} mcu_des_t;

/*typedef struct mcu_rev_t {
    uint16_t revid;
    char  rev;
    bool limit_flash_size_1M;
} mcu_rev_t;*/
