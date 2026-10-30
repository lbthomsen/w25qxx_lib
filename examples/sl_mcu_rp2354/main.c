/**
 * @file main.c
 * @brief Overclocking RP2350 & W25Qxx Flash Library Test
 * @author STM32World <lth@stm32world.com>
 * @date 2026
 *
 * Copyright (c) 2026 STM32World <lth@stm32world.com>
 *
 * Trying to push the RP2350 to its limits with a simple LED blink, tick,
 * automatic ADC temperature reader, and full W25Qxx SPI flash test suite.
 *
 */

// Include necessary headers from the Pico SDK

#include "hardware/adc.h"    // For ADC access (if needed)
#include "hardware/clocks.h" // For clock frequency information
#include "hardware/dma.h"    // For DMA access (if needed)
#include "hardware/gpio.h"   // For GPIO control
#include "hardware/spi.h"    // For SPI bus access
#include "hardware/timer.h"  // Required for hardware timer access
#include "hardware/vreg.h"   // Needed for voltage scaling
#include "pico/bootrom.h"    // For flash command execution
#include "pico/multicore.h"  // For multicore support
#include "pico/stdlib.h"     // For sleep and stdio initialization

// Include standard I/O for printf
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "w25qxx.h"

#ifndef LED_DELAY
#define LED_DELAY 500 // 500ms
#endif

#ifndef TICK_DELAY
#define TICK_DELAY 1000 // 1000ms = 1 second
#endif

#define PAGE_SIZE 4096

// Pin definitions for W25Qxx SPI bus on RP2350
#define FLASH_SPI spi1
#define FLASH_PIN_MISO 40
#define FLASH_PIN_CS 27
#define FLASH_PIN_SCK 42
#define FLASH_PIN_MOSI 43

volatile uint16_t internal_temp_raw = 0;
volatile float internal_temp_c = 0.0f;

// Volatile variable to mimic STM32's uwTick
static volatile uint32_t systick = 0;

W25QXX_HandleTypeDef w25qxx; // Handler for all w25qxx operations!

// Static buffer in RAM
static uint8_t buf[PAGE_SIZE];

/**
 * @brief Software CRC32 implementation matching standard STM32 HAL Hardware CRC (Poly: 0x04C11DB7)
 */
static uint32_t HAL_CRC_Calculate(const uint32_t *pBuffer, uint32_t BufferLength) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < BufferLength; i++) {
        uint32_t data = pBuffer[i];
        crc ^= data;
        for (uint8_t bit = 0; bit < 32; bit++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ 0x04C11DB7;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Dump hex to serial console
 */
void dump_hex(char *header, uint32_t start, uint8_t *buf, uint32_t len) {
    uint32_t i = 0;

    printf("%s\n", header);

    for (i = 0; i < len; ++i) {

        if (i % 16 == 0) {
            printf("0x%08lx: ", start);
        }

        printf("%02x ", buf[i]);

        if ((i + 1) % 16 == 0) {
            printf("\n");
        }

        ++start;
    }
}

void fill_buffer(uint8_t pattern, uint8_t *buf, uint32_t len) {
    switch (pattern) {
    case 0:
        memset(buf, 0, len);
        break;
    case 1:
        memset(buf, 0xaa, len); // 10101010
        break;
    case 2:
        for (uint32_t i = 0; i < len; ++i)
            buf[i] = i % 256;
        break;
    default:
        printf("Programmer is a moron\n");
    }
}

uint8_t check_buffer(uint8_t pattern, uint8_t *buf, uint32_t len) {

    uint8_t ret = 1;

    switch (pattern) {
    case 0:
        for (uint32_t i = 0; i < len; ++i) {
            if (buf[i] != 0)
                ret = 0;
        }
        break;
    case 1:
        for (uint32_t i = 0; i < len; ++i) {
            if (buf[i] != 0xaa)
                ret = 0;
        }
        break;
    case 2:
        for (uint32_t i = 0; i < len; ++i) {
            if (buf[i] != i % 256)
                ret = 0;
        }
        break;
    default:
        printf("Programmer is a moron\n");
    }

    return ret;
}

uint32_t get_sum(uint8_t *buf, uint32_t len) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; ++i) {
        sum += buf[i];
    }
    return sum;
}

/**
 * @brief Callback for the repeating timer.
 * Works on both ARM and RISC-V.
 */
bool on_timer_tick(struct repeating_timer *t) {
    systick++;
    return true; // Keep the timer running
}

/**
 * @brief Universal tick initialization using the SDK timer pool.
 */
void universal_tick_init() {
    static struct repeating_timer timer;
    add_repeating_timer_ms(-1, on_timer_tick, NULL, &timer);
}

// Perform initialisation
int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

/**
 * @brief SPI Hardware Initialization Function
 */
static void MX_SPI1_Init(void) {
    spi_init(FLASH_SPI, 10 * 1000 * 1000); // 10 MHz SPI clock speed

    // Explicitly configure CS Pin
    gpio_init(FLASH_PIN_CS);
    gpio_set_dir(FLASH_PIN_CS, GPIO_OUT);
    gpio_put(FLASH_PIN_CS, 1);

    gpio_set_function(FLASH_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(FLASH_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(FLASH_PIN_MOSI, GPIO_FUNC_SPI);
}

/**
 * @brief Toggles the state of the default LED.
 */
void pico_toggle_led() {
    gpio_xor_mask64(((uint64_t)1 << PICO_DEFAULT_LED_PIN));
}

// This timer callback fires automatically in the background
bool repeating_timer_callback(struct repeating_timer *t) {
    adc_select_input(8);
    internal_temp_raw = adc_read();
    return true;
}

void init_automatic_temp_sensor() {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(8);

    static struct repeating_timer timer;
    add_repeating_timer_ms(-500, repeating_timer_callback, NULL, &timer);
}

/**
 * @brief Entry point for Core 1.
 */
void core1_entry() {
    printf("Core 1: Booting...\n");

    uint32_t now, loop_cnt = 0, next_tick = TICK_DELAY + (TICK_DELAY / 2);

    while (1) {
        now = systick;

        if (now >= next_tick) {
            printf("Core 1 tick %lu (loop = %lu)\n", now, loop_cnt);
            loop_cnt = 0;
            next_tick = now + TICK_DELAY;
        }

        ++loop_cnt;
        tight_loop_contents();
    }
}

/**
 * @brief Main entry point for Core 0.
 */
int main() {

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    stdio_init_all();
    uart_set_baudrate(uart0, 921600);
    sleep_ms(50);

    printf("\n\n\nCore 0: Booting...\n");
    printf("Running on %s at %d MHz\n",
#ifdef __riscv
           "RISC-V",
#else
           "Arm Cortex-M33",
#endif
           frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) / 1000);

    MX_SPI1_Init();
    universal_tick_init();
    init_automatic_temp_sensor();

    // multicore_launch_core1(core1_entry); // Disabled as it just mess up the test output on the console

    printf("\n--------\nCore and peripherals has been initialized\n");

    sleep_ms(1000);

    W25QXX_result_t res;

    res = w25qxx_init(&w25qxx, FLASH_SPI, 0, FLASH_PIN_CS);

    if (res == W25QXX_Ok) {
        printf("W25QXX successfully initialized\n");
        printf("Manufacturer       = 0x%2x\n", w25qxx.manufacturer_id);
        printf("Device             = 0x%4x\n", w25qxx.device_id);
        printf("Block size         = 0x%04lx (%lu)\n", w25qxx.block_size, w25qxx.block_size);
        printf("Block count        = 0x%04lx (%lu)\n", w25qxx.block_count, w25qxx.block_count);
        printf("Sector size        = 0x%04lx (%lu)\n", w25qxx.sector_size, w25qxx.sector_size);
        printf("Sectors per block  = 0x%04lx (%lu)\n", w25qxx.sectors_in_block, w25qxx.sectors_in_block);
        printf("Page size          = 0x%04lx (%lu)\n", w25qxx.page_size, w25qxx.page_size);
        printf("Pages per sector   = 0x%04lx (%lu)\n", w25qxx.pages_in_sector, w25qxx.pages_in_sector);
        printf("Total size (in kB) = 0x%04lx (%lu)\n", (w25qxx.block_count * w25qxx.block_size) / 1024, (w25qxx.block_count * w25qxx.block_size) / 1024);
    } else {
        printf("Unable to initialize w25qxx\n");
    }

    sleep_ms(100);

    for (uint8_t run = 0; run <= 2; ++run) {

        printf("\n-------------\nRun %d\n", run);
        printf("Reading first page\n");

        res = w25qxx_read(&w25qxx, 0, (uint8_t *)&buf, sizeof(buf));
        if (res == W25QXX_Ok) {
            dump_hex("First page at start", 0, (uint8_t *)&buf, sizeof(buf));
        } else {
            printf("Unable to read w25qxx\n");
        }

        printf("Erasing first page\n");

        if (w25qxx_erase(&w25qxx, 0, sizeof(buf)) == W25QXX_Ok) {
            printf("Reading first page\n");
            if (w25qxx_read(&w25qxx, 0, (uint8_t *)&buf, sizeof(buf)) == W25QXX_Ok) {
                dump_hex("After erase", 0, (uint8_t *)&buf, sizeof(buf));
            }
        }

        fill_buffer(run, buf, sizeof(buf));

        printf("Writing first page\n");

        if (w25qxx_write(&w25qxx, 0, (uint8_t *)&buf, sizeof(buf)) == W25QXX_Ok) {
            printf("Reading first page\n");
            if (w25qxx_read(&w25qxx, 0, (uint8_t *)&buf, sizeof(buf)) == W25QXX_Ok) {
                dump_hex("After write", 0, (uint8_t *)&buf, sizeof(buf));
            }
        }
    }

    // Stress test
    uint32_t start;
    uint32_t sectors = w25qxx.block_count * w25qxx.sectors_in_block;

    printf("Stress testing w25qxx device: sectors = %lu\n", sectors);
    printf("Doing chip erase\n");

    start = systick;
    w25qxx_chip_erase(&w25qxx);

    printf("Done erasing - took %lu ms\n", systick - start);

    fill_buffer(0, buf, sizeof(buf));

    printf("Writing all zeroes %lu sectors\n", sectors);

    start = systick;
    for (uint32_t i = 0; i < sectors; ++i) {
        w25qxx_write(&w25qxx, i * w25qxx.sector_size, buf, sizeof(buf));
    }

    printf("Done writing - took %lu ms\n", systick - start);
    printf("Reading %lu sectors\n", sectors);

    start = systick;
    for (uint32_t i = 0; i < sectors; ++i) {
        w25qxx_read(&w25qxx, i * w25qxx.sector_size, buf, sizeof(buf));
    }

    printf("Done reading - took %lu ms\n", systick - start);
    printf("Validating buffer .... ");
    if (check_buffer(0, buf, sizeof(buf))) {
        printf("OK\n");
    } else {
        printf("Not OK\n");
    }
    printf("Doing chip erase\n");

    start = systick;
    w25qxx_chip_erase(&w25qxx);

    printf("Done erasing - took %lu ms\n", systick - start);

    fill_buffer(1, buf, sizeof(buf));

    printf("Writing 10101010 %lu sectors\n", sectors);

    start = systick;
    for (uint32_t i = 0; i < sectors; ++i) {
        w25qxx_write(&w25qxx, i * w25qxx.sector_size, buf, sizeof(buf));
    }

    printf("Done writing - took %lu ms\n", systick - start);
    printf("Reading %lu sectors\n", sectors);

    start = systick;
    for (uint32_t i = 0; i < sectors; ++i) {
        w25qxx_read(&w25qxx, i * w25qxx.sector_size, buf, sizeof(buf));
    }

    printf("Done reading - took %lu ms\n", systick - start);
    printf("Validating buffer ... ");
    if (check_buffer(1, buf, sizeof(buf))) {
        printf("OK\n");
    } else {
        printf("Not OK\n");
    }
    printf("Erasing %lu sectors sequentially\n", sectors);

    start = systick;
    for (uint32_t i = 0; i < sectors; ++i) {
        w25qxx_erase(&w25qxx, i * w25qxx.sector_size, sizeof(buf));
        if ((i > 0) && (i % 100 == 0)) {
            printf("Done %4lu sectors - total time = %3lu s\n", i, (systick - start) / 1000);
        }
    }

    printf("Done erasing - took %lu ms\n", systick - start);

    uint32_t now, loop_cnt = 0, next_blink = LED_DELAY, next_tick = TICK_DELAY, next_test = 10000, offset_address = 0;

    while (true) {

        now = systick;

        if (now >= next_blink) {
            pico_toggle_led();
            next_blink = now + LED_DELAY;
        }

        if (now >= next_tick) {

            volatile float v_ref = 3.3f;
            volatile float adc_steps = 4096.0f;
            volatile float temp_base = 27.0f;
            volatile float slope_offset = 0.706f;
            volatile float slope = 0.001721f;

            float voltage = (float)internal_temp_raw * (v_ref / adc_steps);
            internal_temp_c = temp_base - (voltage - slope_offset) / slope;

            printf("Core 0 tick %lu (loop = %lu raw = %u c = %.2f)\n", now, loop_cnt, (unsigned int)internal_temp_raw, internal_temp_c);
            loop_cnt = 0;
            next_tick = now + TICK_DELAY;
        }

        if (now >= next_test) {

            printf("---------------\nReading page at address     : 0x%08lx\n", offset_address);

            res = w25qxx_read(&w25qxx, offset_address, (uint8_t *)&buf, sizeof(buf));
            if (res == W25QXX_Ok) {
                printf("Reading old value           : 0x%08lx\n", HAL_CRC_Calculate((uint32_t *)&buf, sizeof(buf) / 4));
            } else {
                printf("Unable to read w25qxx\n");
            }

            if (w25qxx_erase(&w25qxx, offset_address, sizeof(buf)) == W25QXX_Ok) {
                if (w25qxx_read(&w25qxx, offset_address, (uint8_t *)&buf, sizeof(buf)) == W25QXX_Ok) {
                    printf("After erase                 : 0x%08lx\n", HAL_CRC_Calculate((uint32_t *)&buf, sizeof(buf) / 4));
                }
            }

            fill_buffer(2, buf, sizeof(buf));

            printf("Writing page value          : 0x%08lx\n", HAL_CRC_Calculate((uint32_t *)&buf, sizeof(buf) / 4));

            if (w25qxx_write(&w25qxx, offset_address, (uint8_t *)&buf, sizeof(buf)) == W25QXX_Ok) {
                if (w25qxx_read(&w25qxx, offset_address, (uint8_t *)&buf, sizeof(buf)) == W25QXX_Ok) {
                    printf("Reading back                : 0x%08lx\n", HAL_CRC_Calculate((uint32_t *)&buf, sizeof(buf) / 4));
                }
            }

            printf("Test time                   : %lu ms\n", systick - now);

            offset_address += PAGE_SIZE / 4;

            if (offset_address + PAGE_SIZE > w25qxx.block_count * w25qxx.block_size)
                offset_address = 0;

            next_test = now + 10000;
        }

        ++loop_cnt;

        tight_loop_contents();
    }
}

// vim: ts=4 et nowrap