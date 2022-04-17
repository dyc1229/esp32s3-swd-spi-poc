#pragma once

#include <esp_err.h>
#include <driver/gpio.h>

esp_err_t swd_spi_init(gpio_num_t swclk, gpio_num_t swdio, uint32_t freq_hz, spi_host_device_t host);
esp_err_t swd_spi_wait_till_ready(int32_t timeout_cycles);
void swd_spi_send_cycles(uint32_t clk_cycles, uint32_t swclk_level);
void swd_spi_send_bits(uint32_t bits, size_t bit_len);
void swd_spi_send_send_swj_sequence();
esp_err_t IRAM_ATTR swd_spi_recv_pkt(uint8_t reg, uint32_t *out_word, uint16_t retry_cnt = 1000);
esp_err_t IRAM_ATTR swd_spi_read_dp(uint8_t reg, uint32_t *out_word);
uint8_t swd_spi_calc_parity_req(uint8_t reg);
uint32_t swd_spi_calc_parity_32(uint32_t reg);
esp_err_t IRAM_ATTR swd_spi_read_idcode(uint32_t *idcode);