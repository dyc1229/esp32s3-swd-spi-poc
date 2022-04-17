#include <esp_err.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <soc/spi_struct.h>
#include "swd_spi.h"

#ifdef TAG
#undef TAG
#endif

#define TAG "swd_spi"

static spi_device_handle_t spi_handle = NULL;
static volatile spi_dev_t *spi_dev = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);

esp_err_t swd_spi_init(gpio_num_t swclk, gpio_num_t swdio, uint32_t freq_hz, spi_host_device_t host)
{
    spi_bus_config_t spi_bus_config = {};
    spi_bus_config.mosi_io_num     = swdio; // SWD I/O
    spi_bus_config.miso_io_num     = -1; // not connected
    spi_bus_config.sclk_io_num     = swclk; // SWD CLK
    spi_bus_config.quadwp_io_num   = -1;
    spi_bus_config.quadhd_io_num   = -1;
    spi_bus_config.max_transfer_sz = 0;
    spi_bus_config.flags = SPICOMMON_BUSFLAG_IOMUX_PINS;

    esp_err_t ret = spi_bus_initialize(host, &spi_bus_config, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Bus init fail");
        return ret;
    }


    spi_device_interface_config_t spi_dev_inf_config = {};
    spi_dev_inf_config.command_bits        = 0;
    spi_dev_inf_config.address_bits        = 0;
    spi_dev_inf_config.dummy_bits          = 0;
    spi_dev_inf_config.mode                = 0;
    spi_dev_inf_config.duty_cycle_pos      = 0;
    spi_dev_inf_config.cs_ena_pretrans     = 0;
    spi_dev_inf_config.cs_ena_posttrans    = 0;
    spi_dev_inf_config.clock_speed_hz      = (int)freq_hz;
    spi_dev_inf_config.spics_io_num        = -1;
    spi_dev_inf_config.flags               = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST;
    spi_dev_inf_config.queue_size          = 24;
    spi_dev_inf_config.pre_cb              = NULL;
    spi_dev_inf_config.post_cb             = NULL;

    ret = spi_bus_add_device(host, &spi_dev_inf_config, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device init fail");
        return ret;
    }


    ret = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI lock fail");
        return ret;
    }

    switch (host) {
        case SPI1_HOST: {
            spi_dev = (volatile spi_dev_t *)(DR_REG_SPI1_BASE);
            break;
        }

        case SPI2_HOST: {
            spi_dev = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);
            break;
        }

        case SPI3_HOST: {
            spi_dev = (volatile spi_dev_t *)(DR_REG_SPI3_BASE);
            break;
        }

        default: {
            ESP_LOGE(TAG, "Unknown SPI peripheral selected");
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ret;
}

esp_err_t swd_spi_wait_till_ready(int32_t timeout_cycles)
{
    while(timeout_cycles >= 0 && spi_dev->cmd.usr != 0) {
        timeout_cycles--;
    }

    if (timeout_cycles < 0) {
        return ESP_ERR_TIMEOUT;
    } else {
        return ESP_OK;
    }
}

void swd_spi_send_cycles(uint32_t clk_cycles, uint32_t swclk_level)
{
    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 1;
    spi_dev->user.usr_miso = 0;
    spi_dev->misc.ck_idle_edge = 0;
//    spi_dev->user.usr_hold_pol = 1;
//    spi_dev->user.usr_dout_hold = 0;
    spi_dev->user.ck_out_edge = 0;

    for (uint32_t idx = 0; idx < 18; idx++) {
        spi_dev->data_buf[idx] = swclk_level;
    }

    spi_dev->ms_dlen.ms_data_bitlen = clk_cycles - 1;
    spi_dev->cmd.usr = 1; // Trigger Tx!!
}

void swd_spi_send_bits(uint32_t bits, size_t bit_len)
{
    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 1;
    spi_dev->user.usr_miso = 0;
    spi_dev->misc.ck_idle_edge = 0;
    spi_dev->user.ck_out_edge = 0;
    spi_dev->ms_dlen.ms_data_bitlen = bit_len - 1;
    spi_dev->data_buf[0] = bits;
    spi_dev->cmd.usr = 1; // Trigger Tx!!
}

void swd_spi_send_send_swj_sequence()
{
    const uint16_t seq = 0xE79E;
    swd_spi_send_cycles(51, 0xffffffff);
    while(spi_dev->cmd.usr != 0);
    swd_spi_send_cycles(seq, 16);
    while(spi_dev->cmd.usr != 0);
    swd_spi_send_cycles(51, 0xffffffff);
    while(spi_dev->cmd.usr != 0);
}

esp_err_t swd_spi_recv_pkt(uint8_t reg, uint32_t *out_word, uint16_t retry_cnt)
{
    return 0;
}

esp_err_t swd_spi_read_dp(uint8_t reg, uint32_t *out_word)
{
    return 0;
}

uint8_t swd_spi_calc_parity_req(uint8_t reg)
{
    return 0;
}

uint32_t swd_spi_calc_parity_32(uint32_t reg)
{
    return 0;
}

esp_err_t swd_spi_read_idcode(uint32_t *idcode)
{
    return 0;
}
