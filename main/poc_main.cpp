#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <hal/spi_types.h>
#include "swd_spi_raw.h"

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(swd_spi_init(GPIO_NUM_1, GPIO_NUM_2, 800000, SPI2_HOST));
    ESP_ERROR_CHECK(swd_spi_reset());
    ESP_ERROR_CHECK(swd_spi_switch());
    ESP_ERROR_CHECK(swd_spi_reset());

    uint32_t idcode = 0;
    ESP_ERROR_CHECK(swd_spi_read_idcode(&idcode));
    ESP_LOGI("main", "IDCODE = 0x%x", idcode);

    vTaskDelay(portMAX_DELAY);
}
