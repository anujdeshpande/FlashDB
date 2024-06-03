/*
 * Copyright (c) 2022, kaans, <https://github.com/kaans>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include <esp_partition.h>

#include <string.h>
#include <fal.h>

#define FLASH_ERASE_MIN_SIZE (4 * 1024)

#define LOCKER_ENABLE
#ifdef LOCKER_ENABLE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "soc/spi_pins.h"
#include "hal/spi_types.h"
#include "esp_system.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
static SemaphoreHandle_t s_lock = NULL;
static char* TAG="fal";
const static esp_partition_t *partition;

#define LOCK()                                 \
    do                                         \
    {                                          \
        xSemaphoreTake(s_lock, portMAX_DELAY); \
    } while (0)

#define UNLOCK()                \
    do                          \
    {                           \
        xSemaphoreGive(s_lock); \
    } while (0)
#else
#define LOCK()
#define UNLOCK()
#endif


/* static esp_flash_t* example_init_ext_flash(void) */
static esp_flash_t* init_ext_flash(void)
{
	/* These values taken from pins.h */
    const spi_bus_config_t bus_config = {
        .mosi_io_num = GPIO_NUM_7,
        .miso_io_num = GPIO_NUM_4,
        .sclk_io_num = GPIO_NUM_6,
    };

    const esp_flash_spi_device_config_t device_config = {
        .host_id = SPI2_HOST,
        .cs_io_num = GPIO_NUM_5,
        /* .io_mode = SPI_FLASH_DIO, */ /* Not sure what this does - commented for now*/
        .freq_mhz = 10,
    };

    ESP_LOGI(TAG, "Initializing external SPI Flash");
    ESP_LOGI(TAG, "Pin assignments:");
    ESP_LOGI(TAG, "MOSI: %2d   MISO: %2d   SCLK: %2d   CS: %2d",
        bus_config.mosi_io_num, bus_config.miso_io_num,
        bus_config.sclk_io_num, device_config.cs_io_num
    );

    // Initialize the SPI bus
    ESP_LOGI(TAG, "DMA CHANNEL: %d", SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));

    // Add device to the SPI bus
    esp_flash_t* ext_flash;
    ESP_ERROR_CHECK(spi_bus_add_flash_device(&ext_flash, &device_config));

    // Probe the Flash chip and initialize it
    esp_err_t err = esp_flash_init(ext_flash);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize external Flash: %s (0x%x)", esp_err_to_name(err), err);
        return NULL;
    }

    // Print out the ID and size
    uint32_t id;
    ESP_ERROR_CHECK(esp_flash_read_id(ext_flash, &id));
    ESP_LOGI(TAG, "Initialized external Flash, size=%" PRIu32 " KB, ID=0x%" PRIx32, ext_flash->size / 1024, id);

    return ext_flash;
}

static int init(void)
{
#ifdef LOCKER_ENABLE
    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateCounting(1, 1);
        assert(s_lock != NULL);
    }
#endif

    // the values passed to esp_partition_find_first() must correspond to the
    // values set in partitions.csv for the partition named "flashdb".
    /* partition = esp_partition_find_first(0x40, 0x00, "flashdb"); */
    esp_flash_t* flash = init_ext_flash();
    if (flash == NULL) {
	    ESP_LOGE(TAG,"Error in ext flash init");
	    return -1;
    }
    const size_t offset = 0;
    esp_err_t ret = esp_partition_register_external(flash,  offset,  flash->size, "flashdb", 0x40, 0x00, &partition);
    assert(partition != NULL);

    return 1;
}

static int read(long offset, uint8_t *buf, size_t size)
{
    esp_err_t ret;

    LOCK();
    ret = esp_partition_read(partition, offset, buf, size);
    UNLOCK();

    return ret;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
    esp_err_t ret;

    LOCK();
    ret = esp_partition_write(partition, offset, buf, size);
    UNLOCK();

    return ret;
}

static int erase(long offset, size_t size)
{
    esp_err_t ret;
    int32_t erase_size = ((size - 1) / FLASH_ERASE_MIN_SIZE) + 1;

    LOCK();
    ret = esp_partition_erase_range(partition, offset, erase_size * FLASH_ERASE_MIN_SIZE);
    UNLOCK();

    return ret;
}

const struct fal_flash_dev nor_flash0 =
    {
        .name = NOR_FLASH_DEV_NAME,
        .addr = 0x0,                      // address is relative to beginning of partition; 0x0 is start of the partition
        .len = 32 * 1024,                 // size of the partition as specified in partitions.csv
        .blk_size = FLASH_ERASE_MIN_SIZE, // must be 4096 bytes
        .ops = {init, read, write, erase},
        .write_gran = 1, // 1 byte write granularity
};
