/*
 * Copyright (c) 2022, kaans, <https://github.com/kaans>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include <sfud.h>
#include <string.h>
#include <fal.h>

#define FLASH_ERASE_MIN_SIZE (4 * 1024)

#define LOCKER_ENABLE
#ifdef LOCKER_ENABLE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lock = NULL;

static sfud_flash* sfud_dev = NULL;

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

static int init(void)
{
#ifdef LOCKER_ENABLE
    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateCounting(1, 1);
        assert(s_lock != NULL);
    }
#endif
    sfud_flash *flash = sfud_get_device_table() + 0;
    sfud_device_init(flash);
    sfud_dev = flash;
    if (NULL == sfud_dev)
    {
        return -1;
    }

    /* update the flash chip information */
    nor_flash0.blk_size = sfud_dev->chip.erase_gran;
    nor_flash0.len = sfud_dev->chip.capacity;

    return 0;
}

static int read(long offset, uint8_t *buf, size_t size)
{
    esp_err_t ret;

    LOCK();
    ret = sfud_read(sfud_dev,nor_flash0.addr+offset,size, buf);
    UNLOCK();

    return ret;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
    LOCK();
    if (sfud_write(sfud_dev, nor_flash0.addr + offset, size, buf) != SFUD_SUCCESS)
    {
        return -1;
    }

    UNLOCK();

    return 0;
}

static int erase(long offset, size_t size)
{
    int32_t erase_size = ((size - 1) / FLASH_ERASE_MIN_SIZE) + 1;

    LOCK();
    if (sfud_erase(sfud_dev, nor_flash0.addr + offset, size) != SFUD_SUCCESS)
    {
        return -1;
    }   
    UNLOCK();

    return 0;
}

struct fal_flash_dev nor_flash0 =
    {
        .name = NOR_FLASH_DEV_NAME,
        .addr = 0,                      // address is relative to beginning of partition; 0x0 is start of the partition
        .len = 8*1024*1024,                 // size of the partition as specified in partitions.csv
        .blk_size = FLASH_ERASE_MIN_SIZE, // must be 4096 bytes
        .ops = {init, read, write, erase},
        .write_gran = 1, // 1 byte write granularity
};
