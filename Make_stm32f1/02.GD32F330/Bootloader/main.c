/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

    \version 2017-06-06, V1.0.0, firmware for GD32F3x0
    \version 2019-06-01, V2.0.0, firmware for GD32F3x0
    \version 2020-09-30, V2.1.0, firmware for GD32F3x0
    \version 2022-01-06, V2.2.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f3x0.h"
#include "systick.h"
#include <stdio.h>
#include <gd32f3x0.h>
#include "main.h"

#include "host_data_layer.h"
#include "app_debug.h"
#include "SEGGER_RTT.h"
#include "board_hw.h"
#include "app_cli.h"
#include "app_debug.h"
#include "flash_if.h"
#include "ota_update.h"

#define VERIFY_CRC      1

typedef void (*jump_func)(void);
jump_func jump_to_app;
static volatile uint32_t m_sys_tick = 0;
static uint32_t rtt_debug_output(const void *data, uint32_t size);
static void task_wdt(void);
//static uint32_t lp_usart_put(const void *data, uint32_t size);

static volatile uint32_t m_delay_ms = 0;
void sys_increase_tick(void)
{
    ++m_sys_tick;
    if (m_delay_ms)
    {
        --m_delay_ms;
    }
    // host_data_layer_on_1ms_callback();
}

void sys_delay_ms(uint32_t ms)
{
    __disable_irq();
    m_delay_ms = ms;
    __enable_irq();
    while (m_delay_ms);
}

uint32_t sys_get_ms(void)
{
    return m_sys_tick;
}

static uint32_t rtt_debug_output(const void *data, uint32_t size)
{
    return SEGGER_RTT_Write(0, data, size);
}

uint32_t utilities_crc_sum(const uint8_t* data_p, uint32_t length)
{
    uint32_t crc = 0;
    while (length--)
    {
        crc += *data_p++;
    }
    
    return crc;
}

uint32_t HAL_GetTick(void)
{
    return m_sys_tick;
}

void SysTick_Handler(void)
{
    sys_increase_tick();
}


int main(void)
{
    /* configure systick */
    systick_config();
    
    app_debug_init(sys_get_ms, (void*)0);
    app_debug_register_callback_print(rtt_debug_output);
    board_hw_initialize();
    APP_DEBUG_INFO("Bootloader started\r\n");
    ota_information_t *ota_flag = (ota_information_t*)OTA_INFORMATION_START_ADDR;
	
    if (ota_flag->ota_flag == OTA_UPDATE_FLAG_UPDATE_NEW_FIRMWARE
        && ota_flag->size < OTA_UPDATE_APPLICATION_SIZE)
    {
#if VERIFY_CRC
        uint32_t crc;
        uint32_t crc_addr = OTA_UPDATE_DOWNLOAD_IMAGE_START_ADDR + sizeof(ota_image_header_t);
        uint32_t crc_size = ota_flag->size - sizeof(ota_image_header_t)-4;
        crc = utilities_crc_sum((uint8_t*)crc_addr, crc_size);
        // Copy firmware from download region to main region
        if (crc == ota_flag->crc32)
#else
        if (1)
#endif
        {            
            board_hw_watchdog_feed();
            flash_if_erase(OTA_UPDATE_APPLICATION_START_ADDR, (ota_flag->size + FLASH_IF_PAGE_SIZE - 1));
            flash_if_copy(OTA_UPDATE_APPLICATION_START_ADDR, (uint32_t*)(crc_addr), (crc_size+3)/4);
#if VERIFY_CRC
            // Verify crc
            crc = utilities_crc_sum((uint8_t*)crc_addr, crc_size);
            if (crc != ota_flag->crc32)
            {
                board_hw_reset();
            }
            else        // OTA success
#endif
            {
                board_hw_watchdog_feed();
                flash_if_erase(OTA_INFORMATION_START_ADDR, OTA_UPDATE_INFORMATION_SIZE);
                board_hw_reset();
            }
        }
    }
    while (1)
    {
        board_hw_watchdog_feed();
        __disable_irq();
        jump_to_app = (jump_func)(*(volatile unsigned int *)(OTA_UPDATE_BOOTLOADER_SIZE + 4));
        __set_MSP(*(volatile unsigned int*)OTA_UPDATE_BOOTLOADER_SIZE);
        jump_to_app();
    }
}

