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
#include "ota_update.h"
#include "flash_if.h"

#define VERIFY_CRC      1


static volatile uint32_t m_sys_tick = 0;
static uint32_t rtt_debug_output(const void *data, uint32_t size);
//static uint32_t lp_usart_put(const void *data, uint32_t size);
typedef void (*jump_func)(void);
jump_func jump_to_app;
uint32_t utilities_crc_sum(const uint8_t* data_p, uint32_t length)
{
    uint32_t crc = 0;
    while (length--)
    {
        crc += *data_p++;
    }
    
    return crc;
}


static volatile uint32_t m_delay_ms = 0;
void sys_increase_tick(void)
{
    ++m_sys_tick;
    if (m_delay_ms)
    {
        --m_delay_ms;
    }
}

void sys_delay_ms(uint32_t ms)
{
    NVIC_DisableIRQ(SysTick_IRQn);
    m_delay_ms = ms;
    NVIC_EnableIRQ(SysTick_IRQn);
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

//static uint32_t lp_usart_put(const void *data, uint32_t size)
//{
//    return board_hw_fm_lcd_send((uint8_t*)data, size);
//}


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
    app_debug_set_level(DEBUG_LEVEL_INFO);
    DEBUG_WARN("Bootloader %s, %s, HW %s, FW %s\r\n", 
            __DATE__, __TIME__, OTA_UPDATE_DEFAULT_HEADER_DATA_HARDWARE, OTA_UPDATE_FW_VERSION);
    board_hw_initialize();
//    app_debug_register_callback_print(lp_usart_put);
    host_data_layer_initialize();
    ota_information_t *ota_flag = (ota_information_t*)OTA_INFORMATION_START_ADDR;
    
    if (ota_flag->ota_flag == OTA_UPDATE_FLAG_UPDATE_NEW_FIRMWARE
        && ota_flag->size < OTA_UPDATE_APPLICATION_SIZE)
    {
        while (1)
        {
            host_data_layer_polling_task((void*)0);
        }
//#if VERIFY_CRC
//        uint32_t crc;
//        uint32_t crc_addr = OTA_UPDATE_DOWNLOAD_IMAGE_START_ADDR + sizeof(ota_image_header_t);
//        uint32_t crc_size = ota_flag->size - sizeof(ota_image_header_t)-4;
//        crc = utilities_crc_sum((uint8_t*)crc_addr, crc_size);
//        // Copy firmware from download region to main region
//        if (crc == ota_flag->crc32)
//#else
//        if (1)
//#endif
//        {            
//            board_hw_watchdog_feed();
//            flash_if_erase(OTA_UPDATE_APPLICATION_START_ADDR, (ota_flag->size + FLASH_IF_PAGE_SIZE - 1));
//            flash_if_copy(OTA_UPDATE_APPLICATION_START_ADDR, (uint32_t*)(crc_addr), (crc_size+3)/4);
//#if VERIFY_CRC
//            // Verify crc
//            crc = utilities_crc_sum((uint8_t*)crc_addr, crc_size);
//            if (crc != ota_flag->crc32)
//            {
//                board_hw_reset();
//            }
//            else        // OTA success
//#endif
//            {
//                board_hw_watchdog_feed();
//                flash_if_erase(OTA_INFORMATION_START_ADDR, OTA_UPDATE_INFORMATION_SIZE);
//                board_hw_reset();
//            }
//        }
    }
    
    board_hw_watchdog_feed();
    __disable_irq();
    jump_to_app = (jump_func)(*(volatile unsigned int *)(OTA_UPDATE_BOOTLOADER_SIZE + 4));
    __set_MSP(*(volatile unsigned int*)OTA_UPDATE_BOOTLOADER_SIZE);
    jump_to_app();
}
