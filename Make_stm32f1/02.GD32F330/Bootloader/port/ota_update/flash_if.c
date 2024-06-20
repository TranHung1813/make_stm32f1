#include "flash_if.h"
#include "main.h"
#include "app_debug.h"
#include "ota_update.h"
#include "gd32f3x0.h"
#include "board_hw.h"

#define ABS_RETURN(x, y) (((x) < (y)) ? (y) : (x))

uint8_t flash_if_init(void)
{
    return 1;
}

flash_if_error_t flash_if_erase(uint32_t addr, uint32_t size)
{
    uint32_t retval = 0;
    uint32_t begin_addr = addr;
    
    __disable_irq();
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    for (uint32_t i = 0; i < (size+FLASH_IF_PAGE_SIZE-1)/FLASH_IF_PAGE_SIZE; i++)
    {
//         DEBUG_INFO("Erase flash at addr 0x%08X\r\n", addr);
        // Erase the flash pages
        retval += fmc_page_erase(addr);
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
        addr += FLASH_IF_PAGE_SIZE;
        board_hw_watchdog_feed();
    }
    fmc_lock();
    
    __enable_irq();
    for (uint32_t i = 0; i < FLASH_IF_PAGE_SIZE/4; i++)
    {
        if (*((volatile uint32_t*)begin_addr) != 0xFFFFFFFF)
        {
            retval++;
            break;
        }
    }
    APP_DEBUG_VERBOSE("Erase done\r\n");
    if (retval)
    {
        return FLASH_IF_ERASE_KO;
    }
    return FLASH_IF_OK;
}

//uint32_t flash_if_erase_ota_info_page()
//{
//    flash_if_init();

//    FLASH_EraseInitTypeDef desc;
//    uint32_t result = FLASH_IF_OK;
//    uint32_t page_error;

//    HAL_FLASH_Unlock();

//    desc.PageAddress = OTA_INFO_START_ADDR;
//    desc.TypeErase = FLASH_TYPEERASE_PAGES;

//    desc.NbPages = 1;
//    if (HAL_FLASHEx_Erase(&desc, &page_error) != HAL_OK)
//    {
//        DEBUG_ERROR("OTA erase error\r\n");
//        result = FLASH_IF_ERASE_KO;
//    }

//    HAL_FLASH_Lock();

//    return result;
//}

//uint32_t flash_if_write_ota_info_page(uint32_t *data, uint32_t nb_of_word)
//{
//    flash_if_erase_ota_info_page();
//    flash_if_write(OTA_INFO_START_ADDR, data, nb_of_word);
//    return 0;
//}

flash_if_error_t flash_if_copy(uint32_t destination, uint32_t *source, uint32_t nb_of_word)
{
    uint32_t retval = 0;
    uint32_t readback = 0;
    __disable_irq();
    fmc_unlock();
    for (uint32_t i = 0; i < nb_of_word; i++)
    {
        retval += fmc_word_program(destination, source[i]);
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
        readback = *((uint32_t*)destination);
        if (retval || readback != source[i])
        {
            retval = FLASH_IF_WRITING_ERROR;
            APP_DEBUG_ERROR("Compare content failed at addr 0x%08X\r\n", destination);
            break;
        }
        destination += 4;
    }
    fmc_lock();
    __enable_irq();
    if (retval)
    {
        APP_DEBUG_ERROR("FLASH_IF_WRITING_ERROR at addr 0x%08X\r\n", destination);
        retval = FLASH_IF_WRITING_ERROR;
    }
    // __enable_irq();
    board_hw_watchdog_feed();
    return FLASH_IF_OK;
}
