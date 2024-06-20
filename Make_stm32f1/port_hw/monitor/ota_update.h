#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <stdbool.h>
#include <stdint.h>
#include "version_control.h"

/**
 * Bootloader memory organization
 *
 *   Address					Size					Desc
 * 	0x80000000				    8KB					Bootloader code
 * 	0x80002000				    27KB				Application
 * 	0x80008C00				    27KB				Download firmware
 * 	0x8000F800				    0x1000			    OTA download information
 * 	0x8000FC00				    0x1000			    Flash reserve
 */
 
 /**
  * Download firmware region detail
  * -------------------------------
  *         16 bytes header
  * -------------------------------  
  *         Raw firmware
  * -------------------------------
  *         4 bytes CRC32
  * -------------------------------
  */

#define FLASH_SIZE                                      (0x10000)
 
#define OTA_UPDATE_BOOTLOADER_SIZE						(0x4000)
#define OTA_UPDATE_APPLICATION_SIZE						0xB800
#define OTA_UPDATE_DOWNLOAD_FIRMWARE_SIZE				OTA_UPDATE_APPLICATION_SIZE
#define OTA_UPDATE_INFORMATION_SIZE						0x400

#define OTA_UPDATE_FLASH_BASE							((uint32_t)0x08000000U)
#define OTA_UPDATE_APPLICATION_START_ADDR				(OTA_UPDATE_FLASH_BASE + OTA_UPDATE_BOOTLOADER_SIZE)
#define OTA_UPDATE_DOWNLOAD_IMAGE_START_ADDR			OTA_UPDATE_APPLICATION_START_ADDR // (OTA_UPDATE_APPLICATION_START_ADDR + OTA_UPDATE_APPLICATION_SIZE)
#define OTA_INFORMATION_START_ADDR						(FLASH_SIZE - OTA_UPDATE_INFORMATION_SIZE - OTA_UPDATE_INFORMATION_SIZE)

#define OTA_UPDATE_FLAG_UPDATE_NEW_FIRMWARE				0x12345688
#define OTA_UPDATE_FLAG_UPDATE_COMPLETE					0x98761234
#define OTA_UPDATE_FLAG_INVALID							0xFFFFFFFF

#define OTA_UPDATE_MD5_CHECKSUM_SIZE					16
#define OTA_UPDATE_DEFAULT_HEADER_SIZE					16
#define OTA_UPDATE_DEFAULT_HEADER_DATA_FIRMWARE  		VERSION_CONTROL_HEADER "LP2"       // must 3 bytes
#define OTA_UPDATE_DEFAULT_HEADER_DATA_HARDWARE  		VERSION_CONTROL_HW       // must 3 bytes
#define OTA_UPDATE_FW_VERSION                           VERSION_CONTROL_FW       // must 3 bytes
#define OTA_UPDATE_CHECK_HEADER							1   

#ifndef OTA_ERASE_ALL_FLASH_BEFORE_WRITE
#define OTA_ERASE_ALL_FLASH_BEFORE_WRITE                0
#endif

typedef union
{
    struct
    {
        char header[3];
        char firmware_version[3];
        char hardware_version[3];
        uint32_t firmware_size;         // fw size =  image_size + 16 byte md5, fw excluded header
        uint8_t release_year;           // From 2000
        uint8_t release_month;
        uint8_t release_date;
    } __attribute__((packed)) name;
    uint8_t raw[16];
} __attribute__((packed)) ota_image_header_t;



typedef struct
{
	uint32_t ota_flag;
    uint32_t size;
    uint32_t crc32;
} ota_information_t;

/*!
 * @brief		Start ota update process
 * @param[in]	expected_size : Firmware size, included header signature
 * @retval 		TRUE : Operation success
 *         		FALSE : Operation failed
 */
bool ota_update_start(uint32_t expected_size);

/*!
 * @brief		Write data to flash
 * @param[in]	data : Data write to flash
 * @param[in]	length : Size of data in bytes
 * @note 		Flash write address will automatic increase inside function
 * @retval		TRUE : Operation success
 *				FALSE : Operation failed
 */
bool ota_update_write_next(uint8_t *data, uint32_t length);

/*!
 * @brief Finish ota process
 * @param[in] status TRUE : All data downloaded success
 *                   FALSE : A problem occurs
 */
bool ota_update_finish(bool status);

/*!
 * @brief		Check ota update status
 * @retval		TRUE : OTA is running
 * 				FALSE : OTA is not running
 */
bool ota_update_is_running(void);

/*!
 * @brief		Get current ota update
 * @retval		OTA config in flash
 */
ota_information_t *ota_update_get_config(void);

/*!
 * @brief		Set ota image size
 * @param[in]	size : Size of image
 */
void ota_update_set_expected_size(uint32_t size);

/*!
 * @brief		Write all remain data into flash
 */
bool ota_update_commit_flash(void);

/*!
 * @brief		Verify image checksum
 * @param[in]	addr : Start addr of image
 * @param[in]	length : Size of image, included 16 bytes checksum at the end
 * @retval		TRUE Valid image
 * 				FALSE Invalid image
 */
bool ota_update_verify_checksum(uint32_t addr, uint32_t length);

/*!
 * @brief		Get downloaded firmware in percent
 * @retval		Downloaded percent
 */
uint8_t ota_get_downloaded_percent(void);

/*!
 * @brief		Rollback to factory firmware
 */
void ota_update_rollback_to_factory_firmware(void);

/**
 * @brief		Write ota firmware header to flash
 * @param[in]	addr Address will stored data
 * @param[in]	header Header data
 */
void ota_update_write_header(uint32_t addr, ota_information_t *header);

/*!
 * @brief		Enter bootloader
 */
void ota_update_enter_bootloader(void);

#endif /* OTA_UPDATE_H */
