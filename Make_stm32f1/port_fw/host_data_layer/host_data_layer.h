#ifndef HOST_DATA_LAYER_H
#define HOST_DATA_LAYER_H

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    HOST_POWER_STARTING,
    HOST_POWER_UP,
    HOST_POWER_PULSE_PWR_KEY,
    HOST_POWER_RUNNING,
    HOST_POWER_WAIT_FOR_VIN,
    HOST_POWER_DOWN,
    HOST_POWER_FLASH_MODE
} host_power_state_t;

typedef union
{
    struct
    {
        uint32_t led_vol                            : 6;
        uint32_t speaker_on_off                     : 1;
        uint32_t relay1                             : 1;
        uint32_t relay2                             : 1;
        uint32_t fan                                : 1;
        uint32_t mic_line_sw                        : 1;
        uint32_t meeting_led                        : 1;
        uint32_t switch_in                          : 1;
        uint32_t switch_out                         : 1;
        uint32_t lte_led_r                          : 1;
        uint32_t lte_led_b                          : 1;
        uint32_t eth_led_r                          : 1;
        uint32_t eth_led_b                          : 1;
        uint32_t pa                                 : 1;
        uint32_t sw_mic                             : 1;
        uint32_t if_4g                              : 1;
        uint32_t if_wifi                            : 1;
        uint32_t if_eth                             : 1;
        uint32_t operation_mode                     : 3;
        uint32_t nextion_enable                     : 1;
        uint32_t sw_mic_in_from_mic_line_or_hp      : 1;        // config_mic
        uint32_t sw_line_out_in_from_mic_line_or_hp : 1;        // con hp mach ATTH 7 inch
        uint32_t sw_lineout_enable                  : 1;        // en_hph_out
        uint32_t sw_pa_out_from_mic_line_or_hp      : 1;        // config hp mach node PA
        uint32_t class_d                            : 1;
    } __attribute__((packed)) name;
    uint32_t value;
} __attribute__((packed)) host_data_layer_output_t;

typedef union
{
    struct
    {
        uint16_t input0         : 1;
        uint16_t input1         : 1;
        uint16_t input2         : 1;
        uint16_t input3         : 1;
        uint16_t input4         : 1;        // bit nay khong dung nhung de tuong thich nguoc voi ATTH -> van them vao
        uint16_t button_on_air  : 1;
        uint16_t reserve        : 10;
    } __attribute__((packed)) name;
    uint16_t value;
} __attribute__((packed)) host_data_layer_input_t;

typedef union
{
    struct
    {
        uint8_t d_vol_err         : 1;
        uint8_t class_d           : 4;
        uint8_t reserve           : 3;
    } __attribute__((packed)) name;
    uint8_t value;
} __attribute__((packed)) host_data_layer_debug_flag_t;

typedef struct
{
    uint8_t screen_id;
    uint16_t vin_mv;
    uint16_t vbus_24v_mv;
    uint8_t headphone;
    host_data_layer_input_t input;
    host_data_layer_output_t output;
    uint8_t hardware_version[3];
    uint8_t firmware_version[3];
    uint8_t mic;        // dont care
    uint8_t line_in;    // dont care
    uint16_t v3v8_mv;    // dont care
    host_data_layer_debug_flag_t error_flag;
} __attribute__((packed)) host_data_layer_ping_msg_t;

typedef struct
{
    uint8_t type;       // 0 = unknown, 1 = master, 2 = slave
    uint8_t band;
    uint8_t csq;
    uint8_t temperature;
} __attribute__((packed)) host_data_layer_mode_report_t;

/**
 * @brief       Host data layer 1ms tick callback
 */ 
void host_data_layer_on_1ms_callback(void);


/**
 * @brief       Host data layer initialize
 */ 
void host_data_layer_initialize(void);

/**
 * @brief       Host data layer polling task
 * @param[in]   arg Unuse
 */
void host_data_layer_polling_task(void *arg);

/**
 * @brief       Request host id
 */
void host_data_layer_request_ip(void);

/**
 * @brief       Set device in test mode
 * @param[in]   TRUE Set device in test mode
 *              FALSE Exit test mode
 */
void host_data_layer_set_test_mode(bool test);

/**
 * @brief       Set host power mode
 */
void host_data_layer_set_power_mode(host_power_state_t mode);

/**
 * @brief       Set uart host timeout
 * @note        Internal test only
 */
void host_data_layer_set_uart_timeout(uint32_t timeout);
      
/**
 * @brief       Set ping host timeout
 * @note        Internal test only
 */
void host_data_layer_set_ping_timeout(uint32_t timeout);

//void get_gsm_info (gsm_info_t* gsm_info);

#endif /* HOST_DATA_LAYER_H */
