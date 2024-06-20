#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "host_data_layer.h"
#include "lwrb/lwrb.h"
#include "app_debug.h"
#include "board_hw.h"
#include "min.h"
#include "min_id.h"
#include "lpf.h"
#include "version_control.h"
#include "app_cli.h"
#include "SEGGER_RTT.h"
#include "ota_update.h"
#include "flash_if.h"

#define HOST_KEEP_ALIVE_TIMEOUT_FIRST_TIME_MS               (100000)
#define HOST_KEEP_ALIVE_TIMEOUT_SECOND_TIME_MS              (150000)
#define HOST_PING_SHORT_INTERVAL_MS                         (50)
#define HOST_PING_LONG_INTERVAL_MS                          (1000)
#define HOST_POWER_ON_MS                                    (1800000)
#define HOST_POWER_OFF_MS                                   (100000)
#define LCD_FM_RX_BUFFER_SIZE                               96
#define VIN_LOST_THRESHOLD_MV                               9000

#define CORE_POWER_ON()                                     board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0)
#define CORE_POWER_OFF()                                    board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1)
#define MAX_HEARTBEAT_COUNTER_IN_RUN_MODE                   1800    // 180s
#define MAX_HEARTBEAT_COUNTER_AFTER_MCU_BOOT                3600    // 360s
#define MAX_HEARTBEAT_COUNTER_AFTER_MCU_SOFTWARE_RESET      50    // 5s
#define DEFAULT_HOST_UART_TIMEOUT_DO_RESET                  36000   // 1h
#define DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU            (72000) // 2h

#define FM_LCD_ENABLE                                       1
#define V_ADC_ENDIAN_VIN_INDEX                              0
#define V_ADC_ENDIAN_3V8_INDEX                              1
#define V_ADC_ENDIAN_5V_INDEX                               2
#define V_ADC_ENDIAN_VAR_RESISTOR_INDEX                     3

#define STREAM_STATE_IDLE                                   0
#define STREAM_STATE_PREPARE                                1
#define STREAM_STATE_RUNNING                                2
#define STREAM_STATE_STOP                                   3
#define STREAM_STATE_MIC_LINE                               4
#define STREAM_STATE_FM                                     5
#define STREAM_STATE_NO_OPERATION                           6
#define CLI_ENABLE                                          0

#define MAX_ROOM_SUPPORT                                    12
#define MAX_FOLDER_SUPPORT                                  4
#define LCD_MAX_DIRECTORY_NAME_LEN                          64

#define ROOM_IDLE                                           0
#define ROOM_SELECTED                                       1
#define ROOM_MEETING                                        2

#define LCD_HOME_SCREEN                                     1
#define LCD_INFO_SCREEN                                     2
#define LCD_SETTING_SCREEN                                  3
#define LCD_MUSIC_SCREEN                                    4

#define TEST_LCD_NEXTION                                    0
#ifndef OTA_ENABLE
#define OTA_ENABLE                                          1
#endif

#define NEXTION_LCD_MAX_FOLDER_COUNT                        4   


// static uint8_t m_nextion_end_frame[3] = {0xFF, 0xFF, 0xFF};

// Ring buffer host
static lwrb_t m_ring_buffer_host_rx;
uint8_t m_uart_host_buffer[512+128];
static uint32_t m_min_host_rx_buffer[(MIN_MAX_PAYLOAD+3)/4];
uint32_t m_ring_buffer_host_full = 0;
static int32_t m_count_host_boot_time = -1;
static int32_t m_is_vin_detect = -1;
static inline void host_data_layer_send(uint8_t *data, uint32_t length);
static volatile uint32_t m_usart_monitor_error_counter = 0;
static volatile uint32_t m_skip_serial_data_when_module_power_on = 0;
// Host min protocol context
static min_context_t m_min_host_context;
static min_frame_cfg_t m_min_host_setting = MIN_DEFAULT_CONFIG();
static bool uart_tx_to_host(void *ctx, uint8_t data);
static void on_host_frame_callback(void *ctx, min_msg_t *frame);
static void on_host_timeout_callback(void *ctx);
static volatile uint32_t m_last_time_send_ping = 0;
volatile uint32_t m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_AFTER_MCU_BOOT;
static uint32_t m_host_keep_alive_uart = DEFAULT_HOST_UART_TIMEOUT_DO_RESET;     // 1h
static uint32_t m_host_die_counter_force_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
static host_power_state_t m_host_power_state = HOST_POWER_RUNNING;
static uint32_t ignore_continues_write_to_flash = 0;
static host_data_layer_ping_msg_t m_host_ping_msg;
static host_data_layer_output_t m_current_output_not_swap_endian;
uint32_t m_serial_frame_err_counter = 0;
uint32_t m_total_serial_frame = 0;

void host_data_layer_initialize(void)
{   
#if DIGITAL_POT
    // Reset volume
    board_hw_digital_pot_init();
    board_hw_digital_pot_set(0);
#endif
    
    lwrb_init(&m_ring_buffer_host_rx, m_uart_host_buffer, sizeof(m_uart_host_buffer));
    
    m_min_host_setting.get_ms = sys_get_ms;
    m_min_host_setting.last_rx_time = 0x00;
    m_min_host_setting.rx_callback = on_host_frame_callback;
    m_min_host_setting.invalid_crc_callback = NULL;
    m_min_host_setting.timeout_callback = on_host_timeout_callback;
    m_min_host_setting.timeout_not_seen_rx = 7;
    m_min_host_setting.tx_byte = uart_tx_to_host;
    m_min_host_setting.use_timeout_method = 1;

    m_min_host_context.callback = &m_min_host_setting;
    m_min_host_context.rx_frame_payload_buf = (uint8_t*)m_min_host_rx_buffer;
    min_init_context(&m_min_host_context);
    
   
//#if FM_LCD_ENABLE
//    lwrb_init(&m_ring_buffer_fm, m_fm_uart_raw_rx_buffer, sizeof(m_fm_uart_raw_rx_buffer));
//    // FM
//    m_min_fm_setting.get_ms = sys_get_ms;
//    m_min_fm_setting.last_rx_time = 0x00;
//    /* Not using the rx_callback then*/
//    m_min_fm_setting.rx_callback = on_fm_frame_callback;
//    m_min_fm_setting.invalid_crc_callback = on_fm_frame_callback;
//    m_min_fm_setting.timeout_not_seen_rx = 10;
//    m_min_fm_setting.tx_byte = fm_lcd_uart_send_byte;
//    m_min_fm_setting.use_timeout_method = 1;
//    //m_min_fm_context.cb = &m_min_fm_setting;
//    m_min_fm_context.rx_frame_payload_buf = m_min_fm_rx_buffer;
//    m_min_fm_context.callback = &m_min_fm_setting;
//    min_init_context(&m_min_fm_context);
//    
//#else   // init buffer for serial debugging
//    lwrb_init(&m_ring_buffer_debug_uart, m_debug_rx_buffer, sizeof(m_debug_rx_buffer));
//#endif // FM_LCD_ENABLE
    
    char *p = VERSION_CONTROL_HW;
    
    m_host_ping_msg.hardware_version[0] = *p++ - '0';
    m_host_ping_msg.hardware_version[1] = *p++ - '0';
    m_host_ping_msg.hardware_version[2] = *p++ - '0';
    
    m_host_ping_msg.screen_id = LCD_HOME_SCREEN;
    
    p = VERSION_CONTROL_FW;
    m_host_ping_msg.firmware_version[0] = *p++ - '0';
    m_host_ping_msg.firmware_version[1] = *p++ - '0';
    m_host_ping_msg.firmware_version[2] = *p++ - '0';
     
    board_hardware_get_reset_reason();
        
    if (board_hw_is_power_on_reset())
    {
        board_hw_clear_power_on_reset_flag();
        board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0);
        m_host_power_state = HOST_POWER_UP;
        m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_AFTER_MCU_BOOT;
        DEBUG_WARN("Power on reset\r\n");
    }
    else
    {
        board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1);
        board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 0);
        m_host_power_state = HOST_POWER_RUNNING;
        m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_AFTER_MCU_SOFTWARE_RESET;
        DEBUG_WARN("Software reset\r\n");
    }
        
//    // CLI
#if CLI_ENABLE
    static app_cli_cb_t cli_cb;
    cli_cb.printf = cli_printf;
    cli_cb.puts = cli_puts;
    app_cli_start(&cli_cb);
#endif
    
    // Set digital vol
//#if DIGITAL_POT
//    board_hw_digital_pot_set(100);
//#endif
//    board_hw_critical_log_t err;
//    if (board_hw_get_critical_error(&err))
//    {
//        if (err.name.is_heartbeat_err)
//        {
//            DEBUG_ERROR("Ping IO err\r\n");
//        }
//        if (err.name.is_uart_communication_error)
//        {
//            DEBUG_ERROR("UART communication err\r\n");
//        }
//        if (err.name.is_vin_lost)
//        {
//            DEBUG_ERROR("Vin lost err\r\n");
//        }
//        m_reset_counter = err.name.reset_counter+1;
//    }
//    else
//    {
//        err.value = 0;
//    }
//    DEBUG_WARN("Reset counter %u\r\n", err.name.reset_counter);
    m_current_output_not_swap_endian.name.pa = 0;
    board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
}

#if FM_LCD_ENABLE
#endif // FM_LCD_ENABLE


void update_peripheral_input(void)
{        
    static host_data_layer_input_t m_last_input;
    m_host_ping_msg.input.value = 0;
    m_host_ping_msg.input.name.input0 = board_hw_get_input(BOARD_HW_INPUT0);
    m_host_ping_msg.input.name.input1 = board_hw_get_input(BOARD_HW_INPUT1);
    m_host_ping_msg.input.name.input2 = board_hw_get_input(BOARD_HW_INPUT2);
    m_host_ping_msg.input.name.input3 = board_hw_get_input(BOARD_HW_INPUT3);
    m_host_ping_msg.input.name.button_on_air = board_hw_get_input(BOARD_HW_INPUT_BUTTON_ON_AIR);
    
//    uint8_t spk = (m_host_ping_msg.input.name.input0 << 0)
//                | (m_host_ping_msg.input.name.input1 << 1)
//                | (m_host_ping_msg.input.name.input2 << 2)
//                | (m_host_ping_msg.input.name.input3 << 3);
//    DEBUG_WARN("SPK %d, raw %d%d%d%d, PA %d\r\n", spk, m_host_ping_msg.input.name.input0, 
//                                            m_host_ping_msg.input.name.input1,
//                                            m_host_ping_msg.input.name.input2,
//                                            m_host_ping_msg.input.name.input3,
//                                            m_current_output_not_swap_endian.name.pa);
    
    if (m_last_input.value != m_host_ping_msg.input.value)
    {
        DEBUG_VERBOSE("DI changed from 0x%08X to 0x%08X\r\n", m_last_input.value, m_host_ping_msg.input.value);
        m_last_input.value = m_host_ping_msg.input.value;
    }
    
    // Swap input endian
    uint16_t input_swap_endian = m_host_ping_msg.input.value;
    m_host_ping_msg.input.value = ((input_swap_endian & 0xFF) << 8) | (input_swap_endian >> 8);
    
    // Swap output endian
    uint16_t output_swap_endian = m_current_output_not_swap_endian.value;
    m_host_ping_msg.output.value = ((output_swap_endian & 0xFF) << 8) | (output_swap_endian >> 8); 
}


void host_data_layer_set_test_mode(bool test)
{
}

void host_data_layer_set_power_mode(host_power_state_t mode)
{
    DEBUG_WARN("Set power state to %d\r\n", mode);
    m_host_power_state = mode;
    if (m_host_power_state == HOST_POWER_FLASH_MODE)
    {
        board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0);
    }
}


static void on_host_timeout_callback(void *ctx)
{
    
}

static void on_host_frame_callback(void *ctx, min_msg_t *frame)
{
    m_total_serial_frame++;
    m_skip_serial_data_when_module_power_on = 0;
    // Reset timeout
    m_host_keep_alive_uart = DEFAULT_HOST_UART_TIMEOUT_DO_RESET;
    m_host_die_counter_force_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
    m_usart_monitor_error_counter = 0;

    if (m_count_host_boot_time != -1)
    {
        // DEBUG_WARN("Host boot time %u sec\r\n", m_count_host_boot_time / 10);
        m_count_host_boot_time = -1;
    }
       
    switch (frame->id)
    {
#if OTA_ENABLE        
        case MIN_ID_OTA_UPDATE_START:
        {
            /* 4 bytes size */
            DEBUG_WARN("OTA start\r\n");
            // Disable USARTFM RB interrupt
            board_hw_disable_fm_rx();
            bool retval;
            uint8_t *p = (uint8_t *)frame->payload;
            uint32_t firmware_size = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | (p[3]);
            
            board_hw_enable_host_rx(false);
            retval = ota_update_start(firmware_size);
            board_hw_enable_host_rx(true);
            
            if (retval)
            {
                // Send ack
                min_msg_t rsp_msg;
                uint8_t tmp = 0;
                DEBUG_WARN("OTA ACK\r\n");
                rsp_msg.id = MIN_ID_OTA_UPDATE_START; //MIN_ID_OTA_ACK;
                rsp_msg.len = 1;
                rsp_msg.payload = &tmp;
                min_send_frame(&m_min_host_context, &rsp_msg);
            }
            else // Send failed
            {
                min_msg_t rsp_msg;
                uint8_t tmp = 1;
                DEBUG_WARN("OTA start failed\r\n");
                rsp_msg.id = MIN_ID_OTA_FAILED;
                rsp_msg.len = 1;
                rsp_msg.payload = &tmp;
                min_send_frame(&m_min_host_context, &rsp_msg);
            }
        }
        break;

        case MIN_ID_OTA_UPDATE_TRANSFER:
        {
            /* payload */
            uint8_t *p = (uint8_t *)frame->payload;
            board_hw_enable_host_rx(false);
            bool retval = ota_update_write_next(p, frame->len);
            board_hw_enable_host_rx(true);
            if (retval)
            {
                DEBUG_WARN("Do ack\r\n");
                // Send ack
                min_msg_t rsp_msg;
                uint8_t tmp = 0;
                rsp_msg.id = MIN_ID_OTA_UPDATE_TRANSFER; //MIN_ID_OTA_ACK;
                rsp_msg.len = 1;
                rsp_msg.payload = &tmp;
                min_send_frame(&m_min_host_context, &rsp_msg);
            }
            else // Send failed
            {
                DEBUG_WARN("Failed\r\n");
                min_msg_t rsp_msg;
                uint8_t tmp = 1;
                rsp_msg.id = MIN_ID_OTA_UPDATE_TRANSFER ;// MIN_ID_OTA_FAILED;
                rsp_msg.len = 1;
                rsp_msg.payload = &tmp;
                min_send_frame(&m_min_host_context, &rsp_msg);
            }
        }
        break;

        case MIN_ID_OTA_UPDATE_END:
        {
            DEBUG_WARN("OTA end\r\n");
            
            // disable host usart
            board_hw_enable_host_rx(false);
            // usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE);
            uint8_t retval = ota_update_finish(true) ? 0 :1;
//            usart_receive_config(USART_HOST, USART_RECEIVE_ENABLE);
            // Time for master send OTA status to server
            // Send ack
            min_msg_t rsp_msg;
            rsp_msg.id = MIN_ID_OTA_UPDATE_END; //retval ? MIN_ID_OTA_ACK : MIN_ID_OTA_FAILED;
            rsp_msg.len = 1;
            rsp_msg.payload = &retval;
            min_send_frame(&m_min_host_context, &rsp_msg);
            
            board_hw_watchdog_feed();
            sys_delay_ms(2000);
            board_hw_reset();
            while (1);
        }
    //    break;
#endif // OTA_ENABLE
        
        default:
            break;
    }
}


static bool is_vin_detect(void)
{        
    return (m_is_vin_detect == 0) ? false : true;
}


uint8_t fake_power_event = 0;
static void poll_vin_fsm(void)
{
    m_is_vin_detect = 1;
}


static void process_host_power_state(void)
{
    static uint32_t m_power_state_counter = 0;
    
    board_hw_output_set(BOARD_HW_MODULE_RESET, 0);      // dont care about reset pin
    
    if (m_host_die_counter_force_reset_mcu)      // 10000*100ms = 10000s
    {
        m_host_die_counter_force_reset_mcu--;
        if (m_host_die_counter_force_reset_mcu == 0)
        {
            m_host_die_counter_force_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
            board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 0);
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1);
            board_hw_reset();
        }
    }
    
    if (ignore_continues_write_to_flash)
    {
        ignore_continues_write_to_flash--;
    }
    
    if (m_pulse_heartbeat_counter)
    {
        // Reset timeout
        m_pulse_heartbeat_counter--;
        if (m_pulse_heartbeat_counter == 0)
        {
            DEBUG_ERROR("Pulse heartbeat timeout, Shutdown and restart host\r\n");
            m_host_power_state = HOST_POWER_DOWN;
            m_power_state_counter = 0;
            // Save uart lost error code to flash
//            board_hw_critical_log_t err;
//            board_hw_get_critical_error(&err);
//            err.name.is_heartbeat_err = 1;
//            err.name.reset_counter = m_reset_counter;
//            board_hw_set_critical_error(err);  
        }
    }
    
    switch (m_host_power_state)
    {       
        case HOST_POWER_DOWN:
        {         
            // m_skip_serial_data_when_module_power_on = 1;
            // Turn off audio PA
            m_current_output_not_swap_endian.name.pa = 0;
            board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
            
            m_count_host_boot_time = -1;
            // Release pwk to default status   
            board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 0);
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0);
            
            m_pulse_heartbeat_counter = 0;
            
            
            if (!is_vin_detect())
            {
                m_power_state_counter = 0;
            }
            if (m_power_state_counter++ > 40)
            {
                m_power_state_counter = 0;
                DEBUG_WARN("Change pwr state to 'UP'\r\n");
                m_host_power_state = HOST_POWER_UP;
            }
        }
            break;
        
        case HOST_POWER_UP:
        {
            m_current_output_not_swap_endian.name.pa = 0;
            board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1);
            if (!is_vin_detect())
            {
                m_power_state_counter = 0;
            }
            if (m_power_state_counter++ > 40)     // 4s
            {
                m_power_state_counter = 0;
                m_host_power_state = HOST_POWER_PULSE_PWR_KEY;
                board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 1);
                DEBUG_WARN("Change pwr state to 'PULSE PWR KEY'\r\n");
                board_hw_enable_host_rx(false);
            }
            // m_skip_serial_data_when_module_power_on = 1;
        }
            break;

        case HOST_POWER_PULSE_PWR_KEY:
        {
            board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
            if (!is_vin_detect())
            {
                m_power_state_counter = 0;
            }
            
            if (m_power_state_counter++ > 30)     // 3s
            {
                m_skip_serial_data_when_module_power_on = 0;
                m_power_state_counter = 0;
                m_host_power_state = HOST_POWER_RUNNING;
                board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 0);
                m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_IN_RUN_MODE;
                DEBUG_WARN("Change pwr state to 'RUNNING'\r\n");
                m_count_host_boot_time = 0;
                board_hw_enable_host_rx(true);
            }
        }
        break;
        
        case HOST_POWER_RUNNING:
        {   
            if (m_host_keep_alive_uart)       // 1h
            {
                m_host_keep_alive_uart--;
                if (m_host_keep_alive_uart == 0)
                {
                    m_host_keep_alive_uart = DEFAULT_HOST_UART_TIMEOUT_DO_RESET;
                    DEBUG_WARN("Long time not see uart module->reset\r\n");
                    m_host_power_state = HOST_POWER_DOWN;
                    board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0);
                    m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_IN_RUN_MODE;
                    
//                    // Save uart lost error code to flash
//                    board_hw_critical_log_t err;
//                    board_hw_get_critical_error(&err);
//                    err.name.is_uart_communication_error = 1;
//                    board_hw_set_critical_error(err);  
                }
            }
            if (m_count_host_boot_time != -1)
            {
                m_count_host_boot_time++;
            }
        }
            break;
        
        case HOST_POWER_WAIT_FOR_VIN:
        {
            static uint32_t m_wait_for_vin_counter = 0;
            m_wait_for_vin_counter++;
            m_count_host_boot_time = -1;
            
            m_current_output_not_swap_endian.name.pa = 0;
            board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
            
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1); // always on SC20 power
            if (is_vin_detect())
            {
                m_pulse_heartbeat_counter = 0;
                DEBUG_WARN("Change power state to 'DOWN'\r\n");
                m_host_power_state = HOST_POWER_DOWN;
                m_wait_for_vin_counter = 0;
            }
            
            if (m_wait_for_vin_counter == 30*600)       //15p, co the ADC loi
            {
//                board_hw_critical_log_t err;
//                board_hw_get_critical_error(&err);
//                err.name.vin_lost_for_along_time = 1;
//                board_hw_set_critical_error(err);
            }
            
            if (m_wait_for_vin_counter == ((30*600)+20))       //15p, co the ADC loi
            {
                board_hw_reset();
            }
        }
            break;
        
        case HOST_POWER_FLASH_MODE:
            break;
        
        default:
            break;
    }
}


void poll_host_uart_data(void)
{
    uint8_t tmp;
    while (lwrb_read(&m_ring_buffer_host_rx, &tmp, 1))
    {
        min_rx_feed(&m_min_host_context, &tmp, 1);
    }
    min_timeout_poll(&m_min_host_context);
}

static void update_exti_gpio(void)
{
    static uint32_t tick = 0;
    if (sys_get_ms() - tick >= 1234)
    {
        tick = sys_get_ms();
        board_hw_ioex_mode_t new_io;
        // Get current IO value
        new_io = board_hw_ioex_get_current_value();

        // Echo back value
        min_msg_t io_msg;
        io_msg.id = MIN_ID_CTRL_EXT_GPIO;
        io_msg.len = sizeof(new_io);
        io_msg.payload = &new_io;
        min_send_frame(&m_min_host_context, &io_msg);
    }
}

void host_data_layer_polling_task(void *arg)
{
    static uint32_t m_last_time_update_time = 0, m_last_time_measure_input = 0, m_last_time_poll_wdt = 0;
    
    if (sys_get_ms() - m_last_time_measure_input >= (uint32_t)100)
    {
        m_last_time_measure_input = sys_get_ms();
        poll_vin_fsm();
        process_host_power_state();
    }
    
    if (sys_get_ms() - m_last_time_poll_wdt >= (uint32_t)55)
    {
//        update_adc();
//        update_interface();
        
        // Reload watchdog
        m_last_time_poll_wdt = sys_get_ms();
        board_hw_watchdog_feed();
//        update_peripheral_input();
//        if (m_lcd_nextion_enable)
//        {
//            process_rtc_time();
//        }
    }
    
    if (sys_get_ms() - m_last_time_update_time >= (uint32_t)999)
    {    
        m_last_time_update_time = sys_get_ms();
    }
    
    poll_host_uart_data();
//    poll_lcd_fm_uart_data();
//    poll_debug_data();
    update_exti_gpio();
    app_debug_isr_ringbuffer_flush();
}

// Min protocol low layer function, use to send a byte to serial port
static bool uart_tx_to_host(void *ctx, uint8_t data)
{
    (void)ctx;
    host_data_layer_send(&data, 1);
    return true;
}

static inline void host_data_layer_send(uint8_t *data, uint32_t length)
{
    board_hw_host_send(data, length);
}

// 1ms interrupt callback
void host_data_layer_on_1ms_callback(void)
{
}

// Host RX data handler

void board_hw_on_host_rx_callback(uint8_t *data, uint32_t length)
{
    if (length == 0)
    {
        return;
    }
    if (m_skip_serial_data_when_module_power_on)
    {
        lwrb_reset(&m_ring_buffer_host_rx);
        m_usart_monitor_error_counter = 0;
        return;
    }
    // DEBUG_RAW("%02X ", data[0]);
    if (!lwrb_write(&m_ring_buffer_host_rx, data, length))
    {
        DEBUG_ERROR("Ring buffer host full\r\n");
        m_usart_monitor_error_counter++;
        if (m_usart_monitor_error_counter > 30)
        {
            // Maybe hardware uart bug
            // Error when overrun orcus
            board_hw_reset();
        }
        
        lwrb_reset(&m_ring_buffer_host_rx);
    }
}

void board_hw_ping_detect_cb(void)
{
    m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_IN_RUN_MODE;      // 100*10ms timer = 10s
}


void host_data_layer_set_uart_timeout(uint32_t timeout)
{
    m_host_keep_alive_uart = timeout;
}
      

void host_data_layer_set_ping_timeout(uint32_t timeout)
{
    m_pulse_heartbeat_counter = timeout;
}

