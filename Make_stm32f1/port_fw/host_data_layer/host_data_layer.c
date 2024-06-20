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
#define LCD_FM_SCREEN                                       6
#define LCD_LOGIN_SCREEN                                    7
#define LCD_MEETING_SCREEN                                  8

#define TEST_LCD_NEXTION                                    0
#ifndef OTA_ENABLE
#define OTA_ENABLE                                          0
#endif

#define NEXTION_LCD_MAX_FOLDER_COUNT                        4   
#define DEBUG_MIN_PRO                                       0
#define ICON_TYPE_FILE                                      0
#define ICON_TYPE_FOLDER                                    1
#define ICON_TYPE_EMPTY                                     2

#define SHUTDOWN_MODULE_WHEN_POWER_LOST                     1

#define TEST_CLASS_D                                        0

#if TEST_CLASS_D
#define CLASS_D_ACTIVE_LEVEL                                0
#else
#define CLASS_D_ACTIVE_LEVEL                                1
#endif

#if FM_LCD_ENABLE
#define FM_FORWARD_TO_HOST_ENABLE                           0

typedef enum
{
    FM_PROTOCOL_TYPE_UNKNOWN,
    FM_PROTOCOL_TYPE_STRING_FORMAT,
    FM_PROTOCOL_TYPE_MIN_PROTOCOL
} fm_protocol_type_t;

typedef struct
{
    uint8_t buffer[LCD_FM_RX_BUFFER_SIZE];
    uint8_t rx_index;
} lcd_fm_buffer_t;
#endif

static uint8_t m_nextion_end_frame[3] = {0xFF, 0xFF, 0xFF};
static uint32_t m_fm_freq[4] = {965, 100, 1027};
static uint8_t m_fm_vol = 0;
static int8_t m_fm_channel = -1;

// Mode report
static host_data_layer_mode_report_t m_working_report;

// Ring buffer host
static lwrb_t m_ring_buffer_host_rx;
uint8_t m_uart_host_buffer[512+128];
static uint32_t m_min_host_rx_buffer[(MIN_MAX_PAYLOAD+3)/4];
uint32_t m_ring_buffer_host_full = 0;
#if CLI_ENABLE
static void cli_puts(uint8_t *buffer, uint32_t size);
static int cli_printf(const char *msg);
#endif
static int32_t m_count_host_boot_time = -1;
static int32_t m_is_vin_detect = -1;
static inline void host_data_layer_send(uint8_t *data, uint32_t length);
static volatile uint32_t m_usart_monitor_error_counter = 0;
static host_data_layer_output_t m_prev_gpio_output_control =
{
    .value = 0,
};
static volatile uint32_t m_skip_serial_data_when_module_power_on = 0;

#if FM_LCD_ENABLE
static uint8_t m_fm_uart_raw_rx_buffer[LCD_FM_RX_BUFFER_SIZE];
// Ring buffer FM
static lwrb_t m_ring_buffer_fm;
static void on_fm_frame_callback(void *ctx, min_msg_t *frame);
static void on_host_invalid_callback(void *ctx, min_msg_t *frame);
static uint8_t m_min_fm_rx_buffer[MIN_MAX_PAYLOAD];
static void do_ping(void);
void lcd_display_operation_mode(char *mode);
#if TEST_CLASS_D
static bool m_get_class_d_spk = true;
#else
static bool m_get_class_d_spk = false;
static volatile uint32_t m_delay_skip_over_temp = 0;
#endif
// FM buffer
static lcd_fm_buffer_t m_fm_raw_buffer = 
{
    .rx_index = 0
};

// LCD Buffer
static lcd_fm_buffer_t m_nextion_lcd_buffer = 
{
    .rx_index = 0
};

static fm_protocol_type_t m_fm_protocol_type = FM_PROTOCOL_TYPE_UNKNOWN;

// FM min protocol context
static min_context_t m_min_fm_context;
static min_frame_cfg_t m_min_fm_setting = MIN_DEFAULT_CONFIG();

// static void fm_lcd_uart_send(uint8_t *data, uint32_t length);
static bool fm_lcd_uart_send_byte(void *ctx, uint8_t data);
#else
static lwrb_t m_ring_buffer_debug_uart;
static uint8_t m_debug_rx_buffer[64];
#endif

static const uint8_t m_end_of_nextion[3] = {0xFF, 0xFF, 0xFF};
// Host min protocol context
static min_context_t m_min_host_context;
static min_frame_cfg_t m_min_host_setting = MIN_DEFAULT_CONFIG();
static bool uart_tx_to_host(void *ctx, uint8_t data);
static void on_host_frame_callback(void *ctx, min_msg_t *frame);
static void on_host_timeout_callback(void *ctx);
static volatile uint32_t m_last_time_send_ping = 0;
volatile uint32_t m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_AFTER_MCU_BOOT;
static uint8_t do_update_mode_color = 0;

static bool m_test_mode = false;
static uint32_t m_host_keep_alive_uart = DEFAULT_HOST_UART_TIMEOUT_DO_RESET;     // 1h
static uint32_t m_host_die_counter_force_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
static host_power_state_t m_host_power_state = HOST_POWER_RUNNING;
static volatile int32_t m_fm_lcd_usart_idle_timeout = 0;
static uint8_t m_vin_lost_times = 0;
static uint32_t ignore_continues_write_to_flash = 0;
static host_data_layer_ping_msg_t m_host_ping_msg;
static const char *cmd_stop = "SET STOP";
static const char *cmd_fm = "FM";
static const char *cmd_mic_in_line_in = "MIC/LINE IN";
static const char *cmd_internet = "INTERNET";
static const char *cmd_set_mode_master = "SET MODE_M";
static const char *cmd_set_mode_slave = "SET MODE_S";
//static const char *cmd_home = "HOME";
static const char *cmd_setting = "SETTING";
static host_data_layer_output_t m_current_output_not_swap_endian;
// Voltage low pass filter
static void lcd_nextion_write(char *data);
static bool m_lcd_nextion_enable = false;
static uint16_t m_reset_counter = 0;
uint32_t m_serial_frame_err_counter = 0;
uint32_t m_total_serial_frame = 0;
static uint16_t v_adc_endian[BOARD_HW_ADC_COUNTER];

// new variable 
//static gsm_info_t gsm_now;
//static board_status_t status_info_now;
char char_set_recieve[32];

static uint32_t m_fm_lcd_uart_baud;

//void get_gsm_info (gsm_info_t* gsm_info)
//{
//    memcpy (gsm_info, &gsm_now, sizeof (gsm_info_t));
//}
//void get_server_state (uint8_t* state)
//{
//    *state = status_info_now.server_state;
//}

//void get_temperature (uint8_t* temper)
//{
//    *temper = status_info_now.temperature;
//}

static lpf_data_t m_lpf[BOARD_HW_ADC_COUNTER] = 
{
    {
        .gain = 0
    },
    {
        .gain = 0
    },
    {
        .gain = 0
    },
    {
        .gain = 0
    },
};



static uint32_t m_led_blink_indicator_step = 0;

static void update_output()
{
    // Control GPIO
    if (m_current_output_not_swap_endian.name.lte_led_r != m_prev_gpio_output_control.name.lte_led_r)
    {
        DEBUG_VERBOSE("LTE Red -> %d\r\n", m_current_output_not_swap_endian.name.lte_led_r);
    }
    
    if (m_current_output_not_swap_endian.name.lte_led_b != m_prev_gpio_output_control.name.lte_led_b)
    {
        DEBUG_VERBOSE("LTE B -> %d\r\n", m_current_output_not_swap_endian.name.lte_led_b);
    }
    
    if (m_current_output_not_swap_endian.name.eth_led_r != m_prev_gpio_output_control.name.eth_led_r)
    {
        DEBUG_VERBOSE("ETH Red -> %d\r\n", m_current_output_not_swap_endian.name.eth_led_r);
    }
    
    if (m_current_output_not_swap_endian.name.eth_led_b != m_prev_gpio_output_control.name.eth_led_b)
    {
        DEBUG_VERBOSE("ETH B -> %d\r\n", m_current_output_not_swap_endian.name.eth_led_b);
    }
    
    if (m_current_output_not_swap_endian.name.relay1 != m_prev_gpio_output_control.name.relay1)
    {
        DEBUG_VERBOSE("RL 1 -> %d\r\n", m_current_output_not_swap_endian.name.relay1);
    }
    
    if (m_current_output_not_swap_endian.name.relay2 != m_prev_gpio_output_control.name.relay2)
    {
        DEBUG_VERBOSE("RL 2 -> %d\r\n", m_current_output_not_swap_endian.name.relay2);
    }
    
    if (m_current_output_not_swap_endian.name.pa != m_prev_gpio_output_control.name.pa)
    {
        DEBUG_VERBOSE("PA -> %d\r\n", m_current_output_not_swap_endian.name.pa);
    }
    
    if (m_current_output_not_swap_endian.name.switch_in != m_prev_gpio_output_control.name.switch_in)
    {
        DEBUG_VERBOSE("SW IN -> %d\r\n", m_current_output_not_swap_endian.name.switch_in);
    }
    
    if (m_current_output_not_swap_endian.name.switch_out != m_prev_gpio_output_control.name.switch_out)
    {
        DEBUG_VERBOSE("SW OUT -> %d\r\n", m_current_output_not_swap_endian.name.switch_out);
    }
    
    if (m_current_output_not_swap_endian.name.sw_mic != m_prev_gpio_output_control.name.sw_mic)
    {
        DEBUG_VERBOSE("SW MIC -> %d\r\n", m_current_output_not_swap_endian.name.sw_mic);
    }
    
    board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, m_current_output_not_swap_endian.name.lte_led_r);
    board_hw_output_set(BOARD_HW_OUTPUT_LED_1_B, m_current_output_not_swap_endian.name.lte_led_b);
    board_hw_output_set(BOARD_HW_OUTPUT_LED_2_R, m_current_output_not_swap_endian.name.eth_led_r);
    board_hw_output_set(BOARD_HW_OUTPUT_LED_2_B, m_current_output_not_swap_endian.name.eth_led_b);
    board_hw_output_set(BOARD_HW_OUTPUT1, m_current_output_not_swap_endian.name.relay1);
    board_hw_output_set(BOARD_HW_OUTPUT2, m_current_output_not_swap_endian.name.relay2);
    if (m_current_output_not_swap_endian.name.pa)
    {
        board_hw_output_set(BOARD_HW_OUTPUT_PA, 1);
    }
    else if (m_current_output_not_swap_endian.name.class_d == CLASS_D_ACTIVE_LEVEL && m_get_class_d_spk == false)     // class D & ko detect loa
    {
        board_hw_output_set(BOARD_HW_OUTPUT_PA, 0);
    }
    else if (m_current_output_not_swap_endian.name.class_d != CLASS_D_ACTIVE_LEVEL)     // Class AB
    {
        board_hw_output_set(BOARD_HW_OUTPUT_PA, 0);
    }
    
    board_hw_output_set(BOARD_HW_OUTPUT_SW_IN, m_current_output_not_swap_endian.name.switch_in); 
    board_hw_output_set(BOARD_HW_OUTPUT_SW_OUT, m_current_output_not_swap_endian.name.switch_out);  \
    board_hw_output_set(BOARD_HW_OUTPUT_SW_MIC, m_current_output_not_swap_endian.name.sw_mic);
//    DEBUG_VERBOSE("PA %s\r\n", m_current_output_not_swap_endian.name.pa ? "TRUE" : "FALSE");
//    DEBUG_INFO("SW IN %s, SWOUT %s\r\n", m_current_output_not_swap_endian.name.switch_in ? "TRUE" : "FALSE",
//                                        m_current_output_not_swap_endian.name.switch_out ? "TRUE" : "FALSE");
    
}   


static void update_interface()
{
//    int index = 0;
//    static uint8_t if_changed = 0;
//    char tmp[32];
//    char output[32];
//    
////    gsm_info_t gsm_info_now;
//    
//    if (m_host_ping_msg.screen_id != LCD_HOME_SCREEN || m_lcd_nextion_enable == false)
//    {
//        if_changed = 50;
//        return;
//    }
//    
//    memset(tmp, 0, 32);
//    memset(output, 0, 32);
//    
//    if (m_current_output_not_swap_endian.name.if_4g != m_prev_gpio_output_control.name.if_4g)
//    {
//        DEBUG_WARN("4G -> %d\r\n", m_current_output_not_swap_endian.name.if_4g);
//        if_changed = 50;
//        m_prev_gpio_output_control.name.if_4g = m_current_output_not_swap_endian.name.if_4g;
//    }
//    
//    if (m_current_output_not_swap_endian.name.if_wifi != m_prev_gpio_output_control.name.if_wifi)
//    {
//        DEBUG_WARN("WIFI -> %d\r\n", m_current_output_not_swap_endian.name.if_wifi);
//        if_changed = 50;
//    }
//    
//    if (m_current_output_not_swap_endian.name.if_eth != m_prev_gpio_output_control.name.if_eth)
//    {
//        DEBUG_WARN("ETH -> %d\r\n", m_current_output_not_swap_endian.name.if_eth);
//        if_changed = 50;
//    }
//    
//    if (++if_changed < 50)
//    {
//        return;
//    }
//    
//    if (m_current_output_not_swap_endian.name.if_4g)
//    {
//        index += sprintf(tmp+index, "%s", "4G/");
//    }
//    
//    if (m_current_output_not_swap_endian.name.if_wifi)
//    {
//        index += sprintf(tmp+index, "%s", "WIFI/");
//    }
//    
//    if (m_current_output_not_swap_endian.name.if_eth)
//    {
//        index += sprintf(tmp+index, "%s", "ETH/");
//    }
//    
//    if (m_working_report.csq)
//    {
//        index += sprintf(tmp+index, "B%d/Q=%d/", m_working_report.band, m_working_report.csq);
//    }
//    
//    if (index)
//    {
//        tmp[index-1] = 0;       // remove '/'
//    }
//    else
//    {
//        index += sprintf(tmp, "%s", "...");
//    }

//    
//    if_changed = 0;
//    sprintf(output, "t1.txt=\"%s\"", tmp);
//    board_hw_fm_lcd_send((uint8_t*)output, strlen(output));
//    board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//    
////    if (m_working_report.type != 0)
////    {
////        sprintf(tmp, "t12.txt=\"%d\"", m_working_report.temperature); 
////        lcd_nextion_write(tmp);
////    }
}

static void update_operation_mode()
{
    static uint8_t mode_changed = 0;
    static const char *mode[16] = {"IDLE", "PREPARE", "RUNNING", "STOP", "MIC/LINE", "FM", "NO OPERATION", "..."};


#if TEST_LCD_NEXTION
    m_current_output_not_swap_endian.name.nextion_enable = 1;
#endif
    // DEBUG_WARN("Nextion mode -> %d/%d\r\n", m_current_output_not_swap_endian.name.nextion_enable, m_prev_gpio_output_control.name.nextion_enable );
    if (m_current_output_not_swap_endian.name.nextion_enable != m_prev_gpio_output_control.name.nextion_enable
        || (m_lcd_nextion_enable == false && m_current_output_not_swap_endian.name.nextion_enable))
    {
        mode_changed = 11;
        do_update_mode_color = 11;
        DEBUG_WARN("Nextion mode -> %d\r\n", m_current_output_not_swap_endian.name.nextion_enable);
        
#if TEST_LCD_NEXTION
        m_prev_gpio_output_control.name.nextion_enable = 1;
#endif  
        
        if (m_current_output_not_swap_endian.name.nextion_enable && m_lcd_nextion_enable == false)
        {
            DEBUG_WARN("Change FM lcd baudrate to %u\r\n", BOARD_HW_NEXTION_BAUD);
            m_lcd_nextion_enable = true;
            board_hw_change_lcd_fm_baudrate(BOARD_HW_NEXTION_BAUD);
        }
        else if (m_current_output_not_swap_endian.name.nextion_enable == false && m_lcd_nextion_enable == true)
        {
            DEBUG_WARN("Change FM lcd baudrate to %u\r\n", BOARD_HW_FM_BAUD);
            m_lcd_nextion_enable = false;
            board_hw_change_lcd_fm_baudrate(BOARD_HW_FM_BAUD);
        }
        
    }
    else
    {
        
    }
    
    if (m_current_output_not_swap_endian.name.operation_mode != m_prev_gpio_output_control.name.operation_mode)
    {
        DEBUG_WARN("Operation mode -> %d\r\n", m_current_output_not_swap_endian.name.operation_mode);
        mode_changed = 11;
        do_update_mode_color = 11;
    }
    
    if (++mode_changed >= 11 
        && m_current_output_not_swap_endian.name.nextion_enable 
        && m_host_power_state == HOST_POWER_RUNNING)
    {
        mode_changed = 0;
        // Chi xu li LCD operation mode khi screen id = LCD_HOME_SCREEN
        if (m_host_ping_msg.screen_id == LCD_HOME_SCREEN)
        {
            lcd_display_operation_mode((char*)mode[m_current_output_not_swap_endian.name.operation_mode]);
        }    
    }
    
    if (do_update_mode_color++ > 11 && m_current_output_not_swap_endian.name.nextion_enable)
    {
        do_update_mode_color = 0;
        // Chi xu li LCD operation mode khi screen id = LCD_HOME_SCREEN
        if (m_host_ping_msg.screen_id == LCD_SETTING_SCREEN)
        {
//            static const char *highlight_background = "1032";
//            static const char *normal_background = "50712";
//            static const char * t32Color = "t32.bco=";
//            static const char * t33Color = "t33.bco=";
//            static const char * t34Color = "t34.bco=";
//            static const char * t35Color = "t35.bco=";
//            static const char * t36Color = "t36.bco=";
//            static const char * t37Color = "t37.bco=";
//            
//            board_hw_fm_lcd_send((uint8_t*)t32Color, strlen(t32Color));
//            if (m_current_output_not_swap_endian.name.operation_mode <= 3)
//            {
//                board_hw_fm_lcd_send((uint8_t*)highlight_background, strlen(highlight_background));
//            } 
//            else
//            {
//                board_hw_fm_lcd_send((uint8_t*)normal_background, strlen(normal_background));
//            }
//            board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//            
//            board_hw_fm_lcd_send((uint8_t*)t33Color, strlen(t33Color));
//            if (m_current_output_not_swap_endian.name.operation_mode == STREAM_STATE_MIC_LINE)
//            {
//                board_hw_fm_lcd_send((uint8_t*)highlight_background, strlen(highlight_background));
//            } 
//            else
//            {
//                board_hw_fm_lcd_send((uint8_t*)normal_background, strlen(normal_background));
//            }
//            board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//            
//            board_hw_fm_lcd_send((uint8_t*)t34Color, strlen(t34Color));
//            if (m_current_output_not_swap_endian.name.operation_mode == STREAM_STATE_NO_OPERATION)
//            {
//                board_hw_fm_lcd_send((uint8_t*)highlight_background, strlen(highlight_background));
//            } 
//            else
//            {
//                board_hw_fm_lcd_send((uint8_t*)normal_background, strlen(normal_background));
//            }
//            board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//            
//            board_hw_fm_lcd_send((uint8_t*)t35Color, strlen(t35Color));
//            if (m_current_output_not_swap_endian.name.operation_mode == STREAM_STATE_FM)
//            {
//                board_hw_fm_lcd_send((uint8_t*)highlight_background, strlen(highlight_background));
//            } 
//            else
//            {
//                board_hw_fm_lcd_send((uint8_t*)normal_background, strlen(normal_background));
//            }
//            board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//            
//            // Set mode
//            if (m_working_report.type == 1)     // slave
//            {
//                board_hw_fm_lcd_send((uint8_t*)t37Color, strlen(t37Color));
//                board_hw_fm_lcd_send((uint8_t*)highlight_background, strlen(highlight_background));
//                board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//                
//                board_hw_fm_lcd_send((uint8_t*)t36Color, strlen(t36Color));
//                board_hw_fm_lcd_send((uint8_t*)normal_background, strlen(normal_background));
//                board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//            }
//            else
//            {
//                board_hw_fm_lcd_send((uint8_t*)t36Color, strlen(t36Color));
//                board_hw_fm_lcd_send((uint8_t*)highlight_background, strlen(highlight_background));
//                board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//                
//                board_hw_fm_lcd_send((uint8_t*)t37Color, strlen(t37Color));
//                board_hw_fm_lcd_send((uint8_t*)normal_background, strlen(normal_background));
//                board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
//            }
        }
        else if (m_host_ping_msg.screen_id == LCD_FM_SCREEN)                    
        {
//            if (m_fm_channel != -1)
//            {
//                char fm_freq_text[24];
//                sprintf(fm_freq_text, "t52.txt=\"%d.%d\"", m_fm_freq[0]/10, m_fm_freq[0]%10);
//                lcd_nextion_write(fm_freq_text);
//                if (m_fm_channel == 0)
//                {
//                    lcd_nextion_write("t52.bco=1032");
//                }
//                else
//                {
//                    lcd_nextion_write("t52.bco=50712");
//                }
//                
//                if (m_fm_channel == 1)
//                {
//                    lcd_nextion_write("t50.bco=1032");
//                }
//                else
//                {
//                    lcd_nextion_write("t50.bco=50712");
//                }
//                
//                if (m_fm_channel == 2)
//                {
//                    lcd_nextion_write("t53.bco=1032");
//                }
//                else
//                {
//                    lcd_nextion_write("t53.bco=50712");
//                }
//                                
//                sprintf(fm_freq_text, "t50.txt=\"%d.%d\"", m_fm_freq[1]/10, m_fm_freq[1]%10);
//                lcd_nextion_write(fm_freq_text);
//                
//                sprintf(fm_freq_text, "t53.txt=\"%d.%d\"", m_fm_freq[2]/10, m_fm_freq[2]%10);
//                lcd_nextion_write(fm_freq_text);
//                
//                // color
//                
//                // Vol FM
//                sprintf(fm_freq_text, "t51.txt=\"%d%%\"", m_fm_vol);
//                lcd_nextion_write(fm_freq_text);
//            }
        }
    }
}


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
    m_min_host_setting.invalid_crc_callback = on_host_invalid_callback;
    m_min_host_setting.timeout_callback = on_host_timeout_callback;
    m_min_host_setting.timeout_not_seen_rx = 7;
    m_min_host_setting.tx_byte = uart_tx_to_host;
    m_min_host_setting.use_timeout_method = 1;

    m_min_host_context.callback = &m_min_host_setting;
    m_min_host_context.rx_frame_payload_buf = (uint8_t*)m_min_host_rx_buffer;
    min_init_context(&m_min_host_context);
    
   
#if FM_LCD_ENABLE
    lwrb_init(&m_ring_buffer_fm, m_fm_uart_raw_rx_buffer, sizeof(m_fm_uart_raw_rx_buffer));
    // FM
    m_min_fm_setting.get_ms = sys_get_ms;
    m_min_fm_setting.last_rx_time = 0x00;
    /* Not using the rx_callback then*/
    m_min_fm_setting.rx_callback = on_fm_frame_callback;
    m_min_fm_setting.invalid_crc_callback = on_fm_frame_callback;
    m_min_fm_setting.timeout_not_seen_rx = 10;
    m_min_fm_setting.tx_byte = fm_lcd_uart_send_byte;
    m_min_fm_setting.use_timeout_method = 1;
    //m_min_fm_context.cb = &m_min_fm_setting;
    m_min_fm_context.rx_frame_payload_buf = m_min_fm_rx_buffer;
    m_min_fm_context.callback = &m_min_fm_setting;
    min_init_context(&m_min_fm_context);
    
#else   // init buffer for serial debugging
    lwrb_init(&m_ring_buffer_debug_uart, m_debug_rx_buffer, sizeof(m_debug_rx_buffer));
#endif // FM_LCD_ENABLE
    
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
#if DIGITAL_POT
    board_hw_digital_pot_set(100);
#endif
    board_hw_critical_log_t err;
    if (board_hw_get_critical_error(&err))
    {
        if (err.name.is_heartbeat_err)
        {
            DEBUG_ERROR("Ping IO err\r\n");
        }
        if (err.name.is_uart_communication_error)
        {
            DEBUG_ERROR("UART communication err\r\n");
        }
        if (err.name.is_vin_lost)
        {
            DEBUG_ERROR("Vin lost err\r\n");
        }
        m_reset_counter = err.name.reset_counter+1;
    }
    else
    {
        err.value = 0;
    }

    m_current_output_not_swap_endian.name.pa = 0;
    board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
}

#if FM_LCD_ENABLE
static void on_fm_frame_callback(void *ctx, min_msg_t *frame)
{
    m_fm_protocol_type = FM_PROTOCOL_TYPE_MIN_PROTOCOL;
    DEBUG_VERBOSE("Received FM frame id %u, size %u\r\n", frame->id, frame->len);
#if FM_FORWARD_TO_HOST_ENABLE
    switch (frame->id)
    {
        case MIN_ID_PING:
        case MIN_ID_FORWARD:
        {
            min_msg_t msg;
            msg.id = MIN_ID_FORWARD;
            msg.len = frame->len;
            msg.payload = frame->payload;
            min_send_frame(&m_min_host_context, &msg);
        }
            break;
        
        default:
            break;
    }
#endif // FM_FORWARD_TO_HOST_ENABLE
}
#endif // FM_LCD_ENABLE


static void update_adc(void)
{    
    if (!board_hw_has_new_adc_data())
    {
        return;
    }
    uint16_t *adc_ptr = board_hw_get_adc_data();
    uint16_t adc_buffer[BOARD_HW_ADC_COUNTER];
    for (int i = 0; i < BOARD_HW_ADC_COUNTER; i++)
    {  
        adc_buffer [i] = adc_ptr[i];
    }
    
    board_hw_allow_adc_conversion();
    
//    DEBUG_INFO("ADC RAW %d, %d, %d\r\n", 
//                    adc_buffer[V_ADC_ENDIAN_VIN_INDEX], 
//                    adc_buffer[V_ADC_ENDIAN_3V8_INDEX], 
//                    adc_buffer[V_ADC_ENDIAN_5V_INDEX]);
    uint32_t tmp = adc_buffer[V_ADC_ENDIAN_VIN_INDEX]*11;      // 11 = resistor div
    tmp *= 3300;
    tmp /= 4096;

    if (m_lpf[V_ADC_ENDIAN_VIN_INDEX].gain == 0)
    {
        m_lpf[V_ADC_ENDIAN_VIN_INDEX].gain = 20;     // percent
        m_lpf[V_ADC_ENDIAN_VIN_INDEX].estimate_value = tmp;
    }
    else
    {
        lpf_update_estimate(&m_lpf[V_ADC_ENDIAN_VIN_INDEX], (int32_t*)&tmp);
    }
//    
    m_lpf[V_ADC_ENDIAN_VIN_INDEX].estimate_value = tmp;
    
    v_adc_endian[V_ADC_ENDIAN_VIN_INDEX] = m_lpf[V_ADC_ENDIAN_VIN_INDEX].estimate_value + 300;      // 200mv = diode offset
    
    // Swap Vin endian
    m_host_ping_msg.vin_mv = ((v_adc_endian[V_ADC_ENDIAN_VIN_INDEX] & 0xFF) << 8) | (v_adc_endian[V_ADC_ENDIAN_VIN_INDEX] >> 8);
    
    
    // Do low pass filter for V3V8
    tmp = adc_buffer[V_ADC_ENDIAN_3V8_INDEX]*2;            // 2 = resistor div
    tmp *= 3300;
    tmp /= 4096;
    
    if (m_lpf[V_ADC_ENDIAN_3V8_INDEX].gain == 0)
    {
        m_lpf[V_ADC_ENDIAN_3V8_INDEX].gain = 5;     // percent
        m_lpf[V_ADC_ENDIAN_3V8_INDEX].estimate_value = tmp;
    }
    else
    {
        lpf_update_estimate(&m_lpf[V_ADC_ENDIAN_3V8_INDEX], (int32_t*)&tmp);
    }
    // Swap 3v8 endian
    v_adc_endian[V_ADC_ENDIAN_3V8_INDEX] = m_lpf[V_ADC_ENDIAN_3V8_INDEX].estimate_value;
    m_host_ping_msg.v3v8_mv = ((v_adc_endian[V_ADC_ENDIAN_3V8_INDEX] & 0xFF) << 8) | (tmp >> 8);
    
   

    // Do low pass filter for bus 24V
    tmp = adc_buffer[V_ADC_ENDIAN_5V_INDEX]*2;            // 2 = resistor div
    tmp *= 3300;
    tmp /= 4096;
    
    if (m_lpf[V_ADC_ENDIAN_5V_INDEX].gain == 0)
    {
        m_lpf[V_ADC_ENDIAN_5V_INDEX].gain = 10;     // percent
        m_lpf[V_ADC_ENDIAN_5V_INDEX].estimate_value = tmp;
    }
    else
    {
        lpf_update_estimate(&m_lpf[V_ADC_ENDIAN_5V_INDEX], (int32_t*)&tmp);
    }
    
    // Swap 5V endian
    v_adc_endian[V_ADC_ENDIAN_5V_INDEX] = m_lpf[V_ADC_ENDIAN_5V_INDEX].estimate_value;      // 200mv = diode offset
    m_host_ping_msg.vbus_24v_mv = ((v_adc_endian[V_ADC_ENDIAN_5V_INDEX] & 0xFF) << 8) | (tmp >> 8);
    
    tmp = adc_buffer[V_ADC_ENDIAN_VAR_RESISTOR_INDEX];
    tmp *= 3300;
    tmp /= 4096;

    if (m_lpf[V_ADC_ENDIAN_VAR_RESISTOR_INDEX].gain == 0)
    {
        m_lpf[V_ADC_ENDIAN_VAR_RESISTOR_INDEX].gain = 10;     // percent
        m_lpf[V_ADC_ENDIAN_VAR_RESISTOR_INDEX].estimate_value = tmp;
    }
    else
    {
        lpf_update_estimate(&m_lpf[V_ADC_ENDIAN_VAR_RESISTOR_INDEX], (int32_t*)&tmp);
    }
    
    // Analog resistor
    v_adc_endian[V_ADC_ENDIAN_VAR_RESISTOR_INDEX] = m_lpf[V_ADC_ENDIAN_VAR_RESISTOR_INDEX].estimate_value;
    // m_host_ping_msg.v_resistor = ((v_adc_endian[V_ADC_ENDIAN_VAR_RESISTOR_INDEX] & 0xFF) << 8) | (tmp >> 8);
    
    if (m_test_mode  || 1)
    {
        static uint32_t m_log = 0;
        if (m_log++ % 100 == 0 | m_is_vin_detect == 0)
        {
            DEBUG_INFO("Vin %u, v5v %u, 3v8 %u\r\n", 
                        v_adc_endian[V_ADC_ENDIAN_VIN_INDEX], 
                        v_adc_endian[V_ADC_ENDIAN_5V_INDEX], 
                        v_adc_endian[V_ADC_ENDIAN_3V8_INDEX]);
        }
    }
}

void update_peripheral_input(void)
{        
    static host_data_layer_input_t m_last_input;
    m_host_ping_msg.input.value = 0;
    m_host_ping_msg.input.name.input0 = board_hw_get_input(BOARD_HW_INPUT0);
    m_host_ping_msg.input.name.input1 = board_hw_get_input(BOARD_HW_INPUT1);
    m_host_ping_msg.input.name.input2 = board_hw_get_input(BOARD_HW_INPUT2);
    m_host_ping_msg.input.name.input3 = board_hw_get_input(BOARD_HW_INPUT3);
    m_host_ping_msg.input.name.button_on_air = board_hw_get_input(BOARD_HW_INPUT_BUTTON_ON_AIR);
    
    
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
    m_test_mode = test;
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

static void led_blink_when_host_die(void)
{
#if 0
    switch (m_led_blink_indicator_step)
    {
        case 0:
            m_led_blink_indicator_step++;
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_B, 0);
            break;
        
        case 1:
            m_led_blink_indicator_step++;
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 1);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_B, 0);
            break;
        
        case 2:
            m_led_blink_indicator_step++;
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 1);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_B, 0);
            break;
        
        case 3:
            m_led_blink_indicator_step++;
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_R, 1);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_B, 0);
            break;
        
        case 4:
            m_led_blink_indicator_step++;
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_R, 0);
            board_hw_output_set(BOARD_HW_OUTPUT_LED_2_B, 1);
            break;
        
        default:
            m_led_blink_indicator_step = 0;
            break;
    }
#else
    (void)m_led_blink_indicator_step;
#endif
}

int32_t find_index_of_char(char char_to_find, char *buffer_to_find)
{
    uint32_t tmp_cnt = 0;
    uint32_t max_length = 0;

    /* Do dai du lieu */
    max_length = strlen(buffer_to_find);

    for (tmp_cnt = 0; tmp_cnt < max_length; tmp_cnt++)
    {
        if (buffer_to_find[tmp_cnt] == char_to_find)
        {
            return tmp_cnt;
        }
    }
    return -1;
}


static void on_host_invalid_callback(void *ctx, min_msg_t *frame)
{
    //DEBUG_ERROR("Invalid CRC [%d] bytes\r\n%.*s\r\n", frame->len, frame->len, frame->payload);
    m_serial_frame_err_counter++;
#if DEBUG_MIN_PRO
    if (m_total_serial_frame)
    {
        DEBUG_WARN("Min invalid CRC error rate %u/%u = %u%%\r\n", 
                    m_serial_frame_err_counter, m_total_serial_frame, 
                    (m_serial_frame_err_counter*100)/m_total_serial_frame);
    }
#endif
}

static void on_host_timeout_callback(void *ctx)
{
    m_serial_frame_err_counter++;
#if DEBUG_MIN_PRO
    if (m_total_serial_frame)
    {
        DEBUG_WARN("Min timeout callback error rate %u/%u = %u%%\r\n", 
                    m_serial_frame_err_counter, m_total_serial_frame, 
                    (m_serial_frame_err_counter*100)/m_total_serial_frame);
    }
#endif
}

static void lcd_nextion_write(char *data)
{
    board_hw_fm_lcd_send((uint8_t*)data, strlen(data));
    board_hw_fm_lcd_send((uint8_t*)m_nextion_end_frame, 3);
}

uint32_t utilities_get_number_from_string(uint16_t index, char *buffer)
{
	if (!buffer)
	{
		return 0;
	}
    uint32_t value = 0;
    uint16_t tmp = index;

    while (buffer[tmp] && tmp < 1024)
    {
        if (buffer[tmp] >= '0' && buffer[tmp] <= '9')
        {
            value *= 10;
            value += buffer[tmp] - 48;
        }
        else
        {
            break;
        }
        tmp++;
    }

    return value;
}

// Directory
void lcd_set_folder_directory(char *name)
{
    if (!name)
    {
        return;
    }
    char tmp[LCD_MAX_DIRECTORY_NAME_LEN+16];
    
    // Set static text
    snprintf(tmp, sizeof(tmp), "p4t2.txt=\"%s\"", name);
    lcd_nextion_write(tmp);  
      
    return;
}

//void lcd_set_volume_on_meeting_page(uint8_t vol)
//{
//    char tmp[32];
//    sprintf(tmp, "p5t13.txt=\"%u\"", vol); 
//    lcd_nextion_write(tmp);
//}

void lcd_set_checkbox_rom(uint8_t num, uint8_t meet_status)
{
    if (num > MAX_ROOM_SUPPORT || meet_status > 2)
    {
        return;
    }
    
    char tmp[32];
    static uint16_t select_color[3] = {1032, 14856, 528};
    sprintf(tmp, "p5t%d.bco=%d", select_color[num], meet_status); 
    lcd_nextion_write(tmp);
}


void lcd_set_room_name(uint8_t room_num, char *name)
{
    if (!name)
    {
        return;
    }
    
    if (room_num >= MAX_ROOM_SUPPORT)
    {
        return;
    }
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "p5t%d.txt=\"%s\"", room_num, name);
    lcd_nextion_write(tmp);
}

void lcd_set_folder_name(uint8_t folder_count, char *value)
{
    if (folder_count > NEXTION_LCD_MAX_FOLDER_COUNT)
    {
        return;
    }
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "p4g%d.txt=\"%s\"", 1+folder_count, value);
    lcd_nextion_write(tmp);
    
//    // hide icon
//    if (strlen(value) < 2 && value[0] == ' ')
//    {
//        snprintf(tmp, sizeof(tmp), "p4p%d.pic=%u", 4+folder_count, 32);
//        lcd_nextion_write(tmp);
//    }
}

static uint8_t m_folder_type[NEXTION_LCD_MAX_FOLDER_COUNT] = {0, 0, 0, 0};
static uint8_t m_folder_select[NEXTION_LCD_MAX_FOLDER_COUNT] = {0, 0, 0, 0};

void lcd_update_folder(uint8_t folder_count)
{
    char tmp[32];
    
    if (m_folder_select[folder_count] == 1 && m_folder_type[folder_count] == 1)
    {
        //    char tmp[32];
        snprintf(tmp, sizeof(tmp), "p4p%d.pic=%u", 4+folder_count, 76);
    }
    else if (m_folder_select[folder_count] == 1 && m_folder_type[folder_count] == 0)
    {
        snprintf(tmp, sizeof(tmp), "p4p%d.pic=%u", 4+folder_count, 74);
    }
    else if (m_folder_select[folder_count] == 0 && m_folder_type[folder_count] == 1)
    {
        snprintf(tmp, sizeof(tmp), "p4p%d.pic=%u", 4+folder_count, 77);
    }
    else if (m_folder_select[folder_count] == 0 && m_folder_type[folder_count] == 0)
    {
        snprintf(tmp, sizeof(tmp), "p4p%d.pic=%u", 4+folder_count, 75);
    }
    else        // hide
    {
        snprintf(tmp, sizeof(tmp), "p4p%d.pic=%u", 4+folder_count, 78);
    }
    lcd_nextion_write(tmp);
}

void lcd_set_folder_icon(uint8_t folder_count, uint8_t value)
{
    if (value > 2 || folder_count > 4)
    {
        return;
    }
//    
//    uint8_t pic = 0;
//    if (value == 2)
//    {
//        // hide
//        pic = 32;   
//    }
//    else if (value == 1)
//    {
//        pic = 11;
//    }
//    else
//    {
//        pic = 16;
//    }
//    

    m_folder_type[folder_count] = value;
    lcd_update_folder(folder_count);
}

void lcd_set_folder_select(uint8_t folder_count, uint8_t value)
{
    if (value > 1 || folder_count > 4)
    {
        return;
    }

    m_folder_select[folder_count] = value;
    lcd_update_folder(folder_count);
}

void lcd_set_now_playing_song(char *song)
{
    board_hw_fm_lcd_send((uint8_t*)"p4g5.txt=\"", 10);
    board_hw_fm_lcd_send((uint8_t*)(uint8_t*)song, strlen(song));
    board_hw_fm_lcd_send((uint8_t*)"\"", 1);
    board_hw_fm_lcd_send((uint8_t*)m_end_of_nextion, 3);
}

void lcd_set_music_loop(uint8_t is_loop)
{
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "p4p11.pic=%u", is_loop ? 51 : 53);
    lcd_nextion_write(tmp);
}

void lcd_set_music_spk(uint8_t enable)
{
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "p4p10.pic=%u", enable ? 52 : 48);
    lcd_nextion_write(tmp);
}

static uint8_t m_last_audio_status = 0;
void lcd_set_music_playing_icon(uint8_t status)
{
    // 0 = stop, 1 =  playing, 2 = pause
    m_last_audio_status = status;
    if (status == 2)
    {
        lcd_nextion_write("p4p3.pic=69");
    }
    else if (status == 1)
    {
        lcd_nextion_write("p4p3.pic=58");
    }
    else if (status == 0)
    {
        lcd_nextion_write("p4p3.pic=50");
    }
}

void lcd_set_music_playing_text_status(uint8_t status)
{
    // 0 = stop, 1 =  playing, 2 = pause
    if (status == 2)
    {
        lcd_nextion_write("p4t3.txt=\"DỪNG PHÁT\"");
    }
    else if (status == 1)
    {
        lcd_nextion_write("p4t3.txt=\"ĐANG PHÁT\"");
    }
    else if (status == 0)
    {
        lcd_nextion_write("p4t3.txt=\"KHÔNG PHÁT\"");
    }
}

void lcd_set_storage(uint8_t type)
{
    if (type > 2)
    {
        return;
    }
    
    if (type == 0)  // sdc
    {
        lcd_nextion_write("p4t34.pic=36");
        lcd_nextion_write("p4t35.pic=63");
        lcd_nextion_write("p4t36.pic=60");
    }
    else if (type == 1)  // usb
    {
        lcd_nextion_write("p4t34.pic=36");
        lcd_nextion_write("p4t35.pic=62");
        lcd_nextion_write("p4t36.pic=61");
    }
    else if (type == 2) //mmc
    {
        lcd_nextion_write("p4t34.pic=35");
        lcd_nextion_write("p4t35.pic=63");
        lcd_nextion_write("p4t36.pic=61");
    }
}

void lcd_set_call_icon(uint8_t call_state)
{
    // Call state
    // 0 - free, 1 = establishing, 2 = in call
    DEBUG_VERBOSE("Set call state %u\r\n", call_state);
    if (call_state > 3)
    {
        return;
    }
    
    char tmp[32];
    uint8_t pic;
    
    if (call_state == 0)
    {
        pic = 24; 
    }
    else if (call_state == 1)
    {
        pic = 27; 
    }
    else if (call_state == 2)
    {
        pic = 25; 
    }
    else if (call_state == 3)
    {
        pic = 26; 
    }
    
    sprintf(tmp, "p5p0.pic=%d", pic);
    lcd_nextion_write(tmp);
}

void lcd_set_mic_mute_icon(bool enable)
{
    char tmp[32];
    sprintf(tmp, "p5p10.pic=%d", enable ? 52 : 48);
    lcd_nextion_write(tmp);
}

bool utilities_copy_parameters(char *src, char *dst, char comma_begin, char comma_end)
{
    int16_t begin_idx = find_index_of_char(comma_begin, src);
    int16_t end_idx;
    if (comma_begin == comma_end)
    {
        end_idx = find_index_of_char(comma_end, src+1);
        if (end_idx != -1)
        {
            end_idx++;
        }
    }
    else
    {
        end_idx = find_index_of_char(comma_end, src);
    }
    int16_t tmp_cnt, i = 0;

    if (begin_idx == -1 || end_idx == -1)
    {
        return false;
    }

    if (end_idx - begin_idx <= 1)
    {
        return false;
    }

    for (tmp_cnt = begin_idx + 1; tmp_cnt < end_idx; tmp_cnt++)
    {
        dst[i++] = src[tmp_cnt];
    }

    dst[i] = 0;

    return true;
}

void process_ui_msg(char *msg)
{
    // https://docs.google.com/spreadsheets/d/1CnNCpZJ-taqqqL4yKp-eGKMem7It03XueaNuvZ0fp-Q/edit#gid=2071582144
    char *q;
    char *p = msg;
    static char value[256];
    DEBUG_INFO("UI -> %s\r\n", msg);
    snprintf(value, 256, "Host -> LCD : %s\r\n", msg);
    
    // tcp_console_puts(msg);
    // Volume
//    q = strstr(p, "vol=\"");
//    if (q)
//    {
//        q += strlen("vol=\"");
//        if (utilities_copy_parameters(q-1, value, '"', '"'))
//        {
//            lcd_set_volume_on_meeting_page(atoi(value));
//            DEBUG_VERBOSE("LCD set vol %s\r\n", value);
//        }
//    }

//    // IP address
//    q = strstr(p, "ip=\"");
//    if (q)
//    {
//        q += strlen("ip=\"");
//        if (utilities_copy_parameters(q-1, value, '"', '"'))
//        {
//            lcd_set_ip(value);
//            DEBUG_WARN("LCD set IP %s\r\n", value);
//        }
//    }

//    // ROOM
//    for (uint32_t room = 0; room < MAX_ROOM_SUPPORT; room++)
//    {
//        char room_key[12];
//        sprintf(room_key, "r%d=\"", room);
//        q = strstr(p, room_key);
//        if (q)
//        {
//            q += strlen(room_key);
//            if (utilities_copy_parameters(q-1, value, '"', '"'))
//            {
//                lcd_set_room_name(room, value);
//                DEBUG_VERBOSE("LCD set %s%s\"\r\n", room_key, value);
//            }
//        }
//    }

//    // ROOM-check
//    for (uint32_t room = 0; room < MAX_ROOM_SUPPORT; room++)
//    {
//        char room_key[12];
//        sprintf(room_key, "rc%d=\"", room);
//        q = strstr(p, room_key);
//        if (q)
//        {
//            q += strlen(room_key);
//            if (utilities_copy_parameters(q-1, value, '"', '"'))
//            {
//                // meeting_info_t meet;
//                // meet.value = atoi(value); 
//                if (value[0] == '2')
//                {
//                    DEBUG_VERBOSE("LCD select room %d, value %d\"\r\n", room, 2);
//                    lcd_set_checkbox_rom(room, ROOM_MEETING);
//                }
//                else if (value[0] == '1')
//                {
//                    DEBUG_VERBOSE("LCD select room %d, value %d\"\r\n", room, 1);
//                    lcd_set_checkbox_rom(room, ROOM_SELECTED);
//                }
//                else if (value[0] == '0')
//                {
//                    DEBUG_VERBOSE("LCD select room %d, value %d\"\r\n", room, 0);
//                    lcd_set_checkbox_rom(room, ROOM_IDLE);
//                }
//            }
//        }
//    }

//    // Meeting status
//    q = strstr(p, "meet=\"");
//    if (q)
//    {
//        q += strlen("meet=\"");
//        if (utilities_copy_parameters(q-1, value, '"', '"'))
//        {

//        }
//    }
    

    // Current folder working directory
    q = strstr(p, "fw=\"");
     if (q)
    {
        q += strlen("fw=\"");
        if (utilities_copy_parameters(q-1, value, '"', '"'))
        {
            DEBUG_VERBOSE("LCD folder directory %s\"\r\n", value);
            lcd_set_folder_directory(value);
        }
    }

    // Folder name
    for (uint32_t folder_count = 0; folder_count < MAX_FOLDER_SUPPORT; folder_count++)
    {
        char search[12];
        sprintf(search, "f%d=\"", folder_count+30);
        q = strstr(p, search);
        if (q)
        {
            q += strlen(search);
            if (utilities_copy_parameters(q-1, value, '"', '"'))
            {
                lcd_set_folder_name(folder_count, value);
                DEBUG_VERBOSE("LCD folder name %s->%s\"\r\n", search, value);
            }
        }
    }
    // folder icon
    for (uint32_t folder_count = 0; folder_count < MAX_FOLDER_SUPPORT; folder_count++)
    {
        char search[12];
        sprintf(search, "fc%d=\"", folder_count+30);
        q = strstr(p, search);
        if (q)
        {
            q += strlen(search);
            if (utilities_copy_parameters(q-1, value, '"', '"'))
            {
                if (value[0] == (ICON_TYPE_EMPTY+'0'))
                {
                    lcd_set_folder_icon(folder_count, ICON_TYPE_EMPTY);
                }
                else if (value[0] == (ICON_TYPE_FOLDER+'0'))
                {
                    lcd_set_folder_icon(folder_count, ICON_TYPE_FOLDER);
                }
                else if (value[0] == (ICON_TYPE_FILE+'0'))
                {
                    lcd_set_folder_icon(folder_count, ICON_TYPE_FILE);
                }
                DEBUG_VERBOSE("LCD folder icon %s%s\"\r\n", search, value);
            }
        }
    }
    
    // folder select
    for (uint32_t folder_count = 0; 
        folder_count < MAX_FOLDER_SUPPORT; 
        folder_count++)
    {
        char search[12];
        sprintf(search, "fs%d=\"", folder_count+30);
        q = strstr(p, search);
        if (q)
        {
            q += strlen(search);
            if (utilities_copy_parameters(q-1, value, '"', '"'))
            {
                if (value[0] == '1')
                {
                    lcd_set_folder_select(folder_count, 1);
                    DEBUG_VERBOSE("LCD folder select %s%s\"\r\n", search, value);
                }
                else if (value[0] == '0')
                {
                    lcd_set_folder_select(folder_count, 0);
                }
            }
        }
    }

    // FILE
    q = strstr(p, "file=\"");
    if (q)
    {
        q += strlen("file=\"");
        if (utilities_copy_parameters(q-1, value, '"', '"'))
        {
            lcd_set_now_playing_song(value);
            DEBUG_VERBOSE("LCD now playing %s\"\r\n", value);
        }
    }

    // Music loop
    q = strstr(p, "loop=\"");
    if (q)
    {
        q += strlen("loop=\"");
        if (utilities_copy_parameters(q-1, value, '"', '"'))
        {
            if (value[0] == '1')
            {
                lcd_set_music_loop(1);
            }
            else if (value[0] == '0')
            {
                lcd_set_music_loop(0);
            }
        }
    }

//    // music vol
//    q = strstr(p, "volm=\"");
//    if (q)
//    {
//        q += strlen("volm=\"");
//        if (utilities_copy_parameters(q-1, value, '"', '"'))
//        {
//            DEBUG_VERBOSE("LCD music vol %s\"\r\n", value);
//            lcd_set_music_vol(value);
//        }
//    }

    // Music speaker
    q = strstr(p, "spkm=\"");
    if (q)
    {
        q += strlen("spkm=\"");
        if (utilities_copy_parameters(q-1, value, '"', '"'))
        {
            DEBUG_VERBOSE("LCD music speaker %s\"\r\n", value);
            if (value[0] == '1')
            {
                lcd_set_music_spk(1);
            }
            else if (value[0] == '0')
            {
                lcd_set_music_spk(0);
            }
        }
    }

    // Music start/stop, 2 = pause, 1 = playing, 0 = idle
    q = strstr(p, "play=\"");
    if (q)
    {
        q += strlen("play=\"");
        if (utilities_copy_parameters(q-1, value, '"', '"'))
        {
            DEBUG_VERBOSE("LCD music player %s\"\r\n", value);
            if (value[0] == '2')
            {
                lcd_set_music_playing_icon(2);
                lcd_set_music_playing_text_status(2);
            }
            else if (value[0] == '1')
            {
                lcd_set_music_playing_icon(1);
                lcd_set_music_playing_text_status(1);
            }
            else if (value[0] == '0')
            {
                lcd_set_music_playing_icon(0); // icon > ||
                lcd_set_music_playing_text_status(0);
            }
        }
    }

//    // Ethernet icon
//    q = strstr(p, "svr=\"");
//    if (q)
//    {
//        q += strlen("svr=\"");
//        DEBUG_WARN("LCD ENET %.*s\"\r\n", 2, q);
//        if (utilities_copy_parameters(q-1, value, '"', '"'))
//        {
//            if (value[0] == '1')
//            {
//                lcd_set_network_icon(1);
//            }
//            else if (value[0] == '0')
//            {
//                lcd_set_network_icon(0);
//            }
//        }
//    }

    // Disk
    // 0 = SDC, 1 = USB, 2 = MMC
    q = strstr(p, "disk=\"");
    if (q)
    {
        q += strlen("disk=\"");
        if (utilities_copy_parameters(q-1, value, '"', '"'))
        {
            DEBUG_VERBOSE("LCD disk %s\"\r\n", value);
            if (value[0] == '2')    // 0 = SDC, 1 = USB, 2 = internal
            {
                lcd_set_storage(2);
            }
            if (value[0] == '1')
            {
                lcd_set_storage(1);
            }
            else if (value[0] == '0')
            {
                lcd_set_storage(0);
            }
        }
    }

//    // Checkbox : all room
//    if (strstr(p, "rAll=\"1\""))
//    {
//        lcd_set_checkbox_select_all_rom(1);
//    }
//    else if (strstr(p, "rAll=\"0\""))
//    {
//        lcd_set_checkbox_select_all_rom(0);
//    }


//    // CALL - button 'CALL'
//    q = strstr(p, "call=\"");
//    if (q)
//    {
//        uint32_t mode = utilities_get_number_from_string(strlen("call=\""), q);
//        lcd_set_call_icon(mode);
//    }


//    // CALL - Speaker mute
//    q = strstr(p, "spk=\"");
//    if (q)
//    {
//        uint32_t mode = utilities_get_number_from_string(strlen("spk=\""), q);
//        lcd_set_speaker_mute_icon(mode ? true : false);
//    }

    // CALL - Mic
    q = strstr(p, "cmic=\"");
    if (q)
    {
        uint32_t mode = utilities_get_number_from_string(strlen("cmic=\""), q);
        lcd_set_mic_mute_icon(mode ? true : false);
    }
    

//    // mic mode, out2mic, relay0,1
//    q = strstr(p, "micmode=\"");
//    if (q)
//    {
//        uint32_t mode = utilities_get_number_from_string(strlen("micmode=\""), q);
//        lcd_set_mic_mode(mode ? true : false);
//    }
//    q = strstr(p, "out2mic=\"");
//    if (q)
//    {
//        uint32_t mode = utilities_get_number_from_string(strlen("out2mic=\""), q);
//        lcd_set_out2mic_mode(mode ? true : false);
//    }

//    
//    q = strstr(p, "rl0=\"");
//    if (q)
//    {
//        uint32_t relay = utilities_get_number_from_string(strlen("rl0=\""), q);
//        lcd_set_relay0(relay ? true : false);
//    }


//    q = strstr(p, "rl1=\"");
//    if (q)
//    {
//        uint32_t relay = utilities_get_number_from_string(strlen("rl1=\""), q);
//        lcd_set_relay1(relay ? true : false);
//    }    
}

#if TEST_LCD_NEXTION

void test_ui_msg()
{
    static uint32_t i = 0;
    if (i++ < 3)
    {
        lcd_nextion_write("page page4");
    }
    else 
    {
        static char msg[256];
        static int disk = 0;
        static uint8_t loop = 0;
        static uint32_t song;
        static uint32_t play = 0;
        song = sys_get_ms();
        uint32_t folder_icon = song % 2;
        sprintf(msg, "vol=\"50\",fw=\"/usb/nhac/nhac tre/album le quyen\",fc30=\"%d\",fc31=\"%d\",fc32=\"%d\","
                "fc33=\"%d\",file=\"Con mua thang %u\",loop=\"%d\",disk=\"%d\",cmic=\"1\",play=\"%d\""
                "f30=\"nghe nhac\",f31=\"english\",f32=\"nhac cach mang\",f33=\"Song tung mtp.mp3\",spkm=\"%d\"", 
                folder_icon, folder_icon, folder_icon, folder_icon,
                song, loop++ %2,(disk++ % 3), (play++)%3,
                folder_icon);
        process_ui_msg((char*)msg);
    }
}

#endif
/*
                PROCESS ALL ABOUT PASSWORD HERE
*/
#define DEFAULT_PASSWORD "1234"
#define INVALID_PASSWORD "0000"
#define MAX_PWD_LENGTH    4

typedef struct
{
    char pwd_content[MAX_PWD_LENGTH];
    uint8_t pwd_index;
    uint8_t max_retry_count;
} password_t;

static char password_using_now[MAX_PWD_LENGTH] = DEFAULT_PASSWORD;

static password_t password_buffer_now;
void set_password_value (char* pw_content)
{
   memset (password_using_now, 0, sizeof (password_using_now));
   memcpy (password_using_now, pw_content, MAX_PWD_LENGTH);
}

void lcd_goto_login_screen(void);

void put_word_into_password (password_t * pwd_buff, char ch)
{
    pwd_buff->pwd_content[pwd_buff->pwd_index] = ch;
    pwd_buff->pwd_index++;
    
    if (pwd_buff->pwd_index == 1)
    {
        lcd_nextion_write("t1.txt=\"* _ _ _\"");
    }
    else if (pwd_buff->pwd_index == 2)
    {
        lcd_nextion_write("t1.txt=\"* * _ _\"");
    }
    else if (pwd_buff->pwd_index == 3)
    {
        lcd_nextion_write("t1.txt=\"* * * _\"");
    }
    else if (pwd_buff->pwd_index == 4)
    {
        lcd_nextion_write("t1.txt=\"* * * *\"");
    }
    
    if (pwd_buff->pwd_index == 4)
    {
        if (!(memcmp (pwd_buff->pwd_content, password_using_now, MAX_PWD_LENGTH)))
        {
            DEBUG_INFO ("ENTERED RIGHT PASSWORD");
            min_msg_t msg;
            msg.id = MIN_ID_ENTER_PASSWORD_OK;
            msg.len = 0;
            msg.payload = NULL;
            min_send_frame(&m_min_host_context, &msg);
        }
        else
        {
            pwd_buff->max_retry_count++;
            if (pwd_buff->max_retry_count > 5)
            {
                pwd_buff->max_retry_count = 0;
                DEBUG_ERROR ("WRONG PASSWORD > 5 TIMES");
                void lcd_goto_login_screen();
            }
        }
        pwd_buff->pwd_index = 0;
    }
}

/* 
        FUNCTION LCD WITH RELEVANT 
    we code all function relate to lcd here
*/ 




void lcd_goto_login_screen(void)
{
//    lcd_nextion_write("page page7");
//    m_host_ping_msg.screen_id = LCD_LOGIN_SCREEN;
//    lcd_nextion_write("t1.txt=\"_ _ _ _\"");
//    lcd_nextion_write("t0.txt=\"Enter password, please!\"");
}

void lcd_goto_home_screen()
{
    lcd_nextion_write("page page0");
    m_host_ping_msg.screen_id = LCD_HOME_SCREEN;
}

uint32_t process_lcd_keycode(char *msg)
{
    char *p = msg;
    char *q;
    uint32_t counter = 0;
    
    if (!msg)
    {
        return 0;
    }
    
    // 2 = internal storage, 1 =  usb, 0 = sdcard
    q = strstr(p, "PAGE4 INTERNAL_STORAGE");
    if (q)
    {
        DEBUG_WARN("Button 'SELECT INTERNAL STORAGE' pressed\r\n");
        min_msg_t msg;
        uint8_t type = 2;
        msg.id = MIN_ID_SELECT_STORAGE_TYPE;
        msg.len = 1;
        msg.payload = &type;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 USB");
    if (q)
    {
        DEBUG_WARN("Button 'SELECT USB' pressed\r\n");
        min_msg_t msg;
        uint8_t type = 1;
        msg.id = MIN_ID_SELECT_STORAGE_TYPE;
        msg.len = 1;
        msg.payload = &type;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 SDCARD");
    if (q)
    {
        DEBUG_WARN("Button 'SELECT SDCARD' pressed\r\n");
        min_msg_t msg;
        uint8_t type = 0;
        msg.id = MIN_ID_SELECT_STORAGE_TYPE;
        msg.len = 1;
        msg.payload = &type;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 FOLDER_BACK");
    if (q)
    {
        DEBUG_WARN("Button 'FOLDER_BACK' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_PREVIOUS_FOLDER;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 FOLDER_PREV");
    if (q)
    {
        DEBUG_WARN("Button 'FOLDER_PREV' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_FILE_SCROLL_LEFT;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 FOLDER_NEXT");
    if (q)
    {
        DEBUG_WARN("Button 'FOLDER_NEXT' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_FILE_SCROLL_RIGHT;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 SEL_MUSIC_0");
    if (q)
    {
        DEBUG_WARN("Button 'FOLDER 0' pressed\r\n");
        min_msg_t msg;
        uint8_t folder = 30;
        msg.id = MIN_ID_SELECT_FOLDER;
        msg.len = 1;
        msg.payload = &folder;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 SEL_MUSIC_1");
    if (q)
    {
        DEBUG_WARN("Button 'FOLDER 1' pressed\r\n");
        min_msg_t msg;
        uint8_t folder = 31;
        msg.id = MIN_ID_SELECT_FOLDER;
        msg.len = 1;
        msg.payload = &folder;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 SEL_MUSIC_2");
    if (q)
    {
        DEBUG_WARN("Button 'FOLDER 2' pressed\r\n");
        min_msg_t msg;
        uint8_t folder = 32;
        msg.id = MIN_ID_SELECT_FOLDER;
        msg.len = 1;
        msg.payload = &folder;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 SEL_MUSIC_3");
    if (q)
    {
        DEBUG_WARN("Button 'FOLDER 3' pressed\r\n");
        min_msg_t msg;
        uint8_t folder = 33;
        msg.id = MIN_ID_SELECT_FOLDER;
        msg.len = 1;
        msg.payload = &folder;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }

    q = strstr(p, "PAGE4 MUTE");
    if (q)
    {
        DEBUG_WARN("Button 'MUTE' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_MUTE_MUSIC_VOLUME;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 MUSIC_PREV");
    if (q)
    {
        DEBUG_WARN("Button 'MUSIC_PREV' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_PREVIOUS_SONG;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 MUSIC_NEXT");
    if (q)
    {
        DEBUG_WARN("Button 'MUSIC_NEXT' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_NEXT_SONG;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 MUSIC_PLAY");
    if (q)
    {
        DEBUG_WARN("Button 'MUSIC_PLAY/PAUSE' pressed\r\n");
        min_msg_t msg;
        if (m_last_audio_status == 2)   // dang pause -> an nut thi chuyen sang playing:
        {
            msg.id = MIN_ID_MUSIC_PAUSE;
        }
        else if (m_last_audio_status == 1)   // dang playing -> an nut thi chuyen sang pause:
        {
            msg.id = MIN_ID_MUSIC_PAUSE;
        }
        else  // dang stop -> an nut thi chuyen sang playing
        {
            msg.id = MIN_ID_PLAY_STOP_SONG;
        }
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 MUSIC_STOP");
    if (q)
    {
        DEBUG_WARN("Button 'MUSIC_STOP' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_PLAY_STOP_SONG;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 MUSIC_REPEAT");
    if (q)
    {
        DEBUG_WARN("Button 'MUSIC_REPEAT' pressed\r\n");
        min_msg_t msg;
        msg.id = MIN_ID_SELECT_LOOP;
        msg.len = 0;
        min_send_frame(&m_min_host_context, &msg);
        counter++;
    }
    
    q = strstr(p, "PAGE4 GO_HOME");
    if (q)
    {
        DEBUG_WARN("Button 'GO_HOME' pressed\r\n");
        m_host_ping_msg.screen_id = 1;
        lcd_nextion_write("page page1");
        counter++;
    }
    
    q = strstr(p, "PAGE2 TIM");
    if (q)
    {
        if (m_host_ping_msg.screen_id != LCD_INFO_SCREEN)
        {
            m_host_ping_msg.screen_id = LCD_INFO_SCREEN;
            DEBUG_INFO("PAGE 2\r\n");
        }
        counter++;
    }
    
    q = strstr(p, "PAGE3 TIM");
    if (q)
    {
        if (m_host_ping_msg.screen_id != LCD_SETTING_SCREEN)
        {
            m_host_ping_msg.screen_id = LCD_SETTING_SCREEN;
            DEBUG_INFO("PAGE 3\r\n");
        }
        counter++;
    }
    
    q = strstr(p, "PAGE4 TIM");
    if (q)
    {
        if (m_host_ping_msg.screen_id != LCD_MUSIC_SCREEN)
        {
            m_host_ping_msg.screen_id = LCD_MUSIC_SCREEN;
            DEBUG_INFO("PAGE 4\r\n");
        }
        counter++;
    }
    
    q = strstr(p, "PAGE1 TIM");
    if (q)
    {
        if (m_host_ping_msg.screen_id != LCD_HOME_SCREEN)
        {
            m_host_ping_msg.screen_id = LCD_HOME_SCREEN;
            DEBUG_INFO("PAGE 1\r\n");
        }
        counter++;
    }
    
    q = strstr(p, "PAGE5 TIM");
    if (q)
    {
        if (m_host_ping_msg.screen_id != LCD_FM_SCREEN)
        {
            m_host_ping_msg.screen_id = LCD_FM_SCREEN;
            DEBUG_INFO("PAGE 5\r\n");
        }
        counter++;
    }
    
    q = strstr(p, "PAGE6 TIM");
    if (q)
    {
        if (m_host_ping_msg.screen_id != LCD_MEETING_SCREEN)
        {
            m_host_ping_msg.screen_id = LCD_MEETING_SCREEN;
            DEBUG_INFO("MEETING PAGE\r\n");
        }
        counter++;
    }
    
    q = strstr(p, "PAGE7 TIM");
    if (q)
    {
        if (m_host_ping_msg.screen_id != LCD_LOGIN_SCREEN)
        {
            m_host_ping_msg.screen_id = LCD_LOGIN_SCREEN;
            DEBUG_INFO("PAGE 5\r\n");
        }
        counter++;
    }
    
    q = strstr(p, "PAGE7 BUTTON_1");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 1 + 48);
        DEBUG_WARN ("BUTTON 1 PAGE 7 IS PRESSED");
        counter++;
    }
    
    q = strstr(p, "PAGE7 BUTTON_2");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 2 + 48);
        DEBUG_WARN ("BUTTON 2 PAGE 7 IS PRESSED");
        counter++;
    }
    
    
    q = strstr(p, "PAGE7 BUTTON_3");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 3 + 48);
        DEBUG_WARN ("BUTTON 3 PAGE 7 IS PRESSED");
        counter++;
    }    
    
    q = strstr(p, "PAGE7 BUTTON_4");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 4 + 48);
        DEBUG_WARN ("BUTTON 4 PAGE 7 IS PRESSED");
        counter++;
    }
    
    
    q = strstr(p, "PAGE7 BUTTON_5");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 5 + 48);
        DEBUG_WARN ("BUTTON 5 PAGE 7 IS PRESSED");
        counter++;
    }
    
    
    q = strstr(p, "PAGE7 BUTTON_6");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 6 + 48);
        DEBUG_WARN ("BUTTON 6 PAGE 7 IS PRESSED");
        counter++;
    }
    
    
    q = strstr(p, "PAGE7 BUTTON_7");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 7 + 48);
        DEBUG_WARN ("BUTTON 7 PAGE 7 IS PRESSED");
        counter++;
    }
    
    
    q = strstr(p, "PAGE7 BUTTON_8");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 8 + 48);
        DEBUG_WARN ("BUTTON 8 PAGE 7 IS PRESSED");
        counter++;
    }
    
    
    q = strstr(p, "PAGE7 BUTTON_9");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 9 + 48);
        DEBUG_WARN ("BUTTON 9 PAGE 7 IS PRESSED");
        counter++;
    }
        
    
    q = strstr(p, "PAGE7 BUTTON_0");
    if (q)
    {
        put_word_into_password(&password_buffer_now, 0 + 48);
        DEBUG_WARN ("BUTTON 0 PAGE 7 IS PRESSED");
        counter++;
    }
    
    return counter;
}


volatile uint8_t auto_end_line = 0;
bool sent_baudrate = false;
static void on_host_frame_callback(void *ctx, min_msg_t *frame)
{
#if BOARD_HW_HAS_NO_PING
    m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_IN_RUN_MODE;
#endif
    m_total_serial_frame++;
    m_skip_serial_data_when_module_power_on = 0;
    // Reset timeout
    m_host_keep_alive_uart = DEFAULT_HOST_UART_TIMEOUT_DO_RESET;
    m_host_die_counter_force_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
    m_led_blink_indicator_step = 0;
    m_usart_monitor_error_counter = 0;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
#if DEBUG_MIN_PRO
    DEBUG_INFO("Frame cb, ERR/TOTAL = %u/%u\r\n", m_serial_frame_err_counter, m_total_serial_frame);
#endif
    auto_end_line = 0;

    if (m_count_host_boot_time != -1)
    {
        // DEBUG_WARN("Host boot time %u sec\r\n", m_count_host_boot_time / 10);
        m_count_host_boot_time = -1;
    }
       
    switch (frame->id)
    {
        case MIN_ID_GET_GPIO:
        {
            update_peripheral_input();
            min_msg_t msg;
            msg.id = MIN_ID_PING;
            msg.payload = &m_host_ping_msg;
            msg.len = sizeof(m_host_ping_msg);
            min_send_frame(&m_min_host_context, &msg);
        }
        break;

        case MIN_ID_SET_GPIO:
        {
            if (m_test_mode || frame->len != (sizeof(host_data_layer_output_t)))
            {
                DEBUG_ERROR("Invalid frame %u bytes\r\n", frame->len);
                return;
            }
            memcpy((void*)&m_current_output_not_swap_endian.value, frame->payload, sizeof(host_data_layer_output_t));
            if (m_prev_gpio_output_control.value != m_current_output_not_swap_endian.value)
            {
                DEBUG_INFO("GPIO changed from 0x%08X to 0x%08X\r\n", 
                            m_prev_gpio_output_control.value, m_current_output_not_swap_endian.value);
            }
            update_output();
            update_interface();
            //update_operation_mode();
            m_prev_gpio_output_control.value = m_current_output_not_swap_endian.value;
            // mupdate_temperature();
        }
        break;
        
        case MIN_ID_FORWARD:
        {
            DEBUG_WARN("Forward to FM\r\n");
#if FM_LCD_ENABLE
            uint8_t *ptr = frame->payload;
            if (m_fm_protocol_type != FM_PROTOCOL_TYPE_MIN_PROTOCOL)
            {
                DEBUG_INFO("Forward to FM %.*s\r\n", frame->len, (char*)ptr);
            }
#endif
            board_hw_fm_lcd_send(ptr, frame->len);
        }
            break;
        
        case MIN_ID_NEXTION_LCD_FORWARD:
        {
            // uint8_t terminate[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            if (frame->len)
            {
                if (!m_lcd_nextion_enable)
                {
                    DEBUG_WARN("Change FM lcd baudrate to 9600\r\n");
                    m_lcd_nextion_enable = 1;
                    board_hw_change_lcd_fm_baudrate(BOARD_HW_NEXTION_BAUD);
                }
                //char *ptr = frame->payload;
                //DEBUG_VERBOSE("Nextion %.*s\r\n", frame->len-3, ptr);
                // DEBUG_DUMP(ptr, frame->len, "LCD");
                board_hw_fm_lcd_send(frame->payload, frame->len);
    //            board_hw_fm_lcd_send(terminate, sizeof(terminate));
            }
        }
            break;
        
        case MIN_ID_RESET:
            // Force shutdown module
            DEBUG_WARN("Host request mcu force shutdown module");
            if (frame->len == 0)
            {
                if (m_host_power_state != HOST_POWER_DOWN)
                {
                    m_host_power_state = HOST_POWER_DOWN;
                }
            }
            else
            {
                board_hw_reset();
            }
            break;
            
        case MIN_ID_SET_UNIX_TIMESTAMP:
        {
            uint8_t *p = frame->payload;
            uint32_t timestamp = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
            if (timestamp > 1684901278)
            {
                DEBUG_VERBOSE("Timestamp %u\r\n", timestamp);
                board_hw_rtc_set_timestamp(timestamp);
            }
        }
            break;
        
        case MIN_ID_FORWARD_DBG_MSG:
        {
            DEBUG_VERBOSE("Android -> %.*s\r\n", frame->len, (char*)frame->payload);
        }
            break;
        
        case MIN_ID_REQUEST_RESET_REASON:
        {
            // Send reset message to host
            board_hw_critical_log_t err;
            board_hw_get_critical_error(&err);
            uint8_t reset_msg[48];
            memset(reset_msg, 0, sizeof(reset_msg));
            reset_msg[0] = (err.name.reset_counter & 0xFF00) >> 8;
            reset_msg[1] = (err.name.reset_counter & 0xFF);
            reset_msg[2] = err.name.is_vin_lost;
            reset_msg[3] = err.name.is_uart_communication_error;
            reset_msg[4] = err.name.is_heartbeat_err;
            sprintf((char*)&reset_msg[5], "%s", __DATE__);
            
            min_msg_t min_rst_msg;
            min_rst_msg.id = MIN_ID_REQUEST_RESET_REASON;
            min_rst_msg.payload = reset_msg;
            min_rst_msg.len = 5 + strlen((char*)&reset_msg[5])+1;
            min_send_frame(&m_min_host_context, &min_rst_msg);
            
            // Clear luon nguyen nhan reset
            err.name.is_heartbeat_err = 0;
            err.name.is_uart_communication_error = 0;
            err.name.is_vin_lost = 0;
            if (ignore_continues_write_to_flash == 0)
            {
                ignore_continues_write_to_flash = 500;   // 30s
                board_hw_set_critical_error(err);
            }
            
            // send baudrate
           
//            if (board_hw_is_get_baudrate_complete())
//            {
//                min_msg_t min_baudrate_msg;
//                min_baudrate_msg.id = MIN_ID_AUTO_DETECT_BAUD;
//                min_baudrate_msg.payload = &m_fm_lcd_uart_baud;
//                min_baudrate_msg.len = sizeof(uint32_t);
//                min_send_frame(&m_min_host_context, &min_baudrate_msg);
//                DEBUG_WARN("Send fm baud %u\r\n", m_fm_lcd_uart_baud);
//            }
//            
            // DEBUG_WARN("Send reset counter 0x%02X%02X to host\r\n", reset_msg[0], reset_msg[1]);
        }
            break;
        
        case MIN_ID_SET_DIGITAL_VOLUME:
//        case MIN_ID_SET_VOLUME_CALL:
        {
            static uint8_t m_last_vol = 0;
            uint8_t vol = *((uint8_t*)frame->payload);
            if (vol & 0x80)
            {
                DEBUG_INFO("Bypass digital vol\r\n");
                vol = vol & 0X7F;
                board_hw_output_set(BOARD_HW_OUTPUT_BY_PASS_DIGITAL_VOL, true);
            }
            else
            {
                board_hw_output_set(BOARD_HW_OUTPUT_BY_PASS_DIGITAL_VOL, false);
            }
            
            if (m_last_vol != vol)
            {
                m_last_vol = vol;
#if DIGITAL_POT
                m_host_ping_msg.error_flag.name.d_vol_err = board_hw_digital_pot_set(vol) ? 0 : 1;
#endif
                DEBUG_INFO("Set vol %u\r\n", vol);
            }
            
//            if (m_lcd_nextion_enable && m_host_ping_msg.screen_id == LCD_HOME_SCREEN)
//            {
//                char tmp[32];
//                sprintf(tmp, "t4.txt=\"%d%%\"", m_last_vol);
//                lcd_nextion_write(tmp);
//            }
        }
            break;
                
        case MIN_ID_LCD_DISPLAY_LCD_VOLUME:
        {
//            uint8_t vol = *((uint8_t*)frame->payload);
//            if (m_lcd_nextion_enable && m_host_ping_msg.screen_id == LCD_HOME_SCREEN)
//            {
//                char tmp[32];
//                sprintf(tmp, "t4.txt=\"%d%%\"", vol);
//                lcd_nextion_write(tmp);
//            }
//            DEBUG_INFO("Display lcd vol %d\r\n", vol);
        }
            break;
        
        case MIN_ID_CTRL_EXT_GPIO:
        {            
            uint32_t value = *((uint32_t*)frame->payload);
            board_hw_ioex_mode_t new_io;
            new_io.val = value;
            board_hw_ioex_update(new_io);
            
            // Get current IO value
            new_io = board_hw_ioex_get_current_value();
            
            // Echo back value
            min_msg_t io_msg;
            io_msg.id = MIN_ID_CTRL_EXT_GPIO;
            io_msg.len = sizeof(new_io);
            io_msg.payload = &new_io;
            min_send_frame(&m_min_host_context, &io_msg);
        }
            break;
#if OTA_ENABLE        
        case MIN_ID_OTA_UPDATE_START:
        {
            board_hw_disable_fm_rx();
            board_hw_enable_host_rx(false);
            ota_update_enter_bootloader();
            board_hw_ping_irq_control(false);
            board_hw_reset();
//            /* 4 bytes size */
//            DEBUG_WARN("OTA start\r\n");
//            // Disable USARTFM RB interrupt
//            board_hw_disable_fm_rx();
//            bool retval;
//            uint8_t *p = (uint8_t *)frame->payload;
//            uint32_t firmware_size = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | (p[3]);
//            
//            board_hw_enable_host_rx(false);
//            retval = ota_update_start(firmware_size);
//            board_hw_enable_host_rx(true);
//            
//            if (retval)
//            {
//                // Send ack
//                min_msg_t rsp_msg;
//                uint8_t tmp = 0;
//                DEBUG_WARN("OTA ACK\r\n");
//                rsp_msg.id = MIN_ID_OTA_UPDATE_START; //MIN_ID_OTA_ACK;
//                rsp_msg.len = 1;
//                rsp_msg.payload = &tmp;
//                min_send_frame(&m_min_host_context, &rsp_msg);
//            }
//            else // Send failed
//            {
//                min_msg_t rsp_msg;
//                uint8_t tmp = 1;
//                DEBUG_WARN("OTA start failed\r\n");
//                rsp_msg.id = MIN_ID_OTA_FAILED;
//                rsp_msg.len = 1;
//                rsp_msg.payload = &tmp;
//                min_send_frame(&m_min_host_context, &rsp_msg);
//            }
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

        case MIN_ID_FORWARD_FM_MSG_TO_MCU:
        {
            board_hw_fm_lcd_send(frame->payload, frame->len);
        }
            break;
        
        case MIN_ID_UPDATE_UI:
        {
            if (frame->payload && frame->len)
            {
                char *p = frame->payload;
                p[frame->len] = 0; // Consider about buffer overflow
                process_ui_msg(p);
            }
        }
            break;
        
        case MIN_ID_FM_INFO:
        {
            uint8_t *p = frame->payload;
            memcpy(&m_fm_freq, p, 12);
            memcpy(&m_fm_vol, p+12, 1);
            memcpy(&m_fm_channel, p+13, 1);
            DEBUG_VERBOSE("FM freq [%u, %u, %u], vol %d, channel %d\r\n", 
                        m_fm_freq[0], m_fm_freq[1], m_fm_freq[2], m_fm_vol, m_fm_channel);
        }
            break;
        
        case MIN_ID_MODE_REPORT:
        {
            host_data_layer_mode_report_t *mode = (host_data_layer_mode_report_t*)frame->payload;
            memcpy(&m_working_report, mode, sizeof(host_data_layer_mode_report_t));
            DEBUG_VERBOSE("B%d, CSQ=%d, T=%d\r\n", mode->band, mode->csq, mode->temperature);
        }
            break;
        
//        case MIN_ID_SET_DEVICE_NAME:
//        {
////            DEBUG_WARN("Device name %.*s\r\n", frame->len, frame->payload);
//            if (m_lcd_nextion_enable)
//            {
//                board_hw_fm_lcd_send((uint8_t*)frame->payload, frame->len);
//                board_hw_fm_lcd_send((uint8_t*)m_nextion_end_frame, 3);
//            }
//        }
//            break;
        
        case MIN_ID_SPK_UPDATE_CLASS_D_INFO:
            m_get_class_d_spk = true;
            if (frame->len)
            {
                DEBUG_WARN("Delay detect spk %d\r\n", 1000*(*((uint8_t*)frame->payload)));
                __disable_irq();
                m_delay_skip_over_temp = 1000*(*((uint8_t*)frame->payload));       // sec
                __enable_irq();
            }
            break;
				
        default:
            break;
    }
}


static bool is_vin_detect(void)
{        
    return (m_is_vin_detect == 0) ? false : true;
}

static void reset_operation_state()
{
    m_current_output_not_swap_endian.name.operation_mode = 7;
    m_current_output_not_swap_endian.name.if_4g = 0;
    m_current_output_not_swap_endian.name.if_eth = 0;
    m_current_output_not_swap_endian.name.if_wifi = 0;
    m_current_output_not_swap_endian.name.pa = 0;
    m_prev_gpio_output_control.name.if_eth = 0;
    m_prev_gpio_output_control.name.if_wifi = 0;
    m_prev_gpio_output_control.name.if_4g = 0;
    m_prev_gpio_output_control.name.operation_mode = 7;
    m_prev_gpio_output_control.name.pa = 0;
    m_current_output_not_swap_endian.name.if_eth = 0;
    m_current_output_not_swap_endian.name.if_4g = 0;
    m_current_output_not_swap_endian.name.if_wifi = 0;
    m_working_report.type = 0;      // unknown
    m_working_report.csq = 0;
}


uint8_t fake_power_event = 0;
static void poll_vin_fsm(void)
{
    static uint32_t m_vin_event_counter = 0;
    
    // Set default state at startup
    if (m_is_vin_detect == -1)
    {
        m_vin_event_counter = 0;
        if (m_lpf[0].estimate_value > VIN_LOST_THRESHOLD_MV)
        {
            m_is_vin_detect = 1;
        }
        else
        {
            // Vin lost khi chuyen trang thai tu 1 -> 0, con khi bat dau vin chua dat VIN_LOST_THRESHOLD_MV thi khong tinh
            return;
            // m_is_vin_detect = 0;
        }
    }  
    
    if (fake_power_event)
    {
        m_lpf[0].estimate_value = VIN_LOST_THRESHOLD_MV >> 1;
    }
    // Poll power state machine
    if (m_is_vin_detect == 0)
    {
        if (m_lpf[0].estimate_value > VIN_LOST_THRESHOLD_MV 
            && m_vin_event_counter++ > 30)     // 3s
        {
            m_vin_event_counter = 0;
            m_is_vin_detect = 1;
            DEBUG_ERROR("Vin plugged again\r\n");
            board_hw_output_set(BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION, 0);
        }
    }
    else
    {
        m_vin_event_counter = 0;
        
        if (m_lpf[0].estimate_value < VIN_LOST_THRESHOLD_MV)
        {
            m_is_vin_detect = 0;
            if (m_vin_lost_times < 255)
                m_vin_lost_times++;
            DEBUG_INFO("Vin lost %u times\r\nChange pwr state to 'WAIT FOR VIN'\r\n", m_vin_lost_times);
            m_host_power_state = HOST_POWER_WAIT_FOR_VIN;
            __disable_irq();
            if (m_pulse_heartbeat_counter <= MAX_HEARTBEAT_COUNTER_IN_RUN_MODE)
                m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_IN_RUN_MODE;
            __enable_irq();

            reset_operation_state();
            
            lcd_display_operation_mode("POWER LOST");
            
            // Save vin lost error code to flash
            board_hw_critical_log_t err;
            board_hw_get_critical_error(&err);
            err.name.is_vin_lost = 1;
            board_hw_set_critical_error(err);  
        }
    }
    
    board_hw_output_set(BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION, m_is_vin_detect ? 0 : 1);
}


static void process_host_power_state(void)
{
    static uint32_t m_power_state_counter = 0;
    // uint8_t server_state_now;
   
    board_hw_output_set(BOARD_HW_MODULE_RESET, 0);      // dont care about reset pin
    
    if (m_host_die_counter_force_reset_mcu)      // 10000*100ms = 10000s
    {
        m_host_die_counter_force_reset_mcu--;
        if (m_host_die_counter_force_reset_mcu == 0)
        {
            m_host_die_counter_force_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
            DEBUG_ERROR("Long time no see host -> software reset MCU\r\n");
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
        m_led_blink_indicator_step = 0;
        m_pulse_heartbeat_counter--;
        if (m_pulse_heartbeat_counter == 0)
        {
            DEBUG_ERROR("Pulse heartbeat timeout, Shutdown and restart host\r\n");
            m_host_power_state = HOST_POWER_DOWN;
            m_power_state_counter = 0;
            // Save uart lost error code to flash
            board_hw_critical_log_t err;
            board_hw_get_critical_error(&err);
            err.name.is_heartbeat_err = 1;
            err.name.reset_counter = m_reset_counter;
            board_hw_set_critical_error(err);  
        }
    }
    
    switch (m_host_power_state)
    {       
        case HOST_POWER_DOWN:
        {
            reset_operation_state();
            
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
                DEBUG_INFO("Change pwr state to 'UP'\r\n");
                m_host_power_state = HOST_POWER_UP;
            }
        }
            break;
        
        case HOST_POWER_UP:
        {
            m_current_output_not_swap_endian.name.pa = 0;
            board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1);
            if (!is_vin_detect() || v_adc_endian[V_ADC_ENDIAN_3V8_INDEX] < 3300)
            {
                DEBUG_WARN("Vin %s, v3v8 %u\r\n", 
                            is_vin_detect() ? "OK" : "ERR", 
                            v_adc_endian[V_ADC_ENDIAN_3V8_INDEX]);
                m_power_state_counter = 0;
            }
            if (m_power_state_counter++ > 40)     // 4s
            {
                // Clear lcd info
                lcd_display_operation_mode("RESTARTING");
                
                m_power_state_counter = 0;
                m_host_power_state = HOST_POWER_PULSE_PWR_KEY;
                board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 1);
                DEBUG_INFO("Change pwr state to 'PULSE PWR KEY'\r\n");
                board_hw_enable_host_rx(false);
            }
            // m_skip_serial_data_when_module_power_on = 1;
        }
            break;

        case HOST_POWER_PULSE_PWR_KEY:
        {
            reset_operation_state();
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
                DEBUG_INFO("Change pwr state to 'RUNNING'\r\n");
                m_count_host_boot_time = 0;
                board_hw_enable_host_rx(true);
                
                if (m_lcd_nextion_enable)
                {
                    lcd_goto_home_screen();
                }
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
                    
                    // Save uart lost error code to flash
                    board_hw_critical_log_t err;
                    board_hw_get_critical_error(&err);
                    err.name.is_uart_communication_error = 1;
                    board_hw_set_critical_error(err);  
                }
                // get_server_state (&server_state_now);
                update_operation_mode();
                
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
            
#if SHUTDOWN_MODULE_WHEN_POWER_LOST
            if (m_pulse_heartbeat_counter <= (MAX_HEARTBEAT_COUNTER_IN_RUN_MODE-13))  // 13*100
            {
                board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0); // turn off power
            }
            else
            {
                board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1); // always on SC20 power
            }
#endif
            
            m_current_output_not_swap_endian.name.pa = 0;
            board_hw_output_set(BOARD_HW_OUTPUT_PA, m_current_output_not_swap_endian.name.pa);
            
            if (is_vin_detect())
            {
                m_pulse_heartbeat_counter = 0;
                lcd_display_operation_mode("POWER OK");
                DEBUG_WARN("Change power state to 'DOWN'\r\n");
                m_host_power_state = HOST_POWER_DOWN;
                m_wait_for_vin_counter = 0;
            }
            
            if (m_wait_for_vin_counter == 30*600)       //15p, co the ADC loi
            {
                board_hw_critical_log_t err;
                board_hw_get_critical_error(&err);
                err.name.vin_lost_for_along_time = 1;
                board_hw_set_critical_error(err);
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

#if FM_LCD_ENABLE

void remove_trash_nextion_data(uint8_t *buffer_in, uint8_t size, 
                                uint8_t *buffer_out, uint8_t *out_size)
{
    // Remove trash pattern from rx buffer {0x1A, 0xFF, 0xFF, 0xFF, 0xFF}
    uint8_t state = 0, tmp = 0, tmp_size = 0, rewrite = 0;
    for (int i = 0; i < size && buffer_in && buffer_out; i++)
    {
        switch (state)
        {
            case 0:
                if (buffer_in[i] == 0x1A)
                {
                    state = 1;
                    rewrite = 1;
                }
                else
                {
                    tmp = 0;
                    if (buffer_in[i] != 0xFF)
                    {
                        buffer_out[tmp_size++] = buffer_in[i];
                    }
                }
                break;
            case 1:
                if (buffer_in[i] == 0xFF)
                {
                    rewrite = 0;
                    tmp++;
                    if (tmp == 3)
                    {
                        tmp = 0;
                        state = 0;
                    }
                }
                else
                {
                    if (rewrite == 1)
                    {
                        rewrite = 0;
                        buffer_out[tmp_size++] = 0x1A;
                    }
                    tmp = 0;
                    state = 0;
                    if (buffer_in[i] != 0xFF)
                    {
                        buffer_out[tmp_size++] = buffer_in[i];
                    }
                }
            default:
                break;
        }
    }
    *out_size = tmp_size;
}

static void process_new_fm_lcd_data(void)
{
    // Forward data to host module
    if (m_fm_lcd_usart_idle_timeout != -1         // No idle line
        || m_fm_protocol_type == FM_PROTOCOL_TYPE_MIN_PROTOCOL
        || m_fm_raw_buffer.rx_index == 0) 
    {
        return;
    }
    
    
    if (m_lcd_nextion_enable == false && strstr((char*)m_fm_raw_buffer.buffer, "FM|SNR="))
    {
        m_fm_protocol_type = FM_PROTOCOL_TYPE_STRING_FORMAT;
    }
    uint32_t index = m_fm_raw_buffer.rx_index;
    if (m_fm_raw_buffer.rx_index > 5 && strstr((char*)m_fm_raw_buffer.buffer, "GPS="))
    {
        min_msg_t msg;
        msg.id = MIN_ID_FORWARD;
        msg.payload = m_fm_raw_buffer.buffer;
        msg.len = index;
        min_send_frame(&m_min_host_context, &msg);
    }
    else if (index > 1 && index < (63) && m_lcd_nextion_enable)
    {        
        // DEBUG_DUMP(m_fm_raw_buffer.buffer, m_fm_raw_buffer.rx_index, "LCD");
        // Remove trash pattern from rx buffer {0x1A, 0xFF, 0xFF, 0xFF, 0xFF}
        remove_trash_nextion_data(m_fm_raw_buffer.buffer, m_fm_raw_buffer.rx_index,
                                  m_nextion_lcd_buffer.buffer, &m_nextion_lcd_buffer.rx_index);
        
        if (m_nextion_lcd_buffer.rx_index)
        {
            // Avoid buffer overflow
            if (m_nextion_lcd_buffer.rx_index == LCD_FM_RX_BUFFER_SIZE)
            {
                m_nextion_lcd_buffer.rx_index--;
            }
            
            // DEBUG_DUMP(m_nextion_lcd_buffer.buffer, m_nextion_lcd_buffer.rx_index, "LCD");
            m_nextion_lcd_buffer.buffer[m_nextion_lcd_buffer.rx_index] = 0;
            
            if (strstr((char*)m_nextion_lcd_buffer.buffer, cmd_stop))
            {
                do_update_mode_color = 30;
            }
            else if (strstr((char*)m_nextion_lcd_buffer.buffer, cmd_fm))
            {
                do_update_mode_color = 30;
            }
            else if (strstr((char*)m_nextion_lcd_buffer.buffer, cmd_mic_in_line_in))
            {
                do_update_mode_color = 30;
            }
            else if (strstr((char*)m_nextion_lcd_buffer.buffer, cmd_internet))
            {
                do_update_mode_color = 30;
            }
            else if (strstr((char*)m_nextion_lcd_buffer.buffer, cmd_setting))
            {
                do_update_mode_color = 30;
            }
            else if (strstr((char*)m_nextion_lcd_buffer.buffer, cmd_set_mode_master))
            {
                do_update_mode_color = 30;
            }
            else if (strstr((char*)m_nextion_lcd_buffer.buffer, cmd_set_mode_slave))
            {
                do_update_mode_color = 30;
            }
            
            uint32_t process_counter = process_lcd_keycode((char*)m_nextion_lcd_buffer.buffer);
            
            if (process_counter == 0)
            {
                min_msg_t msg;
                msg.id = MIN_ID_NEXTION_LCD_FORWARD;
                msg.payload = m_nextion_lcd_buffer.buffer;
                msg.len = m_nextion_lcd_buffer.rx_index;
                min_send_frame(&m_min_host_context, &msg);
                
                DEBUG_WARN("Forward LCD %.*s to host\r\n", 
                            m_nextion_lcd_buffer.rx_index, 
                            msg.payload);
            }
        }
    }
    // Reset buffer
    memset(m_fm_raw_buffer.buffer, 0, LCD_FM_RX_BUFFER_SIZE);
    
    board_hw_setting_fm_irq_rx(false);
    m_fm_raw_buffer.rx_index = 0;
    m_fm_lcd_usart_idle_timeout = 0;
    board_hw_setting_fm_irq_rx(true);
}
#endif // FM_LCD_ENABLE


void poll_host_uart_data(void)
{
    uint8_t tmp;
    while (lwrb_read(&m_ring_buffer_host_rx, &tmp, 1))
    {
//// #if DEBUG_MIN_PRO
//        DEBUG_RAW("%u ", tmp);
//// #endif
        min_rx_feed(&m_min_host_context, &tmp, 1);
    }
    min_timeout_poll(&m_min_host_context);
}

void poll_lcd_fm_uart_data(void)
{
#if FM_LCD_ENABLE
    uint8_t tmp;
    while (lwrb_read(&m_ring_buffer_fm, &tmp, 1))
    {
        min_rx_feed(&m_min_fm_context, &tmp, 1);
    }
    min_timeout_poll(&m_min_fm_context);
    process_new_fm_lcd_data();
#endif
}

void poll_debug_data()
{
#if CLI_ENABLE
    uint8_t tmp;
    
    while (lwrb_read(&m_ring_buffer_debug_uart, &tmp, 1))
    {
        app_cli_poll(tmp);
    }

    static uint32_t m_last_tick = 0;
    uint32_t now = sys_get_ms();
    if (now - m_last_tick >= 50)
    {
        m_last_tick = now;
        while (SEGGER_RTT_ReadNoLock(0, &tmp, 1))
        {
            app_cli_poll(tmp);
        }
    }
#endif
}

static void update_power_pin_level(void)
{
    min_msg_t msg;
    uint8_t data;
    data = (board_hw_output_get(BOARD_HW_MODULE_PWR_ON) << 0)
            | (board_hw_output_get(BOARD_HW_MODULE_PWR_KEY) << 1)
            | (board_hw_output_get(BOARD_HW_MODULE_RESET) << 2);
    
    msg.id = MIN_ID_POWER_KEY_LEVEL;
    msg.payload = &data;
    msg.len = 1;
    min_send_frame(&m_min_host_context, &msg);
}


static void do_ping(void)
{
    min_msg_t msg;
    msg.id = MIN_ID_PING;
    msg.payload = &m_host_ping_msg;
    msg.len = sizeof(m_host_ping_msg);
    min_send_frame(&m_min_host_context, &msg);
    
    // dev
    update_power_pin_level();
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

void lcd_display_operation_mode(char *mode)
{
//    if (!m_lcd_nextion_enable)
//    {
//        return;
//    }
//    char tmp[64];
//    sprintf(tmp, "t0.txt=\"%s\"",
//             mode);
//    lcd_nextion_write(tmp);
}

void lcd_set_time(uint16_t year, uint8_t month, uint8_t month_day, uint8_t hour, uint8_t minute, uint8_t second)
{
//    if (m_host_ping_msg.screen_id != LCD_HOME_SCREEN)
//    {
//        return;
//    }
//    
//    if (year < 100)
//    {
//        year += 2000;
//    }
//    
//    char date_time[64];
//    int index = 0;
//    index += sprintf(date_time, "t2.txt=\"%02u:%02u:%02u\"",
//             hour, minute, second);

//    lcd_nextion_write(date_time);
//    
//    index = 0;
//    index += sprintf(date_time, "t3.txt=\"%02u/%02u/%04u\"",
//             month_day, month, year);

//    lcd_nextion_write(date_time);
}

static void process_rtc_time(void)
{
    if (!m_lcd_nextion_enable)
    {
        return;
    }
    
    static board_hw_rtc_time_t m_tm_last_time =
    {
        .year = 0,
    };
    board_hw_rtc_time_t *current_time = board_hw_rtc_get();
        
    if (m_tm_last_time.sec != current_time->sec)
    {
        lcd_set_time(current_time->year, current_time->month, current_time->mday,
                    current_time->hour, current_time->min, current_time->sec);

        memcpy(&m_tm_last_time, current_time, sizeof(m_tm_last_time));
    }
}

char *get_spk_err_code(uint8_t code)
{
    code = code & 0x07;
    switch (code)
    {
        case 1:
            return "OPEN";
        case 8:
            return "OVER TEMP";
        case 6:
            return "SHORT";
        case 7:
            return "OK";
        default:
            return "NA";
    }
}
/*
1. Phát thanh
    - không detect đc TT loa, chỉ detect quá nhiệt
    - không phát xung
2. Không phát thanh
    - có detect loa
    - không tìm được quá nhiệt
    - phát xung như cũ
3. Chu trình reset
    - không phát thanh trong 5s
*/
void audio_classd_fsm_poll()
{
    // Ko detect loa khi dang stream
    static uint8_t monitor_cycle = 0;
    uint8_t tmp = 0;
    m_host_ping_msg.input.name.input0 = !board_hw_get_input(BOARD_HW_INPUT0);
    m_host_ping_msg.input.name.input1 = !board_hw_get_input(BOARD_HW_INPUT1);
    m_host_ping_msg.input.name.input2 = !board_hw_get_input(BOARD_HW_INPUT2);
    m_host_ping_msg.input.name.input3 = !board_hw_get_input(BOARD_HW_INPUT3);
    tmp = (m_host_ping_msg.input.name.input0)
          | (m_host_ping_msg.input.name.input1 << 1)
          | (m_host_ping_msg.input.name.input2 << 2)
          | (m_host_ping_msg.input.name.input3 << 3);
    
    tmp = tmp & 0x08;       // over temp
    if (tmp)
    {
        m_host_ping_msg.error_flag.name.class_d |= tmp;
    }
    else
    {
        m_host_ping_msg.error_flag.name.class_d = m_host_ping_msg.error_flag.name.class_d & 0x07;
    }
    
    if (m_current_output_not_swap_endian.name.pa    // dang stream thi ko detect loa
        || (m_current_output_not_swap_endian.name.class_d != CLASS_D_ACTIVE_LEVEL || m_get_class_d_spk == false))
    {
        monitor_cycle = 0;
        return;
    }
    
    // Not streaming
    monitor_cycle++;
    if (monitor_cycle < 200)
    {
        board_hw_output_set(BOARD_HW_OUTPUT_PA, 0);
    }
    else if (monitor_cycle == 200) // 10.5s
    {
        board_hw_output_set(BOARD_HW_OUTPUT_PA, 1);
    }
    if (monitor_cycle == 203) // 150ms
    {
#if TEST_CLASS_D == 0
        m_get_class_d_spk = false;
#endif
        monitor_cycle = 0;

        m_host_ping_msg.input.name.input0 = !board_hw_get_input(BOARD_HW_INPUT0);
        m_host_ping_msg.input.name.input1 = !board_hw_get_input(BOARD_HW_INPUT1);
        m_host_ping_msg.input.name.input2 = !board_hw_get_input(BOARD_HW_INPUT2);
        m_host_ping_msg.input.name.input3 = 1;      // skip overtemp
        
        tmp = (m_host_ping_msg.input.name.input0)
              | (m_host_ping_msg.input.name.input1 << 1)
              | (m_host_ping_msg.input.name.input2 << 2)
              | (m_host_ping_msg.input.name.input3 << 3);
        
        m_host_ping_msg.error_flag.name.class_d = tmp;
        
        /*
            IN1   2   3   4     Result
            1     0   0   0     Open            = 1
            0     1   1   1     Short load      = 6
            1     1   1   1     OK              = 7
            1     0   1   1     Short to GND    = 5
            x     x   x   1     Overtemp        &= 8
         */
        DEBUG_WARN("ClassD I[1-2-3-4] %d%d%d%d = %d, mean %s\r\n", 
                    m_host_ping_msg.input.name.input0,
                    m_host_ping_msg.input.name.input1,
                    m_host_ping_msg.input.name.input2,
                    m_host_ping_msg.input.name.input3,
                    m_host_ping_msg.error_flag.name.class_d,
                    get_spk_err_code(m_host_ping_msg.error_flag.name.class_d));
        board_hw_output_set(BOARD_HW_OUTPUT_PA, 0);
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
        update_operation_mode();
    }
    
    if (sys_get_ms() - m_last_time_poll_wdt >= (uint32_t)50)
    {
        update_adc();
        update_interface();
        
        
        // Reload watchdog
        m_last_time_poll_wdt = sys_get_ms();
        board_hw_watchdog_feed();
        update_peripheral_input();
        process_rtc_time();
        audio_classd_fsm_poll();
    }
    if (sys_get_ms() - m_last_time_update_time >= (uint32_t)999)
    {    
        m_last_time_update_time = sys_get_ms();
        
        do_ping();

        if (m_test_mode)
        {
            led_blink_when_host_die();
            if (m_test_mode)
            {
                board_hw_output_toggle(BOARD_HW_OUTPUT_SW_OUT);
                board_hw_output_toggle(BOARD_HW_OUTPUT1);
                board_hw_output_toggle(BOARD_HW_OUTPUT2);
            }
        }
       
        
#if TEST_LCD_NEXTION
        test_ui_msg();
#endif
    }
    
//    if (m_fm_lcd_uart_baud == 0 || m_fm_lcd_uart_baud == 0xFFFFFFFF)
//        m_fm_lcd_uart_baud = board_hw_auto_measure_baudrate();
    
    poll_host_uart_data();
    poll_lcd_fm_uart_data();
    poll_debug_data();
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

#if FM_LCD_ENABLE

static bool fm_lcd_uart_send_byte(void *ctx, uint8_t data)
{
    board_hw_fm_lcd_send(&data, 1);
    return true;
}

#endif

// 1ms interrupt callback
void host_data_layer_on_1ms_callback(void)
{
#if FM_LCD_ENABLE
    if (m_fm_lcd_usart_idle_timeout > 0)
    {
        m_fm_lcd_usart_idle_timeout--;
        if (m_fm_lcd_usart_idle_timeout == 0)
        {
            m_fm_lcd_usart_idle_timeout = -1;     // ready for processing data
        }
    }
#endif
    if (m_delay_skip_over_temp > 1)
    {
        m_delay_skip_over_temp--;
    }
}

// USART FM : receive data handler
void board_hw_on_fm_lcd_rx_callback(uint8_t *data, uint32_t length)
{
#if FM_LCD_ENABLE
    if (m_fm_lcd_usart_idle_timeout != -1)        // -1 mean processing data
    {
        for (uint32_t i = 0; i < length; i++)
        {
            m_fm_raw_buffer.buffer[m_fm_raw_buffer.rx_index++] = data[i];
            if (m_fm_raw_buffer.rx_index == LCD_FM_RX_BUFFER_SIZE)
            {
                m_fm_raw_buffer.rx_index = 0;
            }
        }
        m_fm_lcd_usart_idle_timeout = 3;
    }
    
    if (m_fm_protocol_type != FM_PROTOCOL_TYPE_STRING_FORMAT)
    {
        if (length != lwrb_write(&m_ring_buffer_fm, data, length))
        {
            DEBUG_ERROR("FM ring buffer full\r\n");
            lwrb_reset(&m_ring_buffer_fm);
        }
    }
#else       // fm uart use as debug port
    for (uint32_t i = 0; i < length; i++)
    {
        if (lwrb_write(&m_ring_buffer_debug_uart, data+i, 1) == 0)
        {  
            // Ringbuffer full
            return;
        }
    }
#endif
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
    DEBUG_INFO("Keep alive uart counter %u\r\n", timeout);
}
      

void host_data_layer_set_ping_timeout(uint32_t timeout)
{
    m_pulse_heartbeat_counter = timeout;
    DEBUG_INFO("Heartbeat counter %u\r\n", m_pulse_heartbeat_counter);
}

#if CLI_ENABLE
static void cli_puts(uint8_t *buffer, uint32_t size)
{
#if FM_LCD_ENABLE == 0
    board_hw_fm_lcd_send(buffer, size);
#endif
    SEGGER_RTT_Write(0, buffer, size);
}

static int cli_printf(const char *msg)
{
    int len = strlen(msg);
#if FM_LCD_ENABLE == 0
    board_hw_fm_lcd_send((uint8_t*)msg, len);
#endif
    SEGGER_RTT_Write(0, (uint8_t*)msg, len);
    return len;
}
#endif
