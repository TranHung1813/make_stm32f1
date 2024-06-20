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
#define HOST_DIE_COUNT_MAX_TIME_RESET                       (3)
#define HOST_PING_SHORT_INTERVAL_MS                         (50)
#define HOST_PING_LONG_INTERVAL_MS                          (1000)
#define HOST_POWER_ON_MS                                    (1800000)
#define HOST_POWER_OFF_MS                                   (100000)
#define LCD_FM_RX_BUFFER_SIZE                               255
#define VIN_LOST_THRESHOLD                                  9000
#define DISABLE_UART_TIMEOUT_AFTER_SMART_MODULE_POWER_ON    (2)         //

#define CORE_POWER_ON()                                     board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0)
#define CORE_POWER_OFF()                                    board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1)
#define MAX_HEARTBEAT_COUNTER_IN_RUN_MODE                   1800    // 180s
#define MAX_HEARTBEAT_COUNTER_AFTER_MCU_BOOT                3600    // 360s
#define MAX_HEARTBEAT_COUNTER_AFTER_MCU_SOFTWARE_RESET      100    // 10s
#define DEFAULT_HOST_UART_TIMEOUT_DO_RESET                  36000   // 1h
#define DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU            (72000) // 2h

#define FM_ENABLE                                           0
#define V_ADC_ENDIAN_VIN_INDEX 0
#define V_ADC_ENDIAN_3V8_INDEX 1
#define V_ADC_ENDIAN_24V_INDEX 2

#if FM_ENABLE
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

// Ring buffer host
static lwrb_t m_ring_buffer_host_rx;
uint8_t m_uart_host_buffer[1024];
static uint8_t m_min_host_rx_buffer[MIN_MAX_PAYLOAD];
uint32_t m_ring_buffer_host_full = 0;
static void cli_puts(uint8_t *buffer, uint32_t size);
static int cli_printf(const char *msg);
static int32_t m_count_host_boot_time = -1;
static int32_t m_is_vin_detect = -1;
static inline void host_data_layer_send(uint8_t *data, uint32_t length);

#if FM_ENABLE
static uint8_t m_fm_uart_raw_rx_buffer[LCD_FM_RX_BUFFER_SIZE];
// Ring buffer FM
static lwrb_t m_ring_buffer_fm;
static void on_fm_frame_callback(void *ctx, min_msg_t *frame);
static uint8_t m_min_fm_rx_buffer[MIN_MAX_PAYLOAD];
static void do_ping(void);
// LCD buffer
static lcd_fm_buffer_t m_fm_lcd_buffer = 
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

// Host min protocol context
static min_context_t m_min_host_context;
static min_frame_cfg_t m_min_host_setting = MIN_DEFAULT_CONFIG();
static bool uart_tx_to_host(void *ctx, uint8_t data);
static void on_host_frame_callback(void *ctx, min_msg_t *frame);


static volatile uint32_t m_last_time_send_ping = 0;
volatile uint32_t m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_AFTER_MCU_BOOT;

static bool m_test_mode = false;
static uint32_t m_host_keep_alive_uart = DEFAULT_HOST_UART_TIMEOUT_DO_RESET;     // 1h
static uint32_t m_host_die_counter_fource_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
static host_power_state_t m_host_power_state = HOST_POWER_RUNNING;
static volatile int32_t m_fm_lcd_idle_timeout = 0;
static uint8_t m_vin_lost_times = 0;
static uint32_t ignore_continues_write_to_flash = 0;
static host_data_layer_ping_msg_t m_host_ping_msg;
// Voltage low pass filter
static lpf_data_t m_lpf[3] = 
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
};

static bool m_lcd_nextion_enable = false;

static uint32_t m_led_blink_indicator_step = 0;

static void update_output()
{
    // Control GPIO
    board_hw_output_set(BOARD_HW_OUTPUT_LED_1_R, m_host_ping_msg.output.name.lte_led_r);
    board_hw_output_set(BOARD_HW_OUTPUT_LED_1_B, m_host_ping_msg.output.name.lte_led_b);
    board_hw_output_set(BOARD_HW_OUTPUT_LED_2_R, m_host_ping_msg.output.name.eth_led_r);
    board_hw_output_set(BOARD_HW_OUTPUT_LED_2_B, m_host_ping_msg.output.name.eth_led_b);
    board_hw_output_set(BOARD_HW_OUTPUT1, m_host_ping_msg.output.name.relay1);
    board_hw_output_set(BOARD_HW_OUTPUT2, m_host_ping_msg.output.name.relay2);
    board_hw_output_set(BOARD_HW_OUTPUT_PA, m_host_ping_msg.output.name.pa);
    board_hw_output_set(BOARD_HW_OUTPUT_SW_IN, m_host_ping_msg.output.name.switch_in); 
    board_hw_output_set(BOARD_HW_OUTPUT_SW_OUT, m_host_ping_msg.output.name.switch_out);  
    APP_DEBUG_VERBOSE("PA %s\r\n", m_host_ping_msg.output.name.pa ? "TRUE" : "FALSE");
    APP_DEBUG_INFO("SW IN %s, SWOUT %s\r\n", m_host_ping_msg.output.name.switch_in ? "TRUE" : "FALSE",
                                        m_host_ping_msg.output.name.switch_out ? "TRUE" : "FALSE");
    
}   

static uint16_t m_reset_counter = 0;
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
    m_min_host_setting.timeout_not_seen_rx = 3000;
    m_min_host_setting.tx_byte = uart_tx_to_host;
    m_min_host_setting.use_timeout_method = 1;

    m_min_host_context.callback = &m_min_host_setting;
    m_min_host_context.rx_frame_payload_buf = m_min_host_rx_buffer;
    min_init_context(&m_min_host_context);

#if FM_ENABLE
    lwrb_init(&m_ring_buffer_fm, m_fm_uart_raw_rx_buffer, sizeof(m_fm_uart_raw_rx_buffer));
    // FM
    m_min_fm_setting.get_ms = sys_get_ms;
    m_min_fm_setting.last_rx_time = 0x00;
    /* Not using the rx_callback then*/
    m_min_fm_setting.rx_callback = on_fm_frame_callback;
    m_min_fm_setting.timeout_not_seen_rx = 3000;
    m_min_fm_setting.tx_byte = fm_lcd_uart_send_byte;
    m_min_fm_setting.use_timeout_method = 1;
    //m_min_fm_context.cb = &m_min_fm_setting;
    m_min_fm_context.rx_frame_payload_buf = m_min_fm_rx_buffer;
    m_min_fm_context.callback = &m_min_fm_setting;
    min_init_context(&m_min_fm_context);
#else
    lwrb_init(&m_ring_buffer_debug_uart, m_debug_rx_buffer, sizeof(m_debug_rx_buffer));
#endif
    
    char *p = VERSION_CONTROL_HW;
    
    m_host_ping_msg.hardware_version[0] = *p++ - '0';
    m_host_ping_msg.hardware_version[1] = *p++ - '0';
    m_host_ping_msg.hardware_version[2] = *p++ - '0';
    
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
        APP_DEBUG_INFO("Power on reset\r\n");
    }
    else
    {
        board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1);
        board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 0);
        m_host_power_state = HOST_POWER_RUNNING;
        m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_AFTER_MCU_SOFTWARE_RESET;
        APP_DEBUG_INFO("Software reset\r\n");
    }
    
    // CLI
    static app_cli_cb_t cli_cb;
    cli_cb.printf = cli_printf;
    cli_cb.puts = cli_puts;
    app_cli_start(&cli_cb);
    
    // Set digital vol
#if DIGITAL_POT
    board_hw_digital_pot_set(100);
#endif
    board_hw_critical_log_t err;
    if (board_hw_get_critical_error(&err))
    {
        if (err.name.is_heartbeat_err)
        {
            APP_DEBUG_ERROR("Ping gpio error\r\n");
        }
        if (err.name.is_uart_communication_error)
        {
            APP_DEBUG_ERROR("UART communication error\r\n");
        }
        if (err.name.is_vin_lost)
        {
            APP_DEBUG_ERROR("Found vin lost\r\n");
        }
        m_reset_counter = err.name.reset_counter+1;
    }
    else
    {
        err.value = 0;
    }
    APP_DEBUG_INFO("Reset counter %u\r\n", err.name.reset_counter);
}

#if FM_ENABLE
static void on_fm_frame_callback(void *ctx, min_msg_t *frame)
{
    m_fm_protocol_type = FM_PROTOCOL_TYPE_MIN_PROTOCOL;
    APP_DEBUG_INFO("Received FM frame id %u, size %u\r\n", frame->id, frame->len);
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
}
#endif

static host_data_layer_output_t m_prev_gpio_output_control =
{
    .value = 0,
};


static uint16_t v_adc_endian[3];

static void update_adc(void)
{
    if (!board_hw_has_new_adc_data())
    {
        return;
    }
    uint16_t *adc_ptr = board_hw_get_adc_data();
    uint16_t adc_buffer[3] = {adc_ptr[0], adc_ptr[1], adc_ptr[2]};
    
    board_hw_allow_adc_conversion();
    
    uint32_t tmp = adc_buffer[V_ADC_ENDIAN_VIN_INDEX]*11;      // 11 = resistor div
    tmp *= 3300;
    tmp /= 4096;
    
    tmp += 300;     // diode offset
    
    if (m_lpf[V_ADC_ENDIAN_VIN_INDEX].gain == 0)
    {
        m_lpf[V_ADC_ENDIAN_VIN_INDEX].gain = 5;     // percent
        m_lpf[V_ADC_ENDIAN_VIN_INDEX].estimate_value = tmp;
    }
    else
    {
        lpf_update_estimate(&m_lpf[V_ADC_ENDIAN_VIN_INDEX], (int32_t*)&tmp);
    }
    
    v_adc_endian[V_ADC_ENDIAN_VIN_INDEX] = m_lpf[V_ADC_ENDIAN_VIN_INDEX].estimate_value + 300;      // 200mv = diode offset
    
    // Swap Vin endian
    m_host_ping_msg.vin_mv = ((v_adc_endian[V_ADC_ENDIAN_VIN_INDEX] & 0xFF) << 8) | (v_adc_endian[V_ADC_ENDIAN_VIN_INDEX] >> 8);
    
    
    // Do low pass filter for V3V8
    tmp = adc_buffer[V_ADC_ENDIAN_3V8_INDEX]*2;            // 2 = resistor div
    tmp *= 3300;
    tmp /= 4096;
    
    if (m_lpf[V_ADC_ENDIAN_3V8_INDEX].gain == 0)
    {
        m_lpf[V_ADC_ENDIAN_3V8_INDEX].gain = 1;     // percent
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
    tmp = adc_buffer[V_ADC_ENDIAN_24V_INDEX]*11;            // 11 = resistor div
    tmp *= 3300;
    tmp /= 4096;
    
    if (m_lpf[V_ADC_ENDIAN_24V_INDEX].gain == 0)
    {
        m_lpf[V_ADC_ENDIAN_24V_INDEX].gain = 1;     // percent
        m_lpf[V_ADC_ENDIAN_24V_INDEX].estimate_value = tmp;
    }
    else
    {
        lpf_update_estimate(&m_lpf[V_ADC_ENDIAN_24V_INDEX], (int32_t*)&tmp);
    }
    
    // Swap 5V endian
    v_adc_endian[V_ADC_ENDIAN_24V_INDEX] = m_lpf[V_ADC_ENDIAN_24V_INDEX].estimate_value;      // 200mv = diode offset
    m_host_ping_msg.vbus_24v_mv = ((v_adc_endian[V_ADC_ENDIAN_24V_INDEX] & 0xFF) << 8) | (tmp >> 8);

    if (m_test_mode  || 1)
    {
        static uint32_t m_log = 0;
        if (m_log++ % 100 == 0 | m_is_vin_detect == 0)
        {
            APP_DEBUG_INFO("Vin %u, v5v %u, 3v8 %u\r\n", v_adc_endian[V_ADC_ENDIAN_VIN_INDEX], v_adc_endian[V_ADC_ENDIAN_24V_INDEX], v_adc_endian[V_ADC_ENDIAN_3V8_INDEX]);
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
        APP_DEBUG_INFO("Input changed from 0x%08X to 0x%08X\r\n", m_last_input.value, m_host_ping_msg.input.value);
        m_last_input.value = m_host_ping_msg.input.value;
    }
    
    // Swap input endian
    uint16_t input_swap_endian = m_host_ping_msg.input.value;
    m_host_ping_msg.input.value = ((input_swap_endian & 0xFF) << 8) | (input_swap_endian >> 8);
    
    // Swap output endian
    uint16_t output_swap_endian = m_host_ping_msg.output.value;
    m_host_ping_msg.output.value = ((output_swap_endian & 0xFF) << 8) | (output_swap_endian >> 8); 
}


void host_data_layer_set_test_mode(bool test)
{
    m_test_mode = test;
}

void host_data_layer_set_power_mode(host_power_state_t mode)
{
    APP_DEBUG_INFO("Set power state to %d\r\n", mode);
    m_host_power_state = mode;
    if (m_host_power_state == HOST_POWER_FLASH_MODE)
    {
        board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0);
    }
}

static void led_blink_when_host_die(void)
{
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

bool copy_parameters(char *src, char *dst, char comma_begin, char comma_end)
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

static void on_host_frame_callback(void *ctx, min_msg_t *frame)
{
    APP_DEBUG_VERBOSE("On min frame callback, id 0x%02X, len %u\r\n", frame->id, frame->len);
    // Reset timeout
    m_host_keep_alive_uart = DEFAULT_HOST_UART_TIMEOUT_DO_RESET;
    m_host_die_counter_fource_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
    m_led_blink_indicator_step = 0;
    
    if (m_count_host_boot_time != -1)
    {
        APP_DEBUG_INFO("Host boot time %u sec\r\n", m_count_host_boot_time / 10);
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
                APP_DEBUG_INFO("Invalid frame %u bytes\r\n", frame->len);
                return;
            }
            memcpy(&m_host_ping_msg.output, frame->payload, sizeof(host_data_layer_output_t));
            if (m_prev_gpio_output_control.value != m_host_ping_msg.output.value)
            {
                APP_DEBUG_WARN("GPIO changed from 0x%08X to 0x%08X\r\n", 
                                m_prev_gpio_output_control.value, m_host_ping_msg.output.value);
                                m_prev_gpio_output_control.value = m_host_ping_msg.output.value;
            }
            update_output();
        }
        break;

        case MIN_ID_PING:
        {

        }
        break;
        
        case MIN_ID_FORWARD:
        {
#if FM_ENABLE
            uint8_t *ptr = frame->payload;
            APP_DEBUG_INFO("Forward to FM\r\n");
            if (m_fm_protocol_type != FM_PROTOCOL_TYPE_MIN_PROTOCOL)
            {
                APP_DEBUG_INFO("Forward to FM %.*s\r\n", frame->len, (char*)ptr);
            }
#endif
            board_hw_fm_lcd_send(frame->payload, frame->len);
        }
            break;
        
        case MIN_ID_NEXTION_LCD:
        {
            uint8_t terminate[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            if (!m_lcd_nextion_enable)
            {
                APP_DEBUG_INFO("Change FM lcd buadrate\r\n");
                m_lcd_nextion_enable = 1;
                board_hw_change_lcd_fm_baudrate(9600);
            }
            board_hw_fm_lcd_send(frame->payload, frame->len);
            board_hw_fm_lcd_send(terminate, sizeof(terminate));
        }
            break;
        
        case MIN_ID_UPDATE_UI:
        {
//            // debug only, dont care about it
//            char *ptr = frame->payload;
//            if (ptr && frame->len)
//            {
//                APP_DEBUG_INFO("%.*s\r\n", frame->len, ptr);
//                ptr = strstr(ptr, "ip=\"");
//                if (ptr)
//                {
//                    char ip[24];
//                    memset(ip, 0, 24);
//                    ptr += 3;
//                    copy_parameters(ptr, ip, '"', '"');
//                    APP_DEBUG_INFO("IP = %s\r\n", ip);
//                }                    
//            }
        }
            break;
        
        case MIN_ID_RESET:
            // Force shutdown module
            APP_DEBUG_WARN("Host request mcu force shutdown module");
            if (m_host_power_state != HOST_POWER_DOWN)
            {
                m_host_power_state = HOST_POWER_DOWN;
            }
            break;
            
        case MIN_ID_SET_UNIX_TIMESTAMP:
            break;
        
        case MIN_ID_FORWARD_DBG_MSG:
        {
            APP_DEBUG_WARN("Android -> %.*s\r\n", frame->len, (char*)frame->payload);
        }
            break;
        
        case MIN_ID_REQUEST_RESET_REASON:
        {
            // Send reset message to host
            board_hw_critical_log_t err;
            board_hw_get_critical_error(&err);
            uint8_t reset_msg[5];
            reset_msg[0] = (err.name.reset_counter & 0xFF00) >> 8;
            reset_msg[1] = (err.name.reset_counter & 0xFF);
            reset_msg[2] = err.name.is_vin_lost;
            reset_msg[3] = err.name.is_uart_communication_error;
            reset_msg[4] = err.name.is_heartbeat_err;
            
            min_msg_t min_rst_msg;
            min_rst_msg.id = MIN_ID_REQUEST_RESET_REASON;
            min_rst_msg.payload = reset_msg;
            min_rst_msg.len = sizeof(reset_msg);
            min_send_frame(&m_min_host_context, &min_rst_msg);
            
            // Clear luon nguyen nhan reset
            err.name.is_heartbeat_err = 0;
            err.name.is_uart_communication_error = 0;
            err.name.is_vin_lost = 0;
            if (ignore_continues_write_to_flash == 0)
            {
                ignore_continues_write_to_flash = 300;   // 30s
                board_hw_set_critical_error(err);
            }
            APP_DEBUG_INFO("Send reset counter 0x%02X%02X to host\r\n", reset_msg[0], reset_msg[1]);
        }
            break;

        case MIN_ID_SET_VOLUME_CALL:
        {
            static uint8_t m_last_vol = 0;
            uint8_t vol = *((uint8_t*)frame->payload);
            if (m_last_vol != vol)
            {
                m_last_vol = vol;
#if DIGITAL_POT
                board_hw_digital_pot_set(vol);
#endif
                APP_DEBUG_INFO("Set vol %u\r\n", vol);
            }
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
        
        case MIN_ID_OTA_UPDATE_START:
        {
            /* 4 bytes size */
            APP_DEBUG_WARN("OTA start\r\n");
            // Disable USART485 RB interrupt
            usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_DISABLE);
            usart_interrupt_disable(FM_USARTPeripheral, USART_INT_RBNE);
            bool retval;
            uint8_t *p = (uint8_t *)frame->payload;
            uint32_t firmware_size = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | (p[3]);
            
            usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE);
            retval = ota_update_start(firmware_size);
            usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_ENABLE);
            
            if (retval)
            {
                // Send ack
                min_msg_t rsp_msg;
                uint8_t tmp = 0;
                APP_DEBUG_WARN("OTA ACK\r\n");
                rsp_msg.id = MIN_ID_OTA_UPDATE_START; //MIN_ID_OTA_ACK;
                rsp_msg.len = 1;
                rsp_msg.payload = &tmp;
                min_send_frame(&m_min_host_context, &rsp_msg);
            }
            else // Send failed
            {
                min_msg_t rsp_msg;
                uint8_t tmp = 1;
                APP_DEBUG_WARN("OTA start failed\r\n");
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
            usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE);
            usart_interrupt_disable(HOST_USARTPeripheral, USART_INT_RBNE);
            bool retval = ota_update_write_next(p, frame->len);
            usart_interrupt_enable(HOST_USARTPeripheral, USART_INT_RBNE);
            usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_ENABLE);
            if (retval)
            {
                APP_DEBUG_WARN("Do ack\r\n");
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
                APP_DEBUG_WARN("Failed\r\n");
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
            APP_DEBUG_WARN("OTA end\r\n");
            
            // disable host usart
            usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE);
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
            NVIC_SystemReset();
            while (1);
        }
    //    break;


        case MIN_ID_MASTER_OTA_END:
        {
            
        }
        break;
        case MIN_ID_FORWARD_FM_MSG_TO_MCU:
            break;
        default:
            APP_DEBUG_WARN("Unhandled min frame id %u, size %u\r\n", frame->id, frame->payload);
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
    static uint32_t m_vin_event_counter = 0;
    
    // Set default state at startup
    if (m_is_vin_detect == -1)
    {
        m_vin_event_counter = 0;
        if (m_lpf[0].estimate_value > VIN_LOST_THRESHOLD)
        {
            m_is_vin_detect = 1;
        }
        else
        {
            // Vin lost khi chuyen trang thai tu 1 -> 0, con khi bat dau vin chua dat VIN_LOST_THRESHOLD thi khong tinh
            return;
            // m_is_vin_detect = 0;
        }
    }  
    
    if (fake_power_event)
    {
        m_lpf[0].estimate_value = VIN_LOST_THRESHOLD >> 1;
    }
    // Poll power state machine
    if (m_is_vin_detect == 0)
    {
        if (m_lpf[0].estimate_value > VIN_LOST_THRESHOLD && m_vin_event_counter++ > 30)     // 3s
        {
            m_vin_event_counter = 0;
            m_is_vin_detect = 1;
            APP_DEBUG_INFO("Vin plugged again\r\n");
            board_hw_output_set(BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION, 0);
        }
    }
    else
    {
        m_vin_event_counter = 0;
        if (m_lpf[0].estimate_value < VIN_LOST_THRESHOLD)
        {
            m_is_vin_detect = 0;
            if (m_vin_lost_times < 255)
                m_vin_lost_times++;
            APP_DEBUG_WARN("Vin lost %u times\r\n", m_vin_lost_times);
            m_host_power_state = HOST_POWER_WAIT_FOR_VIN;
            
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
    
    board_hw_output_set(BOARD_HW_MODULE_RESET, 0);      // dont care about reset pin
    
    if (m_host_die_counter_fource_reset_mcu)      // 10000*100ms = 10000s
    {
        m_host_die_counter_fource_reset_mcu--;
        if (m_host_die_counter_fource_reset_mcu == 0)
        {
            m_host_die_counter_fource_reset_mcu = DEFAULT_HOST_DIE_COUNTER_FORCE_RESET_MCU;
            APP_DEBUG_ERROR("Long time no see host -> software reset MCU\r\n");
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
            APP_DEBUG_ERROR("Set host to force shutdown\r\n");
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
            m_count_host_boot_time = -1;
            // Release pwk to default status
            board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 0);
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0);
            m_pulse_heartbeat_counter = 0;
            if (!is_vin_detect())
            {
                m_power_state_counter = 0;
            }
            if (m_power_state_counter++ > 50)
            {
                m_power_state_counter = 0;
                APP_DEBUG_WARN("Change power state to UP\r\n");
                m_host_power_state = HOST_POWER_UP;
            }
        }
            break;
        
        case HOST_POWER_UP:
        {
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1);
            if (!is_vin_detect() || v_adc_endian[V_ADC_ENDIAN_3V8_INDEX] < 3300)
            {
                APP_DEBUG_INFO("Vin detect %u, v3v8 %u\r\n", is_vin_detect() ? 1 : 0, v_adc_endian[V_ADC_ENDIAN_3V8_INDEX]);
                m_power_state_counter = 0;
            }
            if (m_power_state_counter++ > 50)     // 5s
            {
                m_power_state_counter = 0;
                m_host_power_state = HOST_POWER_POWER_KEY_SEQ;
                board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 1);
                APP_DEBUG_WARN("Change power state control power key sequence\r\n");
            }
        }
            break;

        case HOST_POWER_POWER_KEY_SEQ:
        {
            if (!is_vin_detect())
            {
                m_power_state_counter = 0;
            }
            
            if (m_power_state_counter++ > 30)     // 3s
            {
                m_power_state_counter = 0;
                m_host_power_state = HOST_POWER_RUNNING;
                board_hw_output_set(BOARD_HW_MODULE_PWR_KEY, 0);
                m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_IN_RUN_MODE;
                APP_DEBUG_WARN("Change power state to RUNNING\r\n");
                m_count_host_boot_time = 0;
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
                    APP_DEBUG_INFO("Long time not see uart module->reset\r\n");
                    m_host_power_state = HOST_POWER_DOWN;
                    board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 0);
                    m_pulse_heartbeat_counter = MAX_HEARTBEAT_COUNTER_IN_RUN_MODE;
                    
                    // Save uart lost error code to flash
                    board_hw_critical_log_t err;
                    board_hw_get_critical_error(&err);
                    err.name.is_uart_communication_error = 1;
                    board_hw_set_critical_error(err);  
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
            
            board_hw_output_set(BOARD_HW_MODULE_PWR_ON, 1); // always on SC20 power
            if (is_vin_detect())
            {
                m_pulse_heartbeat_counter = 0;
                APP_DEBUG_WARN("Change power state to DOWN\r\n");
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

#if FM_ENABLE
static void process_new_fm_lcd_data(void)
{
    // Forward data to host module
    if (m_fm_lcd_idle_timeout != -1         // No idle line
        || m_fm_protocol_type == FM_PROTOCOL_TYPE_MIN_PROTOCOL) 
    {
        return;
    }
    
    if (strstr((char*)m_fm_lcd_buffer.buffer, "FM|SNR="))
    {
        m_fm_protocol_type = FM_PROTOCOL_TYPE_STRING_FORMAT;
    }
    uint32_t index = m_fm_lcd_buffer.rx_index;
    if (m_fm_lcd_buffer.rx_index > 5 && strstr((char*)m_fm_lcd_buffer.buffer, "GPS="))
    {
        min_msg_t msg;
        msg.id = MIN_ID_FORWARD;
        msg.payload = m_fm_lcd_buffer.buffer;
        msg.len = index;
        min_send_frame(&m_min_host_context, &msg);
    }
//        else if (index > 5 && strstr((char*)m_fm_lcd_buffer.Buffer, "{\"test\":1}"))
//        {
//            is_in_test_mode = 1;
//        }
    else if (index < (63) && m_lcd_nextion_enable)
    {
        min_msg_t msg;
        msg.id = MIN_ID_NEXTION_LCD;
        msg.payload = m_fm_lcd_buffer.buffer;
        msg.len = index;
        // UART_Putb(ESP32_UART, FMBuffer.Buffer, index);
        min_send_frame(&m_min_host_context, &msg);
    }
    // Reset buffer
    memset(m_fm_lcd_buffer.buffer, 0, LCD_FM_RX_BUFFER_SIZE);
    m_fm_lcd_buffer.rx_index = 0;
    __disable_irq();
    m_fm_lcd_idle_timeout = 0;
    __enable_irq();
}
#endif

void poll_host_uart_data(void)
{
    uint8_t tmp;
    while (lwrb_read(&m_ring_buffer_host_rx, &tmp, 1))
    {
        min_rx_feed(&m_min_host_context, &tmp, 1);
    }
}

void poll_lcd_fm_uart_data(void)
{
#if FM_ENABLE
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
#if FM_ENABLE == 0
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

static void do_ping(void)
{
    min_msg_t msg;
    msg.id = MIN_ID_PING;
    msg.payload = &m_host_ping_msg;
    msg.len = sizeof(m_host_ping_msg);
    min_send_frame(&m_min_host_context, &msg);
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
        update_adc();
        
        // Reload watchdog
        m_last_time_poll_wdt = sys_get_ms();
        board_hw_watchdog_feed();
        update_peripheral_input();
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
    }
    
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
#if FM_ENABLE
static bool fm_lcd_uart_send_byte(void *ctx, uint8_t data)
{
    board_hw_fm_lcd_send(&data, 1);
    return true;
}


//static void fm_lcd_uart_send(uint8_t *data, uint32_t length)
//{
//    board_hw_fm_lcd_send(data, length);
//}
#endif

// 1ms interrupt callback
void host_data_layer_on_1ms_callback(void)
{
#if FM_ENABLE
    if (m_fm_lcd_idle_timeout > 0)
    {
        m_fm_lcd_idle_timeout--;
        if (m_fm_lcd_idle_timeout == 0)
        {
            m_fm_lcd_idle_timeout = -1;     // ready for processing data
        }
    }
#endif
}

// USART FM : receive data handler
void board_hw_on_fm_lcd_rx_callback(uint8_t *data, uint32_t length)
{
#if FM_ENABLE
    if (m_fm_lcd_idle_timeout != -1)        // -1 mean processing data
    {
        for (uint32_t i = 0; i < length; i++)
        {
            m_fm_lcd_buffer.buffer[m_fm_lcd_buffer.rx_index++] = data[i];
            if (m_fm_lcd_buffer.rx_index == LCD_FM_RX_BUFFER_SIZE)
            {
                m_fm_lcd_buffer.rx_index = 0;
            }
        }
        m_fm_lcd_idle_timeout = 10;
    }
    
    if (m_fm_protocol_type != FM_PROTOCOL_TYPE_STRING_FORMAT)
    {
        if (length == lwrb_write(&m_ring_buffer_fm, data, length))
        {
            APP_DEBUG_ERROR("FM ring buffer full\r\n");
            lwrb_reset(&m_ring_buffer_fm);
        }
    }
#else
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
    if (!lwrb_write(&m_ring_buffer_host_rx, data, length))
    {
        APP_DEBUG_ERROR("Ring buffer host full\r\n");
//        board_hw_reset();     // normally error when overrun orcus
        
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
    APP_DEBUG_INFO("Keep alive uart counter %u\r\n", timeout);
}
      

void host_data_layer_set_ping_timeout(uint32_t timeout)
{
    m_pulse_heartbeat_counter = timeout;
    APP_DEBUG_INFO("Heartbeat counter %u\r\n", m_pulse_heartbeat_counter);
}

static void cli_puts(uint8_t *buffer, uint32_t size)
{
    board_hw_fm_lcd_send(buffer, size);
    SEGGER_RTT_Write(0, buffer, size);
}

static int cli_printf(const char *msg)
{
    int len = strlen(msg);
    board_hw_fm_lcd_send((uint8_t*)msg, len);
    SEGGER_RTT_Write(0, (uint8_t*)msg, len);
    return len;
}
