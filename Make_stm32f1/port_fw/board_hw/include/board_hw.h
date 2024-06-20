#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
    
   
#define BOARD_HW_SMART_MODULE_PWR_CONTROL_PIN                           GPIO_PIN_13
#define BOARD_HW_SMART_MODULE_PWR_CONTROL_PORT                          GPIOC
#define BOARD_HW_ADC_VIN_Pin                                            GPIO_PIN_0
#define BOARD_HW_ADC_VIN_PORT                                           GPIOA
#define BOARD_HW_ADC_3V8_PIN                                            GPIO_PIN_1
#define BOARD_HW_ADC_3V8_PORT                                           GPIOA
//#define FM_USART_TX_Pin                                               GPIO_PIN_2
//#define FM_USART_TX_GPIO_Port                                         GPIOA
//#define FM_USART_RX_Pin                                               GPIO_PIN_3
//#define FM_USART_RX_GPIO_Port                                         GPIOA
#define BOARD_HW_ADC_5V_PIN                                             GPIO_PIN_4
#define BOARD_HW_ADC_5V_PORT                                            GPIOA
#define BOARD_HW_EXT_WDT_PIN                                            GPIO_PIN_5
#define BOARD_HW_EXT_WDT_PORT                                           GPIOA
#define BOARD_HW_SMART_MODULE_RST_PIN                                   GPIO_PIN_6
#define BOARD_HW_SMART_MODULE_RST_PORT                                  GPIOA
#define BOARD_HW_TO_SM_POWER_STATE_PIN                                  GPIO_PIN_7
#define BOARD_HW_TO_SM_POWER_STATE_PORT                                 GPIOA
#define BOARD_HW_HEARTBEAT_PIN                                          GPIO_PIN_2
#define BOARD_HW_HEARTBEAT_PORT                                         GPIOB
#define BOARD_HW_HEARTBEAT_EXTI_IRQn                                    EXTI2_3_IRQn
#define BOARD_HW_SW_OUT_PIN                                             GPIO_PIN_11
#define BOARD_HW_SW_OUT_PORT                                            GPIOB
#define BOARD_HW_SW_IN_PIN                                              GPIO_PIN_10
#define BOARD_HW_SW_IN_PORT                                             GPIOB
#define BOARD_HW_SMART_MODULE_PWR_KEY_PIN                               GPIO_PIN_8
#define BOARD_HW_SMART_MODULE_PWR_KEY_PORT                              GPIOA
#define BOARD_HW_MIN_USART_TX_PIN                                       GPIO_PIN_9
#define BOARD_HW_MIN_USART_TX_PORT                                      GPIOA
#define BOARD_HW_MIN_USART_RX_PIN                                       GPIO_PIN_10
#define BOARD_HW_MIN_USART_RX_PORT                                      GPIOA
#define BOARD_HW_DISABLE_HOST_UART_PIN                                  GPIO_PIN_11
#define BOARD_HW_DISABLE_HOST_UART_PORT                                 GPIOA
#define BOARD_HW_PA_CONTROL_PIN                                         GPIO_PIN_12
#define BOARD_HW_PA_CONTROL_PORT                                        GPIOA
#define BOARD_HW_INPUT1_PIN                                             GPIO_PIN_15
#define BOARD_HW_INPUT1_PORT                                            GPIOA
#define BOARD_HW_INPUT2_PIN                                             GPIO_PIN_3
#define BOARD_HW_INPUT2_PORT                                            GPIOB
#define BOARD_HW_INPUT3_PIN                                             GPIO_PIN_4
#define BOARD_HW_INPUT3_PORT                                            GPIOB
#define BOARD_HW_INPUT4_PIN                                             GPIO_PIN_5
#define BOARD_HW_INPUT4_PORT                                            GPIOB
#define BOARD_HW_OUTPUT1_PIN                                            GPIO_PIN_6
#define BOARD_HW_OUTPUT1_PORT                                           GPIOB
#define BOARD_HW_OUTPUT2_PIN                                            GPIO_PIN_7
#define BOARD_HW_OUTPUT2_PORT                                           GPIOB

#define EXT_OP_PB12_PORT                                                GPIOB
#define EXT_OP_PB12_PIN                                                 GPIO_PIN_12

#define EXT_OP_PB13_PORT                                                GPIOB
#define EXT_OP_PB13_PIN                                                 GPIO_PIN_13

#define EXT_OP_PB14_PORT                                                GPIOB
#define EXT_OP_PB14_PIN                                                 GPIO_PIN_14

#define EXT_OP_PB15_PORT                                                GPIOB
#define EXT_OP_PB15_PIN                                                 GPIO_PIN_15

#define BOARD_HW_BYPASS_DIDITAL_VOL_GPIO_PORT                           GPIOA
#define BOARD_HW_BYPASS_DIDITAL_VOL_GPIO_PIN                            GPIO_PIN_11

#define BOARD_HW_SW_MIC_PORT                                            GPIOB
#define BOARD_HW_SW_MIC_PIN                                             GPIO_PIN_1

#define FM_USARTPeripheral                                              USART1
#define FM_IRQHandler                                                   USART1_IRQHandler
#define HOST_USARTPeripheral                                            USART0
#define HOST_IRQHandler                                                 USART0_IRQHandler

#define GPIO_TOGGLE(port, pin)                                          (GPIO_TG(port) = (uint32_t)pin)
#define GPIO_SET(gpio_periph, pin)                                      (GPIO_BOP(gpio_periph) = (uint32_t)pin)
#define GPIO_RESET(gpio_periph, pin)                                    (GPIO_BC(gpio_periph) = (uint32_t)pin)
#define GPIO_INPUT_GET(gpio_periph, pin)                                (GPIO_ISTAT(gpio_periph)&(pin))

#define BOARD_HW_ADC_COUNTER                                            4
#define BOARD_HW_NEXTION_BAUD                                           9600
#define BOARD_HW_FM_BAUD                                                115200

//#define AUDIO_CLASS_D                                                   1

typedef enum
{
    BOARD_HW_INPUT0,
    BOARD_HW_INPUT1,
    BOARD_HW_INPUT2,
    BOARD_HW_INPUT3,
    BOARD_HW_INPUT_BUTTON_ON_AIR,
    BOARD_HW_INPUT_MAX
} board_hw_input_num_t;

typedef enum
{
    BOARD_HW_OUTPUT_LED_1_R,
    BOARD_HW_OUTPUT_LED_1_B,
    BOARD_HW_OUTPUT_LED_2_R,
    BOARD_HW_OUTPUT_LED_2_B,
    BOARD_HW_OUTPUT_SW_OUT,
    BOARD_HW_OUTPUT1,
    BOARD_HW_OUTPUT2,
    BOARD_HW_OUTPUT_PA,
    BOARD_HW_MODULE_RESET,
    BOARD_HW_MODULE_PWR_ON,
    BOARD_HW_MODULE_PWR_KEY,
    BOARD_HW_OUTPUT_SW_IN,
    BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION,
    BOARD_HW_OUTPUT_SW_MIC,
    BOARD_HW_OUTPUT_EXT_WDT,
    BOARD_HW_OUTPUT_BY_PASS_DIGITAL_VOL,
    BOARD_HW_OUTPUT_MAX
} board_hw_output_num_t;

typedef union
{
    struct
    {
        uint8_t reserve     : 1;
        uint8_t host_die    : 1;
		uint8_t fault       : 1;
        uint8_t pin_reset   : 1;
        uint8_t power_on    : 1;
        uint8_t software    : 1;
        uint8_t watchdog    : 1;
        uint8_t low_power   : 1;
    } __attribute__((packed)) name;
    uint8_t value;
} __attribute__((packed)) board_hw_reset_reason_t;


typedef union
{
    struct 
    {
        uint32_t reset_counter : 16;
        uint32_t is_vin_lost : 1;
        uint32_t is_uart_communication_error : 1;
        uint32_t is_heartbeat_err : 1;
        uint32_t vin_lost_for_along_time : 1;
        uint32_t reserve : 4;
        uint32_t valid_flag : 8;
    }  __attribute__((packed)) name;
    uint32_t value;
} __attribute__((packed)) board_hw_critical_log_t;


typedef union
{
    struct {
        uint32_t mode_p46 : 1;
        uint32_t mode_p57 : 1;
        uint32_t mode_p60 : 1;
        uint32_t mode_p24 : 1;
        uint32_t mode_p42 : 1;
        uint32_t mode_p27 : 1;
        uint32_t mode_p35 : 1;
        uint32_t mode_p32 : 1;
        uint32_t mode_p36 : 1;
        uint32_t mode_p61 : 1;
        uint32_t mode_p62 : 1;
        uint32_t mode_p47 : 1;
        uint32_t mode_unuse : 4;
        
        uint32_t value_p46 : 1;
        uint32_t value_p57 : 1;
        uint32_t value_p60 : 1;
        uint32_t value_p24 : 1;
        uint32_t value_p42 : 1;
        uint32_t value_p27 : 1;
        uint32_t value_p35 : 1;
        uint32_t value_p32 : 1;
        uint32_t value_p36 : 1;
        uint32_t value_p61 : 1;
        uint32_t value_p62 : 1;
        uint32_t value_p47 : 1;
        uint32_t value_unuse : 4;
    } __attribute__((packed)) name;
    uint32_t val;
} __attribute__((packed)) board_hw_ioex_mode_t;


typedef struct
{
    uint8_t year;       // from 2000
    uint8_t month;
    uint8_t	mday;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint32_t unix_timestamp;
} board_hw_rtc_time_t;

/**
 * @brief               Get input level
 * @param[in]           num Input number
 */
uint8_t board_hw_get_input(board_hw_input_num_t num);

/**
 * @brief               Toggle output
 * @param[in]           num output number
 */
void board_hw_output_toggle(board_hw_output_num_t num);

/**
 * @brief               Toggle output
 * @param[in]           num output number
 */
void board_hw_output_set(board_hw_output_num_t num, bool level);

/**
 * @brief               Initialize all board clock, gpio and peripheral
 */
void board_hw_initialize(void);

/**
 * @brief               Reset system
 */
void board_hw_reset(void);

/**
 * @brief               Check host ping status
 * @retval              TRUE Host detect
 *                      FALSE Host dead
 */
bool board_hw_host_ping_detect(void);

/**
 * @brief               Start ADC on board
 */
void board_adc_init(void);

/**
 * @brief               Get reset reason
 * @retval              reset reason
 */
board_hw_reset_reason_t *board_hardware_get_reset_reason(void);

/**
 * @brief               Get ADC data
 * @retval              ADC pointer
 */
uint16_t *board_hw_get_adc_data(void);

/**
 * @brief               Get ADC conversion status
 * @retval              TRUE ADC convert complete, new data availble
 *                      FLASE no new data
 */
bool board_hw_has_new_adc_data(void);

/**
 * @brief               Feed watchdog
 */
void board_hw_watchdog_feed(void);

/**
 * @brief               Send data to host
 * @retval              Number of byte sent
 */
uint32_t board_hw_host_send(uint8_t* raw, uint32_t length);

/**
 * @brief               Send data fm/lcd port
 * @retval              Number of byte sent
 */
uint32_t board_hw_fm_lcd_send(uint8_t* raw, uint32_t length);

/**
 * @brief               Change FM lcd baudrate
 * @param[in]           baudrate New baudrate
 */
void board_hw_change_lcd_fm_baudrate(uint32_t baudrate);

/**
 * @brief               Set host die reason
 */
void board_hw_set_host_die_reason_to_flash(void);

/**
 * @brief               Get reset counter
 * @retval              Reset counter
 */
uint32_t board_hw_get_reset_counter(void);

/**
 * @brief               Check if device reset from power on
 * @retval              TRUE Power on reset
                        FALSE software reset
 */
bool board_hw_is_power_on_reset(void);

/**
 * @brief               Board hw ping detect callback
 */
void board_hw_ping_detect_cb(void);

/**
 * @brief               Check if SC20 alive
 */
uint32_t board_hw_is_sc20_get_ping_timeout_counter(void);

/**
 * @brief               Enable/disable usart on host
 */
void board_hw_usart_host_control(bool enable);

/**
 * @brief               Check if we need to disable uart communication with host
 */
bool board_hw_is_uart_to_host_disable(void);

/**
 * @brief               Reset digital POT
 */
void board_hw_digital_pot_reset(void);

/**
 * @brief               Increase volume by X unit (not x%)
 * @pamam[in]           count : Number of increase step
 */
void board_hw_digital_pot_increase(uint32_t count);

/**
 * @brief               Decrease volume by X unit (not x%)
 * @pamam[in]           count : Number of increase step
 */
void board_hw_digital_pot_decrease(uint32_t count);

/**
 * @brief               Enable/dsiable EXTI interrupt on PING gpio
 * @pamam[in]           TRUE : enable IRQ
                        FALSE : disable IRQ
 */
void board_hw_ping_irq_control(bool enable);

/**
 * @brief               Set POT value
 * @pamam[in]           value : Pot digital value
 */
bool board_hw_digital_pot_set(uint32_t value);

/**
 * @brief               Set critical error
 * @pamam[in]           err Error value
 */
void board_hw_set_critical_error(board_hw_critical_log_t err);

/**
 * @brief               Get critical error
 * @param[in]           Pointer to error
 * @retval              TRUE If valid critical error in flash
 */
bool board_hw_get_critical_error(board_hw_critical_log_t *err);

/**
 * @brief               Allow adc conversion
 */
void board_hw_allow_adc_conversion(void);

/**
 * @brief               Get current EXT IO value
 */
board_hw_ioex_mode_t board_hw_ioex_get_current_value(void);

/**
 * @brief               Update EXT IO value
 */
void board_hw_ioex_update(board_hw_ioex_mode_t ex_io);

/**
 *  @brief              Clear hardware power reset flag
 */
void board_hw_clear_power_on_reset_flag(void);

/**
 *  @brief              Mute digital POT
 */
void board_hw_digital_pot_mute(void);

/**
 *  @brief              Unmute digital POT
 */
void board_hw_digital_pot_unmute(void);

/**
 *  @brief              Init POT module
 */
void board_hw_digital_pot_init(void);

/**
 * @brief               Get output level of gpio
 * @pamam[in]           num GPIO num
 * @retval              GPIO level
 */
uint8_t board_hw_output_get(board_hw_output_num_t num);

/**
 * @brief		Get current timestamp
 * @retval		Current timestamp
 */
board_hw_rtc_time_t *board_hw_rtc_get(void);

/**
 * @brief               Start RTC
 */
void board_hw_rtc_start(void);


/**
 * @brief               Set current timestamp
 */
void board_hw_rtc_set_timestamp(uint32_t timestamp);

/**
 * @brief               Disble FM USART RX
 */
void board_hw_disable_fm_rx(void);

/**
 * @brief               Disble FM USART RX interrupt
 */
void board_hw_setting_fm_irq_rx(bool enable);

/**
 * @brief               Enable/disable host USART RX
 */
void board_hw_enable_host_rx(bool enable);

//void process_calculate_falling_raising_timer (uint32_t timer_counter_now);

//uint32_t board_hw_get_auto_det_baud_timer_counter(void);

/**
 * @brief               Auto measure uart baudrate
 * @retval              baudrate, 0xFFFFFFFF on failed
 */
uint32_t board_hw_auto_measure_baudrate(void);

/**
 * @brief               Check if auto det baudrate is complete
 * @retval              Auto detect complete status
 */
bool board_hw_is_get_baudrate_complete(void);

/**
 *  @brief      Get systick value
 *  @retval     Current systick in ms
 */
uint32_t sys_get_ms(void);

/**
 *  @brief      Delay
 *  @param[in]  Delay in ms
 */
void sys_delay_ms(uint32_t ms);

void sys_increase_tick(void);

#endif /*BOARD_HW_H */

