#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
    
   
#define SMART_MODULE_PWR_CONTROL_Pin                        GPIO_PIN_13
#define SMART_MODULE_PWR_CONTROL_GPIO_Port                  GPIOC
#define ADC_VIN_Pin                                         GPIO_PIN_0
#define ADC_VIN_GPIO_Port                                   GPIOA
#define ADC_3V8_Pin                                         GPIO_PIN_1
#define ADC_3V8_GPIO_Port                                   GPIOA
#define FM_USART_TX_Pin                                     GPIO_PIN_2
#define FM_USART_TX_GPIO_Port                               GPIOA
#define FM_USART_RX_Pin                                     GPIO_PIN_3
#define FM_USART_RX_GPIO_Port                               GPIOA
#define ADC_5V_Pin                                          GPIO_PIN_4
#define ADC_5V_GPIO_Port                                    GPIOA
#define EXT_WDT_Pin                                         GPIO_PIN_5
#define EXT_WDT_GPIO_Port                                   GPIOA
#define SMART_MODULE_RST_Pin                                GPIO_PIN_6
#define SMART_MODULE_RST_GPIO_Port                          GPIOA
#define TO_SM_POWER_STATE_Pin                               GPIO_PIN_7
#define TO_SM_POWER_STATE_GPIO_Port                         GPIOA
#define HEARTBEAT_Pin                                       GPIO_PIN_2
#define HEARTBEAT_GPIO_Port                                 GPIOB
#define HEARTBEAT_EXTI_IRQn                                 EXTI2_3_IRQn
#define SW_OUT_Pin                                          GPIO_PIN_11
#define SW_OUT_GPIO_Port                                    GPIOB
#define SW_IN_Pin                                           GPIO_PIN_10
#define SW_IN_GPIO_Port                                     GPIOB
#define SMART_MODULE_PWR_KEY_Pin                            GPIO_PIN_8
#define SMART_MODULE_PWR_KEY_GPIO_Port                      GPIOA
#define MIN_USART_TX_Pin                                    GPIO_PIN_9
#define MIN_USART_TX_GPIO_Port                              GPIOA
#define MIN_USART_RX_Pin                                    GPIO_PIN_10
#define MIN_USART_RX_GPIO_Port                              GPIOA
#define DISABLE_HOST_UART_PIN_Pin                           GPIO_PIN_11
#define DISABLE_HOST_UART_PIN_GPIO_Port                     GPIOA
#define PA_CONTROL_Pin                                      GPIO_PIN_12
#define PA_CONTROL_GPIO_Port                                GPIOA
#define INPUT1_Pin                                          GPIO_PIN_15
#define INPUT1_GPIO_Port                                    GPIOA
#define INPUT2_Pin                                          GPIO_PIN_3
#define INPUT2_GPIO_Port                                    GPIOB
#define INPUT3_Pin                                          GPIO_PIN_4
#define INPUT3_GPIO_Port                                    GPIOB
#define INPUT4_Pin                                          GPIO_PIN_5
#define INPUT4_GPIO_Port                                    GPIOB
#define OUTPUT1_Pin                                         GPIO_PIN_6
#define OUTPUT1_GPIO_Port                                   GPIOB
#define OUTPUT2_Pin                                         GPIO_PIN_7
#define OUTPUT2_GPIO_Port                                   GPIOB

#define EXT_OP_PB12_PORT		                            GPIOB
#define EXT_OP_PB12_PIN		                                GPIO_PIN_12

#define EXT_OP_PB13_PORT		                            GPIOB
#define EXT_OP_PB13_PIN			                            GPIO_PIN_13

#define EXT_OP_PB14_PORT		                            GPIOB
#define EXT_OP_PB14_PIN			                            GPIO_PIN_14

#define EXT_OP_PB15_PORT		                            GPIOB
#define EXT_OP_PB15_PIN		                                GPIO_PIN_15

#define EXT_OP_PA11_PORT		                            GPIOA
#define EXT_OP_PA11_PIN		                                GPIO_PIN_11

#define FM_USARTPeripheral              USART1
#define FM_IRQHandler                   USART1_IRQHandler
#define HOST_USARTPeripheral            USART0
#define HOST_IRQHandler                 USART0_IRQHandler

typedef enum
{
    BOARD_HW_INPUT0,
    BOARD_HW_INPUT1,
    BOARD_HW_INPUT2,
    BOARD_HW_INPUT3,
    BOARD_HW_INPUT_BUTTON_ON_AIR
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
    BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION
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
void board_hw_digital_pot_set(uint32_t value);

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

#endif /*BOARD_HW_H */

