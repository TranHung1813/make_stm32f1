#include "board_hw.h"
#include "main.h"
#include "app_debug.h"
#include "gd32f3x0_gpio.h"
#include "gd32f3x0_i2c.h"
#include "flash_if.h"

#define APP_FLASH_HEADER_BUFFER_VALID       0x1234

#define FLASH_LOG_LOCK()
#define FLASH_LOG_UNLOCK()

#define LL_GPIO_IsInputPinSet           gpio_input_bit_get
#define LL_GPIO_TogglePin               gpio_bit_toggle
#define LL_GPIO_SetOutputPin(x, y)      gpio_bit_write(x, y, (bit_status)1)
#define LL_GPIO_ResetOutputPin(x, y)    gpio_bit_write(x, y, (bit_status)0)



#define DISABLE_HEARTBEAT_GPIO_IRQ()    nvic_irq_disable(EXTI2_3_IRQn)
#define ENABLE_HEARTBEAT_GPIO_IRQ()     nvic_irq_enable(EXTI2_3_IRQn, 2U, 0U)

#define DISABLE_FM_UART_IRQ()           usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_DISABLE)
#define ENABLE_FM_UART_IRQ()        usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_ENABLE)

#define DISABLE_HOST_UART_IRQ()           usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE)
#define ENABLE_HOST_UART_IRQ()        usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_ENABLE)

#define USE_PT2257 1

#if USE_PT2257
#include "pt2257.h"
#endif

#define IOEX_MODE_OUTPUT 0
#define IOEX_MODE_INPUT  1


typedef struct
{
    uint32_t port_addr;
    uint32_t pin_addr;
    uint8_t mode;
    uint8_t value;
} gpio_ex_info_t;


#define USART_FM_LCD_RX_BUFFER_SIZE     256
#define USART_HOST_RX_BUFFER_SIZE       128
#define USE_DMA_TX_USART_HOST           0
#define EEPROM_FLAG_HOST_DIE            0x12345678
#define APP_EEPROM_RESET_REASON_ADDRESS 0x0800FC00
////#define APP_EEPROM_RESET_COUNTER_ADDRESS 0x08080008
////#define APP_EEPROM_CRITICAL_ADDRESS     0x0808000C

//volatile uint32_t m_last_usart_host_transfer_size = 0;
//static bool m_usart_host_is_enabled = true;
//volatile uint32_t m_uart_host_wait_for_idle = 20;
//static volatile uint32_t m_old_usart_host_dma_rx_pos;
//static volatile uint32_t m_old_usart_fm_lcd_dma_rx_pos = 0;

//// Button on air debounce
//static volatile uint32_t m_debounce_state = 0;
//static volatile uint8_t m_debounce_index = 0;
//static volatile bool m_button_on_air_pressed = false;

//static uint32_t m_last_fm_lcd_baudrate = 115200;
//static uint16_t m_adc_buffer[3];

//#if USE_PT2257
//#define PT2257_ADDR 0x88
//static int pt2257_i2c_init(void);
//static int pt2257_i2c_tx(uint8_t *data, uint32_t size);
//static int pt2257_i2c_rx(uint8_t *data, uint32_t size);
//static int pt2257_i2c_tx_rx(uint8_t *tx, uint8_t *rx_data, uint32_t size);

//static pt2257_drv_t m_pt2257_drv = 
//{
//    .i2c_init = pt2257_i2c_init,
//    .i2c_tx = pt2257_i2c_tx,
//    .i2c_rx = pt2257_i2c_rx,
//    .i2c_tx_rx = pt2257_i2c_tx_rx
//};
//#endif
    
//static gpio_ex_info_t m_ioex_info[5] = 
//{
//    {EXT_OP_PB12_PORT, EXT_OP_PB12_PIN, IOEX_MODE_OUTPUT, 0},
//    {EXT_OP_PB13_PORT, EXT_OP_PB13_PIN, IOEX_MODE_OUTPUT, 0},
//    {EXT_OP_PB14_PORT, EXT_OP_PB14_PIN, IOEX_MODE_OUTPUT, 0},
//    {EXT_OP_PB15_PORT, EXT_OP_PB15_PIN, IOEX_MODE_OUTPUT, 0},
//    {EXT_OP_PA11_PORT, EXT_OP_PA11_PIN, IOEX_MODE_OUTPUT, 0},
//};

//static void ioex_gpio_init()
//{      
//	for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
//    {
//        if (m_ioex_info[i].mode == IOEX_MODE_OUTPUT)
//        {
//            gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
//            gpio_output_options_set(m_ioex_info[i].port_addr, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, m_ioex_info[i].pin_addr);
//            
//            if (m_ioex_info[i].value)
//            {
//                LL_GPIO_SetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//            }
//            else
//            {
//                LL_GPIO_ResetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//            }
//        }
//        else
//        {
//            gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_INPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
//            m_ioex_info[i].value = LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0;
//        }
//    }
//}

//board_hw_ioex_mode_t board_hw_ioex_get_current_value(void)
//{
//    board_hw_ioex_mode_t info;
//    info.val = 0;
//    
//    for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
//    {
//        if (m_ioex_info[i].mode == IOEX_MODE_OUTPUT)
//        {
//            if (m_ioex_info[i].value)
//                info.val |= 1 << (i+16);
//        }
//        else
//        {
//            info.val |= 1 << i;
//            if (LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0)
//                info.val |= 1 << (i+16);
//        }
//    }
//    return info;
//}

//void board_hw_ioex_update(board_hw_ioex_mode_t ex_io)
//{
//    char *mode[] = {"out", "in"};
//	for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
//    {
//        int temp = (ex_io.val & (1 << i)) ? IOEX_MODE_INPUT : IOEX_MODE_OUTPUT;
//        
//        if (m_ioex_info[i].mode != temp)    // Mode changed
//        {
//            APP_DEBUG_WARN("Mode on pin %d changed to %s\r\n", i, mode[temp]);
//            m_ioex_info[i].mode = temp;
//            
//            if (temp == IOEX_MODE_OUTPUT)
//            {
//                // Reinit gpio
//                
//                gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
//                gpio_output_options_set(m_ioex_info[i].port_addr, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, m_ioex_info[i].pin_addr);
//                
//                
//                // Update value
//                m_ioex_info[i].value = (ex_io.val & (1 << (i+16))) ? 1 : 0;
//                if (m_ioex_info[i].value)
//                {
//                    LL_GPIO_SetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//                else
//                {
//                    LL_GPIO_ResetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//            }
//            else
//            {
//                // Reinit gpio
//                gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_INPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
//                m_ioex_info[i].value = (LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0);
//            }
//        }
//        else        // No changed
//        {
//            if (temp == IOEX_MODE_OUTPUT)
//            {
//                temp = (ex_io.val & (1 << (16+i))) ? 1 : 0;
//                if (temp != m_ioex_info[i].value)
//                {
//                    APP_DEBUG_WARN("GPIO on pin %d changed to %d\r\n", i, temp);
//                }
//                m_ioex_info[i].value = temp;
//                if (m_ioex_info[i].value)
//                {
//                    LL_GPIO_SetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//                else
//                {
//                    LL_GPIO_ResetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//            }
//            else
//            {
//                m_ioex_info[i].value = (LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0);
//            }
//        }
//    }
//}


//static void board_hw_usart_host_initialize(void)
//{
//    nvic_irq_enable(USART0_IRQn, 0, 0);
//    /* enable COM GPIO clock */
//    rcu_periph_clock_enable(RCU_GPIOA);

//    /* enable USART clock */
//    rcu_periph_clock_enable(RCU_USART0);

//    /* connect port to USARTx_Tx */
//    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

//    /* connect port to USARTx_Rx */
//    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

//    /* configure USART Tx as alternate function push-pull */
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

//    /* configure USART Rx as alternate function push-pull */
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

//    /* USART configure */
//    usart_deinit(USART0);
//    usart_baudrate_set(USART0, 115200U);
//    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
//    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

//    usart_enable(USART0);
//    usart_interrupt_enable(USART0, USART_INT_RBNE);
//}

//static void board_hw_usart_fm_lcd_initialize(void)
//{
//    // USART1
//    nvic_irq_enable(USART1_IRQn, 0, 0);
//    /* enable COM GPIO clock */
//    rcu_periph_clock_enable(RCU_GPIOA);

//    /* connect port to USARTx_Tx */
//    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);

//    /* connect port to USARTx_Rx */
//    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);

//    /* configure USART Tx as alternate function push-pull */
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);

//    /* configure USART Rx as alternate function push-pull */
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_3);

//    /* enable USART clock */
//    rcu_periph_clock_enable(RCU_USART1);

//    /* USART configure */
//    usart_deinit(FM_USARTPeripheral);
//    usart_word_length_set(FM_USARTPeripheral, USART_WL_8BIT);
//    usart_stop_bit_set(FM_USARTPeripheral, USART_STB_1BIT);
//    usart_parity_config(FM_USARTPeripheral, USART_PM_NONE);
//    usart_baudrate_set(FM_USARTPeripheral, m_last_fm_lcd_baudrate);
//    usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_ENABLE);
//    usart_transmit_config(FM_USARTPeripheral, USART_TRANSMIT_ENABLE);

//    usart_enable(FM_USARTPeripheral);
//    usart_interrupt_enable(FM_USARTPeripheral, USART_INT_RBNE);
////    usart_interrupt_enable(FM_USARTPeripheral, USART_INT_TBE);

//}


//void board_hw_usart_host_control(bool enable)
//{	
//    
//}

//uint32_t board_hw_host_send(uint8_t* raw, uint32_t length)
//{
//    if (!m_usart_host_is_enabled || board_hw_is_uart_to_host_disable())
//    {
//        return 0;
//    }
//    for (uint32_t i = 0; i < length; i++)
//    {
//        usart_data_transmit(HOST_USARTPeripheral, raw[i]);
//        while (RESET == usart_flag_get(HOST_USARTPeripheral, USART_FLAG_TBE));
//    }
//    return length;
//}


//void board_hw_usart_host_idle_countdown(void)
//{
//    if (m_uart_host_wait_for_idle)
//    {
//        m_uart_host_wait_for_idle--;
//    }
//}

//bool board_hw_usart_host_is_idle(void)
//{
//    return m_uart_host_wait_for_idle == 0 ? true : false;
//}

//extern void board_hw_on_host_rx_callback(uint8_t *data, uint32_t length);

///**
//  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 25.
//  */
//void HOST_IRQHandler(void)
//{
//    if (usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_RBNE) != RESET)
//    {
//        uint8_t rx = usart_data_receive(HOST_USARTPeripheral);
//        board_hw_on_host_rx_callback(&rx, 1);
//    }
//    
//    uint32_t tmp = USART_CTL0(HOST_USARTPeripheral);
//        
////    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)
////        && (tmp & USART_CTL0_TBEIE))
////    {
////        /* transmit data */
////        uint8_t c;
////        if (driver_uart1_get_new_data_to_send(&c))
////        {
////            usart_data_transmit(USART0, c);
////        }
////        else
////        {
////            usart_interrupt_disable(USART0, USART_INT_TBE);
////        }
////    }  
//    
//    if (RESET != usart_flag_get(HOST_USARTPeripheral, USART_FLAG_ORERR))
//    {
////        usart_data_receive(USART0);
//		usart_flag_clear(HOST_USARTPeripheral, USART_FLAG_ORERR);
//    } 	
//    
//    if (RESET != usart_flag_get(HOST_USARTPeripheral, USART_FLAG_FERR))
//    {
//        usart_data_receive(HOST_USARTPeripheral);
//		usart_flag_clear(HOST_USARTPeripheral, USART_FLAG_FERR);
//    }    

//    if (RESET != usart_flag_get(HOST_USARTPeripheral, USART_FLAG_NERR))
//    {
//        usart_data_receive(HOST_USARTPeripheral);
//		usart_flag_clear(HOST_USARTPeripheral, USART_FLAG_NERR);
//    }
//    
//    if (RESET != usart_flag_get(HOST_USARTPeripheral, USART_FLAG_PERR))
//    {
//		usart_flag_clear(HOST_USARTPeripheral, USART_FLAG_FERR);
//    }  
//}


//static void board_hw_adc_initialize(void)
//{
//    /* enable ADC clock */
//    rcu_periph_clock_enable(RCU_ADC);
//    /* enable DMA clock */
//    rcu_periph_clock_enable(RCU_DMA);
//    /* config ADC clock */
//    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
//   
//    
//    // DMA
//    /* ADC_DMA_channel configuration */
//    dma_parameter_struct dma_data_parameter;
//    
//    /* ADC DMA_channel configuration */
//    dma_deinit(DMA_CH0);
//    
//    /* initialize DMA single data mode */
//    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA);
//    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
//    dma_data_parameter.memory_addr  = (uint32_t)(&m_adc_buffer);
//    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
//    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
//    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
//    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
//    dma_data_parameter.number       = sizeof(m_adc_buffer)/sizeof(m_adc_buffer[0]);
//    dma_data_parameter.priority     = DMA_PRIORITY_LOW;
//    dma_init(DMA_CH0, &dma_data_parameter);

//    dma_circulation_enable(DMA_CH0);
//  
//    /* enable DMA channel */
//    dma_channel_enable(DMA_CH0);
//    
//    /* ADC contineous function enable */
//    adc_special_function_config(ADC_CONTINUOUS_MODE, ENABLE);
//    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
//    /* ADC trigger config */
//    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE); 
//    /* ADC data alignment config */
//    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
//    /* ADC channel length config */
//    adc_channel_length_config(ADC_REGULAR_CHANNEL, sizeof(m_adc_buffer)/sizeof(m_adc_buffer[0]));
// 
//    /* ADC regular channel config */
//    adc_regular_channel_config(0U, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
//    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

//    adc_regular_channel_config(1U, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);
//    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
//    
//    adc_regular_channel_config(2U, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5);
//    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
//    
//    /* enable ADC interface */
//    adc_enable();
//    sys_delay_ms(2U);
//    /* ADC calibration and reset calibration */
//    adc_calibration_enable();

//    /* ADC DMA function enable */
//    adc_dma_mode_enable();
//    /* ADC software trigger enable */
//    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
//}

void board_hw_initialize(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOF);
    
//    rcu_periph_clock_enable(RCU_ADC);
//    rcu_periph_clock_enable(RCU_USART0);
//    rcu_periph_clock_enable(RCU_USART1);
//    rcu_periph_clock_enable(RCU_DMA);
//    rcu_periph_clock_enable(RCU_I2C0);
    
    // Init watchdog
    rcu_osci_on(RCU_IRC40K);
    
    /* wait till IRC40K is ready */
    rcu_osci_stab_wait(RCU_IRC40K);
    
    fwdgt_config(4095, FWDGT_PSC_DIV256);
   
    FWDGT_CTL = FWDGT_KEY_ENABLE;
    
    /* configure the lvd threshold to 3v */
    pmu_lvd_select(PMU_LVDT_6);
    
    // init gpio

    // Smart module power control
    gpio_mode_set(SMART_MODULE_PWR_CONTROL_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SMART_MODULE_PWR_CONTROL_Pin);
    gpio_output_options_set(SMART_MODULE_PWR_CONTROL_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SMART_MODULE_PWR_CONTROL_Pin);
    LL_GPIO_SetOutputPin(SMART_MODULE_PWR_CONTROL_GPIO_Port, SMART_MODULE_PWR_CONTROL_Pin);   
    
    // Smart module power key
    gpio_mode_set(SMART_MODULE_PWR_KEY_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SMART_MODULE_PWR_KEY_Pin);
    gpio_output_options_set(SMART_MODULE_PWR_KEY_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SMART_MODULE_PWR_KEY_Pin);
    LL_GPIO_ResetOutputPin(SMART_MODULE_PWR_KEY_GPIO_Port, SMART_MODULE_PWR_KEY_Pin);  
    
    // EXT watchdog
    gpio_mode_set(EXT_WDT_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EXT_WDT_Pin);
    gpio_output_options_set(EXT_WDT_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, EXT_WDT_Pin);
    LL_GPIO_SetOutputPin(EXT_WDT_GPIO_Port, EXT_WDT_Pin);   
    
    // Smart module reset
    gpio_mode_set(SMART_MODULE_RST_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SMART_MODULE_RST_Pin);
    gpio_output_options_set(SMART_MODULE_RST_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SMART_MODULE_RST_Pin);
    LL_GPIO_ResetOutputPin(SMART_MODULE_RST_GPIO_Port, SMART_MODULE_RST_Pin);   
    
    // Smart module power lost pin
    gpio_mode_set(TO_SM_POWER_STATE_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TO_SM_POWER_STATE_Pin);
    gpio_output_options_set(TO_SM_POWER_STATE_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TO_SM_POWER_STATE_Pin);
    LL_GPIO_ResetOutputPin(TO_SM_POWER_STATE_GPIO_Port, TO_SM_POWER_STATE_Pin);   
    

    // Heartbeat isr
    rcu_periph_clock_enable(RCU_CFGCMP);
    gpio_mode_set(HEARTBEAT_GPIO_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, HEARTBEAT_Pin);
    
    // Enable EXTI interrupt
//    nvic_irq_enable(EXTI2_3_IRQn, 2U, 0U);
//    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN2);
//    exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
//    exti_interrupt_flag_clear(EXTI_2);
    
    
    // Audio switch in and out
    gpio_mode_set(SW_OUT_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_OUT_Pin);
    gpio_output_options_set(SW_OUT_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SW_OUT_Pin);
    LL_GPIO_ResetOutputPin(SW_OUT_GPIO_Port, SW_OUT_Pin);   
    
    gpio_mode_set(SW_IN_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_IN_Pin);
    gpio_output_options_set(SW_IN_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SW_IN_Pin);
    LL_GPIO_ResetOutputPin(SW_IN_GPIO_Port, SW_IN_Pin);   
    
    // Disable uart min protocol with host, currently unuse, use as gpio    
    gpio_mode_set(DISABLE_HOST_UART_PIN_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DISABLE_HOST_UART_PIN_Pin);
    gpio_output_options_set(DISABLE_HOST_UART_PIN_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, DISABLE_HOST_UART_PIN_Pin);
    LL_GPIO_ResetOutputPin(DISABLE_HOST_UART_PIN_GPIO_Port, DISABLE_HOST_UART_PIN_Pin);   
    
    // PA control pin    
    gpio_mode_set(PA_CONTROL_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PA_CONTROL_Pin);
    gpio_output_options_set(PA_CONTROL_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PA_CONTROL_Pin);
    LL_GPIO_ResetOutputPin(PA_CONTROL_GPIO_Port, PA_CONTROL_Pin);   
    
    // Input audio detect 1234    
    gpio_mode_set(INPUT1_GPIO_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INPUT1_Pin);
    gpio_mode_set(INPUT2_GPIO_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INPUT2_Pin);
    gpio_mode_set(INPUT3_GPIO_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INPUT3_Pin);
    gpio_mode_set(INPUT4_GPIO_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INPUT4_Pin);
    
    // Output opto 12
    gpio_mode_set(OUTPUT1_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUTPUT1_Pin);
    gpio_output_options_set(OUTPUT1_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, OUTPUT1_Pin);
    LL_GPIO_ResetOutputPin(OUTPUT1_GPIO_Port, OUTPUT1_Pin);   
    
    gpio_mode_set(OUTPUT2_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUTPUT2_Pin);
    gpio_output_options_set(OUTPUT2_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, OUTPUT2_Pin);
    LL_GPIO_ResetOutputPin(OUTPUT2_GPIO_Port, OUTPUT2_Pin);   
    
//    board_hw_usart_host_initialize();
//    board_hw_usart_fm_lcd_initialize();
//    board_hw_adc_initialize();
//    ioex_gpio_init();
//    
//#if USE_PT2257
//    pt2257_init(&m_pt2257_drv);
//#endif

}


void board_hw_reset(void)
{
    NVIC_SystemReset();
}

void board_hw_watchdog_feed(void)
{
    FWDGT_CTL = FWDGT_KEY_RELOAD;
    LL_GPIO_TogglePin(EXT_WDT_GPIO_Port, EXT_WDT_Pin);
}


//uint8_t board_hw_get_input(board_hw_input_num_t num)
//{
//    uint8_t val = 0;
//    switch (num)
//    {
//        case BOARD_HW_INPUT0:
//            val = LL_GPIO_IsInputPinSet(INPUT1_GPIO_Port, INPUT1_Pin) ? 0 : 1;
//            break;
//        case BOARD_HW_INPUT1:
//            val = LL_GPIO_IsInputPinSet(INPUT2_GPIO_Port, INPUT2_Pin) ? 0 : 1;
//            break;
//        case BOARD_HW_INPUT2:
//            val = LL_GPIO_IsInputPinSet(INPUT3_GPIO_Port, INPUT3_Pin) ? 0 : 1;
//            break;
//        case BOARD_HW_INPUT3:
//            val = LL_GPIO_IsInputPinSet(INPUT4_GPIO_Port, INPUT4_Pin) ? 0 : 1;
//            break;        
//        case BOARD_HW_INPUT_BUTTON_ON_AIR:
//            val = m_button_on_air_pressed ? 1 : 0;
//            break;
//        default:
//            break;
//    }
//    return val;
//}


void board_hw_output_toggle(board_hw_output_num_t num)
{
    switch (num)
    {
        case BOARD_HW_OUTPUT_LED_1_R:
           // LL_GPIO_TogglePin(LED1_R_GPIO_Port, LED1_R_Pin);
            break;
        case BOARD_HW_OUTPUT_LED_1_B:
           // LL_GPIO_TogglePin(LED1_B_GPIO_Port, LED1_B_Pin);
            break;
        case BOARD_HW_OUTPUT_LED_2_R:
            //LL_GPIO_TogglePin(LED2_R_GPIO_Port, LED2_R_Pin);
            break;
        case BOARD_HW_OUTPUT_LED_2_B:
            //LL_GPIO_TogglePin(LED2_B_GPIO_Port, LED2_B_Pin);
            break;  
        case BOARD_HW_OUTPUT_SW_OUT:
            LL_GPIO_TogglePin(SW_OUT_GPIO_Port, SW_OUT_Pin);
            break;
        case BOARD_HW_OUTPUT1:
            LL_GPIO_TogglePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin);
            break;
        case BOARD_HW_OUTPUT2:
            LL_GPIO_TogglePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin);
            break; 
        case BOARD_HW_OUTPUT_PA:
            LL_GPIO_TogglePin(PA_CONTROL_GPIO_Port, PA_CONTROL_Pin);
            break;        
        case BOARD_HW_OUTPUT_SW_IN:
            LL_GPIO_TogglePin(SW_IN_GPIO_Port, SW_IN_Pin);
            break; 
        default:
            break;
    }
}

void board_hw_output_set(board_hw_output_num_t num, bool level)
{
    if (level)
    {
        switch (num)
        {
            case BOARD_HW_OUTPUT_LED_1_R:
                //LL_GPIO_SetOutputPin(LED1_R_GPIO_Port, LED1_R_Pin);
                break;
            case BOARD_HW_OUTPUT_LED_1_B:
                //LL_GPIO_SetOutputPin(LED1_B_GPIO_Port, LED1_B_Pin);
                break;
            case BOARD_HW_OUTPUT_LED_2_R:
                //LL_GPIO_SetOutputPin(LED2_R_GPIO_Port, LED2_R_Pin);
                break;
            case BOARD_HW_OUTPUT_LED_2_B:
                //LL_GPIO_SetOutputPin(LED2_B_GPIO_Port, LED2_B_Pin);
                break;  
            case BOARD_HW_OUTPUT_SW_OUT:
                LL_GPIO_SetOutputPin(SW_OUT_GPIO_Port, SW_OUT_Pin);
                break;
            case BOARD_HW_OUTPUT1:
                LL_GPIO_SetOutputPin(OUTPUT1_GPIO_Port, OUTPUT1_Pin);
                break;
            case BOARD_HW_OUTPUT2:
                LL_GPIO_SetOutputPin(OUTPUT2_GPIO_Port, OUTPUT2_Pin);
                break; 
            case BOARD_HW_OUTPUT_PA:
                LL_GPIO_SetOutputPin(PA_CONTROL_GPIO_Port, PA_CONTROL_Pin);
                break;    
            case BOARD_HW_MODULE_RESET:
                LL_GPIO_SetOutputPin(SMART_MODULE_RST_GPIO_Port, SMART_MODULE_RST_Pin);
                break;      
            case BOARD_HW_MODULE_PWR_ON:
                LL_GPIO_SetOutputPin(SMART_MODULE_PWR_CONTROL_GPIO_Port, SMART_MODULE_PWR_CONTROL_Pin);
                break;      
            case BOARD_HW_MODULE_PWR_KEY:
                LL_GPIO_SetOutputPin(SMART_MODULE_PWR_KEY_GPIO_Port, SMART_MODULE_PWR_KEY_Pin);
                break;     
            case BOARD_HW_OUTPUT_SW_IN:
                LL_GPIO_SetOutputPin(SW_IN_GPIO_Port, SW_IN_Pin);
            break;         
            case BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION:
                LL_GPIO_SetOutputPin(TO_SM_POWER_STATE_GPIO_Port, TO_SM_POWER_STATE_Pin);
                break;
            
            default:
                break;
        }
    }
    else
    {
        switch (num)
        {
            case BOARD_HW_OUTPUT_LED_1_R:
                //LL_GPIO_ResetOutputPin(LED1_R_GPIO_Port, LED1_R_Pin);
                break;
            case BOARD_HW_OUTPUT_LED_1_B:
                //LL_GPIO_ResetOutputPin(LED1_B_GPIO_Port, LED1_B_Pin);
                break;
            case BOARD_HW_OUTPUT_LED_2_R:
                //LL_GPIO_ResetOutputPin(LED2_R_GPIO_Port, LED2_R_Pin);
                break;
            case BOARD_HW_OUTPUT_LED_2_B:
                //LL_GPIO_ResetOutputPin(LED2_B_GPIO_Port, LED2_B_Pin);
                break;  
            case BOARD_HW_OUTPUT_SW_OUT:
                LL_GPIO_ResetOutputPin(SW_OUT_GPIO_Port, SW_OUT_Pin);
                break;
            case BOARD_HW_OUTPUT1:
                LL_GPIO_ResetOutputPin(OUTPUT1_GPIO_Port, OUTPUT1_Pin);
                break;
            case BOARD_HW_OUTPUT2:
                LL_GPIO_ResetOutputPin(OUTPUT2_GPIO_Port, OUTPUT2_Pin);
                break; 
            case BOARD_HW_OUTPUT_PA:
                LL_GPIO_ResetOutputPin(PA_CONTROL_GPIO_Port, PA_CONTROL_Pin);
                break;        
            case BOARD_HW_MODULE_RESET:
                LL_GPIO_ResetOutputPin(SMART_MODULE_RST_GPIO_Port, SMART_MODULE_RST_Pin);
                break;      
            case BOARD_HW_MODULE_PWR_ON:
                LL_GPIO_ResetOutputPin(SMART_MODULE_PWR_CONTROL_GPIO_Port, SMART_MODULE_PWR_CONTROL_Pin);
                break;      
            case BOARD_HW_MODULE_PWR_KEY:
                LL_GPIO_ResetOutputPin(SMART_MODULE_PWR_KEY_GPIO_Port, SMART_MODULE_PWR_KEY_Pin);
                break; 
            case BOARD_HW_OUTPUT_SW_IN:
                LL_GPIO_ResetOutputPin(SW_IN_GPIO_Port, SW_IN_Pin);
            break;   
            case BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION:
                LL_GPIO_ResetOutputPin(TO_SM_POWER_STATE_GPIO_Port, TO_SM_POWER_STATE_Pin);
                break;
            
            default:
                break;
        }
    }
}

//static board_hw_reset_reason_t m_reset_reason = 
//{
//    .value = 0,
//};

//board_hw_reset_reason_t *board_hardware_get_reset_reason(void)
//{
//    static bool valid = true;
//    if (valid == true)
//    {
//        valid = false;
//        uint32_t tmp = RCU_RSTSCK;
//        
//        m_reset_reason.name.power_on = 1;

//        if (tmp & RCU_RSTSCK_PORRSTF)
//        {
//            m_reset_reason.name.pin_reset = 1;
//            APP_DEBUG_RAW("PIN,");
//        }

//        if (tmp & RCU_RSTSCK_SWRSTF)
//        {
//            m_reset_reason.name.software = 1;
//            m_reset_reason.name.power_on = 0;
//            APP_DEBUG_RAW("SFT,");
//        }


//        if (tmp & RCU_RSTSCK_FWDGTRSTF)
//        {
//            m_reset_reason.name.watchdog = 1;
//            m_reset_reason.name.power_on = 0;
//            APP_DEBUG_RAW("IWD,");
//        }

//        if (tmp & RCU_RSTSCK_WWDGTRSTF)
//        {
//            m_reset_reason.name.watchdog = 1;
//            m_reset_reason.name.power_on = 0;
//            APP_DEBUG_RAW("WWDG,");
//        }

//        if (tmp & RCU_RSTSCK_LPRSTF)
//        {
//            m_reset_reason.name.low_power = 1;
//            m_reset_reason.name.power_on = 1;
//            APP_DEBUG_RAW("LPWR,");
//        }		

////        volatile uint32_t data = *((uint32_t*)APP_EEPROM_RESET_REASON_ADDRESS);
////        if (data == EEPROM_FLAG_HOST_DIE)
////        {
////            m_reset_reason.name.host_die = 1;
////            APP_DEBUG_RAW("HOST DIE,");
////        }
////        
////        APP_DEBUG_RAW("\r\n");

//        rcu_all_reset_flag_clear();
////        board_hw_clear_host_die_reason_to_flash();
//    }
//    
//    return &m_reset_reason;
//}

//bool board_hw_is_power_on_reset(void)
//{
//    return (m_reset_reason.name.power_on) ? true : false;
//}

//void board_hw_clear_power_on_reset_flag(void)
//{
//    m_reset_reason.name.power_on = 0;
//    m_reset_reason.name.pin_reset = 0;
//}

//void usart_fm_lcd_tx_complete_callback(bool status)
//{
//#if USE_DMA_FM_LCD_TX
//    m_usart_fm_lcd_tx_run = false;
//#endif
//}



//void board_hw_change_lcd_fm_baudrate(uint32_t baudrate)
//{
//    if (m_last_fm_lcd_baudrate == baudrate)
//    {
//        return;
//    }
//    m_last_fm_lcd_baudrate = baudrate;
//    
//    usart_interrupt_disable(FM_USARTPeripheral, USART_INT_RBNE);
//    
//    usart_deinit(FM_USARTPeripheral);
//    usart_word_length_set(FM_USARTPeripheral, USART_WL_8BIT);
//    usart_stop_bit_set(FM_USARTPeripheral, USART_STB_1BIT);
//    usart_parity_config(FM_USARTPeripheral, USART_PM_NONE);
//    usart_baudrate_set(FM_USARTPeripheral, m_last_fm_lcd_baudrate);
//    usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_ENABLE);
//    usart_transmit_config(FM_USARTPeripheral, USART_TRANSMIT_ENABLE);

//    usart_enable(FM_USARTPeripheral);
//    usart_interrupt_enable(FM_USARTPeripheral, USART_INT_RBNE);
//    
//}

//extern void board_hw_on_fm_lcd_rx_callback(uint8_t *data, uint32_t length);
//void FM_IRQHandler(void)
//{
//    if (usart_interrupt_flag_get(FM_USARTPeripheral, USART_INT_FLAG_RBNE) != RESET)
//    {
//        uint8_t rx = usart_data_receive(FM_USARTPeripheral);
//        board_hw_on_fm_lcd_rx_callback(&rx, 1);
//    }
//    
//    uint32_t tmp = USART_CTL0(FM_USARTPeripheral);
//        
////    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE)
////        && (tmp & USART_CTL0_TBEIE))
////    {
////        /* transmit data */
////        uint8_t c;
////        if (driver_uart1_get_new_data_to_send(&c))
////        {
////            usart_data_transmit(USART1, c);
////        }
////        else
////        {
////            usart_interrupt_disable(USART1, USART_INT_TBE);
////        }
////    }  
//    
//    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_ORERR))
//    {
////        usart_data_receive(USART1);
//		usart_flag_clear(FM_USARTPeripheral, USART_FLAG_ORERR);
//    } 	
//    
//    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_FERR))
//    {
//        usart_data_receive(FM_USARTPeripheral);
//		usart_flag_clear(FM_USARTPeripheral, USART_FLAG_FERR);
//    }    

//    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_NERR))
//    {
//        usart_data_receive(FM_USARTPeripheral);
//		usart_flag_clear(FM_USARTPeripheral, USART_FLAG_NERR);
//    }
//    
//    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_PERR))
//    {
//		usart_flag_clear(FM_USARTPeripheral, USART_FLAG_FERR);
//    }  
//}

//uint32_t board_hw_fm_lcd_send(uint8_t *raw, uint32_t length)
//{
//    for (uint32_t i = 0; i < length; i++)
//    {
//        usart_data_transmit(FM_USARTPeripheral, raw[i]);
//        while (RESET == usart_flag_get(FM_USARTPeripheral, USART_FLAG_TBE));
//    }
//    return length;
//}


//uint16_t *board_hw_get_adc_data(void)
//{
//    return &m_adc_buffer[0];
//}

bool board_hw_has_new_adc_data(void)
{
    return true;
}

void board_hw_allow_adc_conversion(void)
{
    
}

//extern  void board_hw_ping_detect_cb(void);

///**
//  * @brief This function handles EXTI line 2 and line 3 interrupts.
//  */
//void EXTI2_3_IRQHandler(void)
//{
//    if (exti_interrupt_flag_get(EXTI_2) != RESET)
//    {
//        board_hw_ping_detect_cb();
//        exti_interrupt_flag_clear(EXTI_2);
//        static uint32_t heartbeat_counter = 0;
//        if (heartbeat_counter++ == 60)
//        {
//            heartbeat_counter = 0;
//            APP_DEBUG_INFO("Heartbeat -> OK\r\n");
//        }
//    }
//}

bool board_hw_is_uart_to_host_disable(void)
{
//    return LL_GPIO_IsInputPinSet(DISABLE_HOST_UART_PIN_GPIO_Port, DISABLE_HOST_UART_PIN_Pin) ? true : false;
    return false;
}


//void board_hw_digital_pot_init()
//{
//    m_pt2257_drv.i2c_init();
//}

//void board_hw_digital_pot_reset(void)
//{
////    uint8_t reset_cmd = 0xFF;
////    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, 0x29, &reset_cmd, 1, 2);
////    if (status != HAL_OK)
////    {
////        APP_DEBUG_ERROR("Software reset POT failed %d\r\n", status);
////    }
//}


//void board_hw_digital_pot_set(uint32_t value)
//{
//#if USE_PT2257 == 0
//    
//    if (value > 100)
//    {
//        value = 100;
//    }
//    
//    value *= 255;
//    value /= 100;
////    value = m_look_up_vol[m_look_up_vol];
//    
//    uint8_t vol_cmd[2] = {0x00, value};
//    HAL_StatusTypeDef status;
//    
//    status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, vol_cmd, 2, 2);
//    if (status != HAL_OK)
//    {
//        APP_DEBUG_ERROR("Set POT failed %d\r\n", status);
//        return;
//    }
//    
//    vol_cmd[0] = 0x10;
//    status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, vol_cmd, 2, 2);
//    if (status != HAL_OK)
//    {
//        APP_DEBUG_ERROR("Set POT failed %d\r\n", status);
//        return;
//    }
//#else
//    if (value > 100)
//    {
//        value = 100;
//    }
//    value = 100 - value;
//    
//    if (value)
//        pt2257_set_vol(&m_pt2257_drv, value);
//    else
//        pt2257_mute(&m_pt2257_drv, false);
//#endif
//}

//void board_hw_digital_pot_mute()
//{
//#if USE_PT2257 
//    pt2257_mute(&m_pt2257_drv, false);
//#endif
//}

//void board_hw_digital_pot_unmute()
//{
//#if USE_PT2257
//    pt2257_mute(&m_pt2257_drv, true);
//#endif
//}

//void board_hw_digital_pot_increase(uint32_t count)
//{
//#if USE_PT2257 == 0
//    uint8_t increase_cmd[2] = {0x00, 0x55};
////    for (uint32_t i = 0; i < count; i++)
//    {
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, increase_cmd, 2, 2);
//        if (status != HAL_OK)
//        {
//            APP_DEBUG_ERROR("Vol up failed %d\r\n", status);
//            return;
//        }
//    }
//    
//    increase_cmd[0] = 0x01;
////    for (uint32_t i = 0; i < count; i++)
//    {
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, increase_cmd, 2, 2);
//        if (status != HAL_OK)
//        {
//            APP_DEBUG_ERROR("Vol up failed %d\r\n", status);
//            return;
//        }
//    }
//#endif

//}

//void board_hw_digital_pot_decrease(uint32_t count)
//{
//#if USE_PT2257 == 0
//    uint8_t decrease_cmd[2] = {0x00, 0x23};
//    decrease_cmd[0] |= (1 << 3);
//    for (uint32_t i = 0; i < count; i++)
//    {
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, decrease_cmd, 2, 2);
//        if (status != HAL_OK)
//        {
//            APP_DEBUG_ERROR("Vol down failed %d at %d\r\n", status, i);
//            return;
//        }
//    }
//#endif
//}

//#if USE_PT2257

//static int pt2257_i2c_init(void)
//{
//    /* enable GPIO clock */
//    rcu_periph_clock_enable(RCU_I2C0);

//    /* connect I2C_SCL_GPIO_PIN to I2C_SCL */
//    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_8);
//    /* connect I2C_SDA_GPIO_PIN to I2C_SDA */
//    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_9);
//    /* configure GPIO pins of I2C */
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
//    
//   /* configure I2C clock */
//    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
//    /* configure I2C address */
//    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x01);
//    /* enable I2C0 */
//    i2c_enable(I2C0);
//    /* enable acknowledge */
//    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
//    return 0;
//}

//static int pt2257_i2c_tx(uint8_t *data, uint32_t size)
//{
////    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (PT2257_ADDR), data, size, 10);
////    if (status != HAL_OK)
////    {
////        APP_DEBUG_ERROR("I2C TX failed %d\r\n", status);
////        return -1;
////    }
////    return 0;
//    
//    #define I2C_TIMEOUT ((uint32_t)5)
//    
//    uint32_t i2c_start_ms = sys_get_ms();
//    bool i2c_timeout_err = false;        // 1ms
//    bool retval = false;
////    uint32_t tmp;

//    
//    /* wait until I2C bus is idle */
//    while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
//    {
//        if (sys_get_ms() - i2c_start_ms >= 3*I2C_TIMEOUT)
//        {
//            APP_DEBUG_ERROR("I2C busy\r\n");
//            i2c_timeout_err = true; 
//            break;
//        }
//    }
//    if (i2c_timeout_err)
//    {
//        goto end;
//    }        
//    
//    /* send a start condition to I2C bus */
//    i2c_start_on_bus(I2C0);
//    
//    /* wait until SBSEND bit is set */
//    i2c_start_ms = sys_get_ms();
//    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
//    {
//        if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
//        {
//            i2c_timeout_err = true; 
//			break;
//        }
//    }
//    if (i2c_timeout_err)
//    {
//        goto end;
//    } 
//    
//    /* send slave address to I2C bus */
//    i2c_master_addressing(I2C0, PT2257_ADDR, I2C_RECEIVER);
//    i2c_start_ms = sys_get_ms();

//    
////    /* disable ACK before clearing ADDSEND bit */
////    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
//    
//    
//    /* wait until ADDSEND bit is set */
//    i2c_start_ms = sys_get_ms();
//    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
//    {
//        if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
//        {
//            i2c_timeout_err = true; 
//            break;
//        }
//    }
//    
//    if (i2c_timeout_err)
//    {
//        goto end;
//    }     
//     
//    /* clear ADDSEND bit */
//    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
//    
//    for (uint32_t i = 0; i < size; i++)       // 32 = 4 bytes adc * 8zone
//    {
//        i2c_start_ms = sys_get_ms();
//        
//        if (!i2c_timeout_err)
//        {
//            /* wait until the RBNE bit is set */
//            i2c_data_transmit(I2C0, data[i]);
//            
//            while(!i2c_flag_get(I2C0, I2C_FLAG_TBE))
//            {
//                if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
//                {
//                    i2c_timeout_err = true; 
//                    break;
//                }
//            }
//        }

//        if (i2c_timeout_err)
//        {
//            break;
//        }
//    }
//    
//    /* send a stop condition */
//    i2c_stop_on_bus(I2C0);
//    if (!i2c_timeout_err)
//    {
//        i2c_start_ms = sys_get_ms();
//        while (I2C_CTL0(I2C0)&0x0200)
//        {
//            if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
//            {
//                i2c_timeout_err = true; 
//                break;
//            }
//        }
//    }
////    i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
//    /* enable acknowledge */
//        
//    
//    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
//    
//    if (!i2c_timeout_err)
//    {
//        retval = true;
//    }
//end:
//    i2c_stop_on_bus(I2C0);
//    if (!retval)
//    {   
//        APP_DEBUG_ERROR("I2C failed\r\n");
//        return -1;
//    }
//    return 0;
//}

//static int pt2257_i2c_rx(uint8_t *data, uint32_t size)
//{
//    return 0;
//}

//static int pt2257_i2c_tx_rx(uint8_t *tx, uint8_t *rx_data, uint32_t size)
//{
//    return 0;
//}
//#endif

//void board_hw_ping_irq_control(bool enable)
//{
//    if (enable)
//    {
//        NVIC_EnableIRQ(EXTI2_3_IRQn);
//    }
//    else
//    {
//        NVIC_DisableIRQ(EXTI2_3_IRQn);
//    }
//}

//void board_hw_set_critical_error(board_hw_critical_log_t err)
//{    
//    APP_DEBUG_INFO("Set critital error 0x%08X to flash\r\n", err.value);
//    uint32_t addr = APP_EEPROM_RESET_REASON_ADDRESS;
//    int index = -1;
//    for (uint32_t i = 0; i < FLASH_IF_PAGE_SIZE/sizeof(board_hw_critical_log_t); i++)
//    {
//        board_hw_critical_log_t *reason = (board_hw_critical_log_t*)(addr);
//        if (reason->name.valid_flag == 0xAB)
//        {
//            index = i;
//            addr += 4;// sizeof(board_hw_critical_log_t);
//        }
//    }
//    
//    if (index == -1)
//    {
//        index = 0;
//        goto erase;
//    }
//    index++;
//    if (index < FLASH_IF_SECTOR_SIZE/sizeof(board_hw_critical_log_t))
//    {
//       goto write;
//    }
//erase:

//    // Flash full
//    board_hw_watchdog_feed();
//    index = 0;
//    // Disable some isr
//    DISABLE_HEARTBEAT_GPIO_IRQ();
//    DISABLE_HOST_UART_IRQ();
//    DISABLE_FM_UART_IRQ();
//    
//    flash_if_erase(APP_EEPROM_RESET_REASON_ADDRESS, FLASH_IF_PAGE_SIZE);
//    

//write:
//    // Disable some isr
//    DISABLE_HEARTBEAT_GPIO_IRQ();
//    DISABLE_HOST_UART_IRQ();
//    DISABLE_FM_UART_IRQ();
//    DISABLE_HEARTBEAT_GPIO_IRQ();
//    
//    addr = APP_EEPROM_RESET_REASON_ADDRESS + index*sizeof(board_hw_critical_log_t);
//    
//    err.name.valid_flag = 0xAB;
//    flash_if_copy(addr, (uint32_t*)&err.value, 1);
//    
//    // Enable some isr
//    ENABLE_HOST_UART_IRQ();
//    ENABLE_FM_UART_IRQ();
//    ENABLE_HEARTBEAT_GPIO_IRQ(); 
//}


//bool board_hw_get_critical_error(board_hw_critical_log_t *err)
//{
//    err->value = 0;
//    
//    uint32_t addr = APP_EEPROM_RESET_REASON_ADDRESS;
//    int index = -1;
//    for (uint32_t i = 0; i < FLASH_IF_SECTOR_SIZE/sizeof(board_hw_critical_log_t); i++)
//    {
//        board_hw_critical_log_t *reason = (board_hw_critical_log_t*)(addr);
//        if (reason->name.valid_flag == 0xAB)
//        {
//            index = i;
//            addr += 4;// sizeof(board_hw_critical_log_t);
//            err->value = reason->value;
//        }
//    }
//    
//    if (index == -1)
//    {
//        return false;
//    }
//    
//    if (index >= FLASH_IF_SECTOR_SIZE/sizeof(board_hw_critical_log_t))
//    {
//        // Flash full
//        board_hw_watchdog_feed();
//        index = 0;
//        // Disable some isr
//        DISABLE_HEARTBEAT_GPIO_IRQ();
//        DISABLE_HOST_UART_IRQ();
//        DISABLE_FM_UART_IRQ();
//        
//        flash_if_erase(APP_EEPROM_RESET_REASON_ADDRESS, FLASH_IF_SECTOR_SIZE);
//        
//        // Enable some isr
//        ENABLE_HOST_UART_IRQ();
//        ENABLE_FM_UART_IRQ();
//        ENABLE_HEARTBEAT_GPIO_IRQ();
//    }
//    
//    return true;
//}



