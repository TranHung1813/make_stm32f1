#include "board_hw.h"
#include "main.h"
#include "app_debug.h"
#include "gd32f3x0_gpio.h"
#include "gd32f3x0_i2c.h"
#include "flash_if.h"

#define APP_FLASH_HEADER_BUFFER_VALID       0x1234

#define FLASH_LOG_LOCK()
#define FLASH_LOG_UNLOCK()

#define LL_GPIO_IsInputPinSet           GPIO_INPUT_GET
#define LL_GPIO_TogglePin               GPIO_TOGGLE
#define LL_GPIO_SetOutputPin(x, y)      GPIO_SET(x, y)
#define LL_GPIO_ResetOutputPin(x, y)    GPIO_RESET(x, y)

#define DISABLE_HEARTBEAT_GPIO_IRQ()    nvic_irq_disable(EXTI2_3_IRQn)
#define ENABLE_HEARTBEAT_GPIO_IRQ()     nvic_irq_enable(EXTI2_3_IRQn, 2U, 1U)

#define DISABLE_FM_UART_IRQ()           usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_DISABLE)
#define ENABLE_FM_UART_IRQ()            usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_ENABLE)

#define DISABLE_HOST_UART_IRQ()         usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE)
#define ENABLE_HOST_UART_IRQ()          usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_ENABLE)

#define rcu_periph_clock_enable(x)      RCU_REG_VAL(x) |= BIT(RCU_BIT_POS(x))

#define USE_PT2257 0
#define RTC_BKP_VALUE    0x32F0
#define ADC_DMA 0
#define USART_HOST_DMA 0
#define HOST_UART_BAUD  115200U
#define DEBUG_TX_TO_NEXTION_LCD 0

#if USE_PT2257
#include "pt2257.h"
#endif

#define IOEX_MODE_OUTPUT 0
#define IOEX_MODE_INPUT  1

#define USART_FM_LCD_RX_BUFFER_SIZE     256
#define USE_DMA_TX_USART_HOST           0
#define EEPROM_FLAG_HOST_DIE            0x12345678
#define APP_EEPROM_RESET_REASON_ADDRESS 0x0800FC00
//#define APP_EEPROM_RESET_COUNTER_ADDRESS 0x08080008
//#define APP_EEPROM_CRITICAL_ADDRESS     0x0808000C

typedef board_hw_rtc_time_t rtc_time_t;

typedef struct
{
    uint32_t port_addr;
    uint32_t pin_addr;
    uint8_t mode;
    uint8_t value;
} gpio_ex_info_t;

typedef struct
{
    uint8_t index;
    uint32_t port_addr;
    uint32_t pin_addr;
    uint8_t value;
} gpio_output_info_t;

typedef struct
{
    uint8_t index;
    uint32_t port_addr;
    uint32_t pin_addr;
} gpio_input_info_t;


volatile uint32_t m_last_usart_host_transfer_size = 0;
static bool m_usart_host_is_enabled = true;
volatile uint32_t m_uart_host_wait_for_idle = 20;
board_hw_rtc_time_t m_rtc_time;

#if USE_PT2257
#define PT2257_ADDR (0x88)
static int pt2257_i2c_init(void);
static int pt2257_i2c_tx(uint8_t *data, uint32_t size);
static int pt2257_i2c_rx(uint8_t *data, uint32_t size);
static int pt2257_i2c_tx_rx(uint8_t *tx, uint8_t *rx_data, uint32_t size);

static pt2257_drv_t m_pt2257_drv = 
{
    .i2c_init = pt2257_i2c_init,
    .i2c_tx = pt2257_i2c_tx,
    .i2c_rx = pt2257_i2c_rx,
    .i2c_tx_rx = pt2257_i2c_tx_rx
};
#endif
    
static gpio_ex_info_t m_ioex_info[5] = 
{
    {EXT_OP_PB12_PORT, EXT_OP_PB12_PIN, IOEX_MODE_OUTPUT, 0},
    {EXT_OP_PB13_PORT, EXT_OP_PB13_PIN, IOEX_MODE_OUTPUT, 0},
    {EXT_OP_PB14_PORT, EXT_OP_PB14_PIN, IOEX_MODE_OUTPUT, 0},
    {EXT_OP_PB15_PORT, EXT_OP_PB15_PIN, IOEX_MODE_OUTPUT, 0},
    {EXT_OP_PA11_PORT, EXT_OP_PA11_PIN, IOEX_MODE_OUTPUT, 0},
};


static gpio_output_info_t m_gpio_output[BOARD_HW_OUTPUT_MAX] = 
{
    {BOARD_HW_OUTPUT_LED_1_R, 0, 0, 0},
    {BOARD_HW_OUTPUT_LED_1_B, 0, 0, 0},
    {BOARD_HW_OUTPUT_LED_2_R, 0, 0, 0},
    {BOARD_HW_OUTPUT_LED_2_B, 0, 0, 0},
    {BOARD_HW_OUTPUT_SW_OUT, BOARD_HW_SW_OUT_PORT, BOARD_HW_SW_OUT_PIN, 0},
    {BOARD_HW_OUTPUT1, BOARD_HW_OUTPUT1_PORT, BOARD_HW_OUTPUT1_PIN, 0},
    {BOARD_HW_OUTPUT2, BOARD_HW_OUTPUT2_PORT, BOARD_HW_OUTPUT2_PIN, 0},
    {BOARD_HW_OUTPUT_PA, BOARD_HW_PA_CONTROL_PORT, BOARD_HW_PA_CONTROL_PIN, 0},
    {BOARD_HW_MODULE_RESET, BOARD_HW_SMART_MODULE_RST_PORT, BOARD_HW_SMART_MODULE_RST_PIN, 0},
    {BOARD_HW_MODULE_PWR_ON, BOARD_HW_SMART_MODULE_PWR_CONTROL_PORT, BOARD_HW_SMART_MODULE_PWR_CONTROL_PIN, 0},
    {BOARD_HW_MODULE_PWR_KEY, BOARD_HW_SMART_MODULE_PWR_KEY_PORT, BOARD_HW_SMART_MODULE_PWR_KEY_PIN, 0},
    {BOARD_HW_OUTPUT_SW_IN, BOARD_HW_SW_IN_PORT, BOARD_HW_SW_IN_PIN, 0},
    {BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION, BOARD_HW_TO_SM_POWER_STATE_PORT, BOARD_HW_TO_SM_POWER_STATE_PIN, 0},
    {BOARD_HW_OUTPUT_SW_MIC, BOARD_HW_SW_MIC_PORT, BOARD_HW_SW_MIC_PIN, 0},
    {BOARD_HW_OUTPUT_EXT_WDT, BOARD_HW_EXT_WDT_PORT, BOARD_HW_EXT_WDT_PIN, 0}
};

static gpio_input_info_t m_gpio_input[BOARD_HW_INPUT_MAX] = 
{
    {BOARD_HW_INPUT0, BOARD_HW_INPUT1_PORT, BOARD_HW_INPUT1_PIN },
    {BOARD_HW_INPUT1, BOARD_HW_INPUT2_PORT, BOARD_HW_INPUT2_PIN },
    {BOARD_HW_INPUT2, BOARD_HW_INPUT3_PORT, BOARD_HW_INPUT3_PIN },
    {BOARD_HW_INPUT3, BOARD_HW_INPUT4_PORT, BOARD_HW_INPUT4_PIN },
    {BOARD_HW_INPUT_BUTTON_ON_AIR,          0,              0   },
};

static void ioex_gpio_init()
{      
	for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
    {
        if (m_ioex_info[i].mode == IOEX_MODE_OUTPUT)
        {
            gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
            gpio_output_options_set(m_ioex_info[i].port_addr, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, m_ioex_info[i].pin_addr);
            
            if (m_ioex_info[i].value)
            {
                LL_GPIO_SetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
            }
            else
            {
                LL_GPIO_ResetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
            }
        }
        else
        {
            gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_INPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
            m_ioex_info[i].value = LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0;
        }
    }
}

board_hw_ioex_mode_t board_hw_ioex_get_current_value(void)
{
    board_hw_ioex_mode_t info;
    info.val = 0;
    
    for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
    {
        if (m_ioex_info[i].mode == IOEX_MODE_OUTPUT)
        {
            if (m_ioex_info[i].value)
                info.val |= 1 << (i+16);
        }
        else
        {
            info.val |= 1 << i;
            if (LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0)
                info.val |= 1 << (i+16);
        }
    }
    return info;
}

void board_hw_ioex_update(board_hw_ioex_mode_t ex_io)
{
    char *mode[] = {"out", "in"};
	for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
    {
        int temp = (ex_io.val & (1 << i)) ? IOEX_MODE_INPUT : IOEX_MODE_OUTPUT;
        
        if (m_ioex_info[i].mode != temp)    // Mode changed
        {
            DEBUG_WARN("Mode on pin %d changed to %s\r\n", i, mode[temp]);
            m_ioex_info[i].mode = temp;
            
            if (temp == IOEX_MODE_OUTPUT)
            {
                // Reinit gpio
                
                gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
                gpio_output_options_set(m_ioex_info[i].port_addr, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, m_ioex_info[i].pin_addr);
                
                
                // Update value
                m_ioex_info[i].value = (ex_io.val & (1 << (i+16))) ? 1 : 0;
                if (m_ioex_info[i].value)
                {
                    LL_GPIO_SetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
                }
                else
                {
                    LL_GPIO_ResetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
                }
            }
            else
            {
                // Reinit gpio
                gpio_mode_set(m_ioex_info[i].port_addr, GPIO_MODE_INPUT, GPIO_PUPD_NONE, m_ioex_info[i].pin_addr);
                m_ioex_info[i].value = (LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0);
            }
        }
        else        // No changed
        {
            if (temp == IOEX_MODE_OUTPUT)
            {
                temp = (ex_io.val & (1 << (16+i))) ? 1 : 0;
                if (temp != m_ioex_info[i].value)
                {
                    // DEBUG_WARN("GPIO on pin %d changed to %d\r\n", i, temp);
                }
                m_ioex_info[i].value = temp;
                if (m_ioex_info[i].value)
                {
                    LL_GPIO_SetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
                }
                else
                {
                    LL_GPIO_ResetOutputPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
                }
            }
            else
            {
                m_ioex_info[i].value = (LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0);
            }
        }
    }
}


static void board_hw_usart_host_initialize(void)
{
#if USART_HOST_DMA
    rcu_periph_clock_enable(RCU_DMA);
    dma_parameter_struct dma_init_struct;
    
        // init dma
    dma_deinit(DMA_CH2);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)m_dma_host_usart_buffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 10;
    dma_init_struct.periph_addr = (uint32_t) &USART_RDATA(USART0);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH2, &dma_init_struct);

    dma_circulation_disable(DMA_CH2);
    dma_memory_to_memory_disable(DMA_CH2);
    /* enable DMA channel2 transfer complete interrupt */
    dma_interrupt_enable(DMA_CH2, DMA_INT_HTF);
    dma_interrupt_enable(DMA_CH2, DMA_INT_FTF);
    /* enable DMA channel2 */
    dma_channel_enable(DMA_CH2);
#endif    

    nvic_irq_enable(USART0_IRQn, 0, 0);
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);     // ext voltage translator has pullup resistor
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* enable USART clock */
    rcu_usart_clock_config(RCU_USART0SRC_CKSYS); // RCU_USART0SRC_CKAPB2
    rcu_periph_clock_enable(RCU_USART0);
    
    /* USART configure */
    usart_deinit(HOST_USARTPeripheral);
    usart_baudrate_set(HOST_USARTPeripheral, HOST_UART_BAUD);
    // usart_parity_config(HOST_USARTPeripheral, USART_PM_NONE);
    // usart_word_length_set(HOST_USARTPeripheral, USART_WL_8BIT);
    usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_ENABLE);
    usart_transmit_config(HOST_USARTPeripheral, USART_TRANSMIT_ENABLE);
    usart_oversample_config(HOST_USARTPeripheral, USART_OVSMOD_16);
    usart_stop_bit_set(HOST_USARTPeripheral, USART_STB_1BIT);
//    usart_stop_bit_set(HOST_USARTPeripheral, USART_STB_1_5BIT);
    usart_sample_bit_config(HOST_USARTPeripheral, USART_OSB_1BIT);
    
    usart_interrupt_enable(HOST_USARTPeripheral, USART_INT_RBNE);
    usart_interrupt_enable(HOST_USARTPeripheral, USART_INT_ERR);
#if USART_HOST_DMA
    usart_dma_receive_config(HOST_USARTPeripheral, USART_DENR_ENABLE);
    // USART0 idle
   // while(RESET == usart_flag_get(HOST_USARTPeripheral, USART_FLAG_IDLE));
    usart_flag_clear(HOST_USARTPeripheral, USART_FLAG_IDLE);
    usart_interrupt_enable(HOST_USARTPeripheral, USART_INT_IDLE);
#endif
    
    usart_enable(HOST_USARTPeripheral);
    // board_hw_get_host_usart_clk_source();
}


void board_hw_usart_host_control(bool enable)
{	
    
}

uint32_t board_hw_host_send(uint8_t* raw, uint32_t length)
{
    if (!m_usart_host_is_enabled || board_hw_is_uart_to_host_disable())
    {
        return 0;
    }
    for (uint32_t i = 0; i < length; i++)
    {
        usart_data_transmit(HOST_USARTPeripheral, raw[i]);
        while (RESET == usart_flag_get(HOST_USARTPeripheral, USART_FLAG_TC));
    }
    return length;
}


void board_hw_usart_host_idle_countdown(void)
{
    if (m_uart_host_wait_for_idle)
    {
        m_uart_host_wait_for_idle--;
    }
}

bool board_hw_usart_host_is_idle(void)
{
    return m_uart_host_wait_for_idle == 0 ? true : false;
}

extern void board_hw_on_host_rx_callback(uint8_t *data, uint32_t length);

void HOST_IRQHandler(void)
{    
    if (usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_RBNE) != RESET)
    {
#if USART_HOST_DMA == 0
        uint8_t rx = usart_data_receive(HOST_USARTPeripheral) & 0xFF;
//        DEBUG_RAW("%d ", rx);
        board_hw_on_host_rx_callback(&rx, 1);
#endif
    }
    
    uint32_t tmp = USART_CTL0(HOST_USARTPeripheral);
        
//    if(RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_TBE)
//        && (tmp & USART_CTL0_TBEIE))
//    {
//        /* transmit data */
//        uint8_t c;
//        if (driver_uart1_get_new_data_to_send(&c))
//        {
//            usart_data_transmit(HOST_USARTPeripheral, c);
//        }
//        else
//        {
//            usart_interrupt_disable(HOST_USARTPeripheral, USART_INT_TBE);
//        }
//    }  
    
    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_ERR_ORERR))
    {
//        usart_data_receive(HOST_USARTPeripheral);
		usart_interrupt_flag_clear(HOST_USARTPeripheral, USART_INT_FLAG_ERR_ORERR);
    } 	
    
    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_ERR_FERR))
    {
		usart_interrupt_flag_clear(HOST_USARTPeripheral, USART_INT_FLAG_ERR_FERR);
    }    

    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_ERR_NERR))
    {
		usart_interrupt_flag_clear(HOST_USARTPeripheral, USART_INT_FLAG_ERR_NERR);
    }
    
//    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_ERR_))
//    {
//        DEBUG_ERROR("HOST : PE\r\n");
//		usart_interrupt_flag_clear(HOST_USARTPeripheral, USART_FLAG_FERR);
//    }  
    
    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_IDLE)) 
    {
        /* clear IDLE flag */
        usart_interrupt_flag_clear(HOST_USARTPeripheral, USART_INT_FLAG_IDLE);
#if USART_HOST_DMA        
        /* number of data received */
        m_dma_host_usart_idx = 128 - (dma_transfer_number_get(DMA_CH2));
        board_hw_on_host_rx_callback(m_dma_host_usart_buffer, m_dma_host_usart_idx);

        /* disable DMA and reconfigure */
        dma_channel_disable(DMA_CH2);
        dma_transfer_number_config(DMA_CH2, 128);
        dma_channel_enable(DMA_CH2);
#endif
    }
}



void board_hw_initialize(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOF);
    
    rcu_periph_clock_enable(RCU_ADC);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_USART1);
    rcu_periph_clock_enable(RCU_DMA);
    rcu_periph_clock_enable(RCU_I2C0);
    
    // Init watchdog
    rcu_osci_on(RCU_IRC40K);
    
    /* wait till IRC40K is ready */
    rcu_osci_stab_wait(RCU_IRC40K);
    
    fwdgt_config(4095, FWDGT_PSC_DIV256);
   
    FWDGT_CTL = FWDGT_KEY_ENABLE;
    
    /* configure the lvd threshold to 3v */
    pmu_lvd_select(PMU_LVDT_6);
    
    // init gpio
    for (uint8_t i = 0; i < BOARD_HW_OUTPUT_MAX; i++)
    {
        if (m_gpio_output[i].port_addr && m_gpio_output[i].pin_addr)
        {
            gpio_mode_set(m_gpio_output[i].port_addr, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, m_gpio_output[i].pin_addr);
            gpio_output_options_set(m_gpio_output[i].port_addr, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, m_gpio_output[i].pin_addr);
            if (m_gpio_output[i].value)
                LL_GPIO_SetOutputPin(m_gpio_output[i].port_addr, m_gpio_output[i].pin_addr); 
            else
                LL_GPIO_ResetOutputPin(m_gpio_output[i].port_addr, m_gpio_output[i].pin_addr); 
        }
    }
    

    // Heartbeat isr
    rcu_periph_clock_enable(RCU_CFGCMP);
    gpio_mode_set(BOARD_HW_HEARTBEAT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BOARD_HW_HEARTBEAT_PIN);
    
    // Enable EXTI interrupt
    nvic_irq_enable(EXTI2_3_IRQn, 2U, 0U);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN2);
    exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    
    // Input audio detect 1234    
    for (uint32_t i = 0; i < BOARD_HW_INPUT_MAX; i++)
    {
        if (m_gpio_input[i].port_addr && m_gpio_input[i].pin_addr)
        {
            gpio_mode_set(m_gpio_input[i].port_addr, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, m_gpio_input[i].pin_addr);
        }
    }
//    
//    gpio_mode_set(BOARD_HW_INPUT1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BOARD_HW_INPUT1_PIN);
//    gpio_mode_set(BOARD_HW_INPUT2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BOARD_HW_INPUT2_PIN);
//    gpio_mode_set(BOARD_HW_INPUT3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BOARD_HW_INPUT3_PIN);
//    gpio_mode_set(BOARD_HW_INPUT4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BOARD_HW_INPUT4_PIN);
    
    board_hw_usart_host_initialize();
    ioex_gpio_init();

    
    board_hw_disable_fm_rx();
    board_hw_rtc_start();
}


void board_hw_reset(void)
{
    NVIC_SystemReset();
}

void board_hw_watchdog_feed(void)
{
    FWDGT_CTL = FWDGT_KEY_RELOAD;
    LL_GPIO_TogglePin(BOARD_HW_EXT_WDT_PORT, BOARD_HW_EXT_WDT_PIN);
}


uint8_t board_hw_get_input(board_hw_input_num_t num)
{
    uint8_t val = 0;
    
    for (uint32_t i = 0; i < BOARD_HW_INPUT_MAX; i++)
    {
        if (m_gpio_input[i].index == num && m_gpio_input[i].port_addr && m_gpio_input[i].pin_addr)
        {
            val = LL_GPIO_IsInputPinSet(m_gpio_input[i].port_addr, m_gpio_input[i].pin_addr) ? 1 : 0;
            break;
        }
    }
    
//    switch (num)
//    {
//        case BOARD_HW_INPUT0:
//            val = LL_GPIO_IsInputPinSet(BOARD_HW_INPUT1_PORT, BOARD_HW_INPUT1_PIN) ? 0 : 1;
//            break;
//        case BOARD_HW_INPUT1:
//            val = LL_GPIO_IsInputPinSet(BOARD_HW_INPUT2_PORT, BOARD_HW_INPUT2_PIN) ? 0 : 1;
//            break;
//        case BOARD_HW_INPUT2:
//            val = LL_GPIO_IsInputPinSet(BOARD_HW_INPUT3_PORT, BOARD_HW_INPUT3_PIN) ? 0 : 1;
//            break;
//        case BOARD_HW_INPUT3:
//            val = LL_GPIO_IsInputPinSet(BOARD_HW_INPUT4_PORT, BOARD_HW_INPUT4_PIN) ? 0 : 1;
//            break;        
//        case BOARD_HW_INPUT_BUTTON_ON_AIR:
//            val = m_button_on_air_pressed ? 1 : 0;
//            break;
//        default:
//            break;
//    }

    return val;
}


void board_hw_output_toggle(board_hw_output_num_t num)
{
    for (uint32_t i = 0; i < BOARD_HW_OUTPUT_MAX; i++)
    {
        if (num == m_gpio_output[i].index && m_gpio_output[i].port_addr && m_gpio_output[i].pin_addr)
        {
            GPIO_TG(m_gpio_output[i].port_addr) = m_gpio_output[i].pin_addr;
            return;
        }
    }
    
//    switch (num)
//    {
//        case BOARD_HW_OUTPUT_LED_1_R:
//           // LL_GPIO_TogglePin(LED1_R_GPIO_Port, LED1_R_Pin);
//            break;
//        case BOARD_HW_OUTPUT_LED_1_B:
//           // LL_GPIO_TogglePin(LED1_B_GPIO_Port, LED1_B_Pin);
//            break;
//        case BOARD_HW_OUTPUT_LED_2_R:
//            //LL_GPIO_TogglePin(LED2_R_GPIO_Port, LED2_R_Pin);
//            break;
//        case BOARD_HW_OUTPUT_LED_2_B:
//            //LL_GPIO_TogglePin(LED2_B_GPIO_Port, LED2_B_Pin);
//            break;  
//        case BOARD_HW_OUTPUT_SW_OUT:
//            LL_GPIO_TogglePin(BOARD_HW_SW_OUT_PORT, BOARD_HW_SW_OUT_PIN);
//            break;
//        case BOARD_HW_OUTPUT1:
//            LL_GPIO_TogglePin(BOARD_HW_OUTPUT1_PORT, BOARD_HW_OUTPUT1_PIN);
//            break;
//        case BOARD_HW_OUTPUT2:
//            LL_GPIO_TogglePin(BOARD_HW_OUTPUT2_PORT, BOARD_HW_OUTPUT2_PIN);
//            break; 
//        case BOARD_HW_OUTPUT_PA:
//            LL_GPIO_TogglePin(BOARD_HW_PA_CONTROL_PORT, BOARD_HW_PA_CONTROL_PIN);
//            break;        
//        case BOARD_HW_OUTPUT_SW_IN:
//            LL_GPIO_TogglePin(BOARD_HW_SW_IN_PORT, BOARD_HW_SW_IN_PIN);
//            break; 
//        case BOARD_HW_OUTPUT_SW_MIC:
//                LL_GPIO_TogglePin(BOARD_HW_SW_MIC_PORT, BOARD_HW_SW_MIC_PIN);
//                break;
//        default:
//            break;
//    }
}

void board_hw_output_set(board_hw_output_num_t num, bool level)
{
    for (uint32_t i = 0; i < BOARD_HW_OUTPUT_MAX; i++)
    {
        if (num == m_gpio_output[i].index && m_gpio_output[i].port_addr && m_gpio_output[i].pin_addr)
        {
            if (level)
                GPIO_BOP(m_gpio_output[i].port_addr) = m_gpio_output[i].pin_addr;
            else
                GPIO_BC(m_gpio_output[i].port_addr) = m_gpio_output[i].pin_addr;
            return;
        }
    }
//    if (level)
//    {
//        switch (num)
//        {
//            case BOARD_HW_OUTPUT_LED_1_R:
//                //LL_GPIO_SetOutputPin(LED1_R_GPIO_Port, LED1_R_Pin);
//                break;
//            case BOARD_HW_OUTPUT_LED_1_B:
//                //LL_GPIO_SetOutputPin(LED1_B_GPIO_Port, LED1_B_Pin);
//                break;
//            case BOARD_HW_OUTPUT_LED_2_R:
//                //LL_GPIO_SetOutputPin(LED2_R_GPIO_Port, LED2_R_Pin);
//                break;
//            case BOARD_HW_OUTPUT_LED_2_B:
//                //LL_GPIO_SetOutputPin(LED2_B_GPIO_Port, LED2_B_Pin);
//                break;  
//            case BOARD_HW_OUTPUT_SW_OUT:
//                LL_GPIO_SetOutputPin(BOARD_HW_SW_OUT_PORT, BOARD_HW_SW_OUT_PIN);
//                break;
//            case BOARD_HW_OUTPUT1:
//                LL_GPIO_SetOutputPin(BOARD_HW_OUTPUT1_PORT, BOARD_HW_OUTPUT1_PIN);
//                break;
//            case BOARD_HW_OUTPUT2:
//                LL_GPIO_SetOutputPin(BOARD_HW_OUTPUT2_PORT, BOARD_HW_OUTPUT2_PIN);
//                break; 
//            case BOARD_HW_OUTPUT_PA:
//                LL_GPIO_SetOutputPin(BOARD_HW_PA_CONTROL_PORT, BOARD_HW_PA_CONTROL_PIN);
//                break;    
//            case BOARD_HW_MODULE_RESET:
//                LL_GPIO_SetOutputPin(BOARD_HW_SMART_MODULE_RST_PORT, BOARD_HW_SMART_MODULE_RST_PIN);
//                break;      
//            case BOARD_HW_MODULE_PWR_ON:
//                LL_GPIO_SetOutputPin(BOARD_HW_SMART_MODULE_PWR_CONTROL_PORT, BOARD_HW_SMART_MODULE_PWR_CONTROL_PIN);
//                break;      
//            case BOARD_HW_MODULE_PWR_KEY:
//                LL_GPIO_SetOutputPin(BOARD_HW_SMART_MODULE_PWR_KEY_PORT, BOARD_HW_SMART_MODULE_PWR_KEY_PIN);
//                break;     
//            case BOARD_HW_OUTPUT_SW_IN:
//                LL_GPIO_SetOutputPin(BOARD_HW_SW_IN_PORT, BOARD_HW_SW_IN_PIN);
//            break;         
//            case BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION:
//                LL_GPIO_SetOutputPin(BOARD_HW_TO_SM_POWER_STATE_PORT, BOARD_HW_TO_SM_POWER_STATE_PIN);
//                break;
//            case BOARD_HW_OUTPUT_SW_MIC:
//                LL_GPIO_SetOutputPin(BOARD_HW_SW_MIC_PORT, BOARD_HW_SW_MIC_PIN);
//                break;
//            default:
//                break;
//        }
//    }
//    else
//    {
//        switch (num)
//        {
//            case BOARD_HW_OUTPUT_LED_1_R:
//                //LL_GPIO_ResetOutputPin(LED1_R_GPIO_Port, LED1_R_Pin);
//                break;
//            case BOARD_HW_OUTPUT_LED_1_B:
//                //LL_GPIO_ResetOutputPin(LED1_B_GPIO_Port, LED1_B_Pin);
//                break;
//            case BOARD_HW_OUTPUT_LED_2_R:
//                //LL_GPIO_ResetOutputPin(LED2_R_GPIO_Port, LED2_R_Pin);
//                break;
//            case BOARD_HW_OUTPUT_LED_2_B:
//                //LL_GPIO_ResetOutputPin(LED2_B_GPIO_Port, LED2_B_Pin);
//                break;  
//            case BOARD_HW_OUTPUT_SW_OUT:
//                LL_GPIO_ResetOutputPin(BOARD_HW_SW_OUT_PORT, BOARD_HW_SW_OUT_PIN);
//                break;
//            case BOARD_HW_OUTPUT1:
//                LL_GPIO_ResetOutputPin(BOARD_HW_OUTPUT1_PORT, BOARD_HW_OUTPUT1_PIN);
//                break;
//            case BOARD_HW_OUTPUT2:
//                LL_GPIO_ResetOutputPin(BOARD_HW_OUTPUT2_PORT, BOARD_HW_OUTPUT2_PIN);
//                break; 
//            case BOARD_HW_OUTPUT_PA:
//                LL_GPIO_ResetOutputPin(BOARD_HW_PA_CONTROL_PORT, BOARD_HW_PA_CONTROL_PIN);
//                break;        
//            case BOARD_HW_MODULE_RESET:
//                LL_GPIO_ResetOutputPin(BOARD_HW_SMART_MODULE_RST_PORT, BOARD_HW_SMART_MODULE_RST_PIN);
//                break;      
//            case BOARD_HW_MODULE_PWR_ON:
//                LL_GPIO_ResetOutputPin(BOARD_HW_SMART_MODULE_PWR_CONTROL_GPIO_Port, BOARD_HW_SMART_MODULE_PWR_CONTROL_Pin);
//                break;      
//            case BOARD_HW_MODULE_PWR_KEY:
//                LL_GPIO_ResetOutputPin(BOARD_HW_SMART_MODULE_PWR_KEY_PORT, BOARD_HW_SMART_MODULE_PWR_KEY_PIN);
//                break; 
//            case BOARD_HW_OUTPUT_SW_IN:
//                LL_GPIO_ResetOutputPin(BOARD_HW_SW_IN_PORT, BOARD_HW_SW_IN_PIN);
//            break;   
//            case BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION:
//                LL_GPIO_ResetOutputPin(BOARD_HW_TO_SM_POWER_STATE_PORT, BOARD_HW_TO_SM_POWER_STATE_PIN);
//                break;
//            case BOARD_HW_OUTPUT_SW_MIC:
//                LL_GPIO_ResetOutputPin(BOARD_HW_SW_MIC_PORT, BOARD_HW_SW_MIC_PIN);
//                break;
//            default:
//                break;
//        }
//    }
}

uint8_t board_hw_output_get(board_hw_output_num_t num)
{
    uint8_t level = 0;
    for (uint32_t i = 0; i < BOARD_HW_OUTPUT_MAX; i++)
    {
        if (num == m_gpio_output[i].index && m_gpio_output[i].port_addr && m_gpio_output[i].pin_addr)
        {
            level = m_gpio_output[i].value;
            break;
        }
    }
    
//    switch (num)
//    {
//        case BOARD_HW_OUTPUT_LED_1_R:
//        case BOARD_HW_OUTPUT_LED_1_B:
//        case BOARD_HW_OUTPUT_LED_2_R:
//        case BOARD_HW_OUTPUT_LED_2_B:
//            break;  
//        
//        case BOARD_HW_OUTPUT_SW_OUT:
//            level = gpio_output_bit_get(BOARD_HW_SW_OUT_PORT, BOARD_HW_SW_OUT_PIN);
//            break;
//        case BOARD_HW_OUTPUT1:
//            level = gpio_output_bit_get(BOARD_HW_OUTPUT1_PORT, BOARD_HW_OUTPUT1_PIN);
//            break;
//        case BOARD_HW_OUTPUT2:
//            level = gpio_output_bit_get(BOARD_HW_OUTPUT2_PORT, BOARD_HW_OUTPUT2_PIN);
//            break; 
//        case BOARD_HW_OUTPUT_PA:
//            level = gpio_output_bit_get(BOARD_HW_PA_CONTROL_PORT, BOARD_HW_PA_CONTROL_PIN);
//            break;    
//        case BOARD_HW_MODULE_RESET:
//            level = gpio_output_bit_get(BOARD_HW_SMART_MODULE_RST_PORT, BOARD_HW_SMART_MODULE_RST_PIN);
//            break;      
//        case BOARD_HW_MODULE_PWR_ON:
//            level = gpio_output_bit_get(BOARD_HW_SMART_MODULE_PWR_CONTROL_GPIO_Port, BOARD_HW_SMART_MODULE_PWR_CONTROL_Pin);
//            break;      
//        case BOARD_HW_MODULE_PWR_KEY:
//            level = gpio_output_bit_get(BOARD_HW_SMART_MODULE_PWR_KEY_PORT, BOARD_HW_SMART_MODULE_PWR_KEY_PIN);
//            break;     
//        case BOARD_HW_OUTPUT_SW_IN:
//            level = gpio_output_bit_get(BOARD_HW_SW_IN_PORT, BOARD_HW_SW_IN_PIN);
//        break;         
//        case BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION:
//            level = gpio_output_bit_get(BOARD_HW_TO_SM_POWER_STATE_PORT, BOARD_HW_TO_SM_POWER_STATE_PIN);
//            break;
//        case BOARD_HW_OUTPUT_SW_MIC:
//            level = gpio_output_bit_get(BOARD_HW_SW_MIC_PORT, BOARD_HW_SW_MIC_PIN);
//            break;
//        default:
//            level = 0;
//            break;
//    }
    return level ? 1 : 0;
}

static board_hw_reset_reason_t m_reset_reason = 
{
    .value = 0,
};

board_hw_reset_reason_t *board_hardware_get_reset_reason(void)
{
    static bool valid = true;
    if (valid == true)
    {
        valid = false;
        uint32_t tmp = RCU_RSTSCK;

        m_reset_reason.name.power_on = 1;
        
        if (tmp & RCU_RSTSCK_PORRSTF)
        {
            m_reset_reason.name.pin_reset = 1;
            DEBUG_RAW("PIN,");
        }

        if (tmp & RCU_RSTSCK_SWRSTF)
        {
            m_reset_reason.name.software = 1;
            m_reset_reason.name.power_on = 0;
            DEBUG_RAW("SFT,");
        }


        if (tmp & RCU_RSTSCK_FWDGTRSTF)
        {
            m_reset_reason.name.watchdog = 1;
            m_reset_reason.name.power_on = 0;
            DEBUG_RAW("IWD,");
        }

        if (tmp & RCU_RSTSCK_WWDGTRSTF)
        {
            m_reset_reason.name.watchdog = 1;
            m_reset_reason.name.power_on = 0;
            DEBUG_RAW("WWDG,");
        }

        if (tmp & RCU_RSTSCK_LPRSTF)
        {
            m_reset_reason.name.low_power = 1;
            m_reset_reason.name.power_on = 1;
            DEBUG_RAW("LPWR,");
        }		

//        volatile uint32_t data = *((uint32_t*)APP_EEPROM_RESET_REASON_ADDRESS);
//        if (data == EEPROM_FLAG_HOST_DIE)
//        {
//            m_reset_reason.name.host_die = 1;
//            DEBUG_RAW("HOST DIE,");
//        }
//        
//        DEBUG_RAW("\r\n");

        rcu_all_reset_flag_clear();
//        board_hw_clear_host_die_reason_to_flash();
    }
    
    return &m_reset_reason;
}

bool board_hw_is_power_on_reset(void)
{
    return (m_reset_reason.name.power_on) ? true : false;
}

void board_hw_clear_power_on_reset_flag(void)
{
    m_reset_reason.name.power_on = 0;
    m_reset_reason.name.pin_reset = 0;
}

extern  void board_hw_ping_detect_cb(void);

/**
  * @brief This function handles EXTI line 2 and line 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
    if (exti_interrupt_flag_get(EXTI_2) != RESET)
    {
        board_hw_ping_detect_cb();
        static uint32_t heartbeat_counter = 0;
        if (heartbeat_counter++ == 60)
        {
            heartbeat_counter = 0;
            // DEBUG_INF("HB -> OK\r\n");
        }
        exti_interrupt_flag_clear(EXTI_2);
    }
}

bool board_hw_is_uart_to_host_disable(void)
{
//    return LL_GPIO_IsInputPinSet(BOARD_HW_DISABLE_HOST_UART_PORT, BOARD_HW_DISABLE_HOST_UART_PIN) ? true : false;
    return false;
}


void board_hw_digital_pot_init()
{
#if USE_PT2257
    m_pt2257_drv.i2c_init();
#endif
}

void board_hw_digital_pot_reset(void)
{
//    uint8_t reset_cmd = 0xFF;
//    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, 0x29, &reset_cmd, 1, 2);
//    if (status != HAL_OK)
//    {
//        DEBUG_ERROR("Software reset POT failed %d\r\n", status);
//    }
}


void board_hw_digital_pot_set(uint32_t value)
{
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
//        DEBUG_ERROR("Set POT failed %d\r\n", status);
//        return;
//    }
//    
//    vol_cmd[0] = 0x10;
//    status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, vol_cmd, 2, 2);
//    if (status != HAL_OK)
//    {
//        DEBUG_ERROR("Set POT failed %d\r\n", status);
//        return;
//    }
//#else
//    if (value > 79)
//    {
//        value = 79;
//    }
//    value = 79 - value;
////    DEBUG_INF("Set POT %d\r\n", value);
////    if (value)
//        pt2257_set_vol(&m_pt2257_drv, value);
////    else
////        pt2257_mute(&m_pt2257_drv, false);
//#endif
}

void board_hw_digital_pot_mute()
{
//#if USE_PT2257 
//    pt2257_mute(&m_pt2257_drv, false);
//#endif
}

void board_hw_digital_pot_unmute()
{
//#if USE_PT2257
//    pt2257_mute(&m_pt2257_drv, true);
//#endif
}

void board_hw_digital_pot_increase(uint32_t count)
{
//#if USE_PT2257 == 0
//    uint8_t increase_cmd[2] = {0x00, 0x55};
////    for (uint32_t i = 0; i < count; i++)
//    {
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, increase_cmd, 2, 2);
//        if (status != HAL_OK)
//        {
//            DEBUG_ERROR("Vol up failed %d\r\n", status);
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
//            DEBUG_ERROR("Vol up failed %d\r\n", status);
//            return;
//        }
//    }
//#endif

}

void board_hw_digital_pot_decrease(uint32_t count)
{
//#if USE_PT2257 == 0
//    uint8_t decrease_cmd[2] = {0x00, 0x23};
//    decrease_cmd[0] |= (1 << 3);
//    for (uint32_t i = 0; i < count; i++)
//    {
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, decrease_cmd, 2, 2);
//        if (status != HAL_OK)
//        {
//            DEBUG_ERROR("Vol down failed %d at %d\r\n", status, i);
//            return;
//        }
//    }
//#endif
}

#if USE_PT2257

static int pt2257_i2c_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_I2C0);

    /* connect I2C_SCL_GPIO_PIN to I2C_SCL */
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_8);
    /* connect I2C_SDA_GPIO_PIN to I2C_SDA */
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_9);
    /* configure GPIO pins of I2C */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    
   /* configure I2C clock */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* configure I2C address */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, PT2257_ADDR);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    return 0;
}

static int pt2257_i2c_tx(uint8_t *data, uint32_t size)
{
//    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (PT2257_ADDR), data, size, 10);
//    if (status != HAL_OK)
//    {
//        DEBUG_ERROR("I2C TX failed %d\r\n", status);
//        return -1;
//    }
//    return 0;
    
    #define I2C_TIMEOUT ((uint32_t)5)
    
    uint32_t i2c_start_ms = sys_get_ms();
    bool i2c_timeout_err = false;        // 1ms
    bool retval = false;
//    uint32_t tmp;

    
    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
    {
        if (sys_get_ms() - i2c_start_ms >= 3*I2C_TIMEOUT)
        {
            DEBUG_ERROR("I2C busy\r\n");
            i2c_timeout_err = true; 
            break;
        }
    }
    if (i2c_timeout_err)
    {
        goto end;
    }        
    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    
    /* wait until SBSEND bit is set */
    i2c_start_ms = sys_get_ms();
    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
    {
        if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
        {
            i2c_timeout_err = true; 
			break;
        }
    }
    if (i2c_timeout_err)
    {
        goto end;
    } 
    
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, PT2257_ADDR, I2C_TRANSMITTER);
    i2c_start_ms = sys_get_ms();

    
//    /* disable ACK before clearing ADDSEND bit */
//    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
    
    
    /* wait until ADDSEND bit is set */
    i2c_start_ms = sys_get_ms();
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
    {
        if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
        {
            i2c_timeout_err = true; 
            break;
        }
    }
    
    if (i2c_timeout_err)
    {
        goto end;
    }     
     
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    
    for (uint32_t i = 0; i < size; i++)       // 32 = 4 bytes adc * 8zone
    {
        i2c_start_ms = sys_get_ms();
        
        if (!i2c_timeout_err)
        {
            /* wait until the RBNE bit is set */
            i2c_data_transmit(I2C0, data[i]);
            
            while (!i2c_flag_get(I2C0, I2C_FLAG_TBE))
            {
                if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
                {
                    i2c_timeout_err = true; 
                    break;
                }
            }
        }

        if (i2c_timeout_err)
        {
            break;
        }
    }
    
    /* send a stop condition */
    i2c_stop_on_bus(I2C0);
    if (!i2c_timeout_err)
    {
        i2c_start_ms = sys_get_ms();
        while (I2C_CTL0(I2C0)&0x0200)
        {
            if (sys_get_ms() - i2c_start_ms >= I2C_TIMEOUT)
            {
                i2c_timeout_err = true; 
                break;
            }
        }
    }
//    i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
    /* enable acknowledge */
        
    
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    
    if (!i2c_timeout_err)
    {
        retval = true;
    }
end:
    i2c_stop_on_bus(I2C0);
    if (!retval)
    {   
        DEBUG_ERROR("I2C failed\r\n");
        return -1;
    }
    return 0;
}

static int pt2257_i2c_rx(uint8_t *data, uint32_t size)
{
    return 0;
}

static int pt2257_i2c_tx_rx(uint8_t *tx, uint8_t *rx_data, uint32_t size)
{
    return 0;
}
#endif

void board_hw_ping_irq_control(bool enable)
{
    if (enable)
    {
        NVIC_EnableIRQ(EXTI2_3_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(EXTI2_3_IRQn);
    }
}

void board_hw_set_critical_error(board_hw_critical_log_t err)
{    
    DEBUG_WARN("Set critital err 0x%08X\r\n", err.value);
    uint32_t addr = APP_EEPROM_RESET_REASON_ADDRESS;
    int index = -1;
    for (uint32_t i = 0; i < FLASH_IF_PAGE_SIZE/sizeof(board_hw_critical_log_t); i++)
    {
        board_hw_critical_log_t *reason = (board_hw_critical_log_t*)(addr);
        if (reason->name.valid_flag == 0xAB)
        {
            index = i;
            addr += 4;// sizeof(board_hw_critical_log_t);
        }
    }
    
    if (index == -1)
    {
        index = 0;
        goto erase;
    }
    index++;
    if (index < FLASH_IF_SECTOR_SIZE/sizeof(board_hw_critical_log_t))
    {
       goto write;
    }
erase:
    
    board_hw_watchdog_feed();
    
    // Disable some isr
    DISABLE_HEARTBEAT_GPIO_IRQ();
    DISABLE_HOST_UART_IRQ();
    DISABLE_FM_UART_IRQ();
    
    // Flash full

    index = 0;
    // Disable some isr
    DISABLE_HEARTBEAT_GPIO_IRQ();
    DISABLE_HOST_UART_IRQ();
    DISABLE_FM_UART_IRQ();
    
    flash_if_erase(APP_EEPROM_RESET_REASON_ADDRESS, FLASH_IF_PAGE_SIZE);
    

write:
    // Disable some isr
    DISABLE_HEARTBEAT_GPIO_IRQ();
    DISABLE_HOST_UART_IRQ();
    DISABLE_FM_UART_IRQ();
    
    addr = APP_EEPROM_RESET_REASON_ADDRESS + index*sizeof(board_hw_critical_log_t);
    
    err.name.valid_flag = 0xAB;
    flash_if_copy(addr, (uint32_t*)&err.value, 1);
    
    // Enable some isr
    ENABLE_HOST_UART_IRQ();
    ENABLE_FM_UART_IRQ();
    ENABLE_HEARTBEAT_GPIO_IRQ(); 
    
    DEBUG_WARN("Done\r\n");
}


bool board_hw_get_critical_error(board_hw_critical_log_t *err)
{
    err->value = 0;
    
    uint32_t addr = APP_EEPROM_RESET_REASON_ADDRESS;
    int index = -1;
    for (uint32_t i = 0; i < FLASH_IF_SECTOR_SIZE/sizeof(board_hw_critical_log_t); i++)
    {
        board_hw_critical_log_t *reason = (board_hw_critical_log_t*)(addr);
        if (reason->name.valid_flag == 0xAB)
        {
            index = i;
            addr += 4;// sizeof(board_hw_critical_log_t);
            err->value = reason->value;
        }
    }
    
    if (index == -1)
    {
        return false;
    }
    
    if (index >= FLASH_IF_SECTOR_SIZE/sizeof(board_hw_critical_log_t))
    {
        // Flash full
        board_hw_watchdog_feed();
        index = 0;
        // Disable some isr
        DISABLE_HEARTBEAT_GPIO_IRQ();
        DISABLE_HOST_UART_IRQ();
        DISABLE_FM_UART_IRQ();
        
        flash_if_erase(APP_EEPROM_RESET_REASON_ADDRESS, FLASH_IF_SECTOR_SIZE);
        
        // Enable some isr
        ENABLE_HOST_UART_IRQ();
        ENABLE_FM_UART_IRQ();
        ENABLE_HEARTBEAT_GPIO_IRQ();
    }
    
    return true;
}


/*!
    \brief      RTC configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
static volatile uint32_t prescaler_a = 0, prescaler_s = 0;
rtc_timestamp_struct rtc_timestamp;
rtc_parameter_struct rtc_initpara;
static void rtc_pre_config(void)
{
    rcu_osci_on(RCU_IRC40K);
    rcu_osci_stab_wait(RCU_IRC40K);
    rcu_rtc_clock_config(RCU_RTCSRC_IRC40K);

    prescaler_s = 0x18F;
    prescaler_a = 0x63;

    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();
}

/*!
    \brief      use hyperterminal to setup RTC time and alarm
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void rtc_setup(void)
{
    /* setup RTC time value */
//    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    rtc_initpara.rtc_factor_asyn = prescaler_a;
    rtc_initpara.rtc_factor_syn = prescaler_s;
    rtc_initpara.rtc_year = 0x16;
    rtc_initpara.rtc_day_of_week = RTC_SATURDAY;
    rtc_initpara.rtc_month = RTC_APR;
    rtc_initpara.rtc_date = 0x30;
    rtc_initpara.rtc_display_format = RTC_24HOUR;
    rtc_initpara.rtc_am_pm = RTC_AM;

    /* current time input */
//    tmp_hh = 0;
//	tmp_mm = 0;
//	tmp_ss = 0;
	
    /* RTC current time configuration */
    if(ERROR == rtc_init(&rtc_initpara))
	{    
    }
	else
	{
        RTC_BKP0 = RTC_BKP_VALUE;
    }   
}


void board_hw_rtc_start(void)
{
    /* Enable access to RTC registers in Backup domain */
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
  
    rtc_pre_config();

    /* check if RTC has aready been configured */
    if (RTC_BKP_VALUE != RTC_BKP0)
	{    
        rtc_setup(); 
    }
	else
	{
        /* detect the reset source */
        if (RESET != rcu_flag_get(RCU_FLAG_PORRST))
		{
            // DEBUG_INFO("power on reset occurred....\r\n");
        }
		else if (RESET != rcu_flag_get(RCU_FLAG_EPRST))
		{
            // DEBUG_INFO("external reset occurred....\r\n");
        }
    } 
    
//    rcu_all_reset_flag_clear();
	
//	exti_flag_clear(EXTI_19); 
//    exti_init(EXTI_19,EXTI_INTERRUPT,EXTI_TRIG_RISING);
//    nvic_irq_enable(RTC_IRQn,0);
    
    /* RTC timestamp configuration */
//    rtc_timestamp_enable(RTC_TIMESTAMP_FALLING_EDGE);
//    rtc_interrupt_enable(RTC_INT_TIMESTAMP); 
//    rtc_flag_clear(RTC_STAT_TSF|RTC_STAT_TSOVRF);    
    
}

/*******************************************************************************
**
    Function Name  : counter_to_struct
    * Description    : populates time-struct based on counter-value
    * input          : - counter-value (unit seconds, 0 -> 1.1.2000 00:00:00),
    *                  - Pointer to time-struct
    * Output         : time-struct gets populated, DST not taken into account here
    * Return         : none
    * Based on code from Peter Dannegger found in the mikrocontroller.net forum.
*/
static const uint8_t days_in_month[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static void counter_to_struct(uint32_t sec, rtc_time_t *t, uint8_t cal_year)
{
    #define FIRSTYEAR 2000 // start year
    #define FIRSTDAY 6     // 0 = Sunday
	
    uint16_t day;
    uint8_t year;
    uint16_t day_of_year;
    uint8_t leap400;
    uint8_t month;

    t->sec = sec % 60;
    sec /= 60;
    t->min = sec % 60;
    sec /= 60;
    t->hour = sec % 24;

    if (cal_year == 0)
        return;

    day = (uint16_t)(sec / 24);

    year = FIRSTYEAR % 100;                    // 0..99
    leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3); // 4, 3, 2, 1

    for (;;)
    {
        day_of_year = 365;
        if ((year & 3) == 0)
        {
            day_of_year = 366; // leap year
            if (year == 0 || year == 100 || year == 200)
            { // 100 year exception
                if (--leap400)
                { // 400 year exception
                    day_of_year = 365;
                }
            }
        }
        if (day < day_of_year)
        {
            break;
        }
        day -= day_of_year;
        year++; // 00..136 / 99..235
    }
    t->year = year + FIRSTYEAR / 100 * 100 - 2000; // + century

    if (day_of_year & 1 && day > 58)
    {          // no leap year and after 28.2.
        day++; // skip 29.2.
    }

    for (month = 1; day >= days_in_month[month - 1]; month++)
    {
        day -= days_in_month[month - 1];
    }

    t->month = month; // 1..12
    t->day = day + 1; // 1..31
}

static uint8_t bcd_2_decimal(uint8_t hex)
{
    int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
    return dec;
}     

// Function to convert Decimal to BCD
static uint8_t decimal_2_bcd(uint8_t num)
{
    return (num/10) * 16 + (num % 10);
}


void board_hw_rtc_set_timestamp(uint32_t timestamp)
{
//	DEBUG_INFO("Update timestamp %u\r\n", timestamp);
    timestamp += 25200; // gmt adjust +7
    counter_to_struct(timestamp, &m_rtc_time, 1);
//	/* setup RTC time value */

    rtc_initpara.rtc_factor_asyn = prescaler_a;
    rtc_initpara.rtc_factor_syn = prescaler_s;
    rtc_initpara.rtc_year = decimal_2_bcd(m_rtc_time.year);
    rtc_initpara.rtc_day_of_week = RTC_SATURDAY;
    rtc_initpara.rtc_month = decimal_2_bcd(m_rtc_time.month);
    rtc_initpara.rtc_date = decimal_2_bcd(m_rtc_time.day);
    rtc_initpara.rtc_display_format = RTC_24HOUR;
	rtc_initpara.rtc_hour = decimal_2_bcd(m_rtc_time.hour);
	rtc_initpara.rtc_minute = decimal_2_bcd(m_rtc_time.min);
	rtc_initpara.rtc_second = decimal_2_bcd(m_rtc_time.sec);
    rtc_initpara.rtc_am_pm = RTC_AM;

    /* RTC current time configuration */
    if(ERROR == rtc_init(&rtc_initpara))
	{    
  //      DEBUG_INFO("\r\n** RTC time configuration failed! **\r\n");
    }
	else
	{
//        DEBUG_INFO("\r\n** RTC time configuration success! **\r\n");
//        rtc_show_time();
        RTC_BKP0 = RTC_BKP_VALUE;
    }   
}

board_hw_rtc_time_t *board_hw_rtc_get(void)
{
    rtc_current_time_get(&rtc_initpara);

    m_rtc_time.year = bcd_2_decimal(rtc_initpara.rtc_year) + 1970;
    if (m_rtc_time.year  > 2049)
    {
        m_rtc_time.year = 2020;
    }
    m_rtc_time.month = bcd_2_decimal(rtc_initpara.rtc_month);
    m_rtc_time.day = bcd_2_decimal(rtc_initpara.rtc_date);
    m_rtc_time.hour = bcd_2_decimal(rtc_initpara.rtc_hour);
    m_rtc_time.min = bcd_2_decimal(rtc_initpara.rtc_minute);
    m_rtc_time.sec = bcd_2_decimal(rtc_initpara.rtc_second);

    return &m_rtc_time;
}

void board_hw_setting_fm_irq_rx(bool enable)
{
    if (enable)
    {
        nvic_irq_enable(USART1_IRQn, 1, 0);
    }
    else
    {
        nvic_irq_disable(USART1_IRQn);
    }
}

void board_hw_disable_fm_rx(void)
{
    usart_interrupt_disable(FM_USARTPeripheral, USART_INT_RBNE);
    usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_DISABLE);
}

void board_hw_enable_host_rx(bool enable)
{ 
    if (enable)
    {
        usart_interrupt_enable(HOST_USARTPeripheral, USART_INT_RBNE);
        usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_ENABLE);
    }
    else
    {
        usart_interrupt_disable(HOST_USARTPeripheral, USART_INT_RBNE);
        usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE);
    }
}
