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

#define USE_PT2257                      1
#define RTC_BKP_VALUE                   0x32F0
#define ADC_DMA                         1
#define USART_HOST_DMA                  0
#define HOST_UART_BAUD                  115200U
#define DEBUG_TX_TO_NEXTION_LCD         0

#if USE_PT2257
#include "pt2257.h"
#endif

#define IOEX_MODE_OUTPUT                0
#define IOEX_MODE_INPUT                 1

#define USART_FM_LCD_RX_BUFFER_SIZE     256
#define USE_DMA_TX_USART_HOST           0
#define EEPROM_FLAG_HOST_DIE            0x12345678
#define APP_EEPROM_RESET_REASON_ADDRESS 0x0800FC00
#define DMA_USART_RX_BUFFER_SIZE        512
//#define APP_EEPROM_RESET_COUNTER_ADDRESS 0x08080008
//#define APP_EEPROM_CRITICAL_ADDRESS     0x0808000C
static const uint8_t m_volume_level_to_db[] = 
{ 0, 15, 20, 25, 25, 30, 33, 34, 35, 38,
  40, 40, 40, 40, 40, 41, 41, 41, 42, 42, //
  43, 43, 43, 43, 43, 44, 44, 44, 44, 44, //
  44, 44, 44, 44, 46, 46, 48, 48, 48, 48, //
  50, 50, 52, 52, 53, 53, 54, 55, 56, 58, //
  60, 62, 62, 63, 63, 63, 64, 64, 64, 64,
  65, 65, 65, 65, 65, 66, 66, 67, 67, 67,
  70, 70, 70, 70, 70, 71, 71, 72, 72, 73,
  75, 75, 75, 75, 75, 75, 75, 75, 75, 75,
  78, 78, 78, 78, 78, 78, 78, 78, 79, 79, 79
};

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

static uint32_t m_last_fm_lcd_baudrate = 115200;
static uint16_t m_adc_buffer[BOARD_HW_ADC_COUNTER];
board_hw_rtc_time_t m_rtc_time;

/************************************/
//          AUTO DETECT BAUDRATE    //
/***********************************/
volatile uint32_t auto_det_baud_timer_count = 0;
static uint32_t m_auto_det_baud_falling_time = 0;
static uint32_t m_auto_det_baud_raising_time = 0;
static uint32_t m_auto_det_baud_pre_last_time_pulse = 0;
static uint32_t m_auto_det_baud_last_time_pulse = 0;
//static uint32_t m_auto_det_baud_this_time_pulse = 0;
uint8_t m_auto_det_baud_is_measure_cplt = 0;
static void board_hw_usart_fm_lcd_initialize(uint32_t baudrate);

void process_calculate_falling_raising_timer(uint32_t timer_counter_now)
{
//    DEBUG_INFO ("timer_counter_now :%d, last: %d , pre-last:%d\r\n", timer_counter_now, m_auto_det_baud_last_time_pulse, m_auto_det_baud_pre_last_time_pulse);
    m_auto_det_baud_falling_time = timer_counter_now - m_auto_det_baud_last_time_pulse;
    m_auto_det_baud_raising_time = m_auto_det_baud_last_time_pulse - m_auto_det_baud_pre_last_time_pulse;
    m_auto_det_baud_pre_last_time_pulse = m_auto_det_baud_last_time_pulse;
    m_auto_det_baud_last_time_pulse = timer_counter_now;
//    DEBUG_INFO ("m_auto_det_baud_falling_time = %d, rtaising: %d\r\n", m_auto_det_baud_falling_time, m_auto_det_baud_raising_time);
}

uint32_t board_hw_get_auto_det_baud_timer_counter(void)
{
    return auto_det_baud_timer_count;
}

void turn_off_timer_and_interrupt (void)
{
    timer_deinit(TIMER2);
    exti_interrupt_disable(EXTI_3);
}

void timer_auto_det_baudrate_config(void)
{
    /* ----------------------------------------------------------------------------
    TIMER2 Configuration:
    TIMER2CLK = SystemCoreClock/8400(GD32F330)or 10800(GD32F350) = 10KHz.
    the period is 1s(10000/10000 = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;

    /* enable the peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER2);

    /* deinit a TIMER */
    timer_deinit(TIMER2);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
#ifdef GD32F330
    timer_initpara.prescaler         = 71;
#endif /* GD32F330 */
#ifdef GD32F350
    timer_initpara.prescaler         = 10799;
#endif /* GD32F350 */
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 10000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);

    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    /* enable the TIMER interrupt */
//    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    /* enable a TIMER */
    timer_enable(TIMER2);
}

void uart_pin_interrupt_config(void)
{
    /* enable the key wakeup clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_CFGCMP);

    /* configure button pin as input */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_3);

    /* enable and set key wakeup EXTI interrupt to the higher priority */
    nvic_irq_enable(EXTI2_3_IRQn, 3U, 0U);

    /* connect key wakeup EXTI line to key GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN3);

    /* configure key wakeup EXTI line */
    exti_init(EXTI_3, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_3);
}


bool board_hw_is_get_baudrate_complete(void)
{
    return m_auto_det_baud_is_measure_cplt;
}

uint32_t board_hw_auto_measure_baudrate()
{
    if (m_auto_det_baud_is_measure_cplt)
    {
        return 0xFFFFFFFF;
    }
//    DEBUG_INFO ("m_auto_det_baud_falling_time = %d, rtaising: %d\r\n", m_auto_det_baud_falling_time, m_auto_det_baud_raising_time);
//    uint32_t baudrate_temp = 0;
    if (m_auto_det_baud_falling_time == m_auto_det_baud_raising_time)
    {
        if (m_auto_det_baud_falling_time == 9)//8.86 us for 1 bit
        {
            turn_off_timer_and_interrupt();
            m_auto_det_baud_is_measure_cplt = 1;
            DEBUG_WARN("Auto det baudrate = 115200\r\n");
            board_hw_usart_fm_lcd_initialize(115200);
            return 115200;
        }
        else if (m_auto_det_baud_falling_time == 104) //104.2 us for 1 bit
        {
            turn_off_timer_and_interrupt();
            m_auto_det_baud_is_measure_cplt = 1;
            DEBUG_WARN("Auto det baudrate = 9600\r\n");
            board_hw_usart_fm_lcd_initialize(9600);
            return 9600;
        }
//        baudrate_temp = 1000000000 / m_auto_det_baud_raising_time;
//        if ((baudrate_temp < 115290) && (baudrate_temp > 115090))
//        {
//            *baudrate = 115200;
//        }
//        else if ((baudrate_temp < 9690) && (baudrate_temp > 9550))
//        {
//            *baudrate = 9600;
//        }
    }
    return 0xFFFFFFFF;
}

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
//    {EXT_OP_PA11_PORT, EXT_OP_PA11_PIN, IOEX_MODE_OUTPUT, 0},
};


static gpio_output_info_t m_gpio_output[BOARD_HW_OUTPUT_MAX] = 
{
    {BOARD_HW_OUTPUT_LED_1_R, 0, 0, 0},
    {BOARD_HW_OUTPUT_LED_1_B, 0, 0, 0},
    {BOARD_HW_OUTPUT_LED_2_R, 0, 0, 0},
    {BOARD_HW_OUTPUT_LED_2_B, 0, 0, 0},
    {BOARD_HW_OUTPUT_SW_OUT, BOARD_HW_SW_OUT_PORT, BOARD_HW_SW_OUT_PIN, 0},
    {BOARD_HW_OUTPUT1, BOARD_HW_OUTPUT1_PORT, BOARD_HW_OUTPUT1_PIN, 1},
    {BOARD_HW_OUTPUT2, BOARD_HW_OUTPUT2_PORT, BOARD_HW_OUTPUT2_PIN, 1},
    {BOARD_HW_OUTPUT_PA, BOARD_HW_PA_CONTROL_PORT, BOARD_HW_PA_CONTROL_PIN, 0},
    {BOARD_HW_MODULE_RESET, BOARD_HW_SMART_MODULE_RST_PORT, BOARD_HW_SMART_MODULE_RST_PIN, 0},
    {BOARD_HW_MODULE_PWR_ON, BOARD_HW_SMART_MODULE_PWR_CONTROL_PORT, BOARD_HW_SMART_MODULE_PWR_CONTROL_PIN, 0},
    {BOARD_HW_MODULE_PWR_KEY, BOARD_HW_SMART_MODULE_PWR_KEY_PORT, BOARD_HW_SMART_MODULE_PWR_KEY_PIN, 0},
    {BOARD_HW_OUTPUT_SW_IN, BOARD_HW_SW_IN_PORT, BOARD_HW_SW_IN_PIN, 0},
    {BOARD_HW_OUTPUT_SW_POWER_NOTIFICATION, BOARD_HW_TO_SM_POWER_STATE_PORT, BOARD_HW_TO_SM_POWER_STATE_PIN, 0},
    {BOARD_HW_OUTPUT_SW_MIC, BOARD_HW_SW_MIC_PORT, BOARD_HW_SW_MIC_PIN, 0},
    {BOARD_HW_OUTPUT_EXT_WDT, BOARD_HW_EXT_WDT_PORT, BOARD_HW_EXT_WDT_PIN, 0},
    {BOARD_HW_OUTPUT_BY_PASS_DIGITAL_VOL, BOARD_HW_BYPASS_DIDITAL_VOL_GPIO_PORT, BOARD_HW_BYPASS_DIDITAL_VOL_GPIO_PIN, 0}
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
        if (m_ioex_info[i].port_addr == 0 && m_ioex_info[i].pin_addr == 0)
            continue;
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
        if (m_ioex_info[i].port_addr == 0 && m_ioex_info[i].pin_addr == 0)
            continue;
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
        if (m_ioex_info[i].port_addr == 0 && m_ioex_info[i].pin_addr == 0)
            continue;
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

#if USART_HOST_DMA 
static volatile uint32_t m_old_usart_host_dma_rx_pos;
static volatile uint32_t m_old_usart_fm_lcd_dma_rx_pos = 0;
static uint8_t m_dma_host_usart_rx_buffer[DMA_USART_RX_BUFFER_SIZE];
volatile uint8_t m_dma_host_usart_idx = 0;

void DMA_Channel1_2_IRQHandler(void)
{   
    if (RESET != dma_interrupt_flag_get(DMA_CH2, DMA_INT_FLAG_HTF)) 
    {
        dma_interrupt_flag_clear(DMA_CH2, DMA_INT_FLAG_G);
    }
    
    if (RESET != dma_interrupt_flag_get(DMA_CH2, DMA_INT_FLAG_FTF)) 
    {
        dma_interrupt_flag_clear(DMA_CH2, DMA_INT_FLAG_G);
        DEBUG_WARN("DMA complete callback\r\n");
    }
}
#endif

#if 0
void board_hw_get_host_usart_clk_source(void)
{
    #define SEL_IRC8M                    ((uint32_t)0x00000000U)
    #define SEL_HXTAL                    ((uint32_t)0x00000001U)
    #define SEL_PLL                      ((uint32_t)0x00000002U)

    uint32_t sws = 0U;
    uint32_t cksys_freq = 0U, ahb_freq = 0U, apb2_freq = 0U;
    uint32_t pllmf = 0U, pllmf4 = 0U, pllmf5 = 0U, pllsel = 0U, pllpresel = 0U, prediv = 0U, idx = 0U, clk_exp = 0U;
    /* exponent of AHB, APB1 and APB2 clock divider */
    const uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t apb2_exp[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    sws = GET_BITS(RCU_CFG0, 2, 3);
    switch (sws) 
    {
    /* IRC8M is selected as CK_SYS */
    case SEL_IRC8M:
        cksys_freq = IRC8M_VALUE;
        break;
    /* HXTAL is selected as CK_SYS */
    case SEL_HXTAL:
        cksys_freq = HXTAL_VALUE;
        break;
    /* PLL is selected as CK_SYS */
    case SEL_PLL:
        /* get the value of PLLMF[3:0] */
        pllmf = GET_BITS(RCU_CFG0, 18, 21);
        pllmf4 = GET_BITS(RCU_CFG0, 27, 27);
        pllmf5 = GET_BITS(RCU_CFG1, 31, 31);
        /* high 16 bits */
        /* high 16 bits */
        if ((0U == pllmf4) && (0U == pllmf5)) 
        {
            pllmf += 2U;
        }
        if ((1U == pllmf4) && (0U == pllmf5)) 
        {
            pllmf += 17U;
        }
        if ((0U == pllmf4) && (1U == pllmf5)) 
        {
            pllmf += 33U;
        }
        if ((1U == pllmf4) && (1U == pllmf5)) 
        {
            pllmf += 49U;
        }

        /* PLL clock source selection, HXTAL or IRC48M or IRC8M/2 */
        pllsel = GET_BITS(RCU_CFG0, 16, 16);
        pllpresel = GET_BITS(RCU_CFG1, 30, 30);
        if (0U != pllsel) 
        {
            prediv = (GET_BITS(RCU_CFG1, 0, 3) + 1U);
            if (0U == pllpresel) 
            {
                cksys_freq = (HXTAL_VALUE / prediv) * pllmf;
            } 
            else 
            {
                cksys_freq = (IRC48M_VALUE / prediv) * pllmf;
            }
        } 
        else 
        {
            cksys_freq = (IRC8M_VALUE >> 1) * pllmf;
        }
        break;
    /* IRC8M is selected as CK_SYS */
    default:
        cksys_freq = IRC8M_VALUE;
        break;
    }
    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    ahb_freq = cksys_freq >> clk_exp;


    /* calculate APB2 clock frequency */
    idx = GET_BITS(RCU_CFG0, 11, 13);
    clk_exp = apb2_exp[idx];
    apb2_freq = ahb_freq >> clk_exp;

    
    /* calculate USART clock frequency */
    if (RCU_USART0SRC_CKAPB2 == (RCU_CFG2 & RCU_CFG2_USART0SEL)) 
    {
        DEBUG_WARN("apb2_freq %u\r\n", apb2_freq);
    } 
    else if (RCU_USART0SRC_CKSYS == (RCU_CFG2 & RCU_CFG2_USART0SEL)) 
    {
        DEBUG_WARN("cksys_freq %u\r\n", cksys_freq);
    } 
    else if (RCU_USART0SRC_LXTAL == (RCU_CFG2 & RCU_CFG2_USART0SEL)) 
    {
        DEBUG_WARN("LXTAL_VALUE %u\r\n", LXTAL_VALUE);
    } 
    else if (RCU_USART0SRC_IRC8M == (RCU_CFG2 & RCU_CFG2_USART0SEL)) 
    {
        DEBUG_WARN("IRC8M_VALUE %u\r\n", IRC8M_VALUE);
    } 
    else 
    {
        DEBUG_WARN("Unknown clock src\r\n");
    }
}
#endif


static void board_hw_usart_host_initialize(void)
{
#if USART_HOST_DMA
    rcu_periph_clock_enable(RCU_DMA);
    dma_parameter_struct dma_init_struct;
    
        // init dma
    dma_deinit(DMA_CH2);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)m_dma_host_usart_rx_buffer;
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

static void board_hw_usart_fm_lcd_initialize(uint32_t baudrate)
{
    //set baudrate again
    m_last_fm_lcd_baudrate = baudrate;
    // USART1
    nvic_irq_enable(USART1_IRQn, 1, 0);
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);

    /* USART configure */
    usart_deinit(FM_USARTPeripheral);
    usart_word_length_set(FM_USARTPeripheral, USART_WL_8BIT);
    usart_stop_bit_set(FM_USARTPeripheral, USART_STB_1BIT);
    usart_parity_config(FM_USARTPeripheral, USART_PM_NONE);
    usart_baudrate_set(FM_USARTPeripheral, m_last_fm_lcd_baudrate);
    usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_ENABLE);
    usart_transmit_config(FM_USARTPeripheral, USART_TRANSMIT_ENABLE);
    
    usart_enable(FM_USARTPeripheral);
    usart_interrupt_enable(FM_USARTPeripheral, USART_INT_RBNE);
//    usart_interrupt_enable(FM_USARTPeripheral, USART_INT_TBE);
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

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 25.
  */
void HOST_IRQHandler(void)
{    
    if (usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_RBNE) != RESET)
    {
#if USART_HOST_DMA == 0
        uint8_t rx = usart_data_receive(HOST_USARTPeripheral) & 0xFF;
        board_hw_on_host_rx_callback(&rx, 1);
#endif
    }
    
    uint32_t tmp = USART_CTL0(HOST_USARTPeripheral);
        
//    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_TBE)
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
        DEBUG_ERROR("HOST : ORE\r\n");
        usart_interrupt_flag_clear(HOST_USARTPeripheral, USART_INT_FLAG_ERR_ORERR);
    }
    
    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_ERR_FERR))
    {
        DEBUG_ERROR("HOST : FE\r\n");
        usart_interrupt_flag_clear(HOST_USARTPeripheral, USART_INT_FLAG_ERR_FERR);
    }    

    if (RESET != usart_interrupt_flag_get(HOST_USARTPeripheral, USART_INT_FLAG_ERR_NERR))
    {
        DEBUG_ERROR("HOST : NE\r\n");
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
        m_dma_host_usart_idx = DMA_USART_RX_BUFFER_SIZE - (dma_transfer_number_get(DMA_CH2));
        board_hw_on_host_rx_callback(m_dma_host_usart_rx_buffer, m_dma_host_usart_idx);

        /* disable DMA and reconfigure */
        dma_channel_disable(DMA_CH2);
        dma_transfer_number_config(DMA_CH2, DMA_USART_RX_BUFFER_SIZE);
        dma_channel_enable(DMA_CH2);
#endif
    }
}

#if ADC_DMA == 0
uint16_t adc_channel_sample(uint8_t channel)
{
    /* ADC regular channel config */
    adc_regular_channel_config(0U, channel, ADC_SAMPLETIME_239POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read());
}
#endif


static void board_hw_adc_initialize(void)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC);
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
   
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4);
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0);
    
#if ADC_DMA
    // DMA
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA);
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&m_adc_buffer);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = sizeof(m_adc_buffer)/sizeof(m_adc_buffer[0]);
    dma_data_parameter.priority     = DMA_PRIORITY_LOW;
    dma_init(DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA_CH0);


    /* ADC contineous function enable */
    adc_special_function_config(ADC_CONTINUOUS_MODE, ENABLE);
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE); 
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, sizeof(m_adc_buffer)/sizeof(m_adc_buffer[0]));
 
    /* ADC regular channel config */
    adc_regular_channel_config(0U, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

    adc_regular_channel_config(1U, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    
    adc_regular_channel_config(2U, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5);
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    
    adc_regular_channel_config(3U, ADC_CHANNEL_8, ADC_SAMPLETIME_239POINT5);
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    
    /* enable ADC interface */
    adc_enable();
    sys_delay_ms(2U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable();

    /* ADC DMA function enable */
    adc_dma_mode_enable();
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
    
#else
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
    /* ADC external trigger config */
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable();
    sys_delay_ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable();
#endif
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
    // timer_auto_det_baudrate_config();
    uart_pin_interrupt_config();
    board_hw_usart_fm_lcd_initialize(115200);
    board_hw_adc_initialize();
    ioex_gpio_init();
    
#if USE_PT2257
    board_hw_watchdog_feed();
    sys_delay_ms(200);  // PT2257 require 200ms delay for stable
    board_hw_watchdog_feed();
    pt2257_init(&m_pt2257_drv);
#endif
    
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
    for (uint32_t i = 0; i < BOARD_HW_INPUT_MAX; i++)
    {
        if (m_gpio_input[i].index == num && m_gpio_input[i].port_addr && m_gpio_input[i].pin_addr)
        {
            return LL_GPIO_IsInputPinSet(m_gpio_input[i].port_addr, m_gpio_input[i].pin_addr) ? 1 : 0;
        }
    }
    return 0;
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
}

uint8_t board_hw_output_get(board_hw_output_num_t num)
{
    for (uint32_t i = 0; i < BOARD_HW_OUTPUT_MAX; i++)
    {
        if (num == m_gpio_output[i].index && m_gpio_output[i].port_addr && m_gpio_output[i].pin_addr)
        {
            return m_gpio_output[i].value ? 1 : 0;
        }
    }
    return 0;
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
        DEBUG_WARN("Reset flag 0x%08X\r\n", tmp);
        
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

void usart_fm_lcd_tx_complete_callback(bool status)
{
#if USE_DMA_FM_LCD_TX
    m_usart_fm_lcd_tx_run = false;
#endif
}


void board_hw_change_lcd_fm_baudrate(uint32_t baudrate)
{
    if (m_last_fm_lcd_baudrate == baudrate)
    {
        DEBUG_WARN("Same baudrate\r\n");
        return;
    }
    m_last_fm_lcd_baudrate = baudrate;
    
    usart_interrupt_disable(FM_USARTPeripheral, USART_INT_RBNE);
    
    usart_deinit(FM_USARTPeripheral);
    usart_word_length_set(FM_USARTPeripheral, USART_WL_8BIT);
    usart_stop_bit_set(FM_USARTPeripheral, USART_STB_1BIT);
    usart_parity_config(FM_USARTPeripheral, USART_PM_NONE);
    usart_baudrate_set(FM_USARTPeripheral, m_last_fm_lcd_baudrate);
    usart_receive_config(FM_USARTPeripheral, USART_RECEIVE_ENABLE);
    usart_transmit_config(FM_USARTPeripheral, USART_TRANSMIT_ENABLE);

    usart_enable(FM_USARTPeripheral);
    usart_interrupt_enable(FM_USARTPeripheral, USART_INT_RBNE);
    
}

extern void board_hw_on_fm_lcd_rx_callback(uint8_t *data, uint32_t length);
void FM_IRQHandler(void)
{
    if (usart_interrupt_flag_get(FM_USARTPeripheral, USART_INT_FLAG_RBNE) != RESET)
    {
        uint8_t rx = usart_data_receive(FM_USARTPeripheral);
        board_hw_on_fm_lcd_rx_callback(&rx, 1);
    }
    
    uint32_t tmp = USART_CTL0(FM_USARTPeripheral);
        
//    if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE)
//        && (tmp & USART_CTL0_TBEIE))
//    {
//        /* transmit data */
//        uint8_t c;
//        if (driver_uart1_get_new_data_to_send(&c))
//        {
//            usart_data_transmit(USART1, c);
//        }
//        else
//        {
//            usart_interrupt_disable(USART1, USART_INT_TBE);
//        }
//    }  
    
    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_ORERR))
    {
//        usart_data_receive(USART1);
        usart_flag_clear(FM_USARTPeripheral, USART_FLAG_ORERR);
    } 	
    
    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_FERR))
    {
        usart_data_receive(FM_USARTPeripheral);
        usart_flag_clear(FM_USARTPeripheral, USART_FLAG_FERR);
    }    

    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_NERR))
    {
        usart_data_receive(FM_USARTPeripheral);
        usart_flag_clear(FM_USARTPeripheral, USART_FLAG_NERR);
    }
    
    if (RESET != usart_flag_get(FM_USARTPeripheral, USART_FLAG_PERR))
    {
        usart_flag_clear(FM_USARTPeripheral, USART_FLAG_FERR);
    }  
}

uint32_t board_hw_fm_lcd_send(uint8_t *raw, uint32_t length)
{
#if DEBUG_TX_TO_NEXTION_LCD
    uint32_t found_frame = 0;
    for (uint32_t i = 0; i < length; i++)
    {
        if (raw[i] != 0xFF)
        {
            app_debug_putc(raw[i]);
        }
        else
        {
            found_frame++;
        }
        usart_data_transmit(FM_USARTPeripheral, raw[i]);
        while (RESET == usart_flag_get(FM_USARTPeripheral, USART_FLAG_TBE));
    }
    if (found_frame == 3)
    {
        DEBUG_RAW(" FF FF FF\r\n");
    }
#else
    for (uint32_t i = 0; i < length; i++)
    {
        usart_data_transmit(FM_USARTPeripheral, raw[i]);
        while (RESET == usart_flag_get(FM_USARTPeripheral, USART_FLAG_TC));
    }
#endif    
    return length;
}


uint16_t *board_hw_get_adc_data(void)
{
    return &m_adc_buffer[0];
}

bool board_hw_has_new_adc_data(void)
{
#if ADC_DMA == 0
    m_adc_buffer[0] = adc_channel_sample(ADC_CHANNEL_1);
    m_adc_buffer[1] = adc_channel_sample(ADC_CHANNEL_2);
    m_adc_buffer[2] = adc_channel_sample(ADC_CHANNEL_4);
    m_adc_buffer[3] = adc_channel_sample(ADC_CHANNEL_8);
#endif
    return true;
}

void board_hw_allow_adc_conversion(void)
{
    
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
            DEBUG_VERBOSE("HB -> OK\r\n");
        }
        exti_interrupt_flag_clear(EXTI_2);
    }
    
    if (exti_interrupt_flag_get(EXTI_3) != RESET)
    {
//        DEBUG_INFO("external interrupt -> OK\r\n");
        
        uint32_t timer_val = timer_counter_read(TIMER2);
        process_calculate_falling_raising_timer(timer_val);
        exti_interrupt_flag_clear(EXTI_3);
    }
}

bool board_hw_is_uart_to_host_disable(void)
{
//    return LL_GPIO_IsInputPinSet(BOARD_HW_DISABLE_HOST_UART_PORT, BOARD_HW_DISABLE_HOST_UART_PIN) ? true : false;
    return false;
}


void board_hw_digital_pot_init()
{
    m_pt2257_drv.i2c_init();
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


bool board_hw_digital_pot_set(uint32_t value)
{
#if USE_PT2257 == 0
    
    if (value > 100)
    {
        value = 100;
    }
    
    value *= 255;
    value /= 100;
//    value = m_look_up_vol[m_look_up_vol];
    
    uint8_t vol_cmd[2] = {0x00, value};
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, vol_cmd, 2, 2);
    if (status != HAL_OK)
    {
        DEBUG_ERROR("Set POT failed %d\r\n", status);
        return;
    }
    
    vol_cmd[0] = 0x10;
    status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, vol_cmd, 2, 2);
    if (status != HAL_OK)
    {
        DEBUG_ERROR("Set POT failed %d\r\n", status);
        return;
    }
#else
    
    if (value > 100)
    {
        value = 100;
    }
    uint8_t tmp = m_volume_level_to_db[value];
    if (tmp > 79)
    {
        tmp = 79;
    }
    tmp = 79 - tmp;       // max db = 79
//    DEBUG_INF("Set POT %d\r\n", value);
//    if (value)
    return pt2257_set_vol(&m_pt2257_drv, tmp) == -1 ? false : true;
//    else
//        pt2257_mute(&m_pt2257_drv, false);
#endif
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
#if USE_PT2257 == 0
    uint8_t increase_cmd[2] = {0x00, 0x55};
//    for (uint32_t i = 0; i < count; i++)
    {
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, increase_cmd, 2, 2);
        if (status != HAL_OK)
        {
            DEBUG_ERROR("Vol up failed %d\r\n", status);
            return;
        }
    }
    
    increase_cmd[0] = 0x01;
//    for (uint32_t i = 0; i < count; i++)
    {
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, increase_cmd, 2, 2);
        if (status != HAL_OK)
        {
            DEBUG_ERROR("Vol up failed %d\r\n", status);
            return;
        }
    }
#endif

}

void board_hw_digital_pot_decrease(uint32_t count)
{
#if USE_PT2257 == 0
    uint8_t decrease_cmd[2] = {0x00, 0x23};
    decrease_cmd[0] |= (1 << 3);
    for (uint32_t i = 0; i < count; i++)
    {
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hI2C0, (0x28) << 1, decrease_cmd, 2, 2);
        if (status != HAL_OK)
        {
            DEBUG_ERROR("Vol down failed %d at %d\r\n", status, i);
            return;
        }
    }
#endif
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
//    else
//    {
//        DEBUG_INFO("POT OK\r\n");
//    }
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
    
    DEBUG_VERBOSE("Done\r\n");
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

static volatile uint32_t m_prescaler_a = 0, m_prescaler_s = 0;
// static rtc_timestamp_struct m_rtc_timestamp;
static rtc_parameter_struct m_rtc_init_para;
static void rtc_pre_config(void)
{
    rcu_osci_on(RCU_IRC40K);
    rcu_osci_stab_wait(RCU_IRC40K);
    rcu_rtc_clock_config(RCU_RTCSRC_IRC40K);

    m_prescaler_s = 0x18F;
    m_prescaler_a = 0x63;

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

    m_rtc_init_para.rtc_factor_asyn = m_prescaler_a;
    m_rtc_init_para.rtc_factor_syn = m_prescaler_s;
    m_rtc_init_para.rtc_year = 0x16;
    m_rtc_init_para.rtc_day_of_week = RTC_SATURDAY;
    m_rtc_init_para.rtc_month = RTC_APR;
    m_rtc_init_para.rtc_date = 0x30;
    m_rtc_init_para.rtc_display_format = RTC_24HOUR;
    m_rtc_init_para.rtc_am_pm = RTC_AM;

    /* current time input */
//    tmp_hh = 0;
//	tmp_mm = 0;
//	tmp_ss = 0;
	
    /* RTC current time configuration */
    if (ERROR != rtc_init(&m_rtc_init_para))
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


#define LEAPOCH ((uint32_t)951868800) // (946684800LL + 86400*(31+29))

#define DAYS_PER_400Y (146097)
#define DAYS_PER_100Y (36524)
#define DAYS_PER_4Y   (1461)

void counter_to_struct(uint32_t t, rtc_time_t *tm)
{
    long long days, secs;
    int remdays, remsecs, remyears;
    int qc_cycles, c_cycles, q_cycles;
    int years, months;
    int wday, yday, leap;
    static const char days_in_month[] = {31,30,31,30,31,31,30,31,30,31,31,29};

//	/* Reject time_t values whose year would overflow int */
//	if (t < INT_MIN * 31622400LL || t > INT_MAX * 31622400LL)
//		return -1;

    secs = t - LEAPOCH;
    days = secs / 86400;
    remsecs = secs % 86400;
    if (remsecs < 0) 
    {
        remsecs += 86400;
        days--;
    }

    wday = (3+days)%7;
    if (wday < 0) wday += 7;

    qc_cycles = days / DAYS_PER_400Y;
    remdays = days % DAYS_PER_400Y;
    if (remdays < 0) 
    {
        remdays += DAYS_PER_400Y;
        qc_cycles--;
    }

    c_cycles = remdays / DAYS_PER_100Y;
    if (c_cycles == 4) c_cycles--;
    remdays -= c_cycles * DAYS_PER_100Y;

    q_cycles = remdays / DAYS_PER_4Y;
    if (q_cycles == 25) q_cycles--;
    remdays -= q_cycles * DAYS_PER_4Y;

    remyears = remdays / 365;
    if (remyears == 4) remyears--;
    remdays -= remyears * 365;

    leap = !remyears && (q_cycles || !c_cycles);
    yday = remdays + 31 + 28 + leap;
    if (yday >= 365+leap) yday -= 365+leap;

    years = remyears + (q_cycles << 2) + 100*c_cycles + 400*qc_cycles;           // note x*4 = x << 2

    for (months=0; days_in_month[months] <= remdays; months++)
        remdays -= days_in_month[months];

//	if (years+100 > INT_MAX || years+100 < INT_MIN)
//		return -1;

    tm->year = years + 100;
    tm->month = months + 2;
    if (tm->month >= 12) 
    {
        tm->month -=12;
        tm->year++;
    }
    tm->mday = remdays + 1;
    // tm->tm_wday = wday;
    // tm->tm_yday = yday;

    tm->month += 1;     // convert [0-11] to [1-12]
    tm->hour = remsecs / 3600;
    tm->min = remsecs / 60 % 60;
    tm->sec = remsecs % 60;
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
//  DEBUG_INFO("Update timestamp %u\r\n", timestamp);
    timestamp += 25200; // gmt adjust +7
    counter_to_struct(timestamp, &m_rtc_time);
    DEBUG_INFO("Timestamp [%u] = %d/%d/%d %02d:%02d:%02d\r\n", 
                timestamp, 
                m_rtc_time.year+1900, 
                m_rtc_time.mday, 
                m_rtc_time.month,
                m_rtc_time.hour,
                m_rtc_time.min,
                m_rtc_time.sec);
//	/* setup RTC time value */

    m_rtc_init_para.rtc_factor_asyn = m_prescaler_a;
    m_rtc_init_para.rtc_factor_syn = m_prescaler_s;
    m_rtc_init_para.rtc_year = decimal_2_bcd(m_rtc_time.year);
    m_rtc_init_para.rtc_day_of_week = RTC_SATURDAY;
    m_rtc_init_para.rtc_month = decimal_2_bcd(m_rtc_time.month);
    m_rtc_init_para.rtc_date = decimal_2_bcd(m_rtc_time.mday);
    m_rtc_init_para.rtc_display_format = RTC_24HOUR;
    m_rtc_init_para.rtc_hour = decimal_2_bcd(m_rtc_time.hour);
    m_rtc_init_para.rtc_minute = decimal_2_bcd(m_rtc_time.min);
    m_rtc_init_para.rtc_second = decimal_2_bcd(m_rtc_time.sec);
    m_rtc_init_para.rtc_am_pm = RTC_AM;

    /* RTC current time configuration */
    if (ERROR == rtc_init(&m_rtc_init_para))
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
    rtc_current_time_get(&m_rtc_init_para);
    uint32_t year = bcd_2_decimal(m_rtc_init_para.rtc_year) + 1900;

    m_rtc_time.year = year - 2000;    
    m_rtc_time.month = bcd_2_decimal(m_rtc_init_para.rtc_month);
    m_rtc_time.mday = bcd_2_decimal(m_rtc_init_para.rtc_date);
    m_rtc_time.hour = bcd_2_decimal(m_rtc_init_para.rtc_hour);
    m_rtc_time.min = bcd_2_decimal(m_rtc_init_para.rtc_minute);
    m_rtc_time.sec = bcd_2_decimal(m_rtc_init_para.rtc_second);
    
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
#if USART_HOST_DMA
        dma_channel_disable(DMA_CH2);
        dma_transfer_number_config(DMA_CH2, DMA_USART_RX_BUFFER_SIZE);
        dma_channel_enable(DMA_CH2);
#endif 
        usart_interrupt_enable(HOST_USARTPeripheral, USART_INT_RBNE);
        usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_ENABLE);
    }
    else
    {
#if USART_HOST_DMA
        dma_channel_disable(DMA_CH2);
#endif 
        usart_interrupt_disable(HOST_USARTPeripheral, USART_INT_RBNE);
        usart_receive_config(HOST_USARTPeripheral, USART_RECEIVE_DISABLE);
    }
}
