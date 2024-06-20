#include "board_hw.h"
#include "main.h"
#include "usart.h"
#include "app_debug.h"
#include "stm32f1xx_hal.h"
#include "host_data_layer.h"
#include "iwdg.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
//#include "rtc.h"

//#include "stm32l0xx_hal_flash_ex.h"
//#include "i2c.h"

//#define RTC_ERROR_NONE    0
//#define RTC_ERROR_TIMEOUT 1
typedef board_hw_rtc_time_t rtc_time_t;
#define USE_PT2257 0

#if USE_PT2257
#include "pt2257.h"
#endif

#define IOEX_MODE_OUTPUT 0
#define IOEX_MODE_INPUT  1
//typedef board_hw_rtc_time_t rtc_time_t;

//#define USART_FM_LCD_RX_BUFFER_SIZE     256
//#define USART_HOST_RX_BUFFER_SIZE       128
//#define USE_DMA_TX_USART_HOST           0
#define EEPROM_FLAG_HOST_DIE            0x12345678
#define APP_EEPROM_RESET_REASON_ADDRESS 0x0800FC00
#define APP_EEPROM_RESET_COUNTER_ADDRESS 0x0800FC08
#define APP_EEPROM_CRITICAL_ADDRESS     0x0800FC0C

typedef struct
{
    GPIO_TypeDef *port_addr;
    uint32_t pin_addr;
    uint8_t mode;
    uint8_t value;
} gpio_ex_info_t;

typedef struct
{
    uint8_t index;
    GPIO_TypeDef *port_addr;
    uint32_t pin_addr;
    uint8_t value;
} gpio_output_info_t;

typedef struct
{
    uint8_t index;
    GPIO_TypeDef *port_addr;
    uint32_t pin_addr;
} gpio_input_info_t;

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




//static inline void usart_host_start_receive(uint8_t *data, uint32_t length);
//static uint8_t m_usart_host_rx_buffer[USART_HOST_RX_BUFFER_SIZE];
//static uint8_t m_usart_fm_lcd_rx_buffer[USART_FM_LCD_RX_BUFFER_SIZE];
//volatile uint32_t m_last_usart_host_transfer_size = 0;
//static bool m_usart_host_is_enabled = true;
//volatile uint32_t m_uart_host_wait_for_idle = 20;
//static volatile uint32_t m_old_usart_host_dma_rx_pos;
//static volatile uint32_t m_old_usart_fm_lcd_dma_rx_pos = 0;
//static volatile bool m_usart_fm_lcd_rx_ongoing = false;
//static inline void usart_fm_lcd_start_receive(uint8_t *data, uint32_t length);

//// Button on air debounce
//static volatile uint32_t m_debounce_state = 0;
//static volatile uint8_t m_debounce_index = 0;
//static volatile bool m_button_on_air_pressed = false;
static board_hw_rtc_time_t m_rtc_time;
//static volatile uint32_t m_adc_seq = 0;
//static volatile bool m_adc_has_new_data = false;
static volatile uint16_t m_adc_buffer[BOARD_HW_ADC_COUNTER];

//#if USE_PT2257
//#define PT2257_ADDR 0x88

//static int pt2257_i2c_tx(uint8_t *data, uint32_t size);
//static int pt2257_i2c_rx(uint8_t *data, uint32_t size);
//static int pt2257_i2c_tx_rx(uint8_t *tx, uint8_t *rx_data, uint32_t size);

//static pt2257_drv_t m_pt2257_drv = 
//{
//    .i2c_tx = pt2257_i2c_tx,
//    .i2c_rx = pt2257_i2c_rx,
//    .i2c_tx_rx = pt2257_i2c_tx_rx
//};
//#endif
//    
//static gpio_ex_info_t m_ioex_info[5] = 
//{
//    {EXT_OP_PB12_PORT, EXT_OP_PB12_PIN, IOEX_MODE_OUTPUT, 0},
//    {EXT_OP_PB13_PORT, EXT_OP_PB13_PIN, IOEX_MODE_OUTPUT, 0},
//    {EXT_OP_PB14_PORT, EXT_OP_PB14_PIN, IOEX_MODE_OUTPUT, 0},
//    {EXT_OP_PB15_PORT, EXT_OP_PB15_PIN, IOEX_MODE_OUTPUT, 0},
//};

static void ioex_gpio_init(void);

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
//            if (LL_GPIO_IsInputPinSet((GPIO_TypeDef *)m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0)
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
//            DEBUG_WARN("Mode on pin %d changed to %s\r\n", i, mode[temp]);
//            m_ioex_info[i].mode = temp;
//            
//            if (temp == IOEX_MODE_OUTPUT)
//            {
//                // Reinit gpio
//                LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//                GPIO_InitStruct.Pin = m_ioex_info[i].pin_addr;
//                GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//                GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//                GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//                GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//                LL_GPIO_Init((GPIO_TypeDef *)m_ioex_info[i].port_addr, &GPIO_InitStruct);
//                
//                // Update value
//                m_ioex_info[i].value = (ex_io.val & (1 << (i+16))) ? 1 : 0;
//                if (m_ioex_info[i].value)
//                {
//                    LL_GPIO_SetOutputPin((GPIO_TypeDef *)m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//                else
//                {
//                    LL_GPIO_ResetOutputPin((GPIO_TypeDef *)m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//            }
//            else
//            {
//                // Reinit gpio
//                LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//                GPIO_InitStruct.Pin = m_ioex_info[i].pin_addr;
//                GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//                GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//                GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//                LL_GPIO_Init((GPIO_TypeDef *)m_ioex_info[i].port_addr, &GPIO_InitStruct);
//                
//                m_ioex_info[i].value = (LL_GPIO_IsInputPinSet((GPIO_TypeDef *)m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0);
//            }
//        }
//        else        // No changed
//        {
//            if (temp == IOEX_MODE_OUTPUT)
//            {
//                temp = (ex_io.val & (1 << (16+i))) ? 1 : 0;
//                if (temp != m_ioex_info[i].value)
//                {
//                    // DEBUG_WARN("GPIO on pin %d changed to %d\r\n", i, temp);
//                }
//                m_ioex_info[i].value = temp;
//                if (m_ioex_info[i].value)
//                {
//                    LL_GPIO_SetOutputPin((GPIO_TypeDef *)m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//                else
//                {
//                    LL_GPIO_ResetOutputPin((GPIO_TypeDef *)m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr);
//                }
//            }
//            else
//            {
//                m_ioex_info[i].value = (LL_GPIO_IsInputPinSet((GPIO_TypeDef *)m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0);
//            }
//        }
//    }
//}


static void board_hw_usart_host_initialize(void)
{
//    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
//    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
//    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);
//    
//    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
//    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
//    
//    LL_USART_EnableIT_IDLE(USART2);
//    usart_host_start_receive(m_usart_host_rx_buffer, USART_HOST_RX_BUFFER_SIZE);
}

//static void board_hw_usart_fm_lcd_initialize(void)
//{
//    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_3);
//    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
//    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
//    
//    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
//    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
//    
//    LL_USART_EnableIT_IDLE(LPUART1);
//    usart_fm_lcd_start_receive(m_usart_fm_lcd_rx_buffer, USART_FM_LCD_RX_BUFFER_SIZE);
//}

//#if USE_DMA_TX_USART_HOST
//static inline void config_dma_tx(uint8_t *data, uint32_t len)
//{
//    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
//    
//    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
//                         (uint32_t)data,
//                         (uint32_t)&(USART2->TDR),
//                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//    
//    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, len);
//    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_4);
//    
//    /* Enable DMA TX Interrupt */
//    LL_USART_EnableDMAReq_TX(USART2);
//    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
//}
//#endif

//void board_hw_usart_host_control(bool enable)
//{	
//    if (m_usart_host_is_enabled == enable)
//    {
//        // DEBUG_PRINTF("UART state : no changed\r\n");
//        return;
//    }

//    m_usart_host_is_enabled = enable;
//	
//	if (!m_usart_host_is_enabled)
//	{
//#if USE_DMA_TX_USART_HOST
//        while (m_usart_host_tx_run)
//        {
//            
//        }
//#endif
//		LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

//        GPIO_InitStruct.Pin = MIN_USART_TX_Pin;
//        GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//        LL_GPIO_Init(MIN_USART_TX_GPIO_Port, &GPIO_InitStruct);

//        GPIO_InitStruct.Pin = MIN_USART_RX_Pin;
//        GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//        LL_GPIO_Init(MIN_USART_RX_GPIO_Port, &GPIO_InitStruct);
//        
//        // TX
//        LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_4);
//        LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_4);
//        
//        // RX
//        LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_5);
//        LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_5);
//        LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_5);
//        
//        LL_DMA_ClearFlag_TC4(DMA1);
//        
//        LL_DMA_ClearFlag_TC5(DMA1);
//        LL_DMA_ClearFlag_HT5(DMA1);
//        LL_DMA_ClearFlag_GI5(DMA1);
//        LL_DMA_ClearFlag_TE5(DMA1);
//        
//        NVIC_DisableIRQ(USART2_IRQn);
//        LL_USART_Disable(USART2);
//        /* Peripheral clock enable */
//        LL_APB2_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
//        m_old_usart_host_dma_rx_pos = 0;
//	}
//    else
//    {
//        /* DMA controller clock enable */
//        MX_USART2_UART_Init();
//    }
//}

//#if USE_DMA_TX_USART_HOST
//static inline void config_dma_tx(uint8_t *data, uint32_t len)
//{
//    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
//    
//    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
//                         (uint32_t)data,
//                         (uint32_t)&(USART2->TDR),
//                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//    
//    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, len);
//    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_4);
//    
//    /* Enable DMA TX Interrupt */
//    LL_USART_EnableDMAReq_TX(USART2);
//    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
//}

//static inline void usart_host_hw_transmit_dma(uint8_t *data, uint32_t len)
//{	
//    m_usart_host_tx_run = true;
//    NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
//    config_dma_tx(data, len);
//}
//#endif

//void usart_host_tx_complete_callback(bool status)
//{
//#if USE_DMA_TX_USART_HOST
//    m_usart_host_tx_run = false;
//#endif
//}

//uint32_t board_hw_host_send(uint8_t* raw, uint32_t length)
//{
//    if (!m_usart_host_is_enabled || board_hw_is_uart_to_host_disable())
//    {
//        return 0;
//    }
//#if USE_DMA_TX_USART_HOST
//    usart_host_hw_transmit_dma(raw, length);
//#else
//    for (uint32_t i = 0; i < length; i++)
//    {
//        LL_USART_TransmitData8(USART2, raw[i]);
//        while (0 == LL_USART_IsActiveFlag_TXE(USART2));
//    }
//#endif
//    return length;
//}

//static volatile bool m_usart_host_rx_ongoing = false;
//static inline void usart_host_start_receive(uint8_t *data, uint32_t length)
//{
//    NVIC_DisableIRQ(DMA1_Channel4_5_6_7_IRQn);
//    if (!m_usart_host_rx_ongoing)
//    {  
//        m_usart_host_rx_ongoing = true;
//        NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
//                
//        /* Enable DMA Channel Rx */
//        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
//    
//        LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
//                                (uint32_t)&(USART2->RDR),
//                                (uint32_t)data,
//                                LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//        
//        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, length);
//        LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_4);
//        
//        /* Enable DMA RX Interrupt */
//        LL_USART_EnableDMAReq_RX(USART2);
//        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
//    }
//    else
//    {
//        NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
//    }
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

__WEAK void board_hw_on_host_rx_callback(uint8_t *data, uint32_t length)
{
    
}

//void usart_host_rx_complete_callback(bool status)
//{
//    if (status)
//    {
//        m_uart_host_wait_for_idle = 10;
//        uint32_t pos;

//        /* Calculate current position in buffer */
//        pos = USART_HOST_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
//        if (pos != m_old_usart_host_dma_rx_pos) 
//        {                       /* Check change in received data */
//            if (pos > m_old_usart_host_dma_rx_pos) 
//            {   /* Current position is over previous one */
//                /* We are in "linear" mode */
//                /* Process data directly by subtracting "pointers" */
////                DEBUG_RAW("%.*s", pos - m_old_usart_host_dma_rx_pos, &m_usart_host_rx_buffer[m_old_usart_host_dma_rx_pos]);
//				board_hw_on_host_rx_callback(&m_usart_host_rx_buffer[m_old_usart_host_dma_rx_pos], 
//                                            pos-m_old_usart_host_dma_rx_pos);
//            } 
//            else 
//            {
//                /* We are in "overflow" mode */
//                /* First process data to the end of buffer */
//                /* Check and continue with beginning of buffer */
//                board_hw_on_host_rx_callback(&m_usart_host_rx_buffer[m_old_usart_host_dma_rx_pos], 
//                                            USART_HOST_RX_BUFFER_SIZE - m_old_usart_host_dma_rx_pos);

//                if (pos > 0) 
//                {
//                    board_hw_on_host_rx_callback(&m_usart_host_rx_buffer[0], pos);
//                }
//            }
//            m_old_usart_host_dma_rx_pos = pos;                          /* Save current position as old */
//        }
////        m_usart_host_rx_ongoing = false;
//    }
//    else
//    {
//        m_usart_host_rx_ongoing = false;
//    }
//}

//void usart_host_start_dma_rx(void)
//{
//    usart_host_start_receive(m_usart_host_rx_buffer, USART_HOST_RX_BUFFER_SIZE);
//}

///**
//  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 25.
//  */
//void USART2_IRQHandler(void)
//{
//    if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) 
//    {
//        LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
//        usart_host_rx_complete_callback(true);
//    }
//    
//    if (LL_USART_IsActiveFlag_ORE(USART2))
//    {
//        uint32_t tmp = USART2->RDR;
//        (void)tmp;
//        LL_USART_ClearFlag_ORE(USART2);
//    }
//    
//    if (LL_USART_IsActiveFlag_ORE(USART2))
//    {
//        LL_USART_ClearFlag_FE(USART2);
//    }
//    
//    if (LL_USART_IsActiveFlag_NE(USART2))
//    {
//        LL_USART_ClearFlag_NE(USART2);
//    }
//}

//void DMA1_Channel4_5_6_7_IRQHandler(void)
//{
//    if(LL_DMA_IsActiveFlag_TC4(DMA1))
//    {
//        LL_DMA_ClearFlag_TC4(DMA1);
//        /* Call function Transmission complete Callback for host UART */
//        usart_host_tx_complete_callback(true);
//    }
//    else if(LL_DMA_IsActiveFlag_TE4(DMA1))
//    {
//        LL_DMA_ClearFlag_TE4(DMA1);
//        usart_host_tx_complete_callback(false);
//    }
//    if (LL_DMA_IsActiveFlag_HT5(DMA1))
//    {
//        LL_DMA_ClearFlag_HT5(DMA1);
//		usart_host_rx_complete_callback(true);
//    }
//	if (LL_DMA_IsActiveFlag_TC5(DMA1))
//	{
//		LL_DMA_ClearFlag_GI5(DMA1);
//		usart_host_rx_complete_callback(true);
//	}
//	else if(LL_DMA_IsActiveFlag_TE5(DMA1))
//	{
//        LL_DMA_ClearFlag_TE5(DMA1);
//		usart_host_rx_complete_callback(false);
//	}
//}

uint32_t board_hw_host_send(uint8_t* raw, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        LL_USART_TransmitData8(HOST_USARTPeripheral, raw[i]);
        while (0 == LL_USART_IsActiveFlag_TXE(HOST_USARTPeripheral));
    }
    return length;
}

//static void board_hw_adc_initialize(void)
//{
//    
//}

void board_hw_initialize(void)
{
    board_hw_usart_host_initialize();
    ioex_gpio_init();
    
#if USE_PT2257
    pt2257_init(&m_pt2257_drv);
#endif
}


void board_hw_reset(void)
{
    NVIC_SystemReset();
}

void board_hw_watchdog_feed(void)
{
    WRITE_REG(IWDG->KR, LL_IWDG_KEY_RELOAD);
}


static void ioex_gpio_init()
{      
    for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
    {
        if (m_ioex_info[i].port_addr == 0 && m_ioex_info[i].pin_addr == 0)
            continue;
        if (m_ioex_info[i].mode == IOEX_MODE_OUTPUT)
        {
            LL_GPIO_SetPinMode(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr, LL_GPIO_MODE_OUTPUT);
            
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
            LL_GPIO_SetPinMode(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr, LL_GPIO_MODE_INPUT);
            m_ioex_info[i].value = LL_GPIO_IsInputPinSet(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0;
        }
    }
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
            HAL_GPIO_TogglePin(m_gpio_output[i].port_addr, m_gpio_output[i].pin_addr);
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
                HAL_GPIO_WritePin(m_gpio_output[i].port_addr, m_gpio_output[i].pin_addr, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(m_gpio_output[i].port_addr, m_gpio_output[i].pin_addr, GPIO_PIN_RESET);
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
            if (HAL_GPIO_ReadPin(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr) ? 1 : 0)
                info.val |= 1 << (i+16);
        }
    }
    return info;
}

void board_hw_ioex_update(board_hw_ioex_mode_t ex_io)
{
    return;
//    char *mode[] = {"out", "in"};
//    for (uint32_t i = 0; i < sizeof(m_ioex_info)/sizeof(m_ioex_info[0]); i++)
//    {
//        if (m_ioex_info[i].port_addr == 0 && m_ioex_info[i].pin_addr == 0)
//            continue;
//        int temp = (ex_io.val & (1 << i)) ? IOEX_MODE_INPUT : IOEX_MODE_OUTPUT;
//        
//        if (m_ioex_info[i].mode != temp)    // Mode changed
//        {
//            DEBUG_WARN("Mode on pin %d changed to %s\r\n", i, mode[temp]);
//            m_ioex_info[i].mode = temp;
//            
//            if (temp == IOEX_MODE_OUTPUT)
//            {
//                // Reinit gpio
//                LL_GPIO_SetPinMode(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr, LL_GPIO_MODE_OUTPUT);
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
//                LL_GPIO_SetPinMode(m_ioex_info[i].port_addr, m_ioex_info[i].pin_addr, LL_GPIO_MODE_INPUT);
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
//                    // DEBUG_WARN("GPIO on pin %d changed to %d\r\n", i, temp);
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
}

static board_hw_reset_reason_t m_reset_reason = 
{
    .value = 0,
};

void board_hw_set_host_die_reason_to_flash(void)
{    
//	HAL_FLASH_Unlock();

//    HAL_FLASHEx_DATAEEPROM_Erase(APP_EEPROM_RESET_REASON_ADDRESS);
//	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, APP_EEPROM_RESET_REASON_ADDRESS, EEPROM_FLAG_HOST_DIE);
//	HAL_FLASHEx_DATAEEPROM_Lock();
//    
//    volatile uint32_t data = *((uint32_t*)APP_EEPROM_RESET_REASON_ADDRESS);
//    if (data != EEPROM_FLAG_HOST_DIE)
//    {
//        DEBUG_ERROR("Write eeprom failed\r\n");
//    }
}

void flash_if_init(void)
{
    // TODO
//    /* Unlock the Program memory */
//    HAL_FLASH_Unlock();

//    /* Clear all FLASH flags */
//    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR | FLASH_FLAG_SI |
//                           FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR | FLASH_FLAG_FWWERR |
//                           FLASH_FLAG_NOTZEROERR);
//    /* Unlock the Program memory */
//    HAL_FLASH_Lock();
}

void board_hw_clear_host_die_reason_to_flash(void)
{   
//    volatile uint32_t data = *((uint32_t*)APP_EEPROM_RESET_REASON_ADDRESS);
//    if (data == 0xFFFFFFFF)
//    {
//        return;
//    }
//    
//    flash_if_init();
//    
//	HAL_FLASHEx_DATAEEPROM_Unlock();

//    HAL_FLASHEx_DATAEEPROM_Erase(APP_EEPROM_RESET_REASON_ADDRESS);
//	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, APP_EEPROM_RESET_REASON_ADDRESS, 0xFFFFFFFF);
//	HAL_FLASHEx_DATAEEPROM_Lock();
//    
//    data = *((uint32_t*)APP_EEPROM_RESET_REASON_ADDRESS);
//    if (data != 0xFFFFFFFF)
//    {
//        DEBUG_ERROR("Write eeprom failed\r\n");
//    }
}


static uint32_t m_reset_counter = 0;
uint32_t board_hw_get_reset_counter(void)
{   
    static uint32_t m_first_time = true;
    if (m_first_time == true)
    {
        m_first_time = false;
//        volatile uint32_t data = *((uint32_t*)APP_EEPROM_RESET_COUNTER_ADDRESS);
//        if (data != 0xFFFFFFFF)
//        {
//            flash_if_init();
//            m_first_time = data+1;
//            HAL_FLASHEx_DATAEEPROM_Unlock();
//            HAL_FLASHEx_DATAEEPROM_Erase(APP_EEPROM_RESET_COUNTER_ADDRESS);
//            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, APP_EEPROM_RESET_COUNTER_ADDRESS, m_reset_counter);
//            HAL_FLASHEx_DATAEEPROM_Lock();

//            data = *((uint32_t*)m_reset_counter);
//            if (data != 0xFFFFFFFF)
//            {
//                DEBUG_ERROR("Write reset counter 0x%08X - 0x%08X to eeprom eeprom failed\r\n", m_reset_counter, data);
//            }
//        }        
    }
    return m_reset_counter;
}

board_hw_reset_reason_t *board_hardware_get_reset_reason(void)
{
    static bool valid = true;
    if (valid == true)
    {
        valid = false;
        
//        if (m_reset_reason.value == 0)
//        {
//            DEBUG_RAW("RST:");
//        }
        
        m_reset_reason.name.power_on = 1;

        if (LL_RCC_IsActiveFlag_PINRST())
        {
            m_reset_reason.name.pin_reset = 1;
            DEBUG_RAW("PIN,");
        }

        if (LL_RCC_IsActiveFlag_SFTRST())
        {
            m_reset_reason.name.software = 1;
            m_reset_reason.name.power_on = 0;
            DEBUG_RAW("SFT,");
        }


        if (LL_RCC_IsActiveFlag_IWDGRST())
        {
            m_reset_reason.name.watchdog = 1;
            m_reset_reason.name.power_on = 0;
            DEBUG_RAW("IWD,");
        }

        if (LL_RCC_IsActiveFlag_WWDGRST())
        {
            m_reset_reason.name.watchdog = 1;
            m_reset_reason.name.power_on = 0;
            DEBUG_RAW("WWDG,");
        }

        if (LL_RCC_IsActiveFlag_LPWRRST())
        {
            m_reset_reason.name.low_power = 1;
            m_reset_reason.name.power_on = 1;
            DEBUG_RAW("LPWR,");
        }		
            
        if (LL_RCC_IsActiveFlag_PORRST())
        {
            m_reset_reason.name.power_on = 1;
            DEBUG_RAW("POR,");
        }
        
        volatile uint32_t data = *((uint32_t*)APP_EEPROM_RESET_REASON_ADDRESS);
        if (data == EEPROM_FLAG_HOST_DIE)
        {
            m_reset_reason.name.host_die = 1;
            DEBUG_RAW("HOST DIE,");
        }
        
        DEBUG_RAW("\r\n");

        LL_RCC_ClearResetFlags();
        board_hw_clear_host_die_reason_to_flash();
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

//void usart_fm_lcd_tx_complete_callback(bool status)
//{
//#if USE_DMA_FM_LCD_TX
//    m_usart_fm_lcd_tx_run = false;
//#endif
//}


static uint32_t m_last_fm_lcd_baudrate = 115200;
void board_hw_change_lcd_fm_baudrate(uint32_t baudrate)
{
    if (m_last_fm_lcd_baudrate == baudrate)
    {
        return;
    }
    m_last_fm_lcd_baudrate = baudrate;
//    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//    
//    // Disable usart
//    GPIO_InitStruct.Pin = FM_USART_TX_Pin;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//    LL_GPIO_Init(FM_USART_TX_GPIO_Port, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = FM_USART_RX_Pin;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//    LL_GPIO_Init(FM_USART_RX_GPIO_Port, &GPIO_InitStruct);
//    
//    // TX
//    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
//    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_2);
//    
//    // RX
//    LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_3);
//    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
//    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
//    
//    LL_DMA_ClearFlag_TC2(DMA1);
//    
//    LL_DMA_ClearFlag_TC3(DMA1);
//    LL_DMA_ClearFlag_HT3(DMA1);
//    LL_DMA_ClearFlag_GI3(DMA1);
//    LL_DMA_ClearFlag_TE3(DMA1);
//    
//    NVIC_DisableIRQ(LPUART1_IRQn);
//    LL_USART_Disable(LPUART1);
//    /* Peripheral clock enable */
//    LL_APB2_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_LPUART1);

//    // Enable usart again
//    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};


//    /* Peripheral clock enable */
//    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);

//    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//    /**LPUART1 GPIO Configuration
//    PA2   ------> LPUART1_TX
//    PA3   ------> LPUART1_RX
//    */
//    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
//    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
//    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_5);

//    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

//    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

//    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

//    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

//    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

//    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

//    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

//    /* LPUART1_TX Init */
//    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_5);

//    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

//    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

//    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

//    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

//    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

//    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

//    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

//    LPUART_InitStruct.BaudRate = m_last_fm_lcd_baudrate;
//    LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
//    LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
//    LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
//    LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
//    LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
//    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
}

//__WEAK void board_hw_on_fm_lcd_rx_callback(uint8_t *data, uint32_t length)
//{
//    
//}

//static inline void usart_fm_lcd_start_receive(uint8_t *data, uint32_t length)
//{
//    NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
//    if (!m_usart_fm_lcd_rx_ongoing)
//    {  
//        m_usart_fm_lcd_rx_ongoing = true;
//        NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
//                
//        /* Enable DMA Channel Rx */
//        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
//    
//        LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
//                                (uint32_t)&(LPUART1->RDR),
//                                (uint32_t)data,
//                                LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//        
//        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, length);
//        LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_5);
//        
//        /* Enable DMA RX Interrupt */
//        LL_USART_EnableDMAReq_RX(LPUART1);
//        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
//    }
//    else
//    {
//        NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
//    }
//}


//void usart_fm_lcd_rx_complete_callback(bool status)
//{
//    if (status)
//    {
//        uint32_t pos;

//        /* Calculate current position in buffer */
//        pos = USART_FM_LCD_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
//        if (pos != m_old_usart_fm_lcd_dma_rx_pos) 
//        {                       /* Check change in received data */
//            if (pos > m_old_usart_fm_lcd_dma_rx_pos) 
//            {   /* Current position is over previous one */
//                /* We are in "linear" mode */
//                /* Process data directly by subtracting "pointers" */
////                DEBUG_RAW("%.*s", pos - m_old_usart_fm_lcd_dma_rx_pos, &m_usart_fm_lcd_rx_buffer[m_old_usart_fm_lcd_dma_rx_pos]);
//				board_hw_on_fm_lcd_rx_callback(&m_usart_fm_lcd_rx_buffer[m_old_usart_fm_lcd_dma_rx_pos], 
//                                                pos-m_old_usart_fm_lcd_dma_rx_pos);
//            } 
//            else 
//            {
//                /* We are in "overflow" mode */
//                /* First process data to the end of buffer */
//                /* Check and continue with beginning of buffer */
//                board_hw_on_fm_lcd_rx_callback(&m_usart_fm_lcd_rx_buffer[m_old_usart_fm_lcd_dma_rx_pos], 
//                                            USART_FM_LCD_RX_BUFFER_SIZE - m_old_usart_fm_lcd_dma_rx_pos);

//                if (pos > 0) 
//                {
//                    board_hw_on_fm_lcd_rx_callback(&m_usart_fm_lcd_rx_buffer[0], pos);
//                }
//            }
//            m_old_usart_fm_lcd_dma_rx_pos = pos;                          /* Save current position as old */
//        }
//    }
//    else
//    {
//        m_usart_fm_lcd_rx_ongoing = false;
//    }
//}

//void usart_fm_lcd_start_dma_rx(void)
//{
//    usart_fm_lcd_start_receive(m_usart_fm_lcd_rx_buffer, USART_FM_LCD_RX_BUFFER_SIZE);
//}

uint32_t board_hw_fm_lcd_send(uint8_t* raw, uint32_t length)
{
    return length;
}


/**
  * @brief This function handles LPUART1 global interrupt
  */
void USART3_IRQHandler(void)
{
    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) 
    {
        LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
    }
    
    if (LL_USART_IsActiveFlag_RXNE(USART3))
    {
        // DEBUG_ISR("LPUART1 : Over run\r\n");
        uint8_t tmp = USART3->DR;
        board_hw_on_host_rx_callback(&tmp, 1);
    }
    
    if (LL_USART_IsActiveFlag_ORE(USART3))
    {
        // DEBUG_ISR("LPUART1 : Over run\r\n");
        uint32_t tmp = USART3->DR;
        (void)tmp;
        LL_USART_ClearFlag_ORE(USART3);
    }
    
    if (LL_USART_IsActiveFlag_ORE(USART3))
    {
        // DEBUG_ISR("LPUART1 FE\r\n");
        LL_USART_ClearFlag_FE(USART3);
    }
    
    if (LL_USART_IsActiveFlag_NE(USART3))
    {
        // DEBUG_ISR("LPUART1 NE\r\n");
        LL_USART_ClearFlag_NE(USART3);
    }
}

//void ADC1_IRQHandler(void)
//{
//    /* Check whether ADC group regular end of unitary conversion caused         */
//    /* the ADC interruption.                                                    */
//    if (LL_ADC_IsActiveFlag_EOS(ADC1) != 0)
//    {
//        /* Clear flag ADC group regular end of unitary conversion */
//        LL_ADC_ClearFlag_EOS(ADC1);

//    }

//    if (LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
//    {
//        /* Clear flag ADC group regular end of unitary conversion */
//        LL_ADC_ClearFlag_EOC(ADC1);
//        m_adc_seq++;
//        m_adc_buffer[m_adc_seq-1] = LL_ADC_REG_ReadConversionData12(ADC1);
//        if (m_adc_seq >= 4)
//        {
////            LL_ADC_REG_StopConversion(ADC1);
//            m_adc_seq = 0;
//            m_adc_has_new_data = true;
//        }
//        else
//        {
//            LL_ADC_REG_StartConversion(ADC1);
//        }
//    }
//    
//    /* Check whether ADC group regular overrun caused the ADC interruption */
//    if (LL_ADC_IsActiveFlag_OVR(ADC1) != 0)
//    {
//        // DEBUG_ISR("ADC overrun\r\n");
//        LL_ADC_ClearFlag_OVR(ADC1);
//    }
//}

//static void board_hw_start_adc_conversion(void)
//{
//    LL_ADC_REG_StartConversion(ADC1);
//}

uint16_t *board_hw_get_adc_data(void)
{
    m_adc_buffer[0] = 1250;
    m_adc_buffer[1] = 2450;
    m_adc_buffer[2] = 3050;
    m_adc_buffer[3] = 3300;
    return (uint16_t *)&m_adc_buffer[0];
}

bool board_hw_has_new_adc_data(void)
{
    return true;
}

void board_hw_allow_adc_conversion(void)
{

}

///**
//  * @brief This function handles TIM2 global interrupt, interval is 50ms
//  */
//void TIM2_IRQHandler(void)
//{
//    if (LL_TIM_IsActiveFlag_UPDATE(TIM2))       // 50ms timer
//    {
//        if (m_adc_seq == 0 && m_adc_has_new_data == false)
//        {
//            board_hw_start_adc_conversion();
//        }
//        
////        // Debounce button
////        uint8_t level = LL_GPIO_IsInputPinSet(BUTTON_ON_AIR_GPIO_Port, BUTTON_ON_AIR_Pin);
////        m_debounce_state += (level << m_debounce_index);
////        m_debounce_index++;
////        if (m_debounce_index == 8)
////        {
////            if (m_debounce_state == 0x00)
////            {
////                if (!m_button_on_air_pressed)
////                {
////                    // DEBUG_ISR("Button on air pressed\r\n");
////                }
////                m_button_on_air_pressed = true;
////            }
////            else if (m_debounce_state == 0x08)
////            {
////                if (m_button_on_air_pressed)
////                {
////                    // DEBUG_ISR("Button on air release\r\n");
////                }
////                m_button_on_air_pressed = false;
////            }
////            m_debounce_state = 0;
////            m_debounce_index = 0;
////        }
//        LL_TIM_ClearFlag_UPDATE(TIM2);
//    }
//}

//__WEAK void board_hw_ping_detect_cb(void)
//{
//    
//}

///**
//  * @brief This function handles EXTI line 2 and line 3 interrupts.
//  */
//void EXTI2_3_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

//  /* USER CODE END EXTI2_3_IRQn 0 */
//  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
//  {
//    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
//    /* USER CODE BEGIN LL_EXTI_LINE_2 */
//    board_hw_ping_detect_cb();
//    /* USER CODE END LL_EXTI_LINE_2 */
//  }
//  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

//  /* USER CODE END EXTI2_3_IRQn 1 */
//}

//bool board_hw_is_uart_to_host_disable(void)
//{
//    return LL_GPIO_IsInputPinSet(DISABLE_HOST_UART_PIN_GPIO_Port, DISABLE_HOST_UART_PIN_Pin) ? true : false;
////    return false;
//}

//void board_hw_digital_pot_reset(void)
//{
////    uint8_t reset_cmd = 0xFF;
////    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, 0x29, &reset_cmd, 1, 2);
////    if (status != HAL_OK)
////    {
////        DEBUG_ERROR("Software reset POT failed %d\r\n", status);
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
//    status = HAL_I2C_Master_Transmit(&hi2c1, (0x28) << 1, vol_cmd, 2, 2);
//    if (status != HAL_OK)
//    {
//        DEBUG_ERROR("Set POT failed %d\r\n", status);
//        return;
//    }
//    
//    vol_cmd[0] = 0x10;
//    status = HAL_I2C_Master_Transmit(&hi2c1, (0x28) << 1, vol_cmd, 2, 2);
//    if (status != HAL_OK)
//    {
//        DEBUG_ERROR("Set POT failed %d\r\n", status);
//        return;
//    }
//#else
////    if (value)
//        pt2257_set_vol(&m_pt2257_drv, value);
////    else
////        pt2257_mute(&m_pt2257_drv, false);
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
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (0x28) << 1, increase_cmd, 2, 2);
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
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (0x28) << 1, increase_cmd, 2, 2);
//        if (status != HAL_OK)
//        {
//            DEBUG_ERROR("Vol up failed %d\r\n", status);
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
//        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (0x28) << 1, decrease_cmd, 2, 2);
//        if (status != HAL_OK)
//        {
//            DEBUG_ERROR("Vol down failed %d at %d\r\n", status, i);
//            return;
//        }
//    }
//#endif
//}

//#if USE_PT2257
//static int pt2257_i2c_tx(uint8_t *data, uint32_t size)
//{
//#if 0
//    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (PT2257_ADDR), data, size, 10);
//    if (status != HAL_OK)
//    {
//        DEBUG_ERROR("I2C TX failed %d\r\n", status);
//        return -1;
//    }
//    return 0;
//#else    
//    int ret = 0;
//    /* Master Generate Start condition for a write request :              */
//    /*    - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS                   */
//    /*    - with a auto stop condition generation when transmit all bytes */
//    LL_I2C_HandleTransfer(I2C1, PT2257_ADDR, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
//    uint32_t now = sys_get_ms();

//    
//    /* Loop until STOP flag is raised  */
//    while (!LL_I2C_IsActiveFlag_STOP(I2C1))
//    {
//        /* (2.1) Transmit data (TXIS flag raised) *********************************/

//        /* Check TXIS flag value in ISR register */
//        if (LL_I2C_IsActiveFlag_TXIS(I2C1))
//        {
//            /* Write data in Transmit Data register.
//            TXIS flag is cleared by writing data in TXDR register */
//            LL_I2C_TransmitData8(I2C1, (*data++));
//        }

//        /* Check Systick counter flag to decrement the time-out value */
//        if (LL_SYSTICK_IsActiveCounterFlag()) 
//        {
//            if(sys_get_ms() - now >= (uint32_t)10)
//            {
//                ret = -1;
//                break;
//            }
//        }
//    }
//    LL_I2C_ClearFlag_STOP(I2C1);
//    return ret;
//#endif
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

void board_hw_set_critical_error(board_hw_critical_log_t err)
{
    // TODO
//    uint32_t critical_err;
//    err.name.valid_flag = 0xAB;
//    flash_if_init();
//    
//	HAL_FLASHEx_DATAEEPROM_Unlock();

//    HAL_FLASHEx_DATAEEPROM_Erase(APP_EEPROM_CRITICAL_ADDRESS);
//	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, APP_EEPROM_CRITICAL_ADDRESS, err.value);
//	HAL_FLASHEx_DATAEEPROM_Lock();
//    DEBUG_VERBOSE("Write eeprom critical error %u to flash\r\n", err.value);
//     // critical_err = *((volatile uint32_t*)APP_EEPROM_CRITICAL_ADDRESS);
//    if (critical_err != err.value)
//    {
//        // DEBUG_ERROR("Write eeprom critical error failed %08X-%08X\r\n", err.value, critical_err);
//    }
}

bool board_hw_get_critical_error(board_hw_critical_log_t *err)
{
    uint32_t readback;
    
    flash_if_init();
    
    readback = *((volatile uint32_t*)APP_EEPROM_CRITICAL_ADDRESS);
    err->value = readback;
    
    if (err->name.valid_flag != 0xAB)
    {
        // DEBUG_ERROR("Read eeprom critical error failed 0x%08X\r\n", err.value);
        err->value = 0;
        return false;
    }
    else
    {
        err->value = readback;
        return true;
    }
}

void board_hw_disable_fm_rx(void)
{
    
}

void board_hw_enable_host_rx(bool enable)
{
    
}

void board_hw_setting_fm_irq_rx(bool enable)
{
    
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

//static uint8_t bcd_2_decimal(uint8_t hex)
//{
//    int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
//    return dec;
//}     

//// Function to convert Decimal to BCD
//static uint8_t decimal_2_bcd(uint8_t num)
//{
//    return (num/10) * 16 + (num % 10);
//}


board_hw_rtc_time_t *board_hw_rtc_get(void)
{
    /* Note: need to convert in decimal value in using __LL_RTC_CONVERT_BCD2BIN helper macro */
    /* Display time Format : hh:mm:ss */

//    m_rtc_time.hour = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
//    m_rtc_time.min = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)), 
//    m_rtc_time.sec = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));

//    m_rtc_time.month =  __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC));
//    m_rtc_time.day = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)); 
//    m_rtc_time.year = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC));
    
    return &m_rtc_time;
}

///**
//  * @brief  Enter in initialization mode
//  * @note In this mode, the calendar counter is stopped and its value can be updated
//  * @param  None
//  * @retval RTC_ERROR_NONE if no error
//  */
//uint32_t Enter_RTC_InitMode(void)
//{
//  /* Set Initialization mode */
//  LL_RTC_EnableInitMode(RTC);
//  
//#if (USE_TIMEOUT == 1)
//    Timeout = RTC_TIMEOUT_VALUE;
//#endif /* USE_TIMEOUT */

//  /* Check if the Initialization mode is set */
//  while (LL_RTC_IsActiveFlag_INIT(RTC) != 1)
//  {
//#if (USE_TIMEOUT == 1)
//      if (LL_SYSTICK_IsActiveCounterFlag())
//    {
//        Timeout --;
//    }
//      if (Timeout == 0)
//    {
//      return RTC_ERROR_TIMEOUT;
//    }  
//#endif /* USE_TIMEOUT */
//  }
//  
//  return RTC_ERROR_NONE;
//}

////static uint8_t bcd_2_decimal(uint8_t hex)
////{
////    int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
////    return dec;
////}     

//// Function to convert Decimal to BCD
//static uint8_t decimal_2_bcd(uint8_t num)
//{
//    return (num/10) * 16 + (num % 10);
//}

///**
//  * @brief  Wait until the RTC Time and Date registers (RTC_TR and RTC_DR) are
//  *         synchronized with RTC APB clock.
//  * @param  None
//  * @retval RTC_ERROR_NONE if no error (RTC_ERROR_TIMEOUT will occur if RTC is 
//  *         not synchronized)
//  */
//uint32_t WaitForSynchro_RTC(void)
//{
//  /* Clear RSF flag */
//  LL_RTC_ClearFlag_RS(RTC);

//#if (USE_TIMEOUT == 1)
//    Timeout = RTC_TIMEOUT_VALUE;
//#endif /* USE_TIMEOUT */

//  /* Wait the registers to be synchronised */
//  while(LL_RTC_IsActiveFlag_RS(RTC) != 1)
//  {
//#if (USE_TIMEOUT == 1)
//      if (LL_SYSTICK_IsActiveCounterFlag())
//    {
//        Timeout --;
//    }
//      if (Timeout == 0)
//    {
//      return RTC_ERROR_TIMEOUT;
//    }  
//#endif /* USE_TIMEOUT */
//  }
//  return RTC_ERROR_NONE;
//}

///**
//  * @brief  Exit Initialization mode 
//  * @param  None
//  * @retval RTC_ERROR_NONE if no error
//  */
//uint32_t Exit_RTC_InitMode(void)
//{
//  LL_RTC_DisableInitMode(RTC);
//  
//  /* Wait for synchro */
//  /* Note: Needed only if Shadow registers is enabled           */
//  /*       LL_RTC_IsShadowRegBypassEnabled function can be used */
//  return (WaitForSynchro_RTC());
//}


void board_hw_rtc_set_timestamp(uint32_t timestamp)
{
//    timestamp += 25200; // gmt adjust +7
//    counter_to_struct(timestamp, &m_rtc_time, 1);
//    
//      /*##-1- Disable RTC registers write protection ############################*/
//    LL_RTC_DisableWriteProtection(RTC);


//    /*##-3- Enter in initialization mode ######################################*/
//    if (Enter_RTC_InitMode() != RTC_ERROR_NONE)   
//    {
//        
//    }

//    /*##-4- Configure the Date ################################################*/
//    /* Note: __LL_RTC_CONVERT_BIN2BCD helper macro can be used if user wants to*/
//    /*       provide directly the decimal value:                               */
//    /*       LL_RTC_DATE_Config(RTC, LL_RTC_WEEKDAY_MONDAY,                    */
//    /*                          __LL_RTC_CONVERT_BIN2BCD(31), (...))           */
//    /* Set Date: Monday March 31th 2015 */
//    LL_RTC_DATE_Config(RTC, LL_RTC_WEEKDAY_MONDAY, decimal_2_bcd(m_rtc_time.day), decimal_2_bcd(m_rtc_time.month), decimal_2_bcd(m_rtc_time.year-2000));

//    /*##-5- Configure the Time ################################################*/
//    /* Set Time: 11:59:40 PM*/
//    LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_PM, decimal_2_bcd(m_rtc_time.hour), decimal_2_bcd(m_rtc_time.min), decimal_2_bcd(m_rtc_time.sec));

//    /*##-6- Exit of initialization mode #######################################*/
//    if (Exit_RTC_InitMode() != RTC_ERROR_NONE)   
//    {
//        
//    }

//    /*##-7- Enable RTC registers write protection #############################*/
//    LL_RTC_EnableWriteProtection(RTC);
}

static volatile uint32_t m_delay_ms = 0;
static volatile uint32_t m_sys_tick = 0;
void sys_increase_tick(void)
{
    ++m_sys_tick;
    if (m_delay_ms)
    {
        --m_delay_ms;
    }
    host_data_layer_on_1ms_callback();
}


void sys_delay_ms(uint32_t ms)
{
    NVIC_DisableIRQ(SysTick_IRQn);
    m_delay_ms = ms;
    NVIC_EnableIRQ(SysTick_IRQn);
    while (m_delay_ms);
}

uint32_t sys_get_ms(void)
{
    return uwTick;
}
