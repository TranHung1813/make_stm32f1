/*!
    \file  gd32f3x0.h
    \brief firmware option for gd32f3x0
    
    \version 2017-06-06, V1.0.0, firmware for GD32F3x0
    \version 2019-06-01, V2.0.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/
#ifndef GD32F3X0_LIBOPT_H
#define GD32F3X0_LIBOPT_H

/* Includes */
/* Comment the line below to disable peripheral header file inclusion */
#include "RTE_Components.h"

#ifdef RTE_DEVICE_STDPERIPHERALS_ADC
#include "gd32f3x0_adc.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_CEC
#include "gd32f3x0_cec.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_CTC
#include "gd32f3x0_ctc.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_CRC
#include "gd32f3x0_crc.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_CMP
#include "gd32f3x0_cmp.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_DAC
#include "gd32f3x0_dac.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_DBG
#include "gd32f3x0_dbg.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_DMA
#include "gd32f3x0_dma.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_EXTI
#include "gd32f3x0_exti.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_FMC
#include "gd32f3x0_fmc.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_GPIO
#include "gd32f3x0_gpio.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_SYSCFG
#include "gd32f3x0_syscfg.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_I2C
#include "gd32f3x0_i2c.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_FWDGT
#include "gd32f3x0_fwdgt.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_PMU
#include "gd32f3x0_pmu.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_RCU
#include "gd32f3x0_rcu.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_RTC
#include "gd32f3x0_rtc.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_SPI
#include "gd32f3x0_spi.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_TIMER
#include "gd32f3x0_timer.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_USART
#include "gd32f3x0_usart.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_WWDGT
#include "gd32f3x0_wwdgt.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_MISC
#include "gd32f3x0_misc.h"
#endif
#ifdef RTE_DEVICE_STDPERIPHERALS_TSI
#include "gd32f3x0_tsi.h"
#endif

#endif /* __GD32F3X0_LIBOPT_H */

/******************* (C) COPYRIGHT 2017 GIGADEVICE *****END OF FILE****/
