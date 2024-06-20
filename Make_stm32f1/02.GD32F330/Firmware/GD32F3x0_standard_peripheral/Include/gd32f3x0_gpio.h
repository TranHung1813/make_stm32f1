/*!
    \file    gd32f3x0_gpio.h
    \brief   definitions for the GPIO

    \version 2017-06-06, V1.0.0, firmware for GD32F3x0
    \version 2019-06-01, V2.0.0, firmware for GD32F3x0
    \version 2020-09-30, V2.1.0, firmware for GD32F3x0
    \version 2022-01-06, V2.2.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

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

#ifndef GD32F3X0_GPIO_H
#define GD32F3X0_GPIO_H

#include "gd32f3x0.h"

/* GPIOx(x=A,B,C,D,F) definitions */
#define GPIOA                      (GPIO_BASE + 0x00000000U)
#define GPIOB                      (GPIO_BASE + 0x00000400U)
#define GPIOC                      (GPIO_BASE + 0x00000800U)
#define GPIOD                      (GPIO_BASE + 0x00000C00U)
#define GPIOF                      (GPIO_BASE + 0x00001400U)

/* registers definitions */
#define GPIO_CTL(gpiox)            REG32((gpiox) + 0x00000000U)    /*!< GPIO port control register */
#define GPIO_OMODE(gpiox)          REG32((gpiox) + 0x00000004U)    /*!< GPIO port output mode register */
#define GPIO_OSPD0(gpiox)          REG32((gpiox) + 0x00000008U)    /*!< GPIO port output speed register 0 */
#define GPIO_PUD(gpiox)            REG32((gpiox) + 0x0000000CU)    /*!< GPIO port pull-up/pull-down register */
#define GPIO_ISTAT(gpiox)          REG32((gpiox) + 0x00000010U)    /*!< GPIO port input status register */
#define GPIO_OCTL(gpiox)           REG32((gpiox) + 0x00000014U)    /*!< GPIO port output control register */
#define GPIO_BOP(gpiox)            REG32((gpiox) + 0x00000018U)    /*!< GPIO port bit operation register */
#define GPIO_LOCK(gpiox)           REG32((gpiox) + 0x0000001CU)    /*!< GPIO port configuration lock register */
#define GPIO_AFSEL0(gpiox)         REG32((gpiox) + 0x00000020U)    /*!< GPIO alternate function selected register 0 */
#define GPIO_AFSEL1(gpiox)         REG32((gpiox) + 0x00000024U)    /*!< GPIO alternate function selected register 1 */
#define GPIO_BC(gpiox)             REG32((gpiox) + 0x00000028U)    /*!< GPIO bit clear register */
#define GPIO_TG(gpiox)             REG32((gpiox) + 0x0000002CU)    /*!< GPIO port bit toggle register */
#define GPIO_OSPD1(gpiox)          REG32((gpiox) + 0x0000003CU)    /*!< GPIO port output speed register 1 */

/* bits definitions */
/* GPIO_CTL */
#define GPIO_CTL_CTL0              BITS(0,1)             /*!< pin 0 configuration bits */
#define GPIO_CTL_CTL1              BITS(2,3)             /*!< pin 1 configuration bits */
#define GPIO_CTL_CTL2              BITS(4,5)             /*!< pin 2 configuration bits */
#define GPIO_CTL_CTL3              BITS(6,7)             /*!< pin 3 configuration bits */
#define GPIO_CTL_CTL4              BITS(8,9)             /*!< pin 4 configuration bits */
#define GPIO_CTL_CTL5              BITS(10,11)           /*!< pin 5 configuration bits */
#define GPIO_CTL_CTL6              BITS(12,13)           /*!< pin 6 configuration bits */
#define GPIO_CTL_CTL7              BITS(14,15)           /*!< pin 7 configuration bits */
#define GPIO_CTL_CTL8              BITS(16,17)           /*!< pin 8 configuration bits */
#define GPIO_CTL_CTL9              BITS(18,19)           /*!< pin 9 configuration bits */
#define GPIO_CTL_CTL10             BITS(20,21)           /*!< pin 10 configuration bits */
#define GPIO_CTL_CTL11             BITS(22,23)           /*!< pin 11 configuration bits */
#define GPIO_CTL_CTL12             BITS(24,25)           /*!< pin 12 configuration bits */
#define GPIO_CTL_CTL13             BITS(26,27)           /*!< pin 13 configuration bits */
#define GPIO_CTL_CTL14             BITS(28,29)           /*!< pin 14 configuration bits */
#define GPIO_CTL_CTL15             BITS(30,31)           /*!< pin 15 configuration bits */

/* GPIO_OMODE */
#define GPIO_OMODE_OM0             BIT(0)                /*!< pin 0 output mode bit */
#define GPIO_OMODE_OM1             BIT(1)                /*!< pin 1 output mode bit */
#define GPIO_OMODE_OM2             BIT(2)                /*!< pin 2 output mode bit */
#define GPIO_OMODE_OM3             BIT(3)                /*!< pin 3 output mode bit */
#define GPIO_OMODE_OM4             BIT(4)                /*!< pin 4 output mode bit */
#define GPIO_OMODE_OM5             BIT(5)                /*!< pin 5 output mode bit */
#define GPIO_OMODE_OM6             BIT(6)                /*!< pin 6 output mode bit */
#define GPIO_OMODE_OM7             BIT(7)                /*!< pin 7 output mode bit */
#define GPIO_OMODE_OM8             BIT(8)                /*!< pin 8 output mode bit */
#define GPIO_OMODE_OM9             BIT(9)                /*!< pin 9 output mode bit */
#define GPIO_OMODE_OM10            BIT(10)               /*!< pin 10 output mode bit */
#define GPIO_OMODE_OM11            BIT(11)               /*!< pin 11 output mode bit */
#define GPIO_OMODE_OM12            BIT(12)               /*!< pin 12 output mode bit */
#define GPIO_OMODE_OM13            BIT(13)               /*!< pin 13 output mode bit */
#define GPIO_OMODE_OM14            BIT(14)               /*!< pin 14 output mode bit */
#define GPIO_OMODE_OM15            BIT(15)               /*!< pin 15 output mode bit */

/* GPIO_OSPD0 */
#define GPIO_OSPD0_OSPD0           BITS(0,1)             /*!< pin 0 output max speed bits */
#define GPIO_OSPD0_OSPD1           BITS(2,3)             /*!< pin 1 output max speed bits */
#define GPIO_OSPD0_OSPD2           BITS(4,5)             /*!< pin 2 output max speed bits */
#define GPIO_OSPD0_OSPD3           BITS(6,7)             /*!< pin 3 output max speed bits */
#define GPIO_OSPD0_OSPD4           BITS(8,9)             /*!< pin 4 output max speed bits */
#define GPIO_OSPD0_OSPD5           BITS(10,11)           /*!< pin 5 output max speed bits */
#define GPIO_OSPD0_OSPD6           BITS(12,13)           /*!< pin 6 output max speed bits */
#define GPIO_OSPD0_OSPD7           BITS(14,15)           /*!< pin 7 output max speed bits */
#define GPIO_OSPD0_OSPD8           BITS(16,17)           /*!< pin 8 output max speed bits */
#define GPIO_OSPD0_OSPD9           BITS(18,19)           /*!< pin 9 output max speed bits */
#define GPIO_OSPD0_OSPD10          BITS(20,21)           /*!< pin 10 output max speed bits */
#define GPIO_OSPD0_OSPD11          BITS(22,23)           /*!< pin 11 output max speed bits */
#define GPIO_OSPD0_OSPD12          BITS(24,25)           /*!< pin 12 output max speed bits */
#define GPIO_OSPD0_OSPD13          BITS(26,27)           /*!< pin 13 output max speed bits */
#define GPIO_OSPD0_OSPD14          BITS(28,29)           /*!< pin 14 output max speed bits */
#define GPIO_OSPD0_OSPD15          BITS(30,31)           /*!< pin 15 output max speed bits */

/* GPIO_PUD */
#define GPIO_PUD_PUD0              BITS(0,1)             /*!< pin 0 pull-up or pull-down bits */
#define GPIO_PUD_PUD1              BITS(2,3)             /*!< pin 1 pull-up or pull-down bits */
#define GPIO_PUD_PUD2              BITS(4,5)             /*!< pin 2 pull-up or pull-down bits */
#define GPIO_PUD_PUD3              BITS(6,7)             /*!< pin 3 pull-up or pull-down bits */
#define GPIO_PUD_PUD4              BITS(8,9)             /*!< pin 4 pull-up or pull-down bits */
#define GPIO_PUD_PUD5              BITS(10,11)           /*!< pin 5 pull-up or pull-down bits */
#define GPIO_PUD_PUD6              BITS(12,13)           /*!< pin 6 pull-up or pull-down bits */
#define GPIO_PUD_PUD7              BITS(14,15)           /*!< pin 7 pull-up or pull-down bits */
#define GPIO_PUD_PUD8              BITS(16,17)           /*!< pin 8 pull-up or pull-down bits */
#define GPIO_PUD_PUD9              BITS(18,19)           /*!< pin 9 pull-up or pull-down bits */
#define GPIO_PUD_PUD10             BITS(20,21)           /*!< pin 10 pull-up or pull-down bits */
#define GPIO_PUD_PUD11             BITS(22,23)           /*!< pin 11 pull-up or pull-down bits */
#define GPIO_PUD_PUD12             BITS(24,25)           /*!< pin 12 pull-up or pull-down bits */
#define GPIO_PUD_PUD13             BITS(26,27)           /*!< pin 13 pull-up or pull-down bits */
#define GPIO_PUD_PUD14             BITS(28,29)           /*!< pin 14 pull-up or pull-down bits */
#define GPIO_PUD_PUD15             BITS(30,31)           /*!< pin 15 pull-up or pull-down bits */

/* GPIO_ISTAT */
#define GPIO_ISTAT_ISTAT0          BIT(0)                /*!< pin 0 input status */
#define GPIO_ISTAT_ISTAT1          BIT(1)                /*!< pin 1 input status */
#define GPIO_ISTAT_ISTAT2          BIT(2)                /*!< pin 2 input status */
#define GPIO_ISTAT_ISTAT3          BIT(3)                /*!< pin 3 input status */
#define GPIO_ISTAT_ISTAT4          BIT(4)                /*!< pin 4 input status */
#define GPIO_ISTAT_ISTAT5          BIT(5)                /*!< pin 5 input status */
#define GPIO_ISTAT_ISTAT6          BIT(6)                /*!< pin 6 input status */
#define GPIO_ISTAT_ISTAT7          BIT(7)                /*!< pin 7 input status */
#define GPIO_ISTAT_ISTAT8          BIT(8)                /*!< pin 8 input status */
#define GPIO_ISTAT_ISTAT9          BIT(9)                /*!< pin 9 input status */
#define GPIO_ISTAT_ISTAT10         BIT(10)               /*!< pin 10 input status */
#define GPIO_ISTAT_ISTAT11         BIT(11)               /*!< pin 11 input status */
#define GPIO_ISTAT_ISTAT12         BIT(12)               /*!< pin 12 input status */
#define GPIO_ISTAT_ISTAT13         BIT(13)               /*!< pin 13 input status */
#define GPIO_ISTAT_ISTAT14         BIT(14)               /*!< pin 14 input status */
#define GPIO_ISTAT_ISTAT15         BIT(15)               /*!< pin 15 input status */

/* GPIO_OCTL */
#define GPIO_OCTL_OCTL0            BIT(0)                /*!< pin 0 output bit */
#define GPIO_OCTL_OCTL1            BIT(1)                /*!< pin 1 output bit */
#define GPIO_OCTL_OCTL2            BIT(2)                /*!< pin 2 output bit */
#define GPIO_OCTL_OCTL3            BIT(3)                /*!< pin 3 output bit */
#define GPIO_OCTL_OCTL4            BIT(4)                /*!< pin 4 output bit */
#define GPIO_OCTL_OCTL5            BIT(5)                /*!< pin 5 output bit */
#define GPIO_OCTL_OCTL6            BIT(6)                /*!< pin 6 output bit */
#define GPIO_OCTL_OCTL7            BIT(7)                /*!< pin 7 output bit */
#define GPIO_OCTL_OCTL8            BIT(8)                /*!< pin 8 output bit */
#define GPIO_OCTL_OCTL9            BIT(9)                /*!< pin 9 output bit */
#define GPIO_OCTL_OCTL10           BIT(10)               /*!< pin 10 output bit */
#define GPIO_OCTL_OCTL11           BIT(11)               /*!< pin 11 output bit */
#define GPIO_OCTL_OCTL12           BIT(12)               /*!< pin 12 output bit */
#define GPIO_OCTL_OCTL13           BIT(13)               /*!< pin 13 output bit */
#define GPIO_OCTL_OCTL14           BIT(14)               /*!< pin 14 output bit */
#define GPIO_OCTL_OCTL15           BIT(15)               /*!< pin 15 output bit */

/* GPIO_BOP */
#define GPIO_BOP_BOP0              BIT(0)                /*!< pin 0 set bit */
#define GPIO_BOP_BOP1              BIT(1)                /*!< pin 1 set bit */
#define GPIO_BOP_BOP2              BIT(2)                /*!< pin 2 set bit */
#define GPIO_BOP_BOP3              BIT(3)                /*!< pin 3 set bit */
#define GPIO_BOP_BOP4              BIT(4)                /*!< pin 4 set bit */
#define GPIO_BOP_BOP5              BIT(5)                /*!< pin 5 set bit */
#define GPIO_BOP_BOP6              BIT(6)                /*!< pin 6 set bit */
#define GPIO_BOP_BOP7              BIT(7)                /*!< pin 7 set bit */
#define GPIO_BOP_BOP8              BIT(8)                /*!< pin 8 set bit */
#define GPIO_BOP_BOP9              BIT(9)                /*!< pin 9 set bit */
#define GPIO_BOP_BOP10             BIT(10)               /*!< pin 10 set bit */
#define GPIO_BOP_BOP11             BIT(11)               /*!< pin 11 set bit */
#define GPIO_BOP_BOP12             BIT(12)               /*!< pin 12 set bit */
#define GPIO_BOP_BOP13             BIT(13)               /*!< pin 13 set bit */
#define GPIO_BOP_BOP14             BIT(14)               /*!< pin 14 set bit */
#define GPIO_BOP_BOP15             BIT(15)               /*!< pin 15 set bit */
#define GPIO_BOP_CR0               BIT(16)               /*!< pin 0 clear bit */
#define GPIO_BOP_CR1               BIT(17)               /*!< pin 1 clear bit */
#define GPIO_BOP_CR2               BIT(18)               /*!< pin 2 clear bit */
#define GPIO_BOP_CR3               BIT(19)               /*!< pin 3 clear bit */
#define GPIO_BOP_CR4               BIT(20)               /*!< pin 4 clear bit */
#define GPIO_BOP_CR5               BIT(21)               /*!< pin 5 clear bit */
#define GPIO_BOP_CR6               BIT(22)               /*!< pin 6 clear bit */
#define GPIO_BOP_CR7               BIT(23)               /*!< pin 7 clear bit */
#define GPIO_BOP_CR8               BIT(24)               /*!< pin 8 clear bit */
#define GPIO_BOP_CR9               BIT(25)               /*!< pin 9 clear bit */
#define GPIO_BOP_CR10              BIT(26)               /*!< pin 10 clear bit */
#define GPIO_BOP_CR11              BIT(27)               /*!< pin 11 clear bit */
#define GPIO_BOP_CR12              BIT(28)               /*!< pin 12 clear bit */
#define GPIO_BOP_CR13              BIT(29)               /*!< pin 13 clear bit */
#define GPIO_BOP_CR14              BIT(30)               /*!< pin 14 clear bit */
#define GPIO_BOP_CR15              BIT(31)               /*!< pin 15 clear bit */

/* GPIO_LOCK */
#define GPIO_LOCK_LK0              BIT(0)                /*!< pin 0 lock bit */
#define GPIO_LOCK_LK1              BIT(1)                /*!< pin 1 lock bit */
#define GPIO_LOCK_LK2              BIT(2)                /*!< pin 2 lock bit */
#define GPIO_LOCK_LK3              BIT(3)                /*!< pin 3 lock bit */
#define GPIO_LOCK_LK4              BIT(4)                /*!< pin 4 lock bit */
#define GPIO_LOCK_LK5              BIT(5)                /*!< pin 5 lock bit */
#define GPIO_LOCK_LK6              BIT(6)                /*!< pin 6 lock bit */
#define GPIO_LOCK_LK7              BIT(7)                /*!< pin 7 lock bit */
#define GPIO_LOCK_LK8              BIT(8)                /*!< pin 8 lock bit */
#define GPIO_LOCK_LK9              BIT(9)                /*!< pin 9 lock bit */
#define GPIO_LOCK_LK10             BIT(10)               /*!< pin 10 lock bit */
#define GPIO_LOCK_LK11             BIT(11)               /*!< pin 11 lock bit */
#define GPIO_LOCK_LK12             BIT(12)               /*!< pin 12 lock bit */
#define GPIO_LOCK_LK13             BIT(13)               /*!< pin 13 lock bit */
#define GPIO_LOCK_LK14             BIT(14)               /*!< pin 14 lock bit */
#define GPIO_LOCK_LK15             BIT(15)               /*!< pin 15 lock bit */
#define GPIO_LOCK_LKK              BIT(16)               /*!< pin sequence lock key */

/* GPIO_AFSEL0 */
#define GPIO_AFSEL0_SEL0           BITS(0,3)             /*!< pin 0 alternate function selected */
#define GPIO_AFSEL0_SEL1           BITS(4,7)             /*!< pin 1 alternate function selected */
#define GPIO_AFSEL0_SEL2           BITS(8,11)            /*!< pin 2 alternate function selected */
#define GPIO_AFSEL0_SEL3           BITS(12,15)           /*!< pin 3 alternate function selected */
#define GPIO_AFSEL0_SEL4           BITS(16,19)           /*!< pin 4 alternate function selected */
#define GPIO_AFSEL0_SEL5           BITS(20,23)           /*!< pin 5 alternate function selected */
#define GPIO_AFSEL0_SEL6           BITS(24,27)           /*!< pin 6 alternate function selected */
#define GPIO_AFSEL0_SEL7           BITS(28,31)           /*!< pin 7 alternate function selected */

/* GPIO_AFSEL1 */
#define GPIO_AFSEL1_SEL8           BITS(0,3)             /*!< pin 8 alternate function selected */
#define GPIO_AFSEL1_SEL9           BITS(4,7)             /*!< pin 9 alternate function selected */
#define GPIO_AFSEL1_SEL10          BITS(8,11)            /*!< pin 10 alternate function selected */
#define GPIO_AFSEL1_SEL11          BITS(12,15)           /*!< pin 11 alternate function selected */
#define GPIO_AFSEL1_SEL12          BITS(16,19)           /*!< pin 12 alternate function selected */
#define GPIO_AFSEL1_SEL13          BITS(20,23)           /*!< pin 13 alternate function selected */
#define GPIO_AFSEL1_SEL14          BITS(24,27)           /*!< pin 14 alternate function selected */
#define GPIO_AFSEL1_SEL15          BITS(28,31)           /*!< pin 15 alternate function selected */

/* GPIO_BC */
#define GPIO_BC_CR0                BIT(0)                /*!< pin 0 clear bit */
#define GPIO_BC_CR1                BIT(1)                /*!< pin 1 clear bit */
#define GPIO_BC_CR2                BIT(2)                /*!< pin 2 clear bit */
#define GPIO_BC_CR3                BIT(3)                /*!< pin 3 clear bit */
#define GPIO_BC_CR4                BIT(4)                /*!< pin 4 clear bit */
#define GPIO_BC_CR5                BIT(5)                /*!< pin 5 clear bit */
#define GPIO_BC_CR6                BIT(6)                /*!< pin 6 clear bit */
#define GPIO_BC_CR7                BIT(7)                /*!< pin 7 clear bit */
#define GPIO_BC_CR8                BIT(8)                /*!< pin 8 clear bit */
#define GPIO_BC_CR9                BIT(9)                /*!< pin 9 clear bit */
#define GPIO_BC_CR10               BIT(10)               /*!< pin 10 clear bit */
#define GPIO_BC_CR11               BIT(11)               /*!< pin 11 clear bit */
#define GPIO_BC_CR12               BIT(12)               /*!< pin 12 clear bit */
#define GPIO_BC_CR13               BIT(13)               /*!< pin 13 clear bit */
#define GPIO_BC_CR14               BIT(14)               /*!< pin 14 clear bit */
#define GPIO_BC_CR15               BIT(15)               /*!< pin 15 clear bit */

/* GPIO_TG */
#define GPIO_TG_TG0                BIT(0)                /*!< pin 0 toggle bit */
#define GPIO_TG_TG1                BIT(1)                /*!< pin 1 toggle bit */
#define GPIO_TG_TG2                BIT(2)                /*!< pin 2 toggle bit */
#define GPIO_TG_TG3                BIT(3)                /*!< pin 3 toggle bit */
#define GPIO_TG_TG4                BIT(4)                /*!< pin 4 toggle bit */
#define GPIO_TG_TG5                BIT(5)                /*!< pin 5 toggle bit */
#define GPIO_TG_TG6                BIT(6)                /*!< pin 6 toggle bit */
#define GPIO_TG_TG7                BIT(7)                /*!< pin 7 toggle bit */
#define GPIO_TG_TG8                BIT(8)                /*!< pin 8 toggle bit */
#define GPIO_TG_TG9                BIT(9)                /*!< pin 9 toggle bit */
#define GPIO_TG_TG10               BIT(10)               /*!< pin 10 toggle bit */
#define GPIO_TG_TG11               BIT(11)               /*!< pin 11 toggle bit */
#define GPIO_TG_TG12               BIT(12)               /*!< pin 12 toggle bit */
#define GPIO_TG_TG13               BIT(13)               /*!< pin 13 toggle bit */
#define GPIO_TG_TG14               BIT(14)               /*!< pin 14 toggle bit */
#define GPIO_TG_TG15               BIT(15)               /*!< pin 15 toggle bit */

/* GPIO_OSPD1 */
#define GPIO_OSPD1_SPD0            BIT(0)                /*!< set pin 0 very high output speed when OSPD0 is "11" */
#define GPIO_OSPD1_SPD1            BIT(1)                /*!< set pin 1 very high output speed when OSPD1 is "11" */
#define GPIO_OSPD1_SPD2            BIT(2)                /*!< set pin 2 very high output speed when OSPD2 is "11" */
#define GPIO_OSPD1_SPD3            BIT(3)                /*!< set pin 3 very high output speed when OSPD3 is "11" */
#define GPIO_OSPD1_SPD4            BIT(4)                /*!< set pin 4 very high output speed when OSPD4 is "11" */
#define GPIO_OSPD1_SPD5            BIT(5)                /*!< set pin 5 very high output speed when OSPD5 is "11" */
#define GPIO_OSPD1_SPD6            BIT(6)                /*!< set pin 6 very high output speed when OSPD6 is "11" */
#define GPIO_OSPD1_SPD7            BIT(7)                /*!< set pin 7 very high output speed when OSPD7 is "11" */
#define GPIO_OSPD1_SPD8            BIT(8)                /*!< set pin 8 very high output speed when OSPD8 is "11" */
#define GPIO_OSPD1_SPD9            BIT(9)                /*!< set pin 9 very high output speed when OSPD9 is "11" */
#define GPIO_OSPD1_SPD10           BIT(10)               /*!< set pin 10 very high output speed when OSPD10 is "11" */
#define GPIO_OSPD1_SPD11           BIT(11)               /*!< set pin 11 very high output speed when OSPD11 is "11" */
#define GPIO_OSPD1_SPD12           BIT(12)               /*!< set pin 12 very high output speed when OSPD12 is "11" */
#define GPIO_OSPD1_SPD13           BIT(13)               /*!< set pin 13 very high output speed when OSPD13 is "11" */
#define GPIO_OSPD1_SPD14           BIT(14)               /*!< set pin 14 very high output speed when OSPD14 is "11" */
#define GPIO_OSPD1_SPD15           BIT(15)               /*!< set pin 15 very high output speed when OSPD15 is "11" */

/* constants definitions */
typedef FlagStatus bit_status;

/* output mode definitions */
#define CTL_CLTR(regval)           (BITS(0,1) & ((uint32_t)(regval) << 0))
#define GPIO_MODE_INPUT            CTL_CLTR(0)           /*!< input mode */
#define GPIO_MODE_OUTPUT           CTL_CLTR(1)           /*!< output mode */
#define GPIO_MODE_AF               CTL_CLTR(2)           /*!< alternate function mode */
#define GPIO_MODE_ANALOG           CTL_CLTR(3)           /*!< analog mode */

/* pull-up/pull-down definitions */
#define PUD_PUPD(regval)           (BITS(0,1) & ((uint32_t)(regval) << 0))
#define GPIO_PUPD_NONE             PUD_PUPD(0)           /*!< floating mode, no pull-up and pull-down resistors */
#define GPIO_PUPD_PULLUP           PUD_PUPD(1)           /*!< with pull-up resistor */
#define GPIO_PUPD_PULLDOWN         PUD_PUPD(2)           /*!< with pull-down resistor */

/* GPIO pin definitions */
#define GPIO_PIN_0                 BIT(0)                /*!< GPIO pin 0 */
#define GPIO_PIN_1                 BIT(1)                /*!< GPIO pin 1 */
#define GPIO_PIN_2                 BIT(2)                /*!< GPIO pin 2 */
#define GPIO_PIN_3                 BIT(3)                /*!< GPIO pin 3 */
#define GPIO_PIN_4                 BIT(4)                /*!< GPIO pin 4 */
#define GPIO_PIN_5                 BIT(5)                /*!< GPIO pin 5 */
#define GPIO_PIN_6                 BIT(6)                /*!< GPIO pin 6 */
#define GPIO_PIN_7                 BIT(7)                /*!< GPIO pin 7 */
#define GPIO_PIN_8                 BIT(8)                /*!< GPIO pin 8 */
#define GPIO_PIN_9                 BIT(9)                /*!< GPIO pin 9 */
#define GPIO_PIN_10                BIT(10)               /*!< GPIO pin 10 */
#define GPIO_PIN_11                BIT(11)               /*!< GPIO pin 11 */
#define GPIO_PIN_12                BIT(12)               /*!< GPIO pin 12 */
#define GPIO_PIN_13                BIT(13)               /*!< GPIO pin 13 */
#define GPIO_PIN_14                BIT(14)               /*!< GPIO pin 14 */
#define GPIO_PIN_15                BIT(15)               /*!< GPIO pin 15 */
#define GPIO_PIN_ALL               BITS(0,15)            /*!< GPIO pin all */

/* GPIO mode configuration values */
#define GPIO_MODE_SET(n, mode)     ((uint32_t)((uint32_t)(mode) << (2U * (n))))
#define GPIO_MODE_MASK(n)          ((uint32_t)((uint32_t)0x00000003U << (2U * (n))))

/* GPIO pull-up/pull-down values */
#define GPIO_PUPD_SET(n, pupd)     ((uint32_t)((uint32_t)(pupd) << (2U * (n))))
#define GPIO_PUPD_MASK(n)          ((uint32_t)((uint32_t)0x00000003U << (2U * (n))))

/* GPIO output speed values */
#define GPIO_OSPEED_SET(n, speed)  ((uint32_t)((uint32_t)(speed) << (2U * (n))))
#define GPIO_OSPEED_MASK(n)        ((uint32_t)((uint32_t)0x00000003U << (2U * (n))))

/* GPIO output type */
#define GPIO_OTYPE_PP              ((uint8_t)(0x00U))    /*!< push pull mode */
#define GPIO_OTYPE_OD              ((uint8_t)(0x01U))    /*!< open drain mode */

/* GPIO output max speed value */
#define OSPD_OSPD0(regval)         (BITS(0,1) & ((uint32_t)(regval) << 0))
#define GPIO_OSPEED_2MHZ           OSPD_OSPD0(0)                     /*!< output max speed 2MHz */
#define GPIO_OSPEED_10MHZ          OSPD_OSPD0(1)                     /*!< output max speed 10MHz */
#define GPIO_OSPEED_50MHZ          OSPD_OSPD0(3)                     /*!< output max speed 50MHz */
#define GPIO_OSPEED_MAX            ((uint32_t)0x0000FFFFU)           /*!< GPIO very high output speed, max speed more than 50MHz */

/* GPIO alternate function values */
#define GPIO_AFR_SET(n, af)        ((uint32_t)((uint32_t)(af) << (4U * (n))))
#define GPIO_AFR_MASK(n)           ((uint32_t)((uint32_t)0x0000000FU << (4U * (n))))

/* GPIO alternate function */
#define AF(regval)                 (BITS(0,3) & ((uint32_t)(regval) << 0))
#define GPIO_AF_0                   AF(0)                /*!< alternate function 0 selected */
#define GPIO_AF_1                   AF(1)                /*!< alternate function 1 selected */
#define GPIO_AF_2                   AF(2)                /*!< alternate function 2 selected */
#define GPIO_AF_3                   AF(3)                /*!< alternate function 3 selected */
#define GPIO_AF_4                   AF(4)                /*!< alternate function 4 selected (port A,B only) */
#define GPIO_AF_5                   AF(5)                /*!< alternate function 5 selected (port A,B only) */
#define GPIO_AF_6                   AF(6)                /*!< alternate function 6 selected (port A,B only) */
#define GPIO_AF_7                   AF(7)                /*!< alternate function 7 selected (port A,B only) */

/* function declarations */
/* reset GPIO port */
void gpio_deinit(uint32_t gpio_periph);
/* set GPIO mode */
void gpio_mode_set(uint32_t gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin);
/* set GPIO output type and speed */
void gpio_output_options_set(uint32_t gpio_periph, uint8_t otype, uint32_t speed, uint32_t pin);

/* set GPIO pin bit */
void gpio_bit_set(uint32_t gpio_periph, uint32_t pin);
/* reset GPIO pin bit */
void gpio_bit_reset(uint32_t gpio_periph, uint32_t pin);
/* write data to the specified GPIO pin */
void gpio_bit_write(uint32_t gpio_periph, uint32_t pin, bit_status bit_value);
/* write data to the specified GPIO port */
void gpio_port_write(uint32_t gpio_periph, uint16_t data);

/* get GPIO pin input status */
FlagStatus gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin);
/* get GPIO port input status */
uint16_t gpio_input_port_get(uint32_t gpio_periph);
/* get GPIO pin output status */
FlagStatus gpio_output_bit_get(uint32_t gpio_periph, uint32_t pin);
/* get GPIO port output status */
uint16_t gpio_output_port_get(uint32_t gpio_periph);

/* set GPIO alternate function */
void gpio_af_set(uint32_t gpio_periph, uint32_t alt_func_num, uint32_t pin);
/* lock GPIO pin bit */
void gpio_pin_lock(uint32_t gpio_periph, uint32_t pin);

/* toggle GPIO pin status */
void gpio_bit_toggle(uint32_t gpio_periph, uint32_t pin);
/* toggle GPIO port status */
void gpio_port_toggle(uint32_t gpio_periph);

#endif /* GD32F3X0_GPIO_H */
