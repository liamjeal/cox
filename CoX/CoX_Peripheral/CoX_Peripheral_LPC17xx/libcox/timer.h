//*****************************************************************************
//
//! \file      timer.h
//! \brief     Prototypes for the TIMER Driver.
//! \version   V2.2.1.0
//! \date      $CURRENTTIME$
//! \author    CooCox
//! \copyright
//!
//! Copyright (c)  2011, CooCox
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#ifndef __TIMER_H__
#define __TIMER_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup CoX_Peripheral_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup TIMER
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Cfg_Channels LPC17xx Configure parameters.
//! \brief      parameters for timer function.
//!             The value below can be used in any timer function
//! @{
//
//*****************************************************************************

//! Timer Match Channel 0
#define TIMER_MAT_CH_0          BIT_32_0

//! Timer Match Channel 1
#define TIMER_MAT_CH_1          BIT_32_1

//! Timer Match Channel 2
#define TIMER_MAT_CH_2          BIT_32_2

//! Timer Match Channel 3
#define TIMER_MAT_CH_3          BIT_32_3

//! Timer Capture Channel 0
#define TIMER_CAP_CH_0          BIT_32_4

//! Timer Capture Channel 1
#define TIMER_CAP_CH_1          BIT_32_5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Cfg_Int LPC17xx Interrupt Configure parameters.
//! \brief      Parameters for timer interrupt function.
//!             The value below can be used in the function
//!             \ref TimerIntStatusGet
//!             \ref TimerIntStatusCheck
//!             \ref TimerIntStatusClear
//! @{
//
//*****************************************************************************

//! Match Channel 0 Interrupt
#define TIMER_INT_MAT_CH_0      BIT_32_0

//! Match Channel 1 Interrupt
#define TIMER_INT_MAT_CH_1      BIT_32_1

//! Match Channel 2 Interrupt
#define TIMER_INT_MAT_CH_2      BIT_32_2

//! Match Channel 3 Interrupt
#define TIMER_INT_MAT_CH_3      BIT_32_3

//! Capture Channel 0
#define TIMER_INT_CAP_CH_0      BIT_32_4

//! Capture Channel 1
#define TIMER_INT_CAP_CH_1      BIT_32_5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Cfg_Match LPC17xx Match Configure parameters.
//! \brief      Parameters for timer match configure function.
//!             The value below can be used in the function
//!             \ref TimerMatchCfg
//! @{
//
//*****************************************************************************

//! Timer match configure mask
#define TIMER_MAT_MASK          BIT_MASK(32, 2, 0)

//! Timer match pin action
#define TIMER_MAT_PIN_MASK      BIT_MASK(32, 14, 12)

//! An interrupt is generated when MR matches the value in the TC.
#define TIMER_MAT_INT           BIT_32_0

//! The TC will be reset if MR matches it.
#define TIMER_MAT_RESET         BIT_32_1

//! The TC and PC will be stopped and TCR will be set to 0 if MR matches the TC.
#define TIMER_MAT_STOP          BIT_32_2

//! No action when Match Event occurs.
#define TIMER_MAT_PIN_NONE      BIT_32_14

//! Set pin Low when Match Event occurs.
#define TIMER_MAT_PIN_LOW       BIT_32_12

//! Set pin high when Match Event occurs.
#define TIMER_MAT_PIN_HIGH      BIT_32_13

//! Toggle pin value when Match Event occurs.
#define TIMER_MAT_PIN_TOGGLE    (BIT_32_13 | BIT_32_12)

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Cfg_Capture LPC17xx TimerCapture Configure parameters.
//! \brief      Parameters for Timer Capture mode.
//!             The value below can be used in the function \ref TimerCaptureCfg.
//! @{
//
//*****************************************************************************

//! Rising Edge Capture.
#define TIMER_CFG_CAP_RISING    BIT_32_0

//! Falling Edge Capture.
#define TIMER_CFG_CAP_FALLING   BIT_32_1

//! Rising or Falling Edge Capture.
#define TIMER_CFG_CAP_INT       BIT_32_2

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Cfg_Counter LPC17xx TimerCounter Configure parameters.
//! \brief      Parameters for Timer Counter mode.
//!             The value below can be used in the function \ref TimerCounterCfg.
//! @{
//
//*****************************************************************************

//! Counter clock --> Capture Pin 0 Rising Edge pluse
#define TIMER_CFG_CNT_CAP0_RISING      (BIT_32_0)

//! Counter clock --> Capture Pin 0 Falling Edge pluse
#define TIMER_CFG_CNT_CAP0_FALLING     (BIT_32_1)

//! Counter clock --> Capture Pin 0 Rising or Falling Edge pluse
#define TIMER_CFG_CNT_CAP0_BOTH        (BIT_32_1 | BIT_32_0)

//! Counter clock --> Capture Pin 1 Rising Edge pluse
#define TIMER_CFG_CNT_CAP1_RISING      (BIT_32_2 | BIT_32_0)

//! Counter clock --> Capture Pin 1 Falling Edge pluse
#define TIMER_CFG_CNT_CAP1_FALLING     (BIT_32_2 | BIT_32_1)

//! Counter clock --> Capture Pin 1 Rising or Falling Edge pluse
#define TIMER_CFG_CNT_CAP1_BOTH        (BIT_32_2 | BIT_32_1 | BIT_32_0)

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

///////////////////////////////////////////////////////////////////////////////
//  define for port to cox 'x' prefixed interface
///////////////////////////////////////////////////////////////////////////////
#define TIMER_MODE_ONESHOT             BIT_32_0
#define TIMER_MODE_PERIODIC            BIT_32_1
#define TIMER_MODE_TOGGLE              0
#define TIMER_MODE_CONTINUOUS          BIT_32_2
#define TIMER_MODE_CAPTURE             BIT_32_3
#define TIMER_MODE_PWM                 0

#define TIMER_INT_MATCH                BIT_32_0
#define TIMER_INT_CAP_EVENT            BIT_32_1

#define TIMER_EVENT_MATCH              BIT_32_0
#define TIMER_EVENT_CAP_EVENT          BIT_32_1

#define TIMER_COUNTER_RISING           BIT_32_0
#define TIMER_COUNTER_FALLING          BIT_32_1

#define TIMER_CAP_RISING               TIMER_CFG_CNT_CAP0_RISING
#define TIMER_CAP_FALLING              TIMER_CFG_CNT_CAP0_RISING
#define TIMER_CAP_BOTH                 (TIMER_CFG_CNT_CAP0_RISING | TIMER_CFG_CNT_CAP0_RISING)

#define TIMER_CHANNEL0                 TIMER_MAT_CH_0
#define TIMER_CHANNEL1                 TIMER_CAP_CH_0

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __XTIMER_H__

