//*****************************************************************************
//
//! \file      pwm.h
//! \brief     Prototypes for the PWM Driver.
//! \version   V2.3.0.0
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

#ifndef __PWM_H__
#define __PWM_H__

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
//! \addtogroup PWM
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup  LPC17xx_PWM
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup  LPC17xx_PWM_Parameters
//! @{
//
//*****************************************************************************

//! PWM Channel 0
#define PWM_CH_0                BIT_32_0

//! PWM Channel 1
#define PWM_CH_1                BIT_32_1

//! PWM Channel 2
#define PWM_CH_2                BIT_32_2

//! PWM Channel 3
#define PWM_CH_3                BIT_32_3

//! PWM Channel 4
#define PWM_CH_4                BIT_32_4

//! PWM Channel 5
#define PWM_CH_5                BIT_32_5

//! PWM Channel 6
#define PWM_CH_6                BIT_32_6

//! PWM capture Channel 0
#define PWM_CAP_0               BIT_32_7

//! PWM capture Channel 1
#define PWM_CAP_1               BIT_32_8

//! PWM interrupt channel 0
#define PWM_INT_CH_0            BIT_32_0

//! PWM interrupt channel 1
#define PWM_INT_CH_1            BIT_32_1

//! PWM interrupt channel 2
#define PWM_INT_CH_2            BIT_32_2

//! PWM interrupt channel 3
#define PWM_INT_CH_3            BIT_32_3

//! PWM interrupt channel 4
#define PWM_INT_CH_4            BIT_32_8

//! PWM interrupt channel 5
#define PWM_INT_CH_5            BIT_32_9

//! PWM interrupt channel 6
#define PWM_INT_CH_6            BIT_32_10

//! PWM interrupt capture channel 0
#define PWM_INT_CAP_0           BIT_32_4

//! PWM interrupt capture channel 1
#define PWM_INT_CAP_1           BIT_32_5

//! PWM interrupt flag mask
#define PWM_INT_FLAG_MASK       (PWM_INT_CH_0 | PWM_INT_CH_1  | PWM_INT_CH_2 |       \
                                 PWM_INT_CH_3 | PWM_INT_CH_4  | PWM_INT_CH_5 |       \
                                 PWM_INT_CH_6 | PWM_INT_CAP_0 | PWM_INT_CAP_1)

//! Enable PWM match interrupt
#define PWM_MATCH_INT_EN        BIT_32_0

//! Disable PWM match interrupt
#define PWM_MATCH_INT_DIS       BIT_32_8

//! Reset counter when PWM match event occurs
#define PWM_MATCH_RESET_EN      BIT_32_1

//! Don't reset counter when PWM match event occurs
#define PWM_MATCH_RESET_DIS     BIT_32_9

//! Stop counter when PWM match event occurs
#define PWM_MATCH_STOP_EN       BIT_32_2

//! Don's stop counter when PWM match event occurs
#define PWM_MATCH_STOP_DIS      BIT_32_10

//! Enable Double edge mode
#define PWM_EDGE_DOUBLE         BIT_32_0

//! Enable single edge mode
#define PWM_EDGE_SINGLE         BIT_32_1

//! Enable PWM capture channel 0 falling sample.
#define CH0_FALLING_SAMPLE_EN   BIT_32_0

//! Disable PWM capture channel 0 falling sample.
#define CH0_FALLING_SAMPLE_DIS  BIT_32_8

//! Enable PWM capture channel 0 rising sample.
#define CH0_RISING_SAMPLE_EN    BIT_32_1

//! Disable PWM capture channel 0 rising sample.
#define CH0_RISING_SAMPLE_DIS   BIT_32_9

//! Enable PWM capture channel 0 event interrupt.
#define CH0_EDGE_EVENT_INT_EN   BIT_32_2

//! Disable PWM capture channel 0 event interrupt.
#define CH0_EDGE_EVENT_INT_DIS  BIT_32_10

//! Enable PWM capture channel 1 falling sample.
#define CH1_FALLING_SAMPLE_EN   BIT_32_3

//! Disable PWM capture channel 1 falling sample.
#define CH1_FALLING_SAMPLE_DIS  BIT_32_11

//! Enable PWM capture channel 1 rising sample.
#define CH1_RISING_SAMPLE_EN    BIT_32_4

//! Disable PWM capture channel 1 rising sample.
#define CH1_RISING_SAMPLE_DIS   BIT_32_12

//! Enable PWM capture channel 1 event interrupt.
#define CH1_EDGE_EVENT_INT_EN   BIT_32_5

//! Disable PWM capture channel 1 event interrupt.
#define CH1_EDGE_EVENT_INT_DIS  BIT_32_13

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

///////////////////////////////////////////////////////////////////////////////
//  define for port to cox 'x' prefixed interface
///////////////////////////////////////////////////////////////////////////////
#define PWM_INT_PWM                    BIT_32_0

#define PWM_EVENT_PWM                  BIT_32_0
#define PWM_EVENT_CAP                  BIT_32_1

#define TIMER_MODE_CONTINUOUS          BIT_32_2
#define TIMER_MODE_CAPTURE             BIT_32_3
#define TIMER_MODE_PWM                 0

#define PWM_FREQ_CONFIG(a,b,c)         (a * b)

#define PWM_ONE_SHOT_MODE              BIT_32_30
#define PWM_TOGGLE_MODE                BIT_32_31
#define PWM_OUTPUT_INVERTER_EN         0
#define PWM_OUTPUT_INVERTER_DIS        0
#define PWM_DEAD_ZONE_EN               0
#define PWM_DEAD_ZONE_DIS              0

#define PWM_CHANNEL0                   PWM_CH_0
#define PWM_CHANNEL1                   PWM_CH_1
#define PWM_CHANNEL2                   PWM_CH_2
#define PWM_CHANNEL3                   PWM_CH_3
#define PWM_CHANNEL4                   PWM_CH_4
#define PWM_CHANNEL5                   PWM_CH_5
#define PWM_CHANNEL6                   PWM_CH_6
#define PWM_CHANNEL7                   PWM_CH_7

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

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __PWM_H__
