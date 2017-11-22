/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_rtc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_SetRtcClockSource(void);
/*!
 * @brief Set the alarm which will be trigerred x secs later. The alarm trigger
 *        will print a notification on the console.
 *
 * @param offsetSec  Offset seconds is set for alarm.
 */
static void CommandAlarm(uint8_t offsetSec);

/*!
 * @brief Run the digital clock in 20s and show the digital clock on console.
 *
 */
void CheckSeconds_Update(rtc_datetime_t *datetime);

void Init_RTC (void);
bool set_RTC_DateTime (rtc_datetime_t *datetime);

#define SUCCESS 0
#define FAILURE 1
/*!
 * @brief Get the current date time.
 *
 */
static void CommandGetDatetime(void);

/*!
 * @brief Receive the console input and echo.
 *
 * @param buf  Pointer of buffer.
 * @param size Size of buffer.
 */
static void ReceiveFromConsole(char *buf, uint32_t size);

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile bool g_SecsFlag = false;

rtc_datetime_t date;

/*******************************************************************************
 * Code
 ******************************************************************************/

void BOARD_SetRtcClockSource(void)
{
    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;
}

static void CommandGetDatetime(void)
{
    rtc_datetime_t date;

    RTC_GetDatetime(RTC, &date);
    PRINTF("Current datetime: %04d-%02d-%02d %02d:%02d:%02d\r\n", date.year, date.month, date.day, date.hour,
           date.minute, date.second);
}

void CheckSeconds_Update(rtc_datetime_t *datetime)
{
       /* If seconds interrupt ocurred, print new time */
        if (g_SecsFlag)
        {
            /* Build up the word */
            g_SecsFlag = false;
            //CommandGetDatetime();
            RTC_GetDatetime(RTC, datetime);
         //   PRINTF("Current datetime: %04d-%02d-%02d %02d:%02d:%02d\r\n", datetime->year, datetime->month, datetime->day, datetime->hour,
        //   datetime->minute, datetime->second);
        }
}





/*!
 * @brief Override the RTC Second IRQ handler.
 */
void RTC_Seconds_IRQHandler(void)
{
    g_SecsFlag = true;
}

      /* Init RTC */
void Init_RTC (void)
{
      rtc_config_t rtcConfig;
    /*
     * rtcConfig.wakeupSelect = false;
     * rtcConfig.updateMode = false;
     * rtcConfig.supervisorAccess = false;
     * rtcConfig.compensationInterval = 0;
     * rtcConfig.compensationTime = 0;
     */
      
      if(checkRTC_TimeValid() == true)                  // Ensemble created.. when RTC is valid .. no need to set time again...
      {
        EnableIRQ(RTC_Seconds_IRQn);
        return;
      }
    RTC_GetDefaultConfig(&rtcConfig);
    RTC_Init(RTC, &rtcConfig);
    /* Select RTC clock source */
    BOARD_SetRtcClockSource();

    /* Set a start date time and start RTC */
    date.year = 2017U;
    date.month = 04U;
    date.day = 11U;
    date.hour = 14U;
    date.minute = 25U;
    date.second = 10U;

    /* RTC time counter has to be stopped before setting the date & time in the TSR register */
    RTC_StopTimer(RTC);

    RTC_SetDatetime(RTC, &date);

    /* Enable at the NVIC */
//    EnableIRQ(RTC_IRQn);
    EnableIRQ(RTC_Seconds_IRQn);

    /* Start the RTC time counter */
    RTC_StartTimer(RTC);
    RTC_EnableInterrupts(RTC, kRTC_SecondsInterruptEnable);
}

bool set_RTC_DateTime (rtc_datetime_t *datetime)
{
                date.year = (uint16_t)datetime->year;
                date.month = (uint8_t)datetime->month;
                date.day = (uint8_t)datetime->day;
                date.hour = (uint8_t)datetime->hour;
                date.minute = (uint8_t)datetime->minute;
                date.second = (uint8_t)datetime->second;

                /* RTC time counter has to be stopped before setting the date & time in the TSR register */
                RTC_StopTimer(RTC);
                DisableIRQ(RTC_Seconds_IRQn);
                if (kStatus_Success != RTC_SetDatetime(RTC, &date))
                {
                 //   PRINTF("INVALID DATA");
                  return(FAILURE);
                }
                RTC_StartTimer(RTC);
                EnableIRQ(RTC_Seconds_IRQn);
                return(SUCCESS);
}
