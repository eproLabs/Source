#ifndef _RTC_FUNC_H_
#define _RTC_FUNC_H_

#include "fsl_rtc.h"

extern void CheckSeconds_Update(rtc_datetime_t *datetime);
extern bool set_RTC_DateTime (rtc_datetime_t *datetime);

extern void Init_RTC (void);

extern volatile bool g_SecsFlag;

rtc_datetime_t cur_date_time, getdate, datetime_SDCWrite, datetime_avg;



#endif