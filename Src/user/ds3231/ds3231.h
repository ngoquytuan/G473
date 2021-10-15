#ifndef _DS3231_
#define _DS3231_
#include "stm32g4xx_hal.h"
#include <stdio.h>

//#pragma used+    

#define RTC_WADDR 0b11010000
#define RTC_RADDR 0b11010001

typedef unsigned char uint8_t; 
//typedef unsigned int uint16_t; 

extern struct rtc_time enjoy_time;
extern struct rtc_date enjoy_date;
extern char time_line[20];
//extern char arrcTimeValue[15];

/*  
 * Clock initialization
 */
void rtc3231_init(void);

/**
 * Reading time from clock
 * @time: time structure with reading data
 */
void rtc3231_read_time(struct rtc_time *time);

/**
 * Reading date from clock
 * @date: date structure with reading data
 */
void rtc3231_read_date(struct rtc_date *date);

/**
 * Reading date and time from clock
 * @time: time structure with reading data
 * @date: date structure with reading data
 */
void rtc3231_read_datetime(struct rtc_time *time, struct rtc_date *date);

/**
 * Writing new time in clock
 * @time: time structure for writing data
 */
void rtc3231_write_time(struct rtc_time *time);

/**
 * Writing new date in clock
 * @date: date structure for writing data
 */
void rtc3231_write_date(struct rtc_date *date);

static unsigned char bcd (unsigned char data);
void Fn_Ds3231_Update_Time(void);

//#pragma used-

//#pragma library Ds3231.lib
                              
#endif