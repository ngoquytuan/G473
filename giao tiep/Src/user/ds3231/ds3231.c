#include "ds3231.h"
#include "delay.h"
#include "stdio.h"
#include "Ds3231.h"


struct rtc_time
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
};
struct rtc_time enjoy_time;

struct rtc_date
{
    uint8_t wday;
    uint8_t day;
    uint8_t month;
    uint8_t year;
};

struct rtc_date enjoy_date;
char time_line[20];
char arrcTimeValue[15];

/* @brief: DS3231_init
 * @input: None
 * @output: None
 */



