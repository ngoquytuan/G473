#include "delay.h"
/*******************************************************************************
Noi Dung    :   Tao tre 1ms.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
/*Delay function*/
void delay(__IO uint32_t num)
{
	__IO uint32_t index = 0;
	
	/* Default system clock is 72.00MHz */
	for(index = (9 * 800000 * num); index != 0; index--);
}

void delay_ms(__IO uint32_t num)
{
	__IO uint32_t index = 0;
	
	/* Default system clock is 72MHz */
	for(index = (9 * 800 * num); index != 0; index--);
}
void delay_us(volatile uint32_t num)
{
	volatile uint32_t index = 0;
	//__IO uint32_t temp = 0;
	/* Default system clock is 72MHz */
	for(index = (9 * num); index != 0; index--);
}

