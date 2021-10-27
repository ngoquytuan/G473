#include "stm32g4xx_hal.h"
#include "stdint.h"
#include "lcd.h"
#include "stdio.h"
#include <stdarg.h>

void delay_us(volatile uint32_t num)
{
	volatile uint32_t index = 0;
	//__IO uint32_t temp = 0;
	/* Default system clock is 80MHz */
	for(index = (8 * num); index != 0; index--);
}
/*******************************************************************************
Noi Dung    :   Config IO for LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
void LCD_GPIOConfiguration(void)//for 
{

}

/*******************************************************************************
Noi Dung    :   Gui tin hieu Enable den LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/

 void LCD_Enable(void)
{
 LCD_EN(1);
 delay_us(1);
 LCD_EN(0);
 delay_us(50); 
}

 /*******************************************************************************
Noi Dung    :   Gui 4 bit du lieu den LCD.
Tham Bien   :   Data: 4 bit thap cua Data chua 4 bit du lieu can gui.
Tra Ve      :   Khong.
********************************************************************************/

 void LCD_Send4Bit( char  Data )
{
 LCD_D4((Data>>0)&0x01);
 LCD_D5((Data>>1)&0x01);
 LCD_D6((Data>>2)&0x01);
 LCD_D7((Data>>3)&0x01);
}
 /*******************************************************************************
Noi Dung    :   Gui 1 byte du lieu den LCD.
Tham Bien   :   command: byte du lieu can gui.
Tra Ve      :   Khong.
********************************************************************************/

 void LCD_SendCommand (char  command )
{
LCD_Send4Bit  ( command >>4 );   /* Gui 4 bit cao */
LCD_Enable () ;
LCD_Send4Bit  ( command  );      /* Gui 4 bit thap*/
LCD_Enable () ;
}
 /*******************************************************************************
Noi Dung    :   Khoi tao LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
 void LCD_Init ( void )
{	
	
	LCD_off;
	LCD_RW(0);
	LCD_Send4Bit(0x00);
	//delay_ms(40);
	HAL_Delay(40);
	LCD_RS(0);
	
	LCD_Send4Bit(0x03);
	LCD_Enable();
	//delay_ms(5);
	HAL_Delay(5);
	LCD_Enable();
	delay_us(100);
	LCD_Enable();
	LCD_Send4Bit(0x02);
	LCD_Enable();
	LCD_SendCommand( 0x28 );         // giao thuc 4 bit, hien thi 2 hang, ki tu 5x8
	LCD_SendCommand( 0x0c);          // cho phep hien thi man hinh
	LCD_SendCommand( 0x06 );         // tang ID, khong dich khung hinh
	LCD_Clear();                     // xoa toan bo khung hinh
	LCD_on;
}
 /*******************************************************************************
Noi Dung    :   Thiet lap vi tri con tro LCD.
Tham Bien   :   x: vi tri cot cua con tro. x = 0 - 15.
                y: vi tri hang cua con tro. y= 0,1.
Tra Ve      :   Khong.
********************************************************************************/
void LCD_Gotoxy(char  y, char  x)
{
  uint8_t  address;
  if(!y)
  address = (0x80+x);
  else
  address = (0xC0+x);
  //delay_ms(1);
	HAL_Delay(1);
  LCD_SendCommand(address);
  //delay_ms(5);
	HAL_Delay(5);
}
 /*******************************************************************************
Noi Dung    :   Xoa noi dung hien thi tren LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
void LCD_Clear(void)
{
  LCD_SendCommand(0x01);  
  //delay_ms(5);
	HAL_Delay(5);
}
 /*******************************************************************************
Noi Dung    :   Viet 1 ki tu len LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
 void LCD_Putc ( char  Data )
{
 LCD_RS(1);
 LCD_SendCommand( Data );
 LCD_RS(0);
}
 /*******************************************************************************
Noi Dung    :   Viet 1 chuoi ki tu len LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
void LCD_Puts (char *s)
{
   while (*s)
   {
      LCD_Putc(*s);
      s++;
   }
}
int lcdprintf(const char *fmt,...)
{
	char str[50] = {0};
	uint16_t a = 0;
	__va_list arg;
 	va_start(arg,fmt);
	a=vsprintf(str,fmt,arg);
	va_end(arg);
	LCD_Puts((char *)str);
	return a;
}

/*********************************Ket thuc file*********************************/
