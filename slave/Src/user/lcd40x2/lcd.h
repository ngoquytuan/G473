
#ifndef __LCD__H
#define __LCD__H

//LCD pin mapping for GPStest
//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
#define   LCD_RS(state)       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, state)
#define   LCD_RW(state)       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, state)
#define   LCD_EN(state)       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, state)

#define   LCD_D4(state)       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, state)
#define   LCD_D5(state)       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, state)
#define   LCD_D6(state)       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state)
#define   LCD_D7(state)       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, state)
#define		LCD_on							;//GPIO_PinWrite(GPIOC, 8, 1);
#define		LCD_off							;//GPIO_PinWrite(GPIOC, 8, 0);

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

void LCD_GPIOConfiguration(void);

/*******************************************************************************
Noi Dung    :   Gui tin hieu Enable den LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
 void LCD_Enable(void);
 /*******************************************************************************
Noi Dung    :   Gui 4 bit du lieu den LCD.
Tham Bien   :   Data: 4 bit thap cua Data chua 4 bit du lieu can gui.
Tra Ve      :   Khong.
********************************************************************************/

 void LCD_Send4Bit( char  Data );
 /*******************************************************************************
Noi Dung    :   Gui 1 byte du lieu den LCD.
Tham Bien   :   command: byte du lieu can gui.
Tra Ve      :   Khong.
********************************************************************************/
 void LCD_SendCommand (char  command );
 /*******************************************************************************
Noi Dung    :   Khoi tao LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
 void LCD_Init ( void );
 /*******************************************************************************
Noi Dung    :   Thiet lap vi tri con tro LCD.
Tham Bien   :   x: vi tri cot cua con tro. x = 0 - 15.
                y: vi tri hang cua con tro. y= 0,1.
Tra Ve      :   Khong.
********************************************************************************/
void LCD_Gotoxy(char  y, char  x);
 /*******************************************************************************
Noi Dung    :   Xoa noi dung hien thi tren LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
void LCD_Clear(void);
 /*******************************************************************************
Noi Dung    :   Viet 1 ki tu len LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
 void LCD_Putc ( char  Data );
 /*******************************************************************************
Noi Dung    :   Viet 1 chuoi ki tu len LCD.
Tham Bien   :   Khong.
Tra Ve      :   Khong.
********************************************************************************/
void LCD_Puts (char *s);
 /*******************************************************************************
Noi Dung    :   printf len LCD.
Tham Bien   :   giong nhu printf.
Tra Ve      :   so ki tu duoc in len man hinh LCD.
********************************************************************************/
int lcdprintf(const char *fmt,...);
#endif

