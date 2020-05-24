/*--------------------------------------------------------------------------------------------------

  Name         :  NokiaLCD.h

  Description  :  Header file for Nokia 84x48 graphic LCD driver.

  Author       :  2003-03-08 - Louis Frigon.

  History      :  2003-03-08 - First release.

--------------------------------------------------------------------------------------------------*/
#ifndef _NOKIALCD_H_

#define _NOKIALCD_H_

/*--------------------------------------------------------------------------------------------------
                                  General purpose constants
--------------------------------------------------------------------------------------------------*/
//#define NULL                       0
#define FALSE                      0
#define TRUE                       1

#define LCD_X_RES                  84
#define LCD_Y_RES                  48

//  Mega8 port B pinout for LCD.
#define LCD_DC_PIN                 0x02  //  PB1
#define LCD_CE_PIN                 0x01  //  PB0
#define SPI_MOSI_PIN               0x08  //  PB3
#define LCD_RST_PIN                0x80  //  PD7
#define SPI_CLK_PIN                0x20  //  PB5
//#define LCD_POWER				   0x01  //  PB0

#define LCD_CACHE_SIZE             ((LCD_X_RES * LCD_Y_RES) / 8)


typedef enum
{
    LCD_CMD  = 0,
    LCD_DATA = 1

} LcdCmdData;

typedef enum
{
    PIXEL_OFF =  0,
    PIXEL_ON  =  1,
    PIXEL_XOR =  2

} LcdPixelMode;

typedef enum
{
    FONT_1X = 1,
    FONT_2X = 2

} LcdFontSize;

/*--------------------------------------------------------------------------------------------------
                                 Public function prototypes
--------------------------------------------------------------------------------------------------*/
void LcdInit       ( void );
void LcdPower 	   ( char stat );
void LcdContrast   ( char contrast );
void LcdClear      ( void );
void LcdUpdate     ( void );
void LcdGotoXY     ( char x, char y );
void LcdChr        ( LcdFontSize size, char ch );
void LcdStr        ( LcdFontSize size, char *dataPtr );
void LcdStrConst   ( LcdFontSize size, const char *dataPtr );
void LcdPixel      ( char x, char y, LcdPixelMode mode );
void LcdLine       ( char x1, char y1, char x2, char y2, LcdPixelMode mode );

#endif  //  _NOKIALCD_H_
/*--------------------------------------------------------------------------------------------------
                                         End of file.
--------------------------------------------------------------------------------------------------*/

