#include <stdio.h>
#include <math.h>

#define bool _Bool
#define true 1
#define false 0


 static void LCD_spitx (unsigned char);
 static unsigned char LCD_spirx (void);
//extern void spiinit (void);

 //void LCD_spitx ((uint8_t *) LCDSpiTxData, (uint8_t *) LCDSpiRxData, uint8_t LCD_SPI_TXData_Len)
 //void LCD_spirx ((uint8_t *) LCDSpiTxData, (uint8_t *) LCDSpiRxData, uint8_t LCD_SPI_TXData_Len)  

extern void lcd_command (unsigned char);
extern void lcd_ready (void);
extern void lcd_datas(unsigned char *);


extern void tostring (unsigned int, unsigned char *);
unsigned volatile int lcd_delay_reg;

void msdelay_LCD_prog (unsigned int);
void mswait_LCD_prog (void);

void msdelay1_LCD_prog (unsigned int);
void mswait1_LCD_prog (void);
#define TRANSFER_SIZE 10
char LCD_SPI_TXData_Len = 1;
 
extern char LCDSpiRxData[TRANSFER_SIZE];
extern char LCDSpiTxData[TRANSFER_SIZE];
 
extern void init_LCD_SPI (void);
extern void LCD_SPI_Tx (char * LCDSpiTxData , char * LCDSpiRxData, char LCD_SPI_TXData_Len);
extern volatile bool LCD_SpiTxComplete_Flag;

#define commandchar  0xfe
#define datachar 0xfd
#define statuschar 0xfc

#define lcdbusy 0xff
#define lcdfree 0x01
 
 static void LCD_spitx (unsigned char LCDSpiTX_singleData)
 {
   while(LCD_SpiTxComplete_Flag == false);
   //	msdelay_LCD_prog(1);

   LCDSpiTxData[0] = LCDSpiTX_singleData;
   LCD_SPI_TXData_Len = 1;
   LCD_SPI_Tx (LCDSpiTxData, LCDSpiRxData, LCD_SPI_TXData_Len);
 }
 
static unsigned char LCD_spirx (void)
 {
   while(LCD_SpiTxComplete_Flag == false);
  // 	msdelay_LCD_prog(1);

   LCDSpiTxData[0] = 0xff;
   LCD_SPI_TXData_Len = 1;
   LCD_SPI_Tx (LCDSpiTxData, LCDSpiRxData,LCD_SPI_TXData_Len);
   return (LCDSpiRxData[0]);
 }
 

 void lcd_Spi_Init (void)
 {
      init_LCD_SPI();
 }
 
void lcd_init (void)
{
  	msdelay_LCD_prog(1);
        	msdelay_LCD_prog(1);
                	msdelay_LCD_prog(1);
                        	msdelay_LCD_prog(1);
                                
	LCD_spitx ('+');  					// this does not go to the LCD but goes to the LCD controller
	msdelay_LCD_prog(1);
	LCD_spitx ('E');  					// Acts acts a initialisation character for the LCD controller
	msdelay_LCD_prog(1);
//	lcd_command (0x3c);    			// These initialisations are already done by LCD controller itself 
//	lcd_command (0x0e);
//	lcd_command (0x06);
	lcd_command (0x01);
}

void lcd_command (unsigned char data)
{
	lcd_ready ();
	LCD_spitx(commandchar);
	msdelay_LCD_prog(1);
	LCD_spitx(data);
	msdelay_LCD_prog(1);
	msdelay1_LCD_prog(2);
}

void lcd_data (unsigned char data)
{
	lcd_ready ();
	LCD_spitx(datachar);
	msdelay_LCD_prog(1);
	LCD_spitx(data);
	msdelay_LCD_prog(1);
}

void lcd_ready(void)
{
	unsigned int volatile timedelayreg1_LCD;
	LCD_spitx(statuschar);
	msdelay_LCD_prog(1);
	do
		{
		msdelay_LCD_prog(1);
		}while(LCD_spirx() != lcdfree);
	msdelay_LCD_prog(1);

}

void lcd_datas(unsigned char *s)
{
//	 lcd_ready ();
//	LCD_spics = 0;
//		LCD_spitx(datachar);
//	LCD_spics = 1;
//	msdelay_LCD_prog(1);
//	while(*s)
//		{
//		lcd_ready ();
//	LCD_spics = 0;
//		LCD_spitx(*s++);
//	LCD_spics = 1;
//	msdelay_LCD_prog(1);
//		}
	while(*s)
		{
			lcd_data(*s++);
		}
}

	


void tostring (unsigned int tostring_intvalue, unsigned char *tostring_stringvalue)
{

static volatile unsigned int tostring_tenthou,tostring_thou,tostring_hun,tostring_ten,tostring_one;

tostring_tenthou = tostring_intvalue/10000;
tostring_stringvalue[0] = tostring_tenthou +0x30;

tostring_tenthou = tostring_tenthou*10000;
tostring_thou = (tostring_intvalue-tostring_tenthou)/1000;
tostring_stringvalue[1] = tostring_thou +0x30;

tostring_thou =tostring_thou*1000;
tostring_hun= (tostring_intvalue-(tostring_tenthou+tostring_thou))/100;
tostring_stringvalue[2] = tostring_hun +0x30;

tostring_hun = tostring_hun*100;
tostring_ten = (tostring_intvalue-(tostring_tenthou+tostring_thou+tostring_hun))/10;
tostring_stringvalue[3] = tostring_ten+0x30;

tostring_ten = tostring_ten*10;
tostring_one =  tostring_intvalue-(tostring_tenthou+tostring_thou+tostring_hun+tostring_ten);
tostring_stringvalue[4] =tostring_one +0x30;

tostring_stringvalue[5] = 0x00;
}



void msdelay_LCD_prog (unsigned int msdel2)
{
unsigned volatile int timedelayreg1_LCD;
	unsigned volatile int i2;
	for (i2 = 1; i2<=10000; i2++);
	mswait_LCD_prog ();
//	i += 2000;		// a dummy instruction for correct time delay;
//	i +=20;			// for 64MHz

			timedelayreg1_LCD = 1000;
			timedelayreg1_LCD = timedelayreg1_LCD + 5678;
}
void mswait_LCD_prog (void)
{
	unsigned volatile int j2;
	for (j2= 1; j2<=10; j2++);
}



void msdelay1_LCD_prog (unsigned int msdel2)
{
unsigned volatile int timedelayreg1_LCD;
	unsigned volatile int i21;
	for (i21 = 1; i21<=msdel2; i21++)
	mswait1_LCD_prog ();
//	i += 2000;		// a dummy instruction for correct time delay;
//	i +=20;			// for 64MHz

			timedelayreg1_LCD = 1000;
			timedelayreg1_LCD = timedelayreg1_LCD + 5678;
}
void mswait1_LCD_prog (void)
{
	unsigned volatile int j21;
	for (j21= 1; j21<=7; j21++);
}