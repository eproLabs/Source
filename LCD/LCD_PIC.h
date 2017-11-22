#ifndef _LCD_PIC_H_
#define _LCD_PIC_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */
  
void lcd_init (void);
void lcd_command (unsigned char);
void lcd_data (unsigned char);
void lcd_datas(unsigned char *);
void lcd_Spi_Init(void);

void tostring (unsigned int, unsigned char *);

void LCD_SPI_Tx (uint8_t * , uint8_t * , uint8_t);
void init_LCD_SPI (void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */

