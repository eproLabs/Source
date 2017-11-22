#ifndef _SDCARD_H_
#define _SDCARD_H_

#define SUCCESS 0
#define FAILURE 1


extern void int_SDC_SPI (void);
extern void SDC_Timer_handler(void);
extern int CheckSDCard (void);
extern bool CreateTextFile (char *filename);
extern bool WriteTextFile (char *filename, unsigned char *WriteString);

unsigned char storedata[150];



#endif