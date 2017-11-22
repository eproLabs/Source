#ifndef SDC_TXRX_SPI_H		//Do only once the first time this file is used
#define	SDC_TXRX_SPI_H	

extern void SDC_spitx (unsigned char);
extern unsigned char SDC_spirx (void);

extern volatile bool SDC_SpiTxComplete_Flag;

extern void msdelay_SDC_prog (unsigned int msdel2);

#endif