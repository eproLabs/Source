


#define bool _Bool
#define true 1
#define false 0
#define TRANSFER_SIZE 10
char SDC_SPI_TXData_Len = 1;
 
extern  char SDCSpiRxData[TRANSFER_SIZE];
extern char SDCSpiTxData[TRANSFER_SIZE];

extern void init_SDC_SPI (void);
extern void SDC_SPI_Tx (char * SDCSpiTxData , char * SDCSpiRxData, char SDC_SPI_TXData_Len);
extern volatile bool SDC_SpiTxComplete_Flag;



void SDC_spitx (unsigned char);
unsigned char SDC_spirx (void);


void mswait_SDC_prog (void);
void msdelay_SDC_prog (unsigned int msdel2);

void SDC_spitx (unsigned char SDCSpiTX_singleData)
 {
   while(SDC_SpiTxComplete_Flag == false);
   //	msdelay_SDC_prog(1);

   SDCSpiTxData[0] = SDCSpiTX_singleData;
   SDC_SPI_TXData_Len = 1;
   SDC_SPI_Tx (SDCSpiTxData, SDCSpiRxData, SDC_SPI_TXData_Len);
 //     while(SDC_SpiTxComplete_Flag == false);
 }
 
unsigned char SDC_spirx (void)
 {
   while(SDC_SpiTxComplete_Flag == false);
  // 	msdelay_SDC_prog(1);

   SDCSpiTxData[0] = 0xff;
   SDC_SPI_TXData_Len = 1;
   SDC_SPI_Tx (SDCSpiTxData, SDCSpiRxData,SDC_SPI_TXData_Len);
   while(SDC_SpiTxComplete_Flag == false);
   return (SDCSpiRxData[0]);
 }


void msdelay_SDC_prog (unsigned int msdel2)
{
unsigned volatile int timedelayreg1_SDC;
	unsigned volatile int i2;
	for (i2 = 1; i2<=10000; i2++);
	mswait_SDC_prog ();
//	i += 2000;		// a dummy instruction for correct time delay;
//	i +=20;			// for 64MHz

			timedelayreg1_SDC = 1000;
			timedelayreg1_SDC = timedelayreg1_SDC + 5678;
}
void mswait_SDC_prog (void)
{
	unsigned volatile int j2;
	for (j2= 1; j2<=10; j2++);
}