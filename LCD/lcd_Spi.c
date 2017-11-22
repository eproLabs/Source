
#include "fsl_dspi.h"


#define LCD_DSPI_BASEADDR SPI1
#define LCD_DSPI_CLK_SRC DSPI1_CLK_SRC
#define LCD_DSPI_IRQ SPI1_IRQn
#define LCD_DSPI_PCS kDSPI_Pcs0
#define LCD_DSPI_IRQHandler SPI1_IRQHandler

#define TRANSFER_SIZE 1U                //256U        /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 550000  //000U /*! Transfer baudrate - 500k */


uint8_t LCDSpiRxData[TRANSFER_SIZE] = {0U};
uint8_t LCDSpiTxData[TRANSFER_SIZE] = {0U};

volatile uint32_t LCDSpiTxCount;
volatile uint32_t LCDSpiRxCount;
volatile uint32_t LCDSpiCommand;
uint32_t LCDSpiFifoSize;

//dspi_master_handle_t g_m_handle;
//volatile bool isTransferCompleted = false;



volatile bool LCD_SpiTxComplete_Flag = true;



void init_LCD_SPI (void)
{
    uint32_t srcClock_Hz;
   /// uint32_t errorCount;
    dspi_master_config_t LCDSpiConfig;

    /* LCDSpi config */
    LCDSpiConfig.whichCtar = kDSPI_Ctar0;
    LCDSpiConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
    LCDSpiConfig.ctarConfig.bitsPerFrame = 8;
    LCDSpiConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveLow;       //kDSPI_ClockPolarityActiveHigh;
    LCDSpiConfig.ctarConfig.cpha = kDSPI_ClockPhaseSecondEdge;   //kDSPI_ClockPhaseFirstEdge;
    LCDSpiConfig.ctarConfig.direction = kDSPI_MsbFirst;
    LCDSpiConfig.ctarConfig.pcsToSckDelayInNanoSec = 10000000000U / TRANSFER_BAUDRATE;
    LCDSpiConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    LCDSpiConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;

    LCDSpiConfig.whichPcs = LCD_DSPI_PCS;
    LCDSpiConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    LCDSpiConfig.enableContinuousSCK = false;
    LCDSpiConfig.enableRxFifoOverWrite = false;
    LCDSpiConfig.enableModifiedTimingFormat = false;
    LCDSpiConfig.samplePoint = kDSPI_SckToSin1Clock;    // kDSPI_SckToSin0Clock;

    srcClock_Hz = CLOCK_GetFreq(LCD_DSPI_CLK_SRC);
    DSPI_MasterInit(LCD_DSPI_BASEADDR, &LCDSpiConfig, srcClock_Hz);
        EnableIRQ(LCD_DSPI_IRQ);
}


void LCD_DSPI_IRQHandler(void)
{
    if (LCDSpiRxCount < TRANSFER_SIZE)
    {
        while (DSPI_GetStatusFlags(LCD_DSPI_BASEADDR) & kDSPI_RxFifoDrainRequestFlag)
        {
            LCDSpiRxData[LCDSpiRxCount] = DSPI_ReadData(LCD_DSPI_BASEADDR);
            ++LCDSpiRxCount;

            DSPI_ClearStatusFlags(LCD_DSPI_BASEADDR, kDSPI_RxFifoDrainRequestFlag);

            if (LCDSpiRxCount == TRANSFER_SIZE)
            {
                break;
            }
        }
    }

    if (LCDSpiTxCount < TRANSFER_SIZE)
    {
        while ((DSPI_GetStatusFlags(LCD_DSPI_BASEADDR) & kDSPI_TxFifoFillRequestFlag) &&
               ((LCDSpiTxCount - LCDSpiRxCount) < LCDSpiFifoSize))
        {
            if (LCDSpiTxCount < TRANSFER_SIZE)
            {
                LCD_DSPI_BASEADDR->PUSHR = LCDSpiCommand | LCDSpiTxData[LCDSpiTxCount];
                ++LCDSpiTxCount;
            }
            else
            {
                break;
            }

            /* Try to clear the TFFF; if the TX FIFO is full this will clear */
            DSPI_ClearStatusFlags(LCD_DSPI_BASEADDR, kDSPI_TxFifoFillRequestFlag);
        }
    }

    /* Check if we're done with this transfer.*/
    if ((LCDSpiTxCount == TRANSFER_SIZE) && (LCDSpiRxCount == TRANSFER_SIZE))
    {
        /* Complete the transfer and disable the interrupts */
        DSPI_DisableInterrupts(LCD_DSPI_BASEADDR,
                               kDSPI_RxFifoDrainRequestInterruptEnable | kDSPI_TxFifoFillRequestInterruptEnable);
        LCD_SpiTxComplete_Flag = true;
    }
}

void LCD_SPI_Tx (uint8_t * LCDSpiTxData , uint8_t * LCDSpiRxData, uint8_t LCD_SPI_TXData_Len)
{
  
   /* Start LCDSpi transfer*/
    dspi_command_data_config_t commandData;
    commandData.isPcsContinuous = false;
    commandData.whichCtar = kDSPI_Ctar0;
    commandData.whichPcs = LCD_DSPI_PCS;
    commandData.isEndOfQueue = true;    //false;
    commandData.clearTransferCount = false;

    LCDSpiCommand = DSPI_MasterGetFormattedCommand(&commandData);

    LCDSpiFifoSize = FSL_FEATURE_DSPI_FIFO_SIZEn(LCD_DSPI_BASEADDR);
    LCDSpiTxCount = 0;
    LCDSpiRxCount = 0;

    DSPI_StopTransfer(LCD_DSPI_BASEADDR);
    DSPI_FlushFifo(LCD_DSPI_BASEADDR, true, true);
    DSPI_ClearStatusFlags(LCD_DSPI_BASEADDR, kDSPI_AllStatusFlag);
    DSPI_StartTransfer(LCD_DSPI_BASEADDR);

    /*Fill up the LCDSpi Tx data*/
    while (DSPI_GetStatusFlags(LCD_DSPI_BASEADDR) & kDSPI_TxFifoFillRequestFlag)
    {
        if (LCDSpiTxCount < LCD_SPI_TXData_Len)
        {
            DSPI_MasterWriteData(LCD_DSPI_BASEADDR, &commandData, LCDSpiTxData[LCDSpiTxCount]);
            ++LCDSpiTxCount;
        }
        else
        {
            break;
        }

        /* Try to clear the TFFF; if the TX FIFO is full this will clear */
        DSPI_ClearStatusFlags(LCD_DSPI_BASEADDR, kDSPI_TxFifoFillRequestFlag);
    }

    /*Enable LCDSpi RX interrupt*/
    DSPI_EnableInterrupts(LCD_DSPI_BASEADDR, kDSPI_RxFifoDrainRequestInterruptEnable);

    /* Wait slave received all data. */
  //  while (!isTransferCompleted)
  //  {
   // }  
}