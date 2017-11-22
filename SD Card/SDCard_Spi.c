
#include "fsl_dspi.h"


#define SDC_DSPI_BASEADDR SPI0
#define SDC_DSPI_CLK_SRC DSPI0_CLK_SRC
#define SDC_DSPI_IRQ SPI0_IRQn
#define SDC_DSPI_PCS kDSPI_Pcs0
#define SDC_DSPI_IRQHandler SPI0_IRQHandler

#define TRANSFER_SIZE 1U                //256U        /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 200000U  //000U /*! Transfer baudrate - 500k */


uint8_t SDCSpiRxData[TRANSFER_SIZE] = {0U};
uint8_t SDCSpiTxData[TRANSFER_SIZE] = {0U};

volatile uint32_t SDCSpiTxCount;
volatile uint32_t SDCSpiRxCount;
volatile uint32_t SDCSpiCommand;
uint32_t SDCSpiFifoSize;

//dspi_master_handle_t g_m_handle;
//volatile bool isTransferCompleted = false;



volatile bool SDC_SpiTxComplete_Flag = true;


#define SDC_TRANSFER_BAUDRATE 5500000

void int_SDC_SPI (void)
{
    uint32_t srcClock_Hz;
//    uint32_t errorCount;
    dspi_master_config_t SDCSpiConfig;

    /* SDCSpi config */
    SDCSpiConfig.whichCtar = kDSPI_Ctar1;
    SDCSpiConfig.ctarConfig.baudRate = SDC_TRANSFER_BAUDRATE;
    SDCSpiConfig.ctarConfig.bitsPerFrame = 8;
    SDCSpiConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;       //kDSPI_ClockPolarityActiveLow;       //kDSPI_ClockPolarityActiveHigh;
    SDCSpiConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;  //kDSPI_ClockPhaseSecondEdge
    SDCSpiConfig.ctarConfig.direction = kDSPI_MsbFirst;
    SDCSpiConfig.ctarConfig.pcsToSckDelayInNanoSec =   100000000U / SDC_TRANSFER_BAUDRATE;
    SDCSpiConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 10000000U / SDC_TRANSFER_BAUDRATE;
    SDCSpiConfig.ctarConfig.betweenTransferDelayInNanoSec = 10000000U / SDC_TRANSFER_BAUDRATE;

    SDCSpiConfig.whichPcs = SDC_DSPI_PCS;
    SDCSpiConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    SDCSpiConfig.enableContinuousSCK = false;
    SDCSpiConfig.enableRxFifoOverWrite = false;
    SDCSpiConfig.enableModifiedTimingFormat = false;
    SDCSpiConfig.samplePoint = kDSPI_SckToSin0Clock;    // kDSPI_SckToSin1Clock;

    srcClock_Hz = CLOCK_GetFreq(SDC_DSPI_CLK_SRC);
    DSPI_MasterInit(SDC_DSPI_BASEADDR, &SDCSpiConfig, srcClock_Hz);
    EnableIRQ(SDC_DSPI_IRQ);
}


void SDC_DSPI_IRQHandler(void)
{
    if (SDCSpiRxCount < TRANSFER_SIZE)
    {
        while (DSPI_GetStatusFlags(SDC_DSPI_BASEADDR) & kDSPI_RxFifoDrainRequestFlag)
        {
            SDCSpiRxData[SDCSpiRxCount] = DSPI_ReadData(SDC_DSPI_BASEADDR);
            ++SDCSpiRxCount;

            DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_RxFifoDrainRequestFlag);

            if (SDCSpiRxCount == TRANSFER_SIZE)
            {
                break;
            }
        }
    }

    if (SDCSpiTxCount < TRANSFER_SIZE)
    {
        while ((DSPI_GetStatusFlags(SDC_DSPI_BASEADDR) & kDSPI_TxFifoFillRequestFlag) &&
               ((SDCSpiTxCount - SDCSpiRxCount) < SDCSpiFifoSize))
        {
            if (SDCSpiTxCount < TRANSFER_SIZE)
            {
                SDC_DSPI_BASEADDR->PUSHR = SDCSpiCommand | SDCSpiTxData[SDCSpiTxCount];
                ++SDCSpiTxCount;
            }
            else
            {
                break;
            }

            /* Try to clear the TFFF; if the TX FIFO is full this will clear */
            DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_TxFifoFillRequestFlag);
        }
    }

    /* Check if we're done with this transfer.*/
    if ((SDCSpiTxCount == TRANSFER_SIZE) && (SDCSpiRxCount == TRANSFER_SIZE))
    {
        /* Complete the transfer and disable the interrupts */
        DSPI_DisableInterrupts(SDC_DSPI_BASEADDR,
                               kDSPI_RxFifoDrainRequestInterruptEnable | kDSPI_TxFifoFillRequestInterruptEnable);
        SDC_SpiTxComplete_Flag = true;
    }
}

void SDC_SPI_Tx (uint8_t * SDCSpiTxData , uint8_t * SDCSpiRxData, uint8_t SDC_SPI_TXData_Len)
{
  
   /* Start SDCSpi transfer*/
    dspi_command_data_config_t commandData;
    commandData.isPcsContinuous = false;
    commandData.whichCtar = kDSPI_Ctar1;
    commandData.whichPcs = SDC_DSPI_PCS;
    commandData.isEndOfQueue = true;    //false;
    commandData.clearTransferCount = false;

    SDCSpiCommand = DSPI_MasterGetFormattedCommand(&commandData);

    SDCSpiFifoSize = FSL_FEATURE_DSPI_FIFO_SIZEn(SDC_DSPI_BASEADDR);
    SDCSpiTxCount = 0;
    SDCSpiRxCount = 0;

    DSPI_StopTransfer(SDC_DSPI_BASEADDR);
    DSPI_FlushFifo(SDC_DSPI_BASEADDR, true, true);
    DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_AllStatusFlag);
    DSPI_StartTransfer(SDC_DSPI_BASEADDR);

            SDC_SpiTxComplete_Flag = false;
    /*Fill up the SDCSpi Tx data*/
    while (DSPI_GetStatusFlags(SDC_DSPI_BASEADDR) & kDSPI_TxFifoFillRequestFlag)
    {
        if (SDCSpiTxCount < SDC_SPI_TXData_Len)
        {
            DSPI_MasterWriteData(SDC_DSPI_BASEADDR, &commandData, SDCSpiTxData[SDCSpiTxCount]);
            ++SDCSpiTxCount;
        }
        else
        {
            break;
        }

        /* Try to clear the TFFF; if the TX FIFO is full this will clear */
        DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_TxFifoFillRequestFlag);
    }

    /*Enable SDCSpi RX interrupt*/
    DSPI_EnableInterrupts(SDC_DSPI_BASEADDR, kDSPI_RxFifoDrainRequestInterruptEnable);

    /* Wait slave received all data. */
  //  while (!isTransferCompleted)
  //  {
   // }  
}

