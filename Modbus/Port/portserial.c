/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portserial.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include <stdlib.h>
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "fsl_uart.h"
#include "board.h"

/* ----------------------- Defines ------------------------------------------*/

#define MODBUS_UART UART0
#define MODBUS_UART_CLKSRC UART0_CLK_SRC
#define MODBUS_UART_IRQn UART0_RX_TX_IRQn
#define MODBUS_UART_IRQHandler UART0_RX_TX_IRQHandler
//#define MODBUS_UART_BAUDRATE 115200

#define USART0_ENABLED          ( 1 )
#define USART0_IDX              ( 0 )

#define USART1_ENABLED          ( 1 )
#define USART1_IDX              ( USART0_IDX + USART0_ENABLED * 1 )

#define USART_IDX_LAST          ( USART1_IDX )

#define USART_INVALID_PORT      ( 0xFF )
#define USART_NOT_RE_IDX        ( 3 )
#define USART_DE_IDX            ( 4 )

#define USARTNotREPin
   
#define USARTNotREPin_Set (GPIO_SetPinsOutput(BOARD_RS485_GPIO, 1U << BOARD_RS485RE_GPIO_PIN))
#define USARTNotREPin_Clear (GPIO_ClearPinsOutput(BOARD_RS485_GPIO, 1U << BOARD_RS485RE_GPIO_PIN))

#define USARTDEPin_Set (GPIO_SetPinsOutput(BOARD_RS485_GPIO, 1U << BOARD_RS485DE_GPIO_PIN))
#define USARTDEPin_Clear (GPIO_ClearPinsOutput(BOARD_RS485_GPIO, 1U << BOARD_RS485DE_GPIO_PIN))

/* ----------------------- Static variables ---------------------------------*/

//#if USART1_ENABLED == 1
//const Pin       xUSART0Pins[] = {
//    PIN_USART0_TXD,
//    PIN_USART0_RXD
//};
//#endif
//
//#if USART1_ENABLED == 1
//const Pin       xUSART1NotREPin = { 1 << 25, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };
//const Pin       xUSART1DEPin = { 1 << 24, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };
//
//const Pin       xUSART1Pins[] = {
//    PIN_USART1_TXD,
//    PIN_USART1_RXD,
//    {1 << 23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
//};
//#endif
//
//const struct xUSARTHWMappings_t
//{
//    Usart          *pUsart;
//    unsigned int    xUSARTID;
//    IRQn_Type       xUSARTIrq;
//    const Pin      *USARTNotREPin;
//    const Pin      *USARTDEPin;
//    const Pin      *xUSARTPins;
//    uint32_t        xUSARTPinsCnt;
//
//
//} xUSARTHWMappings[] =
//{
//#if USART0_ENABLED == 1
//    {
//    USART0, ID_USART0, USART0_IRQn, NULL, NULL, &xUSART0Pins[0], PIO_LISTSIZE( xUSART0Pins )},
//#endif
//#if USART1_ENABLED == 1
//    {
//    USART1, ID_USART1, USART1_IRQn, &xUSART1NotREPin, &xUSART1DEPin, &xUSART1Pins[0], PIO_LISTSIZE( xUSART1Pins )},
//#endif
//};

static UCHAR    ucUsedPort = USART_INVALID_PORT;
static uart_config_t mod_config;
bool parityError = false;

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{

    if( xRxEnable )
    {
      //  USART_SetReceiverEnabled( xUSARTHWMappings[ucUsedPort].pUsart, 1 );
      //  USART_EnableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IDR_RXRDY );
        UART_EnableRx(MODBUS_UART, true);
        UART_EnableInterrupts(MODBUS_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable );
    }
    else
    {
       // USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IDR_RXRDY );
      //  USART_SetReceiverEnabled( xUSARTHWMappings[ucUsedPort].pUsart, 0 );
        UART_EnableRx(MODBUS_UART, false);
        UART_DisableInterrupts(MODBUS_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable );
    }

    if( xTxEnable )
    {
//        if( NULL != xUSARTHWMappings[ucUsedPort].USARTNotREPin )
//        {
//            PIO_Set( xUSARTHWMappings[ucUsedPort].USARTNotREPin );
//        }
//        if( NULL != xUSARTHWMappings[ucUsedPort].USARTDEPin )
//        {
//            PIO_Set( xUSARTHWMappings[ucUsedPort].USARTDEPin );
//        }
//        USART_SetTransmitterEnabled( xUSARTHWMappings[ucUsedPort].pUsart, 1 );
//        USART_EnableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXRDY );           //kUART_TxDataRegEmptyInterruptEnable
//        USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXEMPTY );       //kUART_TransmissionCompleteInterruptEnable
      
        USARTNotREPin_Set;
        USARTDEPin_Set;
        UART_EnableTx(MODBUS_UART, true);
        UART_DisableInterrupts(MODBUS_UART, kUART_TransmissionCompleteInterruptEnable);
        UART_EnableInterrupts(MODBUS_UART, kUART_TxDataRegEmptyInterruptEnable);
    }
    else
    {
      //  USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IDR_TXRDY );
      //  USART_EnableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXEMPTY );
        UART_DisableInterrupts(MODBUS_UART, kUART_TxDataRegEmptyInterruptEnable);
        UART_EnableInterrupts(MODBUS_UART, kUART_TransmissionCompleteInterruptEnable);
    }
}

BOOL
xMBPortSerialInit(ULONG ulBaudRate, UCHAR ucDataBits, uart_parity_mode_t eParity )
{
    BOOL            bStatus = FALSE;
   // uint32_t        uiMode = US_MR_USART_MODE_NORMAL;
    
      /*
     * mod_config.baudRate_Bps = 115200U;
     * mod_config.parityMode = kUART_ParityDisabled;
     * mod_config.stopBitCount = kUART_OneStopBit;
     * mod_config.txFifoWatermark = 0;
     * mod_config.rxFifoWatermark = 1;
     * mod_config.enableTx = false;
     * mod_config.enableRx = false;
     */
    UART_GetDefaultConfig(&mod_config);
    mod_config.baudRate_Bps = ulBaudRate;
    mod_config.parityMode = eParity;
    mod_config.enableTx = false;
    mod_config.enableRx = false;

   
    
//    if( ( ucPORT <= USART_IDX_LAST ) )
//    {
//        bStatus = TRUE;
//        switch ( eParity )
//        {
//        case MB_PAR_NONE:
//          //  uiMode |= US_MR_PAR_NONE | US_MR_NBSTOP_2_BIT;
//            mod_config.stopBitCount = kUART_TwoStopBit;
//            break;
//        case MB_PAR_ODD:
//        //    uiMode |= US_MR_PAR_ODD | US_MR_NBSTOP_1_BIT;
//            mod_config.stopBitCount = kUART_OneStopBit;
//            break;
//        case MB_PAR_EVEN:
//         //   uiMode |= US_MR_PAR_EVEN | US_MR_NBSTOP_1_BIT;
//            mod_config.stopBitCount = kUART_OneStopBit;
//            break;
//        default:
//            bStatus = FALSE;
//            break;
//        }
  
        bStatus = TRUE;
        switch ( ucDataBits )                           // This is not applicable.. for our controller.. we will be using 8 bits of transmission
        {
        case 8:
          mod_config.BitCount = 8;              // RTU  Mode
    //        uiMode |= US_MR_CHRL_8_BITS;
            break;
        case 7:
           mod_config.BitCount = 7;             //ASCII Mode
   //         uiMode |= US_MR_CHRL_7_BITS;
            break;
        default:
            bStatus = FALSE;
        }

        if( TRUE == bStatus )
        {
            ucUsedPort = 1;

          //   NVIC_DisableIRQ( xUSARTHWMappings[ucUsedPort].xUSARTIrq );
    
//            PIO_Configure( xUSARTHWMappings[ucUsedPort].xUSARTPins, xUSARTHWMappings[ucUsedPort].xUSARTPinsCnt );
//            if( NULL != xUSARTHWMappings[ucUsedPort].USARTNotREPin )
//            {
//                PIO_Configure( xUSARTHWMappings[ucUsedPort].USARTNotREPin, 1 );
//            }
//            if( NULL != xUSARTHWMappings[ucUsedPort].USARTDEPin )
//            {
//                PIO_Configure( xUSARTHWMappings[ucUsedPort].USARTDEPin, 1 );
//            }
          //  PMC_EnablePeripheral( xUSARTHWMappings[ucUsedPort].xUSARTID );
          // USART_Configure( xUSARTHWMappings[ucUsedPort].pUsart, uiMode, ulBaudRate, BOARD_MCK );
      
          //  NVIC_ClearPendingIRQ(  xUSARTHWMappings[ucUsedPort].xUSARTIrq );
          //  NVIC_SetPriority(  xUSARTHWMappings[ucUsedPort].xUSARTIrq, 0xF << 4 );
          //  NVIC_EnableIRQ(  xUSARTHWMappings[ucUsedPort].xUSARTIrq );   
                 
         USARTNotREPin_Clear;
         USARTDEPin_Clear;
         UART_Init(MODBUS_UART, &mod_config, CLOCK_GetFreq(MODBUS_UART_CLKSRC));
         DisableIRQ(MODBUS_UART_IRQn);
         UART_DisableInterrupts(MODBUS_UART, kUART_TxDataRegEmptyInterruptEnable | kUART_TransmissionCompleteInterruptEnable | kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
         ClearPendingIRQ(MODBUS_UART_IRQn);
         EnableIRQ(MODBUS_UART_IRQn);
         
          /* Enable RX interrupt. */
            //UART_EnableInterrupts(USB_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable );
         
        }
//    }

    return bStatus;
}

void
vMBPortSerialClose( void )
{
    if( USART_INVALID_PORT != ucUsedPort )
    {
//        NVIC_DisableIRQ( xUSARTHWMappings[ucUsedPort].xUSARTIrq );
//        PMC_DisablePeripheral( xUSARTHWMappings[ucUsedPort].xUSARTID );
//        if( NULL != xUSARTHWMappings[ucUsedPort].USARTNotREPin )
//        {
//            PIO_Clear( xUSARTHWMappings[ucUsedPort].USARTDEPin );
//        }
//        if( NULL != xUSARTHWMappings[ucUsedPort].USARTDEPin )
//        {
//            PIO_Clear( xUSARTHWMappings[ucUsedPort].USARTNotREPin );
//        }
        USARTNotREPin_Clear;
        USARTDEPin_Clear;
        NVIC_DisableIRQ(MODBUS_UART_IRQn);
        UART_Deinit(MODBUS_UART);        
        ucUsedPort = USART_INVALID_PORT;
    }            
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  UART_WriteByte(MODBUS_UART,ucByte); 
//    USART1->US_THR = ucByte;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
     parityError = false;
    *pucByte = UART_ReadByte(MODBUS_UART);
    if((UART_GetStatusFlags(MODBUS_UART) & kUART_ParityErrorInRxDataRegFlag) && (mod_config.parityMode != kUART_ParityDisabled))
    {
      parityError = true;
      return FALSE;
    }
//    *pucByte = USART1->US_RHR;
    return TRUE;
}

void
MODBUS_UART_IRQHandler( void )
{
//    uint32_t        uiCSR;
//    uint32_t        uiIMR;
//    uiCSR = xUSARTHWMappings[ucUsedPort].pUsart->US_CSR;
//    uiIMR = xUSARTHWMappings[ucUsedPort].pUsart->US_IMR;
//    uint32_t        uiCSRMasked = uiCSR & uiIMR;
    
    //if( uiCSRMasked & US_CSR_RXRDY )
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(MODBUS_UART))
    {
        pxMBFrameCBByteReceived(  );
    }
    //if( uiCSRMasked & US_CSR_TXRDY )
    if((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(MODBUS_UART)) && (kUART_TxDataRegEmptyInterruptEnable & UART_GetEnabledInterrupts(MODBUS_UART)))
    {
        UART_ClearStatusFlags(MODBUS_UART, kUART_TxDataRegEmptyFlag);
        pxMBFrameCBTransmitterEmpty(  );
    }
    //if( uiCSRMasked & US_CSR_TXEMPTY ) 
    if((kUART_TransmissionCompleteFlag & UART_GetStatusFlags(MODBUS_UART)) && (kUART_TransmissionCompleteInterruptEnable & UART_GetEnabledInterrupts(MODBUS_UART)))
    {
//        if( NULL != xUSARTHWMappings[ucUsedPort].USARTDEPin )
//        {
//            PIO_Clear( xUSARTHWMappings[ucUsedPort].USARTDEPin );
//        }
//        if( NULL != xUSARTHWMappings[ucUsedPort].USARTNotREPin )
//        {
//            PIO_Clear( xUSARTHWMappings[ucUsedPort].USARTNotREPin );
//        }
//        USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXEMPTY );
        UART_ClearStatusFlags(MODBUS_UART, kUART_TransmissionCompleteFlag);
        USARTNotREPin_Clear;
        USARTDEPin_Clear;
        UART_DisableInterrupts(MODBUS_UART, kUART_TransmissionCompleteInterruptEnable);
    }
}

//#if USART1_ENABLED == 1
//void
//USART1_IrqHandler( void )
//{
//    vUSARTHandler(  );
//}
//#endif
//
//#if USART0_ENABLED == 1
//void
//USART0_IrqHandler( void )
//{
//    vUSARTHandler(  );
//}
//#endif
