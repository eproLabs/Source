/*


 This MODBUS PROGRAM HAS BEEN CHECKED in
1. RTU-MODE -> ODD PARITY and EVEN PARITY at different baudrates from 9600
2. ASCII-MODE -> NONE PARITY at different baudrates..

Its not checked in
RTU-MODE -> NONE PARITY and ASCII-MODE -> ODD & EVEN PARITY..


All these checking happeded using modbus simulator softwares on a PC using USB to COM FTD Chip.
Its suspected that ordinary UART in PC doesnot work in the above specified modes (RTU - NO Parity and ASCII - Odd&Even Parity) because of different bit formats..
Anyhow to confirm that these UN-Checked modes are also working, it has be to checked with proper methods..
 *






 * File: $Id: demo.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- AT91SAM3S includes -------------------------------*/
//#include <board.h>
//#include <usart/usart.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port.h"
#include <stdbool.h>

/*--------------------------MODBUS DEFINES ---------------------------------*/

#define MODBUS_MODE MB_RTU
#define MODBUS_SLAVE_ADD 0x0A
#define MODBUS_BAUDRATE 19200
#define MODBUS_PARITY kUART_ParityEven

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START                 ( 1)
#define REG_INPUT_NREGS                 ( 32 )

#define REG_HOLDING_START               ( 1 )
#define REG_HOLDING_NREGS               ( 32 )


/* ----------------------- Static functions ---------------------------------*/
//static void _SetupHardware( void );

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

/* ----------------------- Start implementation -----------------------------*/

    const UCHAR     ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
    eMBErrorCode    eStatus;
 
    
    int Modbus_routine_1( void );// Not used..
        
    bool  Modbus_Init (void);
    int Modbus_routine( void );

    void  Update_Modbus(int16_t *Modbus_Reg, uint8_t No_of_UpdateData);
    
    
/* ----------------------- Start implementation -----------------------------*/    
bool  Modbus_Init (void)
{
      if( MB_ENOERR != ( eStatus = eMBInit( MODBUS_MODE, MODBUS_SLAVE_ADD, 1, MODBUS_BAUDRATE, MODBUS_PARITY) ) )
      {
        return(false);
      }
      if( MB_ENOERR != ( eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 ) ) )
      {
        return(false);
      }
      if( MB_ENOERR != ( eStatus = eMBEnable(  ) ) )
      {
        return(false);
      }
      return(true);
}
      
int Modbus_routine( void )     
{
              
   ( void )eMBPoll(  );  
       return 1;
}

void  Update_Modbus(int16_t *Modbus_Reg, uint8_t No_of_UpdateData)
{
  for(uint8_t Update_Datacount = 0; Update_Datacount < No_of_UpdateData; Update_Datacount++)
  {
         usRegHoldingBuf[Update_Datacount] = Modbus_Reg[Update_Datacount];
         usRegInputBuf[Update_Datacount] = Modbus_Reg[Update_Datacount];
  }
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    
    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}


/// Below code not used.....
int Modbus_routine_1( void )                    // Not used);
{
 //   _SetupHardware(  );
    


 //   for( ;; )
  // {
        if( MB_ENOERR != ( eStatus = eMBInit( MB_RTU, 0x0A, 1, 38400, kUART_ParityDisabled ) ) )
        {
            /* Can not initialize. Add error handling code here. */
        }
        else
        {      
            if( MB_ENOERR != ( eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 ) ) )
            {
                /* Can not set slave id. Check arguments */
            }
            else if( MB_ENOERR != ( eStatus = eMBEnable(  ) ) )
            {
                /* Enable failed. */
            }
            else
            {      
                usRegHoldingBuf[0] = 1;
                do
                {
                    ( void )eMBPoll(  );
            
                    /* Here we simply count the number of poll cycles. */
                    usRegInputBuf[0]++;
                }
                while( usRegHoldingBuf[0] );
                ( void )eMBDisable(  );
                ( void )eMBClose(  );                
            }
        }
 //   }    
    return 1;
}

