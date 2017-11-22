/*
    Set current in micro Amps
  Load cur and Load voltage measured and displayed..
    80
      94
        c0 d4
        
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc16.h"
//#include "fsl_dac.h"
#include "fsl_vref.h"
#include "fsl_lptmr.h"

#include "fsl_port.h"
//#include "fsl_dspi.h"

#include "clock_config.h"
#include "pin_mux.h"


//#include "TFTLCD.h"
//#include "keypad_IDBR.h" 
#include "power_mode_switch.h" 

#include "fsl_smc.h"
#include "fsl_llwu.h"
#include "fsl_rcm.h"
#include "fsl_pmc.h"
#include "fsl_uart.h"
   
#include "lcd_pic.h"
#include "rtc_func.h"
#include "SDCard.h"
#include "Modbus_Main.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
   
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USB_UART UART1
#define USB_UART_CLKSRC UART1_CLK_SRC
#define USB_UART_IRQn UART1_RX_TX_IRQn
#define USB_UART_IRQHandler UART1_RX_TX_IRQHandler
#define USB_UART_BAUDRATE 115200
#define USB_UART_BUFFER_SIZE 200

#define USB_UART_RxStartChar '$'
#define USB_UART_RxEndChar '#' 
#define RX_RTC_DATA_CHAR '@'
#define SET_I_RATIO_DATA_CHAR '&'

#define LPTMR_HANDLER LPTMR0_IRQHandler
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
#define LPTMR_TIME 1000 //100000       //1000    //Time in microseconds




#define I2_SENSE_ADC_BASEADDR ADC1        //ADC0
#define I2_SENSE_ADC_CHANNEL_GROUP 0U
#define I2_SENSE_ADC_USER_CHANNEL 0U /* PTB0, ADC0_SE8 */
#define I2_SENSE_ADC_IRQn ADC1_IRQn
#define I2_SENSE_ADC_IRQ_HANDLER_FUNC ADC1_IRQHandler

#define I1_SENSE_ADC_BASEADDR ADC0        //ADC0
#define I1_SENSE_ADC_CHANNEL_GROUP 0U
#define I1_SENSE_ADC_USER_CHANNEL 0U /* PTB0, ADC0_SE8 */
#define I1_SENSE_ADC_IRQn ADC0_IRQn
#define I1_SENSE_ADC_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define VBat_SENSE_ADC_BASEADDR ADC0        //ADC0
#define VBat_SENSE_ADC_CHANNEL_GROUP 0U
#define VBat_SENSE_ADC_USER_CHANNEL 8U /* PTC2 ADC0__SE4b */
#define VBat_SENSE_ADC_IRQn ADC0_IRQn
#define VBat_SENSE_ADC_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define ADCNooftimes 200

//#define DAC_BASEADDR DAC0

//#define DAC_VREF 1.2
//#define DAC_RES 4096

#define VREF_BRD 3.000
#define SE_12BIT 4096.0

#define BAT_LEVEL1_MAX 3.9  
#define BAT_LEVEL1_MIN 3.88
#define BAT_LEVEL2_MAX 3.7
#define BAT_LEVEL2_MIN 3.68
#define BAT_LEVEL3_MAX 3.62
#define BAT_LEVEL3_MIN 3.6
#define BAT_LEVEL_OFF 3.5

#define DEFBATLEVEL 6
#define BAT_CHARGE 5
#define BAT_3_3 4
#define BAT_2_3 3
#define BAT_1_3 2
#define BAT_0_3 1
#define BAT_0_0 0

#define min_load_cur_val 0.01

#define NoofCurDecimal 100      //(2 decimal places)
#define No_hr_avg_values 60        // if reading taken per minute then for an hour its 60.

#define I_add_error 0   //0.0185
#define calib_divider 0.250

#define Max_Voltage 35                  // This is the MAx voltage reach when its opencircuit..

#define ON true
#define OFF false

#define CHARGE false
#define NOTCHARGE true

#define NoofFaultcounts 5

#define VBAT_CHANNEL true
#define I_CHANNEL false
   
#define POS 0
#define NEG 1
#define INF 2

#define INF_VAL 0x7FFF          // A higher value loaded to denote infinity..

#define usbstartchar "$"
#define usbendchar    "\n"
//#define BACKLIGHT_ON  ((GPIO_ClearPinsOutput(BOARD_BACKLIGHT_GPIO, 1U << BOARD_BACKLIGHT_GPIO_PIN)),(DisplayStat = ON));
//#define BACKLIGHT_OFF  ((GPIO_SetPinsOutput(BOARD_BACKLIGHT_GPIO, 1U << BOARD_BACKLIGHT_GPIO_PIN)), (DisplayStat = OFF));


typedef struct{
  uint8_t milliSec;             // 100mSec
  uint8_t Sec;                  //1 sec
  uint8_t Min;                 // 1 Min
}time_data;

typedef struct _uart_transfer_TxTx
{
    uint8_t data[USB_UART_BUFFER_SIZE];   /*!< The buffer of data to be transfer.*/
    size_t dataSize; /*!< The byte count to be transfer. */
    volatile bool completeflag;
    volatile uint16_t txIndex;
    volatile uint16_t rxIndex;
    volatile uint16_t pendingbytes;
} uart_transfer;

typedef struct
{
uint8_t Rxdata[USB_UART_BUFFER_SIZE];
uint16_t Rxdata_Count;
uint16_t Rxdata_Count_End;
bool StartFlag;
bool ReceivedFlag;
}UART_RXtypedef;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Initialize ADC16 & DAC */
static void ADC_Init(void);

void delay(void);
void dec_to_str (char* str, uint32_t val, size_t digits);
float CalculateSetcurrent(unsigned char * KeyPress_Char, unsigned char KeyPress_Count);
void displaydebugdata (void);
void checkadc (void);
void float_to_str_3_1digit (char* str, float val);
void float_to_str_3_2digit (char* str, float val);
void float_to_str_1_3digit (char* str, float val);
void float_to_str_2_1digit (char* str, float val);
void float_to_str_1_2digit (char* str, float val);
void float_to_str_2_2digit (char* str, float val);
uint32_t CalculateDACValue(float Current);
void LPTMR_Initialisation(void);
void disptime (time_data disp_time);
void ClearInt_Time (void);
void ClearBPInt_Time (void);



void AssignDAC (float Set_Current);
void CalculateTime (float Set_Dosage);
void CalculateRampdownDosage (uint16_t MaxCurrent);
void dispDosage (void);
void dispProcessStarted(void);
void dispProcessCompleted(void);
void dispProcessPaused (void);
void dispProcessPausing(void);
void dispMode(void);
void dispSetCurrent (void);
void dispSetDosage (void);
void dispSetBPTime(void);
void dispLoadCurrent (void);
void dispLoadVoltage (void);
void dispLoadDosage (void);
void dispfault(void);
void dispProcessStopping (void);
void dispProcessStopped (void);
void dispBatEmpty(void);
void dispBatStatus(void);
void dispadjustValue(uint16_t Set_adjust_Value, uint16_t AdjustType);
void disp_data (void);
        void dispdata(float cur1, float cur2);
        void disp_pdiff(float per_diff, uint8_t sign);
        void dispBatVolt (float Bat_Volt);
        void disp_RTC_Time (rtc_datetime_t cur_datetime_disp);
        void float_to_str_3f (char* str, float val);
        void float_to_str_2f (char* str, float val);
        void process_RTC_Data (void);
        void process_I_Ratio_SetData(void);
        void Process_USB_Uart_RxData (void);
        void Tx_RTC_Time (rtc_datetime_t cur_datetime_disp);
       void Calculate_Avg (void);
        void Calculate_Ratio(float Current_1, float Current_2);
        void check_I_Ratio_sec (void);
         void Tx_to_Modbus (void);
        
void ProcessStart (void);
void InitialiseAll (void);
void txdata_usb (void);
void USB_UART_Init(void);
void checkUSB_UARTdata(void);
void USB_UART_Tx(unsigned char *Txdata, uint8_t datasize);

void ChangeCurrent (bool IncDecbit);
void ChangeDosage (bool IncDecbit);
void ChangeBPTime (bool IncDecbit);

void checkkeypad (void);
void checkBatStatus(void);
void checkBatadc (void);

void Init_Pins (void);

void ADC_Deinitialise(void);
void WritetoSDCard (void);

void APP_PowerPreSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode);
void APP_PowerPostSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode);

void APP_SetWakeupConfig(app_power_mode_t targetMode);
void APP_PowerModeSwitch(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode);



/*******************************************************************************
 * Variables
 ******************************************************************************/

unsigned const char InitialPrintString[] = "*** L & T Constructions - Current Measurement System ***\r\n";
unsigned const char InitialStoreString[] = "*** L & T Constructions - Current Measurement System ***\r\nDate(ddmmyyyy),Time(hhmmss),I1(A),I2(A),R,SR(%),Rh,SRh(%),Rd,SRd(%),BAT(V)\r\n";
unsigned const char InitialLCD_OK_Str[] = "LCD Initialisation Done\r\n";
unsigned const char InitialLCD_ER_Str[] = "LCD Initialisation Error\r\n";
unsigned const char InitialMODBUS_OK_Str[] = "MODBUS Initialisation Done\r\n";
unsigned const char InitialMODBUS_ER_Str[] = "MODBUS Initialisation Error\r\n";

unsigned char ErrorString[] = "ERROR\r\n";
unsigned char OKString[] = "OK\r\n";
volatile bool I2_adcConversionDoneFlag = false;
volatile uint32_t V_adcConversionValue = 0;
adc16_channel_config_t V_adcChannelConfigStruct;
volatile bool I1_adcConversionDoneFlag = false;
volatile uint32_t I_adcConversionValue = 0;
adc16_channel_config_t I_adcChannelConfigStruct;
volatile bool VBat_adcConversionDoneFlag = false;
volatile uint32_t VBat_adcConversionValue = 0;
adc16_channel_config_t VBat_adcChannelConfigStruct;

volatile uint32_t lptmrCounter = 0U;

uint8_t KeyPressChar[10];
//DisplayDataConfig CurVal_DataConfig;


volatile time_data Int_time, Int_BPtime;

    
float Load_Current_1, Load_Current_2;
float Prev_Load_Current_1;
float I_Ratio, Perc_Diff;
float I_Ratio_sec, Perc_Diff_sec;
float I_Ratio_hr, I_Ratio_dy;
float Perc_Diff_hr, Perc_Diff_dy;

float Bat_Voltage;
  
bool Onoffbit = OFF;
bool faultbit = false;
bool DosageStartbit = 0;

bool chargebit = NOTCHARGE;
//bool BP_Direction = FORWARD;
bool ADC_Channel = VBAT_CHANNEL;
bool Bat_first_bit = true;
bool timedispfirstbit = false;
bool DisplayStat = OFF;
uint16_t adjust_Value = 0;
float Battery_Voltage;
uint16_t Bat_Level;
//uint16_t PrevAdjustType = DEFADJUSTVALUE;
uint16_t PrevBat_Level = DEFBATLEVEL;
uint8_t perc_diff_sign = POS;

char store_filename[25];
uint8_t dispstr[25];

//uint8_t ProcessState = SHUTDOWN_STATE;
static app_wakeup_source_t s_wakeupSource; // Wakeup source.  

static uint16_t gen_timecount = 0;

bool daytime_flag = false;
uint16_t noofdatacount_dy = 0;
uint32_t Isc1_dy_Avg = 0;
uint32_t Isc2_dy_Avg = 0;
uint16_t noofdatacount_hr = 0;
uint32_t Isc1_hr_Avg = 0;
uint32_t Isc2_hr_Avg = 0;
    
float Isc1_hr = 0, Isc2_hr = 0;
float Isc1_dy = 0, Isc2_dy = 0;
float Set_I_Ratio = 0;

char filename_txt[] = {"Ensemble.txt"};
uart_transfer Uart_Tx_Buf, Uart_Rx_Buf;
UART_RXtypedef Usb_Uart;

  unsigned char RTCString[30];
  unsigned char Current1_String[10];
  unsigned char Current2_String[10];
  unsigned char I_Ratio_String[10];
  unsigned char Perc_Diff_String[10];
  unsigned char Bat_Volt_String[10];
  unsigned char I_Ratio_sec_String[10];
  unsigned char I_Ratio_dy_String[10];
  unsigned char I_Ratio_hr_String[10];
  unsigned char Perc_Diff_sec_String[10];
  unsigned char Perc_Diff_dy_String[10];
  unsigned char Perc_Diff_hr_String[10];
  
  uint32_t Isc1_hr_Avg, Isc2_hr_Avg;
  uint32_t Isc1_dy_Avg, Isc2_dy_Avg;
  
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
 //   volatile float SetCurrent = 0;
    smc_power_state_t curPowerState;
    app_power_mode_t targetPowerMode;
    
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    if (kRCM_SourceWakeup & RCM_GetPreviousResetSources(RCM)) /* Wakeup from VLLS. */
    {
        PMC_ClearPeriphIOIsolationFlag(PMC);
 //       NVIC_ClearPendingIRQ(LLWU_IRQn);
    }
    
    BOARD_InitPins();
    BOARD_BootClockRUN();
 //   BOARD_BootClockHSRUN();
     BOARD_InitDebugConsole();
   
     Init_Pins();
    
     USB_UART_Init();

  //  BP_Forward();
  //  GPIO_ClearPinsOutput(BOARD_HI_POWER_GPIO, 1U << BOARD_EN_35V_GPIO_PIN);

    LED1_INIT(0);
    LED2_INIT(0);
    LED3_INIT(0);
    
    ADC_Init();  
    lcd_Spi_Init();
    int_SDC_SPI();
 
    LED1_ON();
    LED2_ON();
    LED3_ON();
   
    EnableIRQ(I2_SENSE_ADC_IRQn);
    EnableIRQ(I1_SENSE_ADC_IRQn);
//    NVIC_EnableIRQ(APP_WAKEUP_BUTTON_IRQ);
    PRINTF("\r\nSOLAR PANEL ADC\r\n");

    LPTMR_Initialisation();

  //  s_wakeupSource = kAPP_WakeupSourcePin;
    curPowerState = SMC_GetPowerModeState(SMC);
    InitialiseAll();
    Init_RTC();
//    while(1)
//    {
//      DbgConsole_Putchar(DbgConsole_Getchar());
//        for (uint16_t i = 0; i< 50; i++)
//        {
//        delay();
//        } 
//    }
      
        USB_UART_Tx((unsigned char*)InitialPrintString, strlen((const char *)InitialPrintString));
        for (uint16_t i = 0; i< 6; i++)
        {
        delay();
        }
        LED1_OFF();
        LED2_OFF();
        LED3_OFF();
        
        lcd_init();
        lcd_command(0x0c);
        lcd_datas("       L & T");
        lcd_command(0xC0);
        lcd_datas("   CONSTRUCTIONS");
        lcd_command(0x94);
        lcd_datas(" CURRENT MEASUREMENT");
        lcd_command(0xd4);
        lcd_datas("       SYSTEM");
        for (uint16_t i = 0; i< 15; i++)
        {
        delay();
        }
        lcd_command(1);
        
        USB_UART_Tx((unsigned char*)InitialLCD_OK_Str, strlen((const char *)InitialLCD_OK_Str));

        if(Modbus_Init() == true)
        {
          USB_UART_Tx((unsigned char*)InitialMODBUS_OK_Str, strlen((const char *)InitialMODBUS_OK_Str));
        }
        else
        {
          USB_UART_Tx((unsigned char*)InitialMODBUS_ER_Str, strlen((const char *)InitialMODBUS_ER_Str));
        }
     
//       while(CreateTextFile(filename_txt) == FAILURE);

        while(1)
        {
          LED3_OFF();
          CheckSeconds_Update(&cur_date_time);
          checkadc();
          if(Load_Current_1 < min_load_cur_val)
          {
            Load_Current_1 = 0;
          }
          if(Load_Current_2 < min_load_cur_val)
          {
            Load_Current_2 = 0;
          }
              
          float_to_str_2_2digit((char *)Current1_String, Load_Current_1);
          float_to_str_2_2digit((char *)Current2_String, Load_Current_2);  
          
          Calculate_Ratio(Load_Current_1, Load_Current_2);
          I_Ratio_sec = I_Ratio;
          Perc_Diff_sec = Perc_Diff;
          sprintf((char *)I_Ratio_sec_String, "%s", I_Ratio_String);
          sprintf((char *)Perc_Diff_sec_String, "%s", Perc_Diff_String); 
          
          Calculate_Avg();
          
          float_to_str_1_2digit((char *)Bat_Volt_String, Bat_Voltage); 
          
          disp_data();
          txdata_usb();
          WritetoSDCard();
          check_I_Ratio_sec();
          Tx_to_Modbus();
        while(g_SecsFlag != true)
        {
         checkUSB_UARTdata();
         Process_USB_Uart_RxData();
         Modbus_routine();
        }
         Modbus_routine();
         checkUSB_UARTdata();
         Process_USB_Uart_RxData();
 //        CheckSeconds_Update(&cur_date_time);
 //        disp_RTC_Time(cur_date_time);
        }
}
 void Tx_to_Modbus (void)
 {
   int16_t Modbus_Registers[15];
   uint8_t Mod_Regcount = 0;
   Modbus_Registers[Mod_Regcount++] = (int16_t) (Load_Current_1 * 100);        // Remove decimal places
   Modbus_Registers[Mod_Regcount++] = (int16_t) (Load_Current_2 * 100);
   Modbus_Registers[Mod_Regcount++] = (int16_t) ((I_Ratio_sec == INF_VAL) ? INF_VAL :(I_Ratio_sec * 100));  
   Modbus_Registers[Mod_Regcount++] = (int16_t) ((Perc_Diff_sec == INF_VAL) ? INF_VAL :(Perc_Diff_sec * 100)); 
   Modbus_Registers[Mod_Regcount++] = (int16_t) ((I_Ratio_hr == INF_VAL) ? INF_VAL :(I_Ratio_hr * 100));
   Modbus_Registers[Mod_Regcount++] = (int16_t) ((Perc_Diff_hr == INF_VAL) ? INF_VAL :(Perc_Diff_hr * 100));
   Modbus_Registers[Mod_Regcount++] = (int16_t) ((I_Ratio_dy == INF_VAL) ? INF_VAL :(I_Ratio_dy * 100));
   Modbus_Registers[Mod_Regcount++] = (int16_t) ((Perc_Diff_dy == INF_VAL) ? INF_VAL :(Perc_Diff_dy * 100));
   Modbus_Registers[Mod_Regcount++] = (int16_t) (Bat_Voltage * 100); 
   Update_Modbus(Modbus_Registers, Mod_Regcount);
 }

void check_I_Ratio_sec (void)
{
  if(Set_I_Ratio > I_Ratio_sec)
  {
    LED1_ON();
  }
  else
  {
    LED1_OFF();
  }
}

void Process_USB_Uart_RxData (void)
{
  if(Usb_Uart.ReceivedFlag == false)
    return;
  switch(Usb_Uart.Rxdata[0])
  {
  case RX_RTC_DATA_CHAR:
    process_RTC_Data();
    break;
  case SET_I_RATIO_DATA_CHAR:
    process_I_Ratio_SetData();
    break;
  default:
    break;
  }
  Usb_Uart.ReceivedFlag  = false;
}


void process_I_Ratio_SetData(void)
{
  unsigned char temptx[10];
  Set_I_Ratio = atof((char *)&Usb_Uart.Rxdata[1]);
 // sprintf((char*)temptx, "Set:%d\r\n", (uint32_t)(Set_I_Ratio*1000));
//  USB_UART_Tx(temptx, strlen((char *)temptx));
  LED3_ON();
}
            
void process_RTC_Data (void)
{
  uint16_t data1;
  uint16_t rxdatacount = 1;
  data1 = (Usb_Uart.Rxdata[rxdatacount++]-0x30)*10;
  data1 = data1 + (Usb_Uart.Rxdata[rxdatacount++]-0x30);
  getdate.day = data1;
  
  data1 = (Usb_Uart.Rxdata[rxdatacount++]-0x30)*10;
  data1 = data1 + (Usb_Uart.Rxdata[rxdatacount++]-0x30);
  getdate.month = data1;
  
  data1 = (Usb_Uart.Rxdata[rxdatacount++]-0x30)*1000;
  data1 = data1 + ((Usb_Uart.Rxdata[rxdatacount++]-0x30)*100);
  data1 = data1 + ((Usb_Uart.Rxdata[rxdatacount++]-0x30)*10);
  data1 = data1 + ((Usb_Uart.Rxdata[rxdatacount++]-0x30));
  getdate.year = data1;
  
  data1 = (Usb_Uart.Rxdata[rxdatacount++]-0x30)*10;
  data1 = data1 + (Usb_Uart.Rxdata[rxdatacount++]-0x30);
  getdate.hour = data1;
  
  data1 = (Usb_Uart.Rxdata[rxdatacount++]-0x30)*10;
  data1 = data1 + (Usb_Uart.Rxdata[rxdatacount++]-0x30);
  getdate.minute = data1;
  
  data1 = (Usb_Uart.Rxdata[rxdatacount++]-0x30)*10;
  data1 = data1 + (Usb_Uart.Rxdata[rxdatacount]-0x30);
  getdate.second = data1;

//   Tx_RTC_Time(getdate);
  if(rxdatacount != (Usb_Uart.Rxdata_Count_End-1))
  {
   //   USB_UART_Tx(ErrorString, strlen((const char *)ErrorString));
  }
  else
  {
    if(set_RTC_DateTime(&getdate) == SUCCESS)
    {
      LED3_ON();
      InitialiseAll();
    //USB_UART_Tx(OKString, strlen((const char *)OKString));
    }
    else
    {
    //USB_UART_Tx(ErrorString, strlen((const char *)ErrorString));
    }
  }
}



void Calculate_Avg (void)
{
  if(cur_date_time.hour >= 10 && cur_date_time.hour <= 13)
  {
    daytime_flag = true;    
  }
  else if(daytime_flag == true)
  {
    Isc1_dy = (((float)Isc1_dy_Avg)/noofdatacount_dy)/NoofCurDecimal;                     // bring in decimal.
    Isc2_dy = (((float)Isc2_dy_Avg)/noofdatacount_dy)/NoofCurDecimal;                     // bring in decimal.
    noofdatacount_dy = 0;
    Isc1_dy_Avg = 0;
    Isc2_dy_Avg = 0;
    daytime_flag = false;    
    Calculate_Ratio(Isc1_dy, Isc2_dy);
    I_Ratio_dy = I_Ratio;
    Perc_Diff_dy = Perc_Diff;
    sprintf((char *)I_Ratio_dy_String, "%s", I_Ratio_String);
    sprintf((char *)Perc_Diff_dy_String, "%s", Perc_Diff_String);    
  }
  
  if(datetime_avg.hour != cur_date_time.hour)
  {
    if(noofdatacount_hr !=0)
    {
    Isc1_hr = (((float)Isc1_hr_Avg)/noofdatacount_hr)/NoofCurDecimal;                     // bring in decimal.
    Isc2_hr = (((float)Isc2_hr_Avg)/noofdatacount_hr)/NoofCurDecimal;                     // bring in decimal.
    noofdatacount_hr = 0;
    Isc1_hr_Avg = 0;
    Isc2_hr_Avg = 0;
    Calculate_Ratio(Isc1_hr, Isc2_hr);
    I_Ratio_hr = I_Ratio;
    Perc_Diff_hr = Perc_Diff;
    sprintf((char *)I_Ratio_hr_String, "%s", I_Ratio_String);
    sprintf((char *)Perc_Diff_hr_String, "%s", Perc_Diff_String);  
    }
   datetime_avg.hour = cur_date_time.hour;
  }
  if(datetime_avg.minute != cur_date_time.minute)
  {
    Isc1_hr_Avg = Isc1_hr_Avg + (uint32_t)((Load_Current_1*NoofCurDecimal));                   // remove decimal for easy and fast operation
    Isc2_hr_Avg = Isc2_hr_Avg + (uint32_t)((Load_Current_2*NoofCurDecimal));                   // remove decimal for easy and fast operation
    noofdatacount_hr++;
    datetime_avg.minute =  cur_date_time.minute;
    
    if(daytime_flag == true)
    {
     Isc1_dy_Avg = Isc1_dy_Avg + (uint32_t)((Load_Current_1*NoofCurDecimal));                   // remove decimal for easy and fast operation
     Isc2_dy_Avg = Isc2_dy_Avg + (uint32_t)((Load_Current_2*NoofCurDecimal));                   // remove decimal for easy and fast operation
     noofdatacount_dy++;
    }
  }
}


void Calculate_Ratio(float Current_1, float Current_2)
{
  float Perc_Diff_temp;
        if(Current_1 <= 0)
        {
          I_Ratio = INF_VAL;               // Just a higher number to denote infinity
          Perc_Diff = INF_VAL;
          sprintf((char *)I_Ratio_String, "  INF ");
          sprintf((char *)Perc_Diff_String, "  INF ");
        }
        else
        {
          I_Ratio = Current_2/Current_1 ;
          float_to_str_1_3digit((char *)I_Ratio_String,I_Ratio);
          Perc_Diff =  ((Current_1 - Current_2) * 100)/Current_1;
          
          if(Current_1 >= Current_2)
          {
          Perc_Diff_temp =  ((Current_1 - Current_2) * 100)/Current_1;
          Perc_Diff_String[0] = ' ';                                            // space for '+'
          float_to_str_3_1digit((char *)(&Perc_Diff_String[1]),Perc_Diff_temp);
          }
          else
          {
          Perc_Diff_temp = ((Current_2 - Current_1) * 100)/Current_1;
          Perc_Diff_String[0] = '-';
          float_to_str_3_1digit((char *)(&Perc_Diff_String[1]),Perc_Diff_temp);
          }
        }
}

void WritetoSDCard (void)
{
  if(datetime_SDCWrite.day != cur_date_time.day)
  {
    uint16_t year_2digit = cur_date_time.year - (((uint8_t)(cur_date_time.year/100))*100);
    sprintf(store_filename, "%02d%02d%02d%02d%s", cur_date_time.day, cur_date_time.month,year_2digit,cur_date_time.minute,".txt");
    if(WriteTextFile(store_filename,(unsigned char*)InitialStoreString)== SUCCESS)
    {
    datetime_SDCWrite.day = cur_date_time.day;
    LED2_ON();
 //   USB_UART_Tx("Write Good",10);
    }
    else
    {
    LED2_OFF();
 //   USB_UART_Tx("WRite Error", 11);
    }      
  }
 // if(datetime_SDCWrite.second != cur_date_time.second)
  if(datetime_SDCWrite.minute != cur_date_time.minute)
  {
    if(WriteTextFile(store_filename,storedata)== SUCCESS)
    {
//    datetime_SDCWrite.second = cur_date_time.second;
   datetime_SDCWrite.minute = cur_date_time.minute;
    LED2_ON();
 //   USB_UART_Tx("Write Good",10 );
    }
    else
    {
    LED2_OFF();
//   USB_UART_Tx("WRite Error", 11);
    }
  }
}

void txdata_usb (void)
{

         unsigned char TxDataString[150];
          
         sprintf((char *)RTCString, "%02d/%02d/%04d,%02d:%02d:%02d", cur_date_time.day, cur_date_time.month,cur_date_time.year, 
          cur_date_time.hour, cur_date_time.minute, cur_date_time.second);
          
         sprintf((char*)TxDataString, "%s,I1:%s,I2:%s,R:%s,SR:%s,Rh:%s,SRh:%s,Rd:%s,SRd:%s,BAT:%s\r\n",RTCString, Current1_String, Current2_String, I_Ratio_sec_String, Perc_Diff_sec_String,I_Ratio_hr_String, Perc_Diff_hr_String,I_Ratio_dy_String, Perc_Diff_dy_String,Bat_Volt_String);
         sprintf((char *)storedata, "%s,I1=%s,I2=%s,R=%s,SR=%s,Rh=%s,SRh=%s,Rd=%s,SRd=%s,BAT=%s\r\n",RTCString, Current1_String, Current2_String, I_Ratio_sec_String, Perc_Diff_sec_String,I_Ratio_hr_String, Perc_Diff_hr_String,I_Ratio_dy_String, Perc_Diff_dy_String,Bat_Volt_String);
        // Tx_RTC_Time(cur_date_time);
         USB_UART_Tx(TxDataString, strlen((char *)TxDataString));
       //  USB_UART_Tx(storedata, strlen((char *)storedata));                     
//  PRINTF(usbstartchar);
//  float_to_str_3f(String1, Load_Current_1);
//  float_to_str_3f(String2, Load_Current_2);  
//  PRINTF("I1:%0.03f,I2:%0.03f", Load_Current_1, Load_Current_2);
// // sprintf(storedata,"I1:%0.03f,I2:%0.03f\r\n", Load_Current_1, Load_Current_2);
//  PRINTF(",");
//  if(I_Ratio > 1000)
//  {
//    PRINTF("R:INF,PR:INF");
//    sprintf(I_Ratio_String, "INF");
//    sprintf(Perc_Diff_String, "INF");
//  }
//  else
//  {
//    if(perc_diff_sign == NEG)
//    {
//      float perc_diff_temp = (Perc_Diff * (-1));
//      PRINTF("R:%0.03f,PR:%0.02f", I_Ratio, perc_diff_temp);
//      float_to_str_3f(I_Ratio_String, I_Ratio);
//      Perc_Diff_String[0] = '-';
//      float_to_str_2f(&Perc_Diff_String[1], Perc_Diff); 
//    }
//    else
//    {
//      PRINTF("R:%0.03f,PR:%0.02f", I_Ratio, Perc_Diff);
//      float_to_str_3f(I_Ratio_String, I_Ratio);
//      float_to_str_2f(Perc_Diff_String, Perc_Diff); 
//    }
//  }
//  PRINTF(",");
//  PRINTF("BAT:%0.03f", Bat_Voltage);
//  PRINTF(usbendchar);
//  float_to_str_3f(Bat_Volt_String, Bat_Voltage); 
//  sprintf(TxDataString, "I1:%s,I2:%s,R:%s,PR:%s,BAT:%s", String1, String2, I_Ratio_String, Perc_Diff_String,Bat_Volt_String);
//  PRINTF(usbstartchar);
//  PRINTF("%s", TxDataString);
//  PRINTF(usbendchar);
//  sprintf((char*)storedata, "%s\r\n", TxDataString);
 }
//void calcpdiff(float volt, float cur)
//{
//  if(volt > cur)
//  {
//   

void disp_RTC_Time (rtc_datetime_t cur_datetime_disp)
{
  lcd_command(0x80);
  sprintf((char *)dispstr, "%02d-%02d-%04d %02d:%02d:%02d", cur_datetime_disp.day, cur_datetime_disp.month,cur_datetime_disp.year, 
          cur_datetime_disp.hour, cur_datetime_disp.minute, cur_datetime_disp.second);
  lcd_datas(dispstr);
 
}

void Tx_RTC_Time (rtc_datetime_t cur_datetime_disp)
{
  sprintf((char *)RTCString, "%02d-%02d-%04d %02d:%02d:%02d", cur_datetime_disp.day, cur_datetime_disp.month,cur_datetime_disp.year, 
          cur_datetime_disp.hour, cur_datetime_disp.minute, cur_datetime_disp.second);
  USB_UART_Tx(RTCString, strlen((const char *)RTCString));
}

void disp_data (void)
{
 //   lcd_command(1);
    disp_RTC_Time(cur_date_time);
    lcd_command(0xC0);
    lcd_datas("I1:");
    lcd_datas(Current1_String);
    lcd_data('A');
    lcd_command(0xCB);
    lcd_datas("I2:"); 
    lcd_datas(Current2_String);
    lcd_data('A');
    
    lcd_command(0x94);
    lcd_datas("Sh:");
    lcd_datas(Perc_Diff_hr_String);
    lcd_command(0x9E);
    lcd_datas("Sd:");
   lcd_datas(Perc_Diff_dy_String);
 //   lcd_datas(" %");
    
    lcd_command(0xd4);
    lcd_datas("BAT VOLT:");
    lcd_datas(Bat_Volt_String);
    lcd_datas(" V");

}

void checkBatStatus (void)
{
      checkBatadc();
  if(GPIO_ReadPinInput(BOARD_POWER_GPIO, BOARD_CHARGE_GPIO_PIN) == CHARGE)
  {
     Bat_Level = BAT_CHARGE;
  }
  else
  {
    checkBatadc();
  }
  if(Bat_Level != PrevBat_Level)
  {
    if(DisplayStat == ON)
    {
    PrevBat_Level = Bat_Level;
  //  dispBatStatus();
    }
  }
}

void InitialiseAll (void)
{
datetime_avg.day = 0xff;
datetime_avg.hour = 0xff;
datetime_avg.minute = 0xff;
datetime_avg.month = 0xff;
datetime_avg.second = 0xff;
datetime_avg.year = 0xff;

datetime_SDCWrite.day = 0xff;
datetime_SDCWrite.hour = 0xff;
datetime_SDCWrite.minute = 0xff;
datetime_SDCWrite.month = 0xff;
datetime_SDCWrite.second = 0xff;
datetime_SDCWrite.year = 0xff;

I_Ratio = 0;
Perc_Diff = 0;
I_Ratio_sec = 0;
Perc_Diff_sec = 0;
I_Ratio_hr = 0;
I_Ratio_dy = 0;
Perc_Diff_hr = 0;
Perc_Diff_dy = 0;

Bat_Voltage = 0;

Isc1_hr_Avg = 0;
Isc1_dy_Avg = 0;
Isc2_hr_Avg = 0;
Isc2_dy_Avg = 0;

noofdatacount_dy = 0;
noofdatacount_hr = 0;
daytime_flag = false;

   sprintf((char *)I_Ratio_sec_String, "  --- ");
   sprintf((char *)Perc_Diff_sec_String, "  --- ");
   sprintf((char *)I_Ratio_hr_String, "  --- ");
   sprintf((char *)Perc_Diff_hr_String, "  --- ");
   sprintf((char *)I_Ratio_dy_String, "  --- ");
   sprintf((char *)Perc_Diff_dy_String, "  --- ");

}

void ClearInt_Time (void)
{
  Int_time.milliSec = 0;
  Int_time.Sec = 0;
  Int_time.Min = 0; 
}

void ClearBPInt_Time (void)
{
  Int_BPtime.milliSec = 0;
  Int_BPtime.Sec = 0;
  Int_BPtime.Min = 0; 
}


void checkBatadc (void)
{
  uint32_t VBat_voltRead;
  volatile float voltRead;
  VBat_voltRead = 0;
//  adc16_channel_mux_mode_t adc_channel_mux;
//  adc_channel_mux = kADC16_ChannelMuxB;
//  ADC16_SetChannelMuxMode(VBat_SENSE_ADC_BASEADDR, adc_channel_mux);
  ADC_Channel = VBAT_CHANNEL;
  for (uint16_t i=0;i<ADCNooftimes; i++)
  {
        VBat_adcConversionDoneFlag = false;

        ADC16_SetChannelConfig(VBat_SENSE_ADC_BASEADDR, VBat_SENSE_ADC_CHANNEL_GROUP, &VBat_adcChannelConfigStruct);

        while (!VBat_adcConversionDoneFlag)
        {
        }
        VBat_voltRead = VBat_voltRead + VBat_adcConversionValue;
  }
  
 // ////      PRINTF("\r\n\r\nVBAT_ADC Value_dir: %d\r\n", (uint16_t)VBat_adcConversionValue);       
        voltRead = (VBat_voltRead/ADCNooftimes);
 //       PRINTF("\r\nVBAT_ADC Value: %d\r\n", (uint16_t)voltRead);
 //       voltRead = (float)(voltRead * (VREF_BRD / SE_12BIT));
 //       PRINTF("\r\nVBAT_Voltage Value: %f\r\n", voltRead);
        Bat_Voltage = (float)voltRead / 124.13;

        
        if(Bat_first_bit)
        {
          if(Bat_Voltage > BAT_LEVEL1_MAX)
          {
          Bat_Level = BAT_3_3;
          }
          else if(Bat_Voltage >= BAT_LEVEL2_MAX)
          {
            Bat_Level = BAT_2_3;
          }
          else if(Bat_Voltage >= BAT_LEVEL3_MAX)
          {
             Bat_Level = BAT_1_3;
          }
          else if(Bat_Voltage >= BAT_LEVEL_OFF)
          {
              Bat_Level = BAT_0_3;
          }
          else
          {
            Bat_Level = BAT_0_0;
          }
            
          Bat_first_bit = 0;
        }
        else
        {
          if(Bat_Voltage > BAT_LEVEL1_MAX)
          {
          Bat_Level = BAT_3_3;
          }
          else if(Bat_Voltage <= BAT_LEVEL1_MIN)
          {
            if(Bat_Voltage > BAT_LEVEL2_MAX)
            {
            Bat_Level = BAT_2_3;
            }
            else if(Bat_Voltage <= BAT_LEVEL2_MIN)
            {
              if(Bat_Voltage > BAT_LEVEL3_MAX)
              {
              Bat_Level = BAT_1_3;
              }
              else if(Bat_Voltage <= BAT_LEVEL3_MIN)
              {
              if(Bat_Voltage < BAT_LEVEL_OFF)
              Bat_Level = BAT_0_0;
              }
              else
              {
              Bat_Level = BAT_0_3;
              }              
            }
          }
        }
        
        if(Bat_Level == BAT_0_0)
        {
          Onoffbit = OFF;
          DosageStartbit = 0;
          faultbit = 1;
          if(DisplayStat == ON)
          {
        //  dispBatEmpty();        
          delay();
          delay();
          delay();
          }
        }
      
}



void checkadc (void)
{
        uint32_t I2_voltRead, I1_voltRead, VBat_voltRead;
        volatile float voltRead;

        I2_voltRead = 0;
        I1_voltRead = 0;
        VBat_voltRead = 0;

      //  adc_channel_mux = kADC16_ChannelMuxA;
     //   ADC16_SetChannelMuxMode(I1_SENSE_ADC_BASEADDR, adc_channel_mux);
      for (uint16_t i=0;i<ADCNooftimes; i++)
      {
        I2_adcConversionDoneFlag = false;
        ADC16_SetChannelConfig(I2_SENSE_ADC_BASEADDR, I2_SENSE_ADC_CHANNEL_GROUP, &V_adcChannelConfigStruct);

        while (!I2_adcConversionDoneFlag)
        {
        }
        I2_voltRead = I2_voltRead + V_adcConversionValue;
        
        
        
        I1_adcConversionDoneFlag = false;
        ADC_Channel = I_CHANNEL;
        ADC16_SetChannelConfig(I1_SENSE_ADC_BASEADDR, I1_SENSE_ADC_CHANNEL_GROUP, &I_adcChannelConfigStruct);

        while (!I1_adcConversionDoneFlag)
        {
        }
        I1_voltRead = I1_voltRead + I_adcConversionValue;

      }

        voltRead = (I1_voltRead/ADCNooftimes);
 //       PRINTF("\r\nI1_ADC Voltage: %d\r\n", (uint16_t)voltRead);
//        dec_to_str((char *)dispstr, (uint16_t)voltRead, maxNoofCurDigits);

        voltRead = (float)(voltRead * (VREF_BRD / SE_12BIT));
     //   voltRead = voltRead / calib_divider;                 // To get value in mA................................................
        Load_Current_1 = voltRead +I_add_error;
        Load_Current_1 = Load_Current_1 / calib_divider; 
        delay();
//        float_to_str((char *)dispstr, Load_Current_1);

 //      PRINTF("\r\nI1_Value: %0.3f\r\n", Load_Current_1);
        if(DosageStartbit )
       {
      //   if(ProcessState == SHUTDOWN_STATE)
      //   {
      //       Load_Current_1 = 0;
      //   }
  //      dispLoadCurrent();
       }
        
//        dec_to_str((char *)dispstr, (uint16_t)voltRead, maxNoofCurDigits);
       
             voltRead = (I2_voltRead/ADCNooftimes);
 //     PRINTF("\r\nI2_ADC Voltage: %d\r\n", (uint16_t)voltRead);
//        dec_to_str((char *)dispstr, (uint16_t)voltRead, maxNoofCurDigits);
      voltRead = (float)(voltRead * (VREF_BRD / SE_12BIT));
      Load_Current_2 = voltRead;
      Load_Current_2 = Load_Current_2 / calib_divider; 
        delay();
//    float_to_str((char *)dispstr, Load_Current_2);

 //     PRINTF("\r\nI2_Value: %0.3f\r\n", Load_Current_2);
        if(DosageStartbit)
        {
     //     if(ProcessState == SHUTDOWN_STATE)
     //    {
      //      Load_Current_2 = 0;
     //    }
     //   dispLoadVoltage();
        if(Load_Current_2 >= Max_Voltage)
        {
          faultbit= 1;
        }
        }

      checkBatStatus();

}
 
void ADC_Deinitialise(void)
{
    ADC16_Deinit(I2_SENSE_ADC_BASEADDR);  
    ADC16_Deinit(I1_SENSE_ADC_BASEADDR);
}
  
    
static void ADC_Init(void)
{
    adc16_config_t adc16ConfigStruct;
    //dac_config_t dacConfigStruct;
    vref_config_t vrefConfigStruct;
    
    VREF_GetDefaultConfig(&vrefConfigStruct);
    VREF_Init(VREF, &vrefConfigStruct);

    /* Configure the ADC16. */
    /*
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    
        /*V sense ADC*/
    ADC16_Init(I2_SENSE_ADC_BASEADDR, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(I2_SENSE_ADC_BASEADDR, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(I2_SENSE_ADC_BASEADDR))
    {
        PRINTF("\r\nI2_SENSE_ADC_DoAutoCalibration() Done.");
    }
    else
    {
        PRINTF("I2_SENSE_ADC_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
    V_adcChannelConfigStruct.channelNumber = I2_SENSE_ADC_USER_CHANNEL;
    V_adcChannelConfigStruct.enableInterruptOnConversionCompleted = true;

    


#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    V_adcChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
    
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
      /*I sense ADC*/
    ADC16_Init(I1_SENSE_ADC_BASEADDR, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(I1_SENSE_ADC_BASEADDR, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(I1_SENSE_ADC_BASEADDR))
    {
        PRINTF("\r\nI1_SENSE_ADC_DoAutoCalibration() Done.");
    }
    else
    {
        PRINTF("I1_SENSE_ADC_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
   I_adcChannelConfigStruct.channelNumber = I1_SENSE_ADC_USER_CHANNEL;
   I_adcChannelConfigStruct.enableInterruptOnConversionCompleted = true;
    
    VBat_adcChannelConfigStruct.channelNumber = VBat_SENSE_ADC_USER_CHANNEL;
    VBat_adcChannelConfigStruct.enableInterruptOnConversionCompleted = true;
    
           VBat_adcChannelConfigStruct.enableDifferentialConversion = false;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
  I_adcChannelConfigStruct.enableDifferentialConversion = false;
        VBat_adcChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}

void I2_SENSE_ADC_IRQ_HANDLER_FUNC(void)
{

    I2_adcConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    V_adcConversionValue = ADC16_GetChannelConversionValue(I2_SENSE_ADC_BASEADDR, I2_SENSE_ADC_CHANNEL_GROUP); 

}

void I1_SENSE_ADC_IRQ_HANDLER_FUNC(void)
{
//  ADC_Channel = VBAT_CHANNEL;
  if(ADC_Channel == I_CHANNEL)
  {
    I1_adcConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    I_adcConversionValue = ADC16_GetChannelConversionValue(I1_SENSE_ADC_BASEADDR, I1_SENSE_ADC_CHANNEL_GROUP);
  }
  else if(ADC_Channel == VBAT_CHANNEL)
  {
    VBat_adcConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    VBat_adcConversionValue = ADC16_GetChannelConversionValue(VBat_SENSE_ADC_BASEADDR, VBat_SENSE_ADC_CHANNEL_GROUP);
  }
}

void APP_WAKEUP_BUTTON_IRQ_HANDLER(void)
{
//  /*
//    if ((1U << APP_WAKEUP_BUTTON_GPIO_PIN) & PORT_GetPinsInterruptFlags(APP_WAKEUP_BUTTON_PORT))
//    {
//        /* Disable interrupt. */
//   //     PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT, APP_WAKEUP_BUTTON_GPIO_PIN, kPORT_InterruptOrDMADisabled);
//   //     PORT_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_PORT, (1U << APP_WAKEUP_BUTTON_GPIO_PIN));
//    }
//*/
}

void LLWU_IRQHandler(void)
{
//    /* If wakeup by LPTMR. */
//    if (LLWU_GetInternalWakeupModuleFlag(LLWU, LLWU_LPTMR_IDX))
//    {
//        LPTMR_DisableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
//        LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
//        LPTMR_StopTimer(LPTMR0);
//    }
//    /* If wakeup by external pin. */
//    if (LLWU_GetExternalWakeupPinFlag(LLWU, LLWU_WAKEUP_PIN_IDX))
//    {
//        PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT, APP_WAKEUP_BUTTON_GPIO_PIN, kPORT_InterruptOrDMADisabled);
//        PORT_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_PORT, (1U << APP_WAKEUP_BUTTON_GPIO_PIN));
//        LLWU_ClearExternalWakeupPinFlag(LLWU, LLWU_WAKEUP_PIN_IDX);
//    }
}


/*! @brief Set wakeup timeout and wakeup source. */
void APP_SetWakeupConfig(app_power_mode_t targetMode)
{
  
     //   PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT, APP_WAKEUP_BUTTON_GPIO_PIN, APP_WAKEUP_BUTTON_IRQ_TYPE);
}


void dec_to_str (char* str, uint32_t val, size_t digits)
{
  size_t i=1u;
  
  for(; i<=digits; i++)
  {
    str[digits-i] = (char)((val % 10u) + '0');
    val/=10u;
  }

  str[i-1u] = '\0'; // assuming you want null terminated strings?
}

void float_to_str_3_1digit (char* str, float val)
{
val = ((float)(round(val*10)))/10;         // 10 is for 1 digit decimal
    
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 10);   // Turn into integer (123).                       // here the decimal is 2 digits
sprintf (str, "%03d.%01d", d1, d2); //decimal is 1 digits......
}

void float_to_str_3_2digit (char* str, float val)
{
val = ((float)(round(val*100)))/100;         // 100 is for 2 digit decimal
    
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 100);   // Turn into integer (123).                       // here the decimal is 2 digits
sprintf (str, "%03d.%02d", d1, d2); //decimal is 2 digits......
}

void float_to_str_1_3digit (char* str, float val)
{
val = ((float)(round(val*1000)))/1000;         // 1000 is for 3 digit decimal
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 1000);   // Turn into integer (123).                       // here the decimal is 3 digits
sprintf (str, "%01d.%03d", d1, d2); //decimal is 3 digits......
}

void float_to_str_2_1digit (char* str, float val)
{
val = ((float)(round(val*10)))/10;         // 10 is for 1 digits
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 10);   // Turn into integer (123).                       // here the decimal is 1 digits
sprintf (str, "%02d.%01d", d1, d2); //decimal is 1 digits......
}

void float_to_str_2_2digit (char* str, float val)
{
val = ((float)(round(val*100)))/100;         // 10 is for 1 digits
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 100);   // Turn into integer (123).                       // here the decimal is 1 digits
sprintf (str, "%02d.%02d", d1, d2); //decimal is 1 digits......
}

void float_to_str_1_2digit (char* str, float val)
{
val = ((float)(round(val*100)))/100;         // 10 is for 1 digits
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 100);   // Turn into integer (123).                       // here the decimal is 1 digits
sprintf (str, "%01d.%02d", d1, d2); //decimal is 1 digits......
}


 

void delay(void)
{
    volatile uint32_t i,j = 0;
    for (i = 0; i < 800000; ++i)
    {
    //   for (j = 0; j < 8; ++j)
    //   {
        __asm("NOP"); /* delay */
      // }
    }
}

void APP_PowerPreSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode)
{
    /* Wait for debug console output finished. */
    while (!(kUART_TransmissionCompleteFlag & UART_GetStatusFlags((UART_Type *)BOARD_DEBUG_UART_BASEADDR)))
    {
    }
    DbgConsole_Deinit();

    if ((kAPP_PowerModeRun != targetMode) && (kAPP_PowerModeHsrun != targetMode) && (kAPP_PowerModeVlpr != targetMode))
    {
        /*
         * Set pin for current leakage.
         * Debug console RX pin: Set to pinmux to disable.
         * Debug console TX pin: Don't need to change.
         */
        PORT_SetPinMux(DEBUG_CONSOLE_RX_PORT, DEBUG_CONSOLE_RX_PIN, kPORT_PinDisabledOrAnalog);
    }
}

void APP_PowerPostSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode)
{
    smc_power_state_t powerState = SMC_GetPowerModeState(SMC);

    /*
     * For some other platforms, if enter LLS mode from VLPR mode, when wakeup, the
     * power mode is VLPR. But for some platforms, if enter LLS mode from VLPR mode,
     * when wakeup, the power mode is RUN. In this case, the clock setting is still
     * VLPR mode setting, so change to RUN mode setting here.
     */
    if ((kSMC_PowerStateVlpr == originPowerState) && (kSMC_PowerStateRun == powerState))
    {
        //APP_SetClockRunFromVlpr();
    }

    if ((kAPP_PowerModeRun != targetMode) && (kAPP_PowerModeHsrun != targetMode) && (kAPP_PowerModeVlpr != targetMode))
    {
        /*
         * Debug console RX pin is set to disable for current leakage, nee to re-configure pinmux.
         * Debug console TX pin: Don't need to change.
         */
        PORT_SetPinMux(DEBUG_CONSOLE_RX_PORT, DEBUG_CONSOLE_RX_PIN, DEBUG_CONSOLE_RX_PINMUX);
    }

    /*
     * If enter stop modes when MCG in PEE mode, then after wakeup, the MCG is in PBE mode,
     * need to enter PEE mode manually.
     */
    if ((kAPP_PowerModeRun != targetMode) && (kAPP_PowerModeWait != targetMode) && (kAPP_PowerModeVlpw != targetMode) &&
        (kAPP_PowerModeHsrun != targetMode) && (kAPP_PowerModeVlpr != targetMode))
    {
        if (kSMC_PowerStateRun == originPowerState)
        {
            /* Wait for PLL lock. */
            while (!(kMCG_Pll0LockFlag & CLOCK_GetStatusFlags()))
            {
            }
            CLOCK_SetPeeMode();
        }
    }

    /* Set debug console clock source. */
    BOARD_InitDebugConsole();
}


void APP_PowerModeSwitch(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode)
{
    smc_power_mode_vlls_config_t vlls_config;
    vlls_config.enablePorDetectInVlls0 = true;
    smc_power_mode_lls_config_t lls_config;
    lls_config.subMode = kSMC_StopSub3;

    switch (targetPowerMode)
    {
        case kAPP_PowerModeVlpr:
//            APP_SetClockVlpr();
//            SMC_SetPowerModeVlpr(SMC);
//            while (kSMC_PowerStateVlpr != SMC_GetPowerModeState(SMC))
//            {
//            }
            break;

        case kAPP_PowerModeRun:
            /* If enter RUN from HSRUN, fisrt change clock. */
//            if (kSMC_PowerStateHsrun == curPowerState)
//            {
//                APP_SetClockRunFromHsrun();
//            }
//
//            /* Power mode change. */
//            SMC_SetPowerModeRun(SMC);
//            while (kSMC_PowerStateRun != SMC_GetPowerModeState(SMC))
//            {
//            }
//
//            /* If enter RUN from VLPR, change clock after the power mode change. */
//            if (kSMC_PowerStateVlpr == curPowerState)
//            {
//                APP_SetClockRunFromVlpr();
//            }
            break;

        case kAPP_PowerModeHsrun:
//            SMC_SetPowerModeHsrun(SMC);
//            while (kSMC_PowerStateHsrun != SMC_GetPowerModeState(SMC))
//            {
//            }
//
//            APP_SetClockHsrun(); /* Change clock setting after power mode change. */
            break;

        case kAPP_PowerModeWait:
  //          SMC_SetPowerModeWait(SMC);
            break;

        case kAPP_PowerModeStop:
  //          SMC_SetPowerModeStop(SMC, kSMC_PartialStop);
            break;

        case kAPP_PowerModeVlpw:
    //        SMC_SetPowerModeVlpw(SMC);
            break;

        case kAPP_PowerModeVlps:
            SMC_SetPowerModeVlps(SMC);
            break;

        case kAPP_PowerModeLls:
       //     SMC_SetPowerModeLls(SMC, &lls_config);
            break;

        case kAPP_PowerModeVlls0:
     //       vlls_config.subMode = kSMC_StopSub0;
    //        SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        case kAPP_PowerModeVlls1:
      //      vlls_config.subMode = kSMC_StopSub1;
      //      SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        case kAPP_PowerModeVlls2:
     //       vlls_config.subMode = kSMC_StopSub2;
      //      SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        case kAPP_PowerModeVlls3:
      //      vlls_config.subMode = kSMC_StopSub3;
      //      SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        default:
            PRINTF("Wrong value");
            break;
    }
}

 void Init_Pins (void)
 {
   port_pin_config_t input_pull_up_config;
   input_pull_up_config.pullSelect = kPORT_PullUp;
   input_pull_up_config.openDrainEnable = kPORT_OpenDrainDisable;
     
    GPIO_PinInit(BOARD_POWER_GPIO, BOARD_FAULT_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 1});
    GPIO_PinInit(BOARD_POWER_GPIO, BOARD_CHARGE_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 1});

    GPIO_PinInit(BOARD_BACKLIGHT_GPIO, BOARD_BACKLIGHT_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});
    //BACKLIGHT_ON;
    
    GPIO_PinInit(BOARD_SDCARD_CD_GPIO, BOARD_SDCARD_CD_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 1});
 //   PORT_SetPinConfig(BOARD_SDCARD_CD_PORT, BOARD_SDCARD_CD_GPIO_PIN, &input_pull_up_config);
        GPIO_PinInit(BOARD_SDCARD_CS_GPIO, BOARD_SDCARD_CS_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1});
    
    GPIO_PinInit(BOARD_RS485_GPIO, BOARD_RS485DE_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1});
    GPIO_PinInit(BOARD_RS485_GPIO, BOARD_RS485RE_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});
    
 //    Bridge_Shutdown();
  }



void USB_UART_Init(void)
{
  uart_config_t config;

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = USB_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(USB_UART, &config, CLOCK_GetFreq(USB_UART_CLKSRC));
        /* Enable RX interrupt. */
    UART_EnableInterrupts(USB_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable );
    EnableIRQ(USB_UART_IRQn);
}

void USB_UART_Tx(unsigned char *Txdata, uint8_t datasize)
{
          while(Uart_Tx_Buf.pendingbytes > 0);
          for(uint16_t datacount = 0; datacount < datasize; datacount++)
          {
            //while(((Uart_Tx_Buf.rxIndex + 1) % USB_UART_BUFFER_SIZE) == Uart_Tx_Buf.txIndex);/* If ring buffer is not full, add data to ring buffer. */
            Uart_Tx_Buf.data[Uart_Tx_Buf.rxIndex] = Txdata[datacount];   
            Uart_Tx_Buf.rxIndex++;
            Uart_Tx_Buf.rxIndex %= USB_UART_BUFFER_SIZE;
            Uart_Tx_Buf.pendingbytes++;
          }
          UART_EnableInterrupts(USB_UART, kUART_TxDataRegEmptyInterruptEnable);
          
}


void checkUSB_UARTdata(void)
{
  uint8_t rxdata;
    while (Uart_Rx_Buf.pendingbytes > 0)
    {
      rxdata = Uart_Rx_Buf.data[Uart_Rx_Buf.txIndex];
      Uart_Rx_Buf.txIndex = (Uart_Rx_Buf.txIndex + 1) % USB_UART_BUFFER_SIZE;
      Uart_Rx_Buf.pendingbytes--;
      if(rxdata == USB_UART_RxStartChar)
      {
        Usb_Uart.Rxdata_Count = 0;
        Usb_Uart.StartFlag = true;
        Usb_Uart.ReceivedFlag = false;
      }
      else if(rxdata == USB_UART_RxEndChar)
      {
        Usb_Uart.Rxdata[Usb_Uart.Rxdata_Count] = 0x00;           // Just add null character at the end
        Usb_Uart.Rxdata_Count_End = Usb_Uart.Rxdata_Count;            // Count without the NULL Character
        Usb_Uart.ReceivedFlag = true;
        break;
      }
      else if(Usb_Uart.StartFlag == true)
      {
        if(Usb_Uart.Rxdata_Count <  USB_UART_BUFFER_SIZE-1)
        {
        Usb_Uart.Rxdata[Usb_Uart.Rxdata_Count++] = rxdata;
        }
        else
        {
        Usb_Uart.Rxdata_Count = 0;
        Usb_Uart.StartFlag = false;
        }
      }
    }
}
  


void USB_UART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(USB_UART))
    {
        data = UART_ReadByte(USB_UART);
        /* If ring buffer is not full, add data to ring buffer. */
        if (((Uart_Rx_Buf.rxIndex + 1) % USB_UART_BUFFER_SIZE) != Uart_Rx_Buf.txIndex)
        {
            Uart_Rx_Buf.data[Uart_Rx_Buf.rxIndex] = data;
            Uart_Rx_Buf.rxIndex++;
            Uart_Rx_Buf.rxIndex %= USB_UART_BUFFER_SIZE;
            Uart_Rx_Buf.pendingbytes++;
        }
    }
    
    if((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(USB_UART)) && kUART_TxDataRegEmptyInterruptEnable)
    {
            if(Uart_Tx_Buf.pendingbytes)
            {
            UART_WriteByte(USB_UART, Uart_Tx_Buf.data[Uart_Tx_Buf.txIndex]);
            Uart_Tx_Buf.txIndex++;
            Uart_Tx_Buf.txIndex %= USB_UART_BUFFER_SIZE;
            Uart_Tx_Buf.pendingbytes--;
            }
            else
            {
              UART_DisableInterrupts(USB_UART,kUART_TxDataRegEmptyInterruptEnable);
            }
    }
}

void Gen_Time_routine (void)
{
   if(++Int_time.milliSec >= 10)                       // Interrupt comes 0.1Secsonds(i.e every 100mSec)
    {
      Int_time.milliSec = 0;
      if(++Int_time.Sec >= 60)
      {
     //   LED2_TOGGLE();
        Int_time.Sec = 0;
        ++Int_time.Min;
      }
    }
    if(++Int_BPtime.milliSec >= 10)
    {
      Int_BPtime.milliSec = 0;
      if(++Int_BPtime.Sec >= 60)
      {
   //     LED2_TOGGLE();
        Int_BPtime.Sec = 0;
        ++Int_BPtime.Min;
      }
    }
}

void LPTMR_HANDLER(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
    if(gen_timecount >= 100)                   // Called every 100 milliseconds
      {
        gen_timecount = 0;
        Gen_Time_routine();
      }
    else
    {
      gen_timecount++;
    }
    SDC_Timer_handler();
 //   lptmrCounter++;
}


void LPTMR_Initialisation(void)
{
   lptmr_config_t lptmrConfig;
    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);

    /* Initialize the LPTMR */
    LPTMR_Init(LPTMR0, &lptmrConfig);

    /* Set timer period */
    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(LPTMR_TIME, LPTMR_SOURCE_CLOCK));

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(LPTMR0_IRQn);

    PRINTF("Low Power Timer Initialisation\r\n");

    /* Start counting */
    LPTMR_StartTimer(LPTMR0);
  
  
}

//void MODBUS_TMR_HANDLER(void)
//{
//    /* Clear interrupt flag.*/
//    PIT_ClearStatusFlags(MODBUS_TMR, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
//  //  pitIsrFlag = true;
//    LED1_TOGGLE();
//}

//void MODBUS_TMR_Initialisation (void)
//{
//    pit_config_t pitConfig;
//    /*
//     * pitConfig.enableRunInDebug = false;
//     */
//    PIT_GetDefaultConfig(&pitConfig);
//      /* Init pit module */
//    PIT_Init(MODBUS_TMR, &pitConfig);
//
//    /* Set timer period for channel 0 */
//    PIT_SetTimerPeriod(MODBUS_TMR, kPIT_Chnl_0, USEC_TO_COUNT(1000000U, MODBUS_TMR_SOURCE_CLOCK));
//
//    /* Enable timer interrupts for channel 0 */
//    PIT_EnableInterrupts(MODBUS_TMR, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
//
//    /* Enable at the NVIC */
//    EnableIRQ(MODBUS_TMR_IRQ_ID);
//
//    /* Start channel 0 */
//    PIT_StartTimer(MODBUS_TMR, kPIT_Chnl_0);
//}


void disptime (time_data disp_time)
{
  /*
        uint16_t disp_hour, disp_min;
        unsigned char dispstr[4];
        uint16_t disp_hr_x_pos, disp_min_x_pos, disp_sec_x_pos, disp_hr_col_x_pos, disp_min_col_x_pos;
        DisplayDataConfig TimeVal_DataConfig;
        TimeVal_DataConfig = CurVal_DataConfig;
        TimeVal_DataConfig.x_width = 16;
        disp_hour = (uint16_t)(disp_time.Min/60);
        disp_min = disp_time.Min - (disp_hour *60);
        TimeVal_DataConfig.y_pos = TimeVal_Y_pos;
        
        disp_hr_x_pos = TimeVal_X_pos;
        disp_hr_col_x_pos = disp_hr_x_pos + ((1*TimeVal_DataConfig.x_width)-6);
        disp_min_x_pos = disp_hr_col_x_pos + (TimeVal_DataConfig.x_width-6);
        disp_min_col_x_pos = disp_min_x_pos + ((2*TimeVal_DataConfig.x_width)-8);
        disp_sec_x_pos = disp_min_col_x_pos + (TimeVal_DataConfig.x_width-6);
 
        if(!timedispfirstbit)
        {
        TimeVal_DataConfig.x_pos = disp_hr_col_x_pos;
        DisplayChar_big(&TimeVal_DataConfig, ':');
        
        TimeVal_DataConfig.x_pos = disp_min_col_x_pos;
        DisplayChar_big(&TimeVal_DataConfig, ':');
        
        timedispfirstbit = 1;
        }
        
        dec_to_str((char *)dispstr, disp_hour, 1);
        TimeVal_DataConfig.x_pos = disp_hr_x_pos;
        DisplayString_big(&TimeVal_DataConfig, dispstr);
         
        TimeVal_DataConfig.x_pos = disp_min_x_pos;
        dec_to_str((char *)dispstr, disp_min, 2);
        DisplayString_big(&TimeVal_DataConfig, dispstr);
        
        TimeVal_DataConfig.x_pos = disp_sec_x_pos;
        dec_to_str((char *)dispstr, disp_time.Sec, 2);
        DisplayString_big(&TimeVal_DataConfig, dispstr);  
    //    TimeVal_DataConfig.x_pos = TimeVal_DataConfig.x_pos + ((2*TimeVal_DataConfig.x_width)-5);
   //     DisplayChar_big(&TimeVal_DataConfig, ':'); 
   //     TimeVal_DataConfig.x_pos = TimeVal_DataConfig.x_pos + (TimeVal_DataConfig.x_width-5);
   //     dec_to_str((char *)dispstr, disp_time.milliSec, 1);
   //     DisplayString_big(&TimeVal_DataConfig, dispstr); 
  */
}

//void dispdata(float cur1, float cur2)
//{
// // float_to_str((char *)dispstr, volt);
//   float_to_str_1_2digit((char *)String1, cur1);
//    lcd_command(0xC0);
//    lcd_datas("I1:");
//    lcd_datas(String1);
//    lcd_data('A');
//    lcd_command(0xCB);
//    lcd_datas("I2:"); 
// //   float_to_str((char *)dispstr, volt);
//   float_to_str_1_2digit((char *)String2, cur2);  
//    lcd_datas(String2);
//    lcd_data('A');
//}
//
//void disp_pdiff(float per_diff, uint8_t sign)
//{
//    lcd_command(0x94);
//    lcd_datas("SR:");
//    if(sign == INF)
//    {
//          lcd_datas("INF             ");
//          return;
//    }
//    else if(sign == NEG)
//    {
//          lcd_data('-');
//    }
//    else
//    {
//         lcd_data(' ');
//    }
//    float_to_str_3_2digit((char *)dispstr, per_diff);
//    lcd_datas(dispstr);
//    lcd_datas(" %");
//}
//
//void dispBatVolt (float Bat_Volt)
//{
//  lcd_command(0xd4);
//  lcd_datas("BAT VOLT:");
//  float_to_str_1_2digit((char *)dispstr, Bat_Volt); 
//  lcd_datas(dispstr);
//  lcd_datas(" V");
//}



void float_to_str_3f (char* str, float val)
{
val = ((float)(round(val*1000)))/1000;         // 1000 is for 3 digit decimal
    
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 1000);   // Turn into integer (123).                       // here the decimal is 3 digits
sprintf (str, "%d.%03d", d1, d2); //decimal is 3 digits......
}
void float_to_str_2f (char* str, float val)
{
val = ((float)(round(val*100)))/100;         // 100 is for 2 digit decimal
    
int d1 = (int)val;            // Get the integer part (678).
float f2 = val - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
int d2 = (int)(f2 * 100);   // Turn into integer (123).                       // here the decimal is 2 digits
sprintf (str, "%d.%02d", d1, d2); //decimal is 2 digits......
}
