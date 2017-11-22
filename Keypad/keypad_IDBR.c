#include "keypad_IDBR.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
//#include "fsl_gpio.c"




void keypad_InitPins(void)
{
    port_pin_config_t keypad_input_config = {
    kPORT_PullUp,
    kPORT_FastSlewRate,
    kPORT_PassiveFilterDisable, 
    kPORT_LowDriveStrength,
    kPORT_OpenDrainDisable,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister  
  };
 
  port_pin_config_t keypad_output_config = {
    kPORT_PullDisable,
    kPORT_FastSlewRate,
    kPORT_PassiveFilterDisable, 
    kPORT_OpenDrainDisable,
    kPORT_LowDriveStrength,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister  
  }; 
  
   static PORT_Type *const k_portBases[] = PORT_BASE_PTRS;
   uint8_t instance;
   PORT_Type *portBase;
   instance = GPIO_GetInstance(BOARD_KEYPAD_GPIO);
   portBase = k_portBases[instance];
    
  PORT_SetMultiplePinsConfig(portBase, KEYPAD_INPUT_PIN_MASK, &keypad_input_config);
  PORT_SetMultiplePinsConfig(portBase, KEYPAD_OUTPUT_PIN_MASK, &keypad_output_config);
   
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_R1, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_R2, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_R3, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_R4, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_C1, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_C2, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_C3, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_C4, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});

  

 }

void keypad_init_PowerSaver (void)
{
     PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_R1,kPORT_PinDisabledOrAnalog);
          PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_R2,kPORT_PinDisabledOrAnalog);
               PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_R3,kPORT_PinDisabledOrAnalog);
                  //  PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_R1,kPORT_PinDisabledOrAnalog);
               
     PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_C1,kPORT_PinDisabledOrAnalog);
     PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_C2,kPORT_PinDisabledOrAnalog);
   //  PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_C3,kPORT_PinDisabledOrAnalog);
     PORT_SetPinMux(BOARD_KEYPAD_PORT, BOARD_KEYPAD_GPIO_PIN_C4,kPORT_PinDisabledOrAnalog);
   
  GPIO_PinInit(BOARD_KEYPAD_GPIO, BOARD_KEYPAD_GPIO_PIN_C3, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});

}
		
unsigned char key_get (void)
{

        const char keypad_chars[] = {
          1, 2, 3, 12,
          4, 5, 6, 13,
          7, 8, 9, 14,
          10,0,11, 15
        };
    	uint8_t keydata;

//	char overflag;
//	overflag  = 0;
	GPIO_ClearPinsOutput(BOARD_KEYPAD_GPIO, KEYPAD_OUTPUT_PIN_MASK);
        keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
        if(keydata == 0x0f)
        {
           keydata = 0xff;
           return(keydata);
        }
        key_delay();
        keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
        if(keydata == 0x0f)
        {
           keydata = 0xff;
           return(keydata);
        }
        
	GPIO_SetPinsOutput(BOARD_KEYPAD_GPIO, KEYPAD_OUTPUT_PIN_MASK);
        GPIO_ClearPinsOutput(BOARD_KEYPAD_GPIO, (1<<BOARD_KEYPAD_GPIO_PIN_C1));
        key_delay();
        keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
         if(keydata != 0x0f)
        {
           keydata = ~keydata & 0x0f;
           if(keydata == 1)
             return(keypad_chars[0]);
          else if (keydata == 2)
            return(keypad_chars[4]);
          else if (keydata == 4)
            return(keypad_chars[8]);
          else
            return(keypad_chars[12]);
        }
        GPIO_SetPinsOutput(BOARD_KEYPAD_GPIO, KEYPAD_OUTPUT_PIN_MASK);
        GPIO_ClearPinsOutput(BOARD_KEYPAD_GPIO, (1<<BOARD_KEYPAD_GPIO_PIN_C2));
        key_delay();
        keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
        if(keydata != 0x0f)
        {
           keydata = ~keydata & 0x0f;
           if(keydata == 1)
            return(keypad_chars[0+1]);
          else if (keydata == 2)
            return(keypad_chars[4+1]);
          else if (keydata == 4)
            return(keypad_chars[8+1]);
          else
            return(keypad_chars[12+1]);
        }
        
        GPIO_SetPinsOutput(BOARD_KEYPAD_GPIO, KEYPAD_OUTPUT_PIN_MASK);
        GPIO_ClearPinsOutput(BOARD_KEYPAD_GPIO, (1<<BOARD_KEYPAD_GPIO_PIN_C3));
        key_delay();
        keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
        if(keydata != 0x0f)
        {
            keydata = ~keydata & 0x0f;
           if(keydata == 1)
            return(keypad_chars[0+2]);
          else if (keydata == 2)
            return(keypad_chars[4+2]);
          else if (keydata == 4)
            return(keypad_chars[8+2]);
          else
            return(keypad_chars[12+2]);
        }
        
        GPIO_SetPinsOutput(BOARD_KEYPAD_GPIO, KEYPAD_OUTPUT_PIN_MASK);
        GPIO_ClearPinsOutput(BOARD_KEYPAD_GPIO, (1<<BOARD_KEYPAD_GPIO_PIN_C4));
        key_delay();
        keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
        if(keydata != 0x0f)
        {
           keydata = ~keydata & 0x0f;
          if(keydata == 1)
            return(keypad_chars[0+3]);
          else if (keydata == 2)
            return(keypad_chars[4+3]);
          else if (keydata == 4)
            return(keypad_chars[8+3]);
          else
            return(keypad_chars[12+3]);
        }
        
        keydata = 0xff;
        return(keydata);
 }

unsigned char key_press (void)
{
      	uint8_t keydata;
        do
        {
        keydata = key_get();
        }while(keydata == 0xff);
     //   GPIO_ClearPinsOutput(BOARD_KEYPAD_GPIO, KEYPAD_OUTPUT_PIN_MASK);
     //   key_delay();
     //   do {
     //   keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
     //   }   while(keydata == 0x0f);
        key_delay();
        return (keydata);
}

void key_release (void)
{
       	uint8_t keydata;
        GPIO_ClearPinsOutput(BOARD_KEYPAD_GPIO, KEYPAD_OUTPUT_PIN_MASK);
        key_delay();
        do {keydata = (GPIO_ReadPortInput(BOARD_KEYPAD_GPIO) & KEYPAD_INPUT_PIN_MASK) >> No_of_Shifts;          // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.
            }while(keydata != 0x0f);
        key_delay();
        key_delay();
        key_delay();
}

void key_delay(void)
{
  for(uint32_t i=0; i<500000; i++)
  {
            __asm("NOP"); /* delay */
  }
  
  
}