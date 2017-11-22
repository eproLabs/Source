

//#include "board.h"

//#define BOARD_KEYPAD_GPIO GPIOC
//#define BOARD_KEYPAD_GPIO_PIN_C1 4U  C4
//#define BOARD_KEYPAD_GPIO_PIN_C2 5U  R4   
//#define BOARD_KEYPAD_GPIO_PIN_C3 6U   R3  
//#define BOARD_KEYPAD_GPIO_PIN_R1 7U     R2
//#define BOARD_KEYPAD_GPIO_PIN_R2 8U     R1
//#define BOARD_KEYPAD_GPIO_PIN_R3 9U     C3
//#define BOARD_KEYPAD_GPIO_PIN_R4 10U    C2
//#define BOARD_KEYPAD_GPIO_PIN_C4 11U  C1          // NOT USED

#define BOARD_KEYPAD_PORT PORTC
#define BOARD_KEYPAD_GPIO GPIOC
#define BOARD_KEYPAD_GPIO_PIN_C4 4U             // Not Used
#define BOARD_KEYPAD_GPIO_PIN_R1 5U    
#define BOARD_KEYPAD_GPIO_PIN_R2 6U    
#define BOARD_KEYPAD_GPIO_PIN_R3 7U    
#define BOARD_KEYPAD_GPIO_PIN_R4 8U     
#define BOARD_KEYPAD_GPIO_PIN_C3 9U    
#define BOARD_KEYPAD_GPIO_PIN_C2 10U  
#define BOARD_KEYPAD_GPIO_PIN_C1 11U   

#define No_of_Shifts BOARD_KEYPAD_GPIO_PIN_R1   // Here BOARD_KEYPAD_GPIO_PIN_R4 is the number of times the port value has to be shifted to store the value in the last four bytes.

#define KEYPAD_OUTPUT_PIN_MASK ((1<<BOARD_KEYPAD_GPIO_PIN_C1)| (1<<BOARD_KEYPAD_GPIO_PIN_C2) |(1<<BOARD_KEYPAD_GPIO_PIN_C3)|(1<<BOARD_KEYPAD_GPIO_PIN_C4))

#define KEYPAD_INPUT_PIN_MASK ((1<<BOARD_KEYPAD_GPIO_PIN_R1)| (1<<BOARD_KEYPAD_GPIO_PIN_R2) |(1<<BOARD_KEYPAD_GPIO_PIN_R3)|(1<<BOARD_KEYPAD_GPIO_PIN_R4))

#define Can 10
#define Ent 11

//
//#define SET_KEY 1
//#define MODE_KEY 2
//#define ONOFF_KEY 3
//#define DOSINC_KEY 4
//#define DOSDEC_KEY 7
//#define CURINC_KEY 5
//#define CURDEC_KEY 8
//#define BPINC_KEY 6
//#define BPDEC_KEY 9
//#define START_KEY 10
//#define PAUSE_KEY 0
//#define STOP_KEY 11

#define SET_KEY 10
#define MODE_KEY 0
#define ONOFF_KEY 11
#define DOSINC_KEY 7
#define DOSDEC_KEY 4
#define CURINC_KEY 8
#define CURDEC_KEY 5
#define BPINC_KEY 9
#define BPDEC_KEY 6
#define START_KEY 1
#define PAUSE_KEY 2
#define STOP_KEY 3


void keypad_InitPins(void);
unsigned char key_get (void);
unsigned char key_press(void); // blocking function
void key_release (void); // blocking function
void key_delay(void);
void keypad_init_PowerSaver (void);
