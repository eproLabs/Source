#ifndef _MODBUS_MAIN_H_
#define _MODBUS_MAIN_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */
 
  
  
extern int Modbus_routine( void );
extern bool  Modbus_Init (void);
  
extern void  Update_Modbus(int16_t *Modbus_Reg, uint8_t No_of_UpdateData);
  
  
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif 

