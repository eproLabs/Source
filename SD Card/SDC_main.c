/*
IBEX UK LTD http://www.ibexuk.com
Electronic Product Design Specialists
RELEASED SOFTWARE

The MIT License (MIT)

Copyright (c) 2013, IBEX UK Ltd, http://ibexuk.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
//Project Name:		FAT FILING SYSTEM FAT16 & FAT 32 DRIVER
//NXP LPC2365 SAMPLE PROJECT C CODE FILE




//----- INCLUDE FILES FOR THIS SOURCE CODE FILE -----
#include "sdglobal.h"					//Global data type definitions (see https://github.com/ibexuk/C_Generic_Header_File )
#define	MAIN_C						//(Define used for following header file to flag that it is the header file for this source file)
#include "SDC_main.h"				//(Include header file for this source file)

//----- OTHER PROJECT FILES REQUIRED BY THIS SOURCE CODE FILE -----
#include "mem-ffs.h"

//----- COMPILER LIBRARY FILES REQUIRED BY THIS SOURCE CODE FILE -----
#include <stdio.h>					//(Needed for printf)
//#include <stdlib.h>
#include <string.h>					//Used in the example project code

#include "fsl_debug_console.h"




const char filename_all[] = {"*.*"};
const char filename_test_txt[] = {"ETTest1.txt"};
const char filename_test_csv[] = {"ETTest2.csv"};

const char read_access_mode[] = {"r"};
const char write_access_mode[] = {"w"};
const char append_access_mode[] = {"a"};
const char read_write_access_mode[] = {"r+"};
const char write_read_access_mode[] = {"w+"};
const char append_read_access_mode[] = {"a+"};

const char text_file_line1_string[] = {"L&T Constructions - Solar Panel Monitoring\r\n********************************************************\r\n\r\n"};  //('\r' = carriage return, '\n' = line feed)
const char text_file_line2_string[] = {"********************************************************\r\n"};
//const char spreadsheet_file_no_text_file_string[] = {"The text.txt file could not be read!"};

static FFS_FILE *our_file_0;
BYTE b_temp;
char c_string_buffer[60];
int i_temp;

int CheckSDCard(void);
bool CreateTextFile (char *filename);
bool WriteTextFile (char *filename, unsigned char *WriteString);
//***********************************
//***********************************
//********** MAIN FUNCTION **********
//***********************************
//***********************************

int CheckSDCard (void)
{
  ffs_process();
  return(ffs_card_ok);    
}

bool CreateTextFile (char *filename)
{
   CheckSDCard();
   if(!ffs_card_ok)
   {
     return(FAILURE);
   } 
   our_file_0 = ffs_fopen(filename, append_access_mode);
   if (our_file_0 != 0)
   {
        ffs_fwrite(text_file_line1_string, sizeof(char), (int)strlen(text_file_line1_string), our_file_0);
        if(ffs_fclose(our_file_0))
        {
         return(FAILURE);
        }
   }
   else
   {
     	ffs_fclose(our_file_0);
        return(FAILURE);
   }  
   PRINTF("Created\r\n");

   return(SUCCESS);
}

bool WriteTextFile (char *filename, unsigned char *WriteString)
{
   CheckSDCard();
   if(!ffs_card_ok)
   {
     return(FAILURE);
   }
   our_file_0 = ffs_fopen(filename, append_read_access_mode);
   if (our_file_0 != 0)
   {
        ffs_fwrite(WriteString, sizeof(char), (int)strlen((char const *)WriteString), our_file_0);
        if (ffs_fclose(our_file_0))
	{
          return(FAILURE);
        }
   }
   else
   {
	ffs_fclose(our_file_0);
        return(FAILURE);
   }  
      PRINTF("Written\r\n");
      return(SUCCESS);
}
//
//int sdcard_routine(void)
//{
//
//
//	//**********************
//	//**********************
//	//***** INITIALISE *****
//	//**********************
//	//**********************
//	//sd_initialise();
//
//
//
//        PRINTF("SD Card Initialisation\r\n");
//
//	//*********************
//	//*********************
//	//***** MAIN LOOP *****
//	//*********************
//	//*********************
//	
//		//----- PROCESS MODE -----
//		process_mode();
//
//
//		//----- PROCESS FAT FILING SYSTEM -----
//		ffs_process();
//
//return(0);
//}
//


void process_mode (void)
{




	//----- IF CARD IS REMOVED ENSURE WE RESET BACK WAITING FOR CARD TO BE INSERTED -----
	if ((test_application_state != TA_PROCESS_WAIT_FOR_CARD) && (!ffs_card_ok))
		test_application_state = TA_PROCESS_WAIT_FOR_CARD;

	//---------------------------------
	//----- PROCESS STATE MACHINE -----
	//---------------------------------
	switch (test_application_state)
	{
	case TA_PROCESS_WAIT_FOR_CARD:
		//-------------------------------------------
		//----- WAITING FOR CARD TO BE INSERTED -----
		//-------------------------------------------
		LED1(1);
		LED2(0);
		if (ffs_card_ok)
		{
			//A CARD HAS BEEN INSERTED AND IS READY TO ACCESS
			test_application_state = TA_PROCESS_CARD_INSERTED;
		}
		break;

	case TA_PROCESS_CARD_INSERTED:
		//----------------------------
		//----- CARD IS INSERTED -----
		//----------------------------
		LED1(0);
		LED2(1);
		if (1)  //SWITCH_1_NEW_PRESS)
		{
			//A CARD HAS BEEN INSERTED AND IS READY TO ACCESS
			test_application_state = TA_PROCESS_OPERATE_CARD;
		}
		break;

	case TA_PROCESS_OPERATE_CARD:
//		//-------------------------------------------------
//		//----- DELETE ALL FILES AND CREATE NEW FILES -----
//		//-------------------------------------------------
//
//		//ALL FILES IN THE ROOT DIRECTORY ARE DELETED
//		//A NEW FILE CALLED TEST.TXT IS CREATED CONTAINING EXAMPLE TEST DATA.
//		//A NEW SPREADSHEET FILE CALLED TEST.CSV IS CREATED CONTAINING TEST DATA FROM THE TEST.TXT FILE
//
//		//----- DELETE ALL FILES IN THE ROOT DIRECTORY -----
//		while (ffs_remove(filename_all) == 0)
//			;
//
//		//----- CREATE NEW FILE FOR WRITING -----
//		our_file_0 = ffs_fopen(filename_test_txt, write_access_mode);
//		if (our_file_0 != 0)
//		{
//			//----- FILE WAS SUCESSFULLY CREATED -----
//
//			//----- WRITE STRING DATA -----
//			if (ffs_fputs(text_file_line1_string, our_file_0) == FFS_EOF)
//			{
//				test_application_state = TA_PROCESS_ERROR;	//Write error
//				break;
//			}
//
//			//----- WRITE A DATA BLOCK -----
//			ffs_fwrite(text_file_line2_string, sizeof(char), (int)strlen(text_file_line2_string), our_file_0);
//
//			//----- WRITE INDIVIDUAL BYTES OF DATA -----
//			for (b_temp = '0'; b_temp <= '9'; b_temp++)
//			{
//				if (ffs_fputc((int)b_temp, our_file_0) == FFS_EOF)
//				{
//					test_application_state = TA_PROCESS_ERROR;	//Write error
//					break;
//				}
//			}
//
//			//----- CLOSE THE FILE -----
//			if (ffs_fclose(our_file_0))
//			{
//				test_application_state = TA_PROCESS_ERROR;	//Error - could not close
//				break;
//			}
//
//		}
//		else
//		{
//			//----- ERROR - THE FILE COULD NOT BE CREATED -----
//			test_application_state = TA_PROCESS_ERROR;
//			break;
//		}
//
//		//----- CREATE NEW FILE FOR WRITING -----
//		our_file_0 = ffs_fopen(filename_test_csv, write_access_mode);
//		if (our_file_0 != 0)
//		{
//			//----- FILE WAS SUCESSFULLY CREATED -----
//
//			//----- OPEN THE TEST.TXT FILE FOR READING IF IT EXISTS -----
//			our_file_1 = ffs_fopen(filename_test_txt, read_access_mode);
//			if (our_file_1 != 0)
//			{
//				//----- THE TEST.TXT FILE DOES EXIST -----
//				//WRITE TO THE NEW SPREADSHEET FILE USING DATA FROM THE TEXT FILE
//
//				//READ THE FIRST LINE FROM THE TEXT FILE
//				if (ffs_fgets(c_string_buffer, 30, our_file_1) == 0)
//				{
//					test_application_state = TA_PROCESS_ERROR;		//Error - end of line not detected
//					break;
//				}
//
//				//WRITE TO SPREADSHEET FILE
//				if (ffs_fputs_char(c_string_buffer, our_file_0) == FFS_EOF)
//				{
//					test_application_state = TA_PROCESS_ERROR;		//Write error
//					break;
//				}
//
//				//READ NEXT LINE AS A DATA BLOCK FROM THE TEXT FILE
//				ffs_fread(c_string_buffer, sizeof(char), (int)strlen(text_file_line2_string), our_file_1);
//
//				//WRITE TO SPREADSHEET FILE
//				ffs_fwrite(c_string_buffer, sizeof(char), (int)strlen(text_file_line2_string), our_file_0);
//
//				//WRITE SEVERAL COLUMNS  THE SPREADSHEET FILE
//				ffs_fputc((int)'\r', our_file_0);
//				ffs_fputc((int)'\n', our_file_0);
//
//				for (b_temp = 'A'; b_temp <= 'Z'; b_temp++)
//				{
//					ffs_fputc((int)b_temp, our_file_0);
//					ffs_fputc((int)',', our_file_0);
//				}
//
//				//CHECK TO SEE IF ANY ERROR HAS OCCURED
//				if (ffs_ferror(our_file_0))
//				{
//					test_application_state = TA_PROCESS_ERROR;
//					break;
//				}
//
//
//				//WRITE THE LAST CHARACTER OF THE TEXT FILE TO THE START OF THE NEXT ROW
//				ffs_fputc((int)'\r', our_file_0);
//				ffs_fputc((int)'\n', our_file_0);
//
//				ffs_fseek(our_file_1, 0, FFS_SEEK_END);
//				i_temp = ffs_fgetc(our_file_1);
//				if (i_temp == FFS_EOF)
//				{
//					test_application_state = TA_PROCESS_ERROR;		//Read error
//					break;
//				}
//
//				ffs_fputc(i_temp, our_file_0);
//
//				ffs_fputc((int)'\r', our_file_0);
//				ffs_fputc((int)'\n', our_file_0);
//
//				//CLOSE THE TEST.TXT FILE
//				if (ffs_fclose(our_file_1))
//				{
//					test_application_state = TA_PROCESS_ERROR;		//Error - could not close
//					break;
//				}
//			}
//			else
//			{
//				//----- THE TEST.TXT FILE DOES NOT EXIST -----
//				//WRITE TO THE NEW SPREADSHEET FILE WITHOUT DATA FROM THE TEXT FILE
//
//				if (ffs_fputs(spreadsheet_file_no_text_file_string, our_file_0) == FFS_EOF)
//				{
//					test_application_state = TA_PROCESS_ERROR;		//Write error
//					break;
//				}
//			}
//
//
//
//			//----- CLOSE THE SPREADSHEET FILE -----
//			if (ffs_fclose(our_file_0))
//			{
//				test_application_state = TA_PROCESS_ERROR;		//Error - could not close
//				break;
//			}
//		}
//		else
//		{
//			//----- ERROR - THE FILE COULD NOT BE CREATED -----
//			test_application_state = TA_PROCESS_ERROR;
//			break;
//		}
//
//		//----- SUCCESS -----
//		test_application_state = TA_PROCESS_CARD_OPERTATION_DONE;
		break;

	case TA_PROCESS_CARD_OPERTATION_DONE:
//		//-----------------------------------------------------------------------------
//		//----- OPERATION DONE - INDICATE SUCCESS AND WAIT FOR CARD TO BE REMOVED -----
//		//-----------------------------------------------------------------------------
//		LED1(0);
//		LED2(0);
		break;


	case TA_PROCESS_ERROR:
//		//--------------------------------------------------------------------------
//		//----- ERROR OCCURED - INDICATE ERROR AND WAIT FOR CARD TO BE REMOVED -----
//		//--------------------------------------------------------------------------
//		LED1(1);
//		LED2(1);
//
//		//Try and close the files if they are open
//		ffs_fclose(our_file_0);
//		ffs_fclose(our_file_1);

		break;



	} //switch (test_application_state)



}















//*************************************************************************************************************************************
//*************************************************************************************************************************************
//*************************************************************************************************************************************
//*************************************************************************************************************************************
//*************************************************************************************************************************************
//*************************************************************************************************************************************
//*************************************************************************************************************************************

//************************************************
//************************************************
//********** INTERRUPT VECTOR LOCATIONS **********
//************************************************
//************************************************

//*******************************************
//*******************************************
//********** TIMER 0 HEARTBEAT IRQ **********
//*******************************************
//*******************************************
void SDC_Timer_handler (void)
{
	static BYTE hb_10ms_timer = 0;
	static BYTE hb_100ms_timer = 0;
	static WORD hb_1sec_timer = 0;


	//T0IR = 0x3f;							//Reset irq

	//-----------------------------
	//-----------------------------
	//----- HERE EVERY 1 mSec -----
	//-----------------------------
	//-----------------------------



	hb_10ms_timer++;
	if (hb_10ms_timer == 10)
	{
		//------------------------------
		//------------------------------
		//----- HERE EVERY 10 mSec -----
		//------------------------------
		//------------------------------
		hb_10ms_timer = 0;


		//----- GENERAL USE 10mS TIMER -----
		if (general_use_10ms_timer)
			general_use_10ms_timer--;


		//----- READ SWITCHES FLAG -----
		read_switches_flag = 1;

		//----- USER MODE 10mS TIMER -----
		if (user_mode_10ms_timer)
			user_mode_10ms_timer--;


		//----- FAT FILING SYSTEM DRIVER TIMER -----
		if (ffs_10ms_timer)
			ffs_10ms_timer--;

	} //if (hb_10ms_timer == 10)

	hb_100ms_timer++;
	if (hb_100ms_timer == 100)
	{
		//-------------------------------
		//-------------------------------
		//----- HERE EVERY 100 mSec -----
		//-------------------------------
		//-------------------------------
		hb_100ms_timer = 0;

		//----- GENERAL USE 100mS TIMER -----
		if (general_use_100ms_timer)
			general_use_100ms_timer--;


	} //if (hb_100ms_timer == 100)

	hb_1sec_timer++;
	if (hb_1sec_timer == 1000)
	{
		//----------------------------
		//----------------------------
		//----- HERE EVERY 1 Sec -----
		//----------------------------
		//----------------------------
		hb_1sec_timer = 0;





	} //if (hb_1sec_timer == 1000)


}	





