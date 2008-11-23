/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
// #include "user_Serialdrv.h"

extern unsigned int rc_ana_in01_value, rc_ana_in02_value, rc_ana_in03_value, rc_ana_in04_value;
extern unsigned int rc_ana_in05_value, rc_ana_in06_value, rc_ana_in07_value, rc_ana_in08_value;
unsigned int Auto_Loop;
unsigned int Auto_Step;

extern rotcounter leftcounter;
/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/

/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata")

void InterruptHandlerLow ()     
{
	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}



//  ***  IFI Code Starts Here***
//                              
//  unsigned char int_byte;       
//  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
//  { 
//    INTCON3bits.INT2IF = 0;
//  }
//  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
//  {
//    INTCON3bits.INT3IF = 0;
//  }
//  else if (INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
//  {
//    int_byte = PORTB;          /* You must read or write to PORTB */
//    INTCONbits.RBIF = 0;     /*     and clear the interrupt flag         */
//  }                                        /*     to clear the interrupt condition.  */
//  else
//  { 
//    CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
//  }
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
 static unsigned int Auto_Mode=0;
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

  Auto_Mode=RC_dip_8;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_7;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_6;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_5;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_4;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_3;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_2; 
  Auto_Mode=(Auto_Mode << 1) | RC_dip_1;
  Auto_Step=1;
  Auto_Loop=0;

//Auto_Mode=13;

RC_Motor_Drive_Left=255;
RC_Motor_Drive_Right=0;
RC_Trans=0;

if (Auto_Mode==1 || Auto_Mode==2 || Auto_Mode==3 || Auto_Mode==4 || Auto_Mode==5 || Auto_Mode==6 || Auto_Mode==7 || Auto_Mode==8 || Auto_Mode==21 || Auto_Mode==22) {
		RC_Motor_Drive_Left=255;
		RC_Motor_Drive_Right=0;
		RC_Trans=255;
}

if (Auto_Mode==0){
		RC_Motor_Drive_Left=127;
		RC_Motor_Drive_Right=127;
		RC_Trans=255;		
}


  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
//        printf("Auto_Mode: %d, Auto_Step: %d, Auto_Loop: %d\r\n", Auto_Mode, Auto_Step, Auto_Loop);
        Auto_Loop++;

		switch (Auto_Mode) {
			case 1:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==62){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 2:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==104){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 3:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==62){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(255,0);
						}
						break;
					case 3: 
						if (Auto_Loop==10){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==42){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 4:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==62){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(0,255);
						}
						break;
					case 3: 
						if (Auto_Loop==10){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==42){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 5:
					switch (Auto_Step){
					case 1:
						if (Auto_Loop==62){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(0,255);
						}
						break;
					case 3: 
						if (Auto_Loop==10){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==32){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 6:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==81){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(255,0);
						}
						break;
					case 3: 
						if (Auto_Loop==15){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==42){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 7:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==62){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(255,0);
						}
						break;
					case 3: 
						if (Auto_Loop==10){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==32){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 8:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==81){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(0,255);
						}
						break;
					case 3: 
						if (Auto_Loop==15){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==42){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 11:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==113){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 12:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==104){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 13:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==113){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(255,0);
						}
						break;
					case 3: 
						if (Auto_Loop==15){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==68){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 14:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==113){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(0,255);
						}
						break;
					case 3: 
						if (Auto_Loop==15){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==68){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 15:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==113){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(0,255);
						}
						break;
					case 3: 
						if (Auto_Loop==15){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==46){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 16:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==157){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(255,0);
						}
						break;
					case 3: 
						if (Auto_Loop==20){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==68){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 17:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==113){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(255,0);
						}
						break;
					case 3: 
						if (Auto_Loop==15){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==46){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 18:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==157){
							Auto_Out(127,127);
						}
						break;
					case 2: 
						if (Auto_Loop==20){
							Auto_Out(0,255);
						}
						break;
					case 3: 
						if (Auto_Loop==20){
							Auto_Out(127,127);
						}
						break;
					case 4:
						if (Auto_Loop==20){
							Auto_Out(255,255);
						}
						break;
					case 5: 
						if (Auto_Loop==68){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 21:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==48){
							Auto_Out(127,100);
						}
						break;
					case 2: 
						if (Auto_Loop==18){
							Auto_Out(127,110);
						}
						break;
					case 3: 
						if (Auto_Loop==16){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 22:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==48){
							Auto_Out(100,127);
						}
						break;
					case 2: 
						if (Auto_Loop==18){
							Auto_Out(110,127);
						}
						break;
					case 3: 
						if (Auto_Loop==16){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 23:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==115){
							Auto_Out(127,100);
						}
						break;
					case 2: 
						if (Auto_Loop==38){
							Auto_Out(127,110);
						}
						break;
					case 3: 
						if (Auto_Loop==19){
							Auto_Out(127,105);
						}
						break;
					case 4: 
						if (Auto_Loop==76){
							Auto_Out(127,127);
						}
						break;
				}
				break;
			case 24:
				switch (Auto_Step){
					case 1:
						if (Auto_Loop==115){
							Auto_Out(100,127);
						}
						break;
					case 2: 
						if (Auto_Loop==38){
							Auto_Out(110,127);
						}
						break;
					case 3: 
						if (Auto_Loop==76){
							Auto_Out(127,127);
						}
						break;
				}
				break;
		}
				 
	Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
  //RC_Motor_Drive_Left=RC_Motor_Drive_Right=127;
}

void Auto_Out(unsigned char left, unsigned char right){
	//255 is full foward
	RC_Motor_Drive_Left=left;					
	RC_Motor_Drive_Right=255-right;
	Auto_Step++; 
	Auto_Loop=0;
}	
		

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/


void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */

 int tmp;

relay1_fwd=1; //Go go cooling faaans.


Get_Analog_Inputs();
/* leftcounter.zero=512;
 tmp=leftcounter.val;
 leftcounter.val=Get_Analog_Value(rc_ana_in01);
 if (leftcounter.dir==1 && leftcounter.val>leftcounter.zero && tmp<leftcounter.zero) {
	leftcounter.count++;
 }else  if (leftcounter.dir==-1 && leftcounter.val<leftcounter.zero && tmp>leftcounter.zero) {
	leftcounter.count--;
 }
 if (tmp<leftcounter.val) {
	leftcounter.dir=1;
 }else if (tmp>leftcounter.val) {
	leftcounter.dir=-1;
 }
*/
}

void Get_Analog_Inputs(void){
rc_ana_in01_value=Get_Analog_Value(rc_ana_in01);
rc_ana_in02_value=Get_Analog_Value(rc_ana_in02);
rc_ana_in03_value=Get_Analog_Value(rc_ana_in03);
rc_ana_in04_value=Get_Analog_Value(rc_ana_in04);
rc_ana_in05_value=1023-Get_Analog_Value(rc_ana_in05);
rc_ana_in06_value=Get_Analog_Value(rc_ana_in06);
rc_ana_in07_value=Get_Analog_Value(rc_ana_in07);
rc_ana_in08_value=Get_Analog_Value(rc_ana_in08);
}



/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
