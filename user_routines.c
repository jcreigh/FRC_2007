/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

//#include "math.h"
#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "pwm.h"
//#include "camera.h"
//#include "tracking.h"
//#include "terminal.h"
#include "user_routines.h"

extern unsigned char aBreakerWasTripped;
unsigned char State_Arm_Position=Unknown_State;
unsigned char State_Mode=Unknown_State;
unsigned char State_Hand=Unknown_State;
unsigned char State_Light=Unknown_State;
unsigned char State_Ramp=Unknown_State;
unsigned char State_Trans=Unknown_State;
unsigned char State_Driver_Override=Unknown_State;
unsigned char State_Arm_Override=Unknown_State;
unsigned char State_Error=Unknown_State;
unsigned char State_Ring=Unknown_State;
unsigned char State_Target=Unknown_State;
unsigned char State_Camera_Lock=Unknown_State;
unsigned char State_Within_Range=Unknown_State;
unsigned char State_Bot_Circle=Unknown_State;
unsigned char State_Bot_Auto=Unknown_State;
unsigned char State_Arm_Deployed=Unknown_State;
unsigned char State_Move=Unknown_State;
unsigned char State_Turret_Position=Unknown_State;

unsigned char State_Arm_Activate=Off;
unsigned char State_Arm_Activate_Released=True;

unsigned char State_Turret_Activate=Off;
unsigned char State_Turret_Activate_Released=True;


unsigned char State_Hand_Activate=Off;
unsigned int RC_Turret_Last_Value;
unsigned char State_Turret_Release=True;


unsigned int rc_ana_in01_value, rc_ana_in02_value, rc_ana_in03_value, rc_ana_in04_value;
unsigned int rc_ana_in05_value, rc_ana_in06_value, rc_ana_in07_value, rc_ana_in08_value;

motor RC_Arm_Joint_1;
motor RC_Arm_Joint_2;
motor RC_Arm_Joint_3;
//motor RC_Drive;
motor RC_Drive_Left;
motor RC_Drive_Right;
motor RC_Turret;
motor RC_Wrist;

unsigned int Bot_Distance_Spider;

rotcounter leftcounter;

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/

/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */

  //For programming remotely
  digital_io_18 = OUTPUT;   
  rc_dig_out18=1;


/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
//  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  // changed so PWM() can control PWM outputs 13 through 16
  Setup_PWM_Output_Type(USER_CCP,USER_CCP,USER_CCP,USER_CCP);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  // initialize the CCP PWM hardware
  Initialize_PWM();

  // initialize the serial ports
  Init_Serial_Port_One();
  Init_Serial_Port_Two();

#ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
#endif

#ifdef TERMINAL_SERIAL_PORT_2    
 stdout_serial_port = SERIAL_PORT_TWO;
#endif

stdout_serial_port = SERIAL_PORT_ONE;

//Initialize Motors

RC_Arm_Joint_1.motor=&RC_Motor_Arm_Joint_1;
RC_Arm_Joint_1.value=&RC_Pot_Arm_Joint_1;
RC_Arm_Joint_1.motor_sign=-1;
RC_Arm_Joint_1.KP=616.0/1000.0;
RC_Arm_Joint_1.KI=1770.0/1000.0;
RC_Arm_Joint_1.KD=744.0/1000.0;
//RC_Arm_Joint_1.max_value=1000;//920;
//RC_Arm_Joint_1.min_value=0;//539;

RC_Arm_Joint_2.motor=&RC_Motor_Arm_Joint_2;
RC_Arm_Joint_2.value=&RC_Pot_Arm_Joint_2;
RC_Arm_Joint_2.motor_sign=1;
RC_Arm_Joint_2.KP=231.0/250.0;
RC_Arm_Joint_2.KI=114.0/125.0;
RC_Arm_Joint_2.KD=539.0/500.0;
//RC_Arm_Joint_2.max_value=1000;//;431;
//RC_Arm_Joint_2.min_value=0;//30;

RC_Arm_Joint_3.motor=&RC_Motor_Wrist_Joint_1;
RC_Arm_Joint_3.motor2=&RC_Motor_Wrist_Joint_2;
RC_Arm_Joint_3.value=&RC_Pot_Arm_Joint_3;
RC_Arm_Joint_3.motor_sign=-1;
RC_Arm_Joint_3.motor2_sign=1;
RC_Arm_Joint_3.KP=498.0/1000.0;
RC_Arm_Joint_3.KI=394.0/1000.0;
RC_Arm_Joint_3.KD=324.0/1000.0;

RC_Wrist.motor=&RC_Motor_Wrist_Joint_1;
RC_Wrist.motor2=&RC_Motor_Wrist_Joint_2;
RC_Wrist.motor_sign=1;
RC_Wrist.motor2_sign=1;

RC_Turret.motor=&RC_Motor_Turret;
RC_Turret.value=&RC_Pot_Turret;
RC_Turret.motor_sign=1;
RC_Turret.KP=616.0/1000.0;
RC_Turret.KI=1770.0/1000.0;
RC_Turret.KD=744.0/1000.0;
//RC_Turret.max_value=1000;//920;
//RC_Turret.min_value=0;//539;

RC_Drive_Left.motor=&RC_Motor_Drive_Left;
RC_Drive_Left.motor_sign=1;
RC_Drive_Left.value=&RC_Pot_Drive_Left;

RC_Drive_Right.motor=&RC_Motor_Drive_Right;
RC_Drive_Right.motor_sign=1;
RC_Drive_Right.value=&RC_Pot_Drive_Right;


//  stdout_serial_port = NUL;



  Putdata(&txdata);            /* DO NOT CHANGE! */

//  ***  IFI Code Starts Here***
//
//  Serial_Driver_Initialize();
//
//  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
 static unsigned int Auto_Mode=0;
	Getdata(&rxdata);
#ifdef DEBUG
printf("\033[2J");
#endif
/*	Camera_Handler();
	Camera2_Handler();
	Servo_Track();
	Servo2_Track();*/
	LabView();
	Handle_OI();
	LabView_Out();


RC_Turret.KP=Get_Analog_Value(rc_ana_in14)*.001*2;
RC_Turret.KI=Get_Analog_Value(rc_ana_in15)*.001*2;
RC_Turret.KD=Get_Analog_Value(rc_ana_in16)*.001*2;


//#Ring Awareness
	if (RC_lim_manipulator) {
		State_Ring=True;
	}else{
		State_Ring=False;
	}
//	Which_Camera_Has_Closest_Target() //Bot_Distance_Spider, State_Camera_Lock=Camera_Left,Camera_Right
	State_Target=False;
	if (Bot_Distance_Spider<Circle_1_Distance){
		State_Target=True;
	}

	Move_Bot();
	
	Move_Turret();

	Move_Arm();

	Move_Hand();

	if (State_Ramp==Mode_Ramp_Deployed_State){
		//Deploy_Ramp();
	}

//	PWM(pwm13,pwm14,pwm15,pwm16);


RC_Arm_Joint_1.hold_value=((unsigned int)OI_seg_1)*4;
RC_Arm_Joint_2.hold_value=((unsigned int)OI_seg_2)*4;
RC_Arm_Joint_3.hold_value=((unsigned int)OI_seg_3)*4;
/*if (RC_Arm_Joint_1.hold_value>OI_seg_1_max) {RC_Arm_Joint_1.hold_value=(unsigned int)OI_seg_1_max;}
if (RC_Arm_Joint_1.hold_value<OI_seg_1_min) {RC_Arm_Joint_1.hold_value=(unsigned int)OI_seg_1_min;}
if (RC_Arm_Joint_2.hold_value>OI_seg_2_max) {RC_Arm_Joint_2.hold_value=(unsigned int)OI_seg_2_max;}
if (RC_Arm_Joint_2.hold_value<OI_seg_2_min) {RC_Arm_Joint_2.hold_value=(unsigned int)OI_seg_2_min;}
if (RC_Arm_Joint_3.hold_value>OI_seg_3_max) {RC_Arm_Joint_3.hold_value=(unsigned int)OI_seg_3_max;}
if (RC_Arm_Joint_3.hold_value<OI_seg_3_min) {RC_Arm_Joint_3.hold_value=(unsigned int)OI_seg_3_min;}
*/
/*
  Auto_Mode=RC_dip_8;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_7;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_6;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_5;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_4;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_3;
  Auto_Mode=(Auto_Mode << 1) | RC_dip_2; 
  Auto_Mode=(Auto_Mode << 1) | RC_dip_1;

printf("%d, %d %d %d %d %d %d %d %d blah\r\n", Auto_Mode, RC_dip_1, RC_dip_2, RC_dip_3, RC_dip_4, RC_dip_5, RC_dip_6, RC_dip_7, RC_dip_8);
*/

#ifdef DEBUG_values
printf("\r\n\r\nStates- Override: %d\t\t\r\n", State_Arm_Override);
printf("Left Drive: %d\t\t Right Drive: %d\t\t Turret    : %d\r\n", *RC_Drive_Left.value, *RC_Drive_Right.value, *RC_Turret.value);
printf("Left Drive: %d\t\t Right Drive: %d\t\t Turret    : %d\r\n", *RC_Drive_Left.motor, *RC_Drive_Right.motor, *RC_Turret.motor);
printf("Arm J 1   : %d\t\t Arm J 2    : %d\t\t Arm J 3   : %d\r\n", *RC_Arm_Joint_1.value, *RC_Arm_Joint_2.value, *RC_Arm_Joint_3.value);
printf("Arm J 1   : %d\t\t Arm J 2    : %d\r\n", RC_Motor_Arm_Joint_1, RC_Motor_Arm_Joint_2);
printf("Arm Hold 1: %d\t\t Arm Hold 2 : %d\t\t Arm Hold 3: %d\r\n", RC_Arm_Joint_1.hold_value, RC_Arm_Joint_2.hold_value, RC_Arm_Joint_3.hold_value);
printf("Seg 1 Max : %d\t\t Seg 2 Max  : %d\t\t Seg 3 Max : %d\r\n",OI_seg_1_max,OI_seg_2_max,OI_seg_3_max);
printf("Seg 1 Min : %d\t\t Seg 2 Min  : %d\t\t Seg 3 Min : %d\r\n",OI_seg_1_min,OI_seg_2_min,OI_seg_3_min);
printf("Wrist J 2 : %d\r\n", RC_Pot_Wrist_Joint_2);
printf("Wrist J 1 : %d\t\t Wrist J 2  : %d\r\n", RC_Motor_Wrist_Joint_1, RC_Motor_Wrist_Joint_2);
printf("Manip     : %d\r\n", RC_Pot_Manipulator);
printf("OI 1      : %d\t\t OI 2       : %d\t\t OI 3  : %d\r\n",OI_seg_1,OI_seg_2,OI_seg_3);
printf("Turret    : %d\r\n", OI_Turret);
//printf("Wrist L   : %d\t\t Wrist R    : %d\r\n Turret L     : %d\t\t Turret R    : %d\r\n",OI_Button_Wrist_Left,OI_Button_Wrist_Right,OI_Button_Turret_Left,OI_Button_Turret_Right);
#endif

//printf("Arm Joint 1: %d\t Arm Joint 2: %d\t Arm Joint 3: %d\r\n", RC_Pot_Arm_Joint_1, RC_Pot_Arm_Joint_2, RC_Pot_Wrist_Joint_1);

#ifdef DEBUG_PID
printf("P: %d/1000 \t\tI: %d/1000 \t\tD: %d/1000\r\n", 2*Get_Analog_Value(rc_ana_in14),2*Get_Analog_Value(rc_ana_in15),2*Get_Analog_Value(rc_ana_in16));
#endif
	Putdata(&txdata);
}

void Handle_OI(void){
#ifdef DEBUG_trace
printf("Entering Handle_OI -");
#endif
	if (OI_Button_Arm_Bottom && !OI_Button_Arm_Shift){
		State_Arm_Position=Arm_Bottom_State;
	}else if (OI_Button_Arm_Middle && !OI_Button_Arm_Shift){
		State_Arm_Position=Arm_Middle_State;
	}else if (OI_Button_Arm_Top && !OI_Button_Arm_Shift){
		State_Arm_Position=Arm_Top_State;
	}else if (OI_Button_Arm_Floor && !OI_Button_Arm_Shift){
		State_Arm_Position=Arm_Floor_State;
	}else if (OI_Button_Arm_Home && !OI_Button_Arm_Shift){
		State_Arm_Position=Arm_Home_State;
	}

	if (OI_Button_Arm_Bottom && OI_Button_Arm_Shift){
		State_Turret_Position=Turret_Left_State;
	}else if (OI_Button_Arm_Middle && OI_Button_Arm_Shift){
		State_Turret_Position=Turret_Center_State;
	}else if (OI_Button_Arm_Top && OI_Button_Arm_Shift){
		State_Turret_Position=Turret_Right_State;
	}
	
	if (OI_Button_Place_Remove && State_Mode==Mode_Remove_State){
		State_Mode=Mode_Place_State;
	}else if(OI_Button_Place_Remove && State_Mode==Mode_Place_State && !State_Ring){
		State_Mode=Mode_Remove_State;
	}

	if (OI_Button_Wrist_Left || OI_Button_Wrist_Right) {
		State_Hand_Activate=Off;
	}else{
		State_Hand_Activate=On;
	}

	if (OI_Button_Trans) {
		State_Trans=Mode_Trans_High_State;
	}else{
		State_Trans=Mode_Trans_Low_State;
	}

	if (OI_Button_Arm_Activate && State_Arm_Activate == Off && State_Arm_Activate_Released == True) {
		State_Arm_Activate=On;
		State_Arm_Activate_Released=False;
	}else if (OI_Button_Arm_Activate && State_Arm_Activate == On && State_Arm_Activate_Released == True) {
		State_Arm_Activate=Off;
		State_Arm_Activate_Released=False;
	}else if(!OI_Button_Arm_Activate){
		State_Arm_Activate_Released=True;
	}

	if ((OI_Button_Turret_Activate && State_Turret_Activate == Off && State_Turret_Activate_Released == True) && State_Arm_Activate==On) {
		State_Turret_Activate=On;
		State_Turret_Activate_Released=False;	
	}else if ((OI_Button_Turret_Activate && State_Turret_Activate == On && State_Turret_Activate_Released == True) || State_Arm_Activate==Off) {
		State_Turret_Activate=Off;
		State_Turret_Activate_Released=False;
	}else if(!OI_Button_Turret_Activate){
		State_Turret_Activate_Released=True;
	}
//	printf("Arm Activated: %d, Released: %d\r\n", State_Arm_Activate==On, State_Arm_Activate_Released==On);
//	printf("Turret Activated: %d, Released: %d\r\n", State_Turret_Activate==On, State_Turret_Activate_Released==On);

if (State_Arm_Activate==On){
		OI_Arm_Green=On;
		if (State_Turret_Activate==On){
			OI_Arm_Red=On;
		}else{
			OI_Arm_Red=Off;
		}
}else{
		OI_Arm_Green=Off;
		OI_Arm_Red=On;
}

//OI_Button_Light is a toggle. 
	if (OI_Button_Light) {
		State_Light=Mode_Light_State;
	}else{
		State_Light=Mode_No_Light_State;
	}

	if (OI_Button_Ramp_Deploy && State_Ramp==Mode_Ramp_Deploy_Okay_State){
		State_Ramp=Mode_Ramp_Deployed_State;
	}

	State_Driver_Override=OI_Button_Driver_Override;
	State_Arm_Override=OI_Button_Arm_Override;
	if (State_Arm_Override){
		State_Arm_Position=Unknown_State;	
		State_Turret_Position=Unknown_State;
	}
#ifdef DEBUG_trace
printf("Leaving Handle_OI -");
#endif
}

void Move_Bot(void){
	unsigned double outputX;
	unsigned double outputY;
	static int loopcount;
#ifdef DEBUG_trace
printf("Entering Move_Bot -");
#endif
	State_Bot_Circle=2;
	State_Bot_Auto=False;
	if (!State_Driver_Override && State_Ring && State_Arm_Position!=Arm_Home_State && State_Arm_Position!=Arm_Floor_State && State_Target){
		if (State_Arm_Deployed){
			State_Bot_Circle=0;
			State_Bot_Auto=True;
		}else{
			State_Bot_Circle=1;
			State_Bot_Auto=True;
		}
	}
    if (RC_Trans!=(State_Trans*255)) {RC_Trans=(State_Trans*255),loopcount=0;} 

	if (State_Bot_Auto && State_Ramp!=Mode_Ramp_Deployed_State){
		State_Move=Mode_Move_Auto_State;
	}else if(State_Ramp==Mode_Ramp_Deployed_State){
		State_Move=Mode_Move_Ramp_State;
	}else{
		State_Move=Mode_Move_OI_State;
		OI_Drive_x = 255 - OI_Drive_x; //Reverse Inputs
		OI_Drive_y = 255 - OI_Drive_y;

		OI_Drive_x=joy_condition(OI_Drive_x);
		OI_Drive_y=joy_condition(OI_Drive_y);
		outputX=OI_Drive_x;
		outputY=OI_Drive_y;
		//outputX = (((((double)OI_p4_x-127)*((double)OI_p4_x - 127)*((double)OI_p4_x - 127)) / 16129) + 127);
		//outputY = (((((double)OI_p4_y-127)*((double)OI_p4_y - 127)*((double)OI_p4_y - 127)) / 16129) + 127);	

		out(RC_Drive_Right, Limit_Mix(2000 + (int)outputY + (int)outputX - 127),0);
    	out(RC_Drive_Left,Limit_Mix(2000 - (int)outputY + (int)outputX + 127),0); 
	}
    if (loopcount<5) {RC_Motor_Drive_Right = RC_Motor_Drive_Left = 127;loopcount++;}
#ifdef DEBUG_trace
printf("Leaving Move_Bot -");
#endif
}

void out(motor mot, unsigned char val1, unsigned char val2){
#ifdef DEBUG_trace
printf("Entering Out -");
#endif
	if (mot.motor_sign==-1){
		*mot.motor=255-val1;	
	}else{
		*mot.motor=val1;		
	}
	if (mot.motor_sign==-1){
		*mot.motor2=255-val2;	
	}else{
		*mot.motor2=val2;		
	}
#ifdef DEBUG_trace
printf("Leaving Out -");
#endif
}

void Move_Turret(void){
//const unsigned char turretPower[] = {127, 127, 140, 145, 150, 157, 165, 172, 180, 187, 195, 205, 215, 215, 235, 255};
//const unsigned char turretPower[] = {127, 127, 140, 142, 145, 147, 155, 162, 174, 183, 195, 205, 215, 215, 235, 255};
const unsigned char turretPower[] = {127, 127, 140, 142, 144, 146, 149, 152, 155, 160, 165, 170, 170, 170, 180, 180};
signed char a = 1;
signed char in = (signed char)((signed int)(255-OI_Turret)-127);

#ifdef DEBUG_trace
printf("Entering Move_Turret -");
#endif
//THIS WILL CHANGE. 
	RC_Motor_Turret=127;
	if (State_Arm_Activate && State_Turret_Activate) {
		if (in<-15){
			in=-15;
		}
		if (in>15){
			in=15;
		}
		if (in < 0){a=-1;}else{a=1;}
		RC_Motor_Turret=a*turretPower[a*in];




/*
		if (OI_Button_Hat_Up){
			RC_Turret.hold_value+=5;
		}else if (OI_Button_Hat_Down){
			RC_Turret.hold_value-=5;
		}	
		if (OI_Button_Turret){
			RC_Turret.hold_value=RC_Turret_Last_Value+((signed int)OI_Turret-127)*4;
		}else{
			RC_Turret_Last_Value=RC_Turret.hold_value;
		}
#ifdef DEBUG_PID
	printf("Turret:\r\n");
#endif
		Move_Motor(RC_Turret);


		if (OI_Button_Turret){
			RC_Motor_Turret=255-OI_Turret;
			if (RC_Motor_Turret>142) {RC_Motor_Turret=162;}
			if (RC_Motor_Turret<112) {RC_Motor_Turret=92;}
		}
*/

	}

//		RC_Motor_Turret=255-joy_condition(OI_p2_x)+127;

#ifdef DEBUG_trace
printf("Leaving Move_Turret -");
#endif
}

void Move_Arm(void){
//THIS WILL CHANGE. 
	static signed int loopcount_j1;
	static signed int loopcount_j2;
	RC_Motor_Arm_Joint_1=127;
	RC_Motor_Arm_Joint_2=127;
#ifdef DEBUG_trace
printf("Entering Move_Arm -");
#endif
	if (State_Arm_Override){
//		RC_Arm_Joint_1.hold_value=RC_Pot_Arm_Joint_1;
//		RC_Arm_Joint_2.hold_value=RC_Pot_Arm_Joint_2;
		
	}
//	if (State_Arm_Override && !OI_Button_Arm_Shift){
	if (OI_Button_Arm_Override && !OI_Button_Arm_Shift){
		*RC_Arm_Joint_1.motor=OI_p2_y;
		*RC_Arm_Joint_2.motor=255-OI_p2_x;
		*RC_Turret.motor=(unsigned char)((((unsigned int)OI_p2_wheel)-127)/2+127);
	}
	
	if (State_Arm_Override && OI_Button_Arm_Shift){
//		*RC_Arm_Joint_2.motor=255-OI_p2_y;
	}

//if (!State_Arm_Override && State_Arm_Position!=Unknown_State){
if (State_Arm_Activate){
#ifdef DEBUG_PID
	printf("Joint 1:\r\n");
#endif
	Move_Motor(RC_Arm_Joint_1);
#ifdef DEBUG_PID
	printf("Joint 2:\r\n");
#endif
	Move_Motor(RC_Arm_Joint_2);
}

if (RC_Motor_Arm_Joint_1 >= 155 || RC_Motor_Arm_Joint_1 < 125) { loopcount_j1 = 0; }
if (RC_Motor_Arm_Joint_2 <= 120 || RC_Motor_Arm_Joint_2 < 125) { loopcount_j2 = 0; }
if (loopcount_j1 < 39 && RC_Motor_Arm_Joint_1 > 125 && RC_Motor_Arm_Joint_1 < 155) {RC_Motor_Arm_Joint_1=155; loopcount_j1++;}
if (loopcount_j2 < 39 && RC_Motor_Arm_Joint_2 < 129 && RC_Motor_Arm_Joint_2 > 120) {RC_Motor_Arm_Joint_2=120; loopcount_j2++;}
#ifdef DEBUG_trace
printf("Leaving Move_Arm -");
#endif
}

void Move_Hand(void){
#ifdef DEBUG_trace
printf("Entering Move_Hand -");
#endif
//THIS WILL CHANGE 
	RC_Motor_Wrist_Joint_1=RC_Motor_Wrist_Joint_2=127;

	if (State_Arm_Override && OI_Button_Arm_Shift){
		RC_Motor_Wrist_Joint_1=joy_condition(OI_p2_y);
		RC_Motor_Wrist_Joint_2=joy_condition(OI_p2_x);
	}
	if (State_Arm_Activate && State_Hand_Activate) {
#ifdef DEBUG_PID
	printf("Wrist:\r\n");
#endif
     Move_Motor(RC_Arm_Joint_3);
    }else if(State_Arm_Activate){
		if (OI_Button_Wrist_Left) {
			out(RC_Wrist,RC_Wrist_Rot_Speed,RC_Wrist_Rot_Speed);
		}else if (OI_Button_Wrist_Right) {
			out(RC_Wrist,255-RC_Wrist_Rot_Speed,255-RC_Wrist_Rot_Speed);
		}
	}

#ifdef DEBUG_trace
printf("Leaving Move_Hand -");
#endif
}

void Move_Motor(motor mot){
	signed int error;
	signed int delta_err;
	signed int integral_err;
	signed int p_out;
	//static signed int prev_err;
	signed int i_out;
	signed int d_out;
	signed int output;
#ifdef DEBUG_trace
printf("Entering Move_Motor -");
#endif
	error=*mot.value-mot.hold_value;
	delta_err=mot.prev_err-error;
	mot.integral_err+=error;

	mot.prev_err = error;

	if (mot.integral_err>MAX_INTEGRAL_ERROR){
		mot.integral_err = MAX_INTEGRAL_ERROR;
	}else if (mot.integral_err<-MAX_INTEGRAL_ERROR){
		mot.integral_err = -MAX_INTEGRAL_ERROR;
	}
    if (error<5 && error>-5) {mot.integral_err=0;}
	p_out = (int)(error * mot.KP);
	i_out = (int)(mot.integral_err * mot.KI);
	d_out = (int)(delta_err * mot.KD);

	output = p_out + i_out + d_out;

	if(output > 127){
		output = 127;
	}else if(output < -127){
		output = -127;
	}
#ifdef DEBUG_PID
	printf(": %d, Hold: %d, Error: %d, DeltaErr, %d, IntegErr: %d\r\n PrevErr: %d, Out: %d\r\n", *mot.value, mot.hold_value, error, delta_err, mot.integral_err,mot.prev_err,output+127);
	printf("KP: %d/1000, KI: %d/1000, KD: %d/1000\r\n", (int)(mot.KP*1000), (int)(mot.KI*1000), (int)(mot.KD*1000));
#endif

	mot.prev_err = error;

/*if (*arm.value>arm.max_value){
	output=0;
}else if (*arm.value<arm.min_value){
	output=0;
}*/
	*mot.motor=(mot.motor_sign*(unsigned char)output + 127);
	*mot.motor2=(mot.motor2_sign*(unsigned char)output + 127);
	
	mot.prev_err = error;
#ifdef DEBUG_trace
  printf("Leaving Move_Motor -");
#endif
}

unsigned char joy_condition(unsigned char in) {
	signed int out = ((signed int)in)-127;			
	if (out > joy_deadzone)	{
		if (out > joy_max){
		}else{
 			return (unsigned char)(out+127);		
 		}
	}else if (out < -joy_deadzone){					// joystick is negative
		if (out < -joy_max){
			return (unsigned char)(-joy_max+127);	
		}else{
			return (unsigned char)(out+127);	
		}
	}
	return 127;
}

void debug(unsigned int v){
	unsigned char r = v&255;
	unsigned char l = (v>>8)&255;
	//stdout_serial_port = SERIAL_PORT_TWO;
	printf("%c%c",l,r);
	//stdout_serial_port = SERIAL_PORT_ONE;

}

void debug_long (unsigned long v){
	unsigned char lr = v&255;
	unsigned char ll = (v>>8)&255;
	unsigned char ur = (v>>16)&255;
	unsigned char ul = (v>>24)&255;
	//stdout_serial_port = SERIAL_PORT_TWO;
	printf("%c%c%c%c",ul,ur,ll,lr);
	//stdout_serial_port = SERIAL_PORT_ONE;
}

void LabView_Out(){

	debug(32896);	
	debug(32896);	
	debug(18);
	
	debug( OI_Drive_x);			//0
	debug( OI_Drive_y);
	debug(*RC_Drive_Left.motor);	
	debug(*RC_Drive_Right.motor);
	debug(State_Arm_Override);		
	debug(disabled_mode);			//5
	debug(autonomous_mode);
	debug(OI_seg_1);			
	debug(OI_seg_2);
	debug(RC_Motor_Arm_Joint_1);
	debug(RC_Motor_Arm_Joint_2);		//10
	debug(RC_Motor_Wrist_Joint_1);		
	debug(*RC_Arm_Joint_1.value);		
	debug(*RC_Arm_Joint_2.value);
	debug(*RC_Arm_Joint_3.value);		
	debug(RC_Arm_Joint_1.hold_value);	//15
	debug(RC_Arm_Joint_2.hold_value);	
	debug(RC_Arm_Joint_3.hold_value);		

}
	


void LabView(void)
{
// LabView Input
// *** Put EEPROM writes here ***
//
// Need Left Motor values
//
	unsigned char bytecount=0;
	unsigned char data=0;
	unsigned char data2=0;
	unsigned int variablecount=0;
	unsigned char currentvar=0;
	unsigned char inputpktstate=0;
	static int inputmagics=0;
	static unsigned int i = 0;
	static unsigned int j = 0;

	/* LabView Interface - Get variables */
	bytecount=Serial_Port_One_Byte_Count();
	if(bytecount>0){
		for (j=0;j<bytecount;j++){
			data=Read_Serial_Port_One();
			if (data!=128 && inputpktstate<4){
					inputpktstate=0;
			}else if (data==128 && inputpktstate<4){	
				inputpktstate++;
			}else if(inputpktstate==4 && variablecount==0){
				data2=Read_Serial_Port_One();
				variablecount=data2;
				j++;
				currentvar=0;
			}else if (variablecount>0){
				data2=Read_Serial_Port_One();
				j++;
				switch (currentvar){
					case 0:				// Program Mode
						if (data2==1){
							rc_dig_out18=0;
						}
						break;
					case 1:				// P Value - Joint 1
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_1.KP=((float)inputmagics)/10000;
						break;	
					case 2:				// I Value - Joint 1
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_1.KI=((float)inputmagics)/10000;
						break;	
					case 3:				// D Value - Joint 1
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_1.KD=((float)inputmagics)/10000;
						break;	
					case 4:				// P Value - Left Motor 
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_2.KP=((float)inputmagics)/10000;
						break;	
					case 5:				// I Value - Left Motor
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_2.KI=((float)inputmagics)/10000;
						break;	
					case 6:				// D Value - Left Motor
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_2.KD=((float)inputmagics)/10000;
						break;	
					case 7:				// P Value - Arm Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_3.KP=((float)inputmagics)/10000;
						break;	
					case 8:				// I Value - Arm Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_3.KI=((float)inputmagics)/10000;
						break;	
					case 9:				// D Value - Arm Motor Up
						inputmagics=data;inputmagics=inputmagics<<8;inputmagics+=data2;
						RC_Arm_Joint_3.KD=((float)inputmagics)/10000;
						break;	

				}
				currentvar++;
			}
			if (currentvar>variablecount){
				variablecount=0; inputpktstate=0;
			}
		}
	}
}

