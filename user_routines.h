/*******************************************************************************
* FILE NAME: user_routines.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to user_routines.c and
*  user_routines_fast.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_


/*******************************************************************************
                            MACRO DECLARATIONS
*******************************************************************************/
/* Add your macros (aliases and constants) here.                              */
/* Do not edit the ones in ifi_aliases.h                                      */
/* Macros are substituted in at compile time and make your code more readable */
/* as well as making it easy to change a constant value in one place, rather  */
/* than at every place it is used in your code.                               */
/*
 EXAMPLE CONSTANTS:
#define MAXIMUM_LOOPS   5
#define THE_ANSWER      42
#define TRUE            1
#define FALSE           0
#define PI_VAL          3.1415

 EXAMPLE ALIASES:
#define LIMIT_SWITCH_1  rc_dig_int1  (Points to another macro in ifi_aliases.h)
#define MAIN_SOLENOID   solenoid1    (Points to another macro in ifi_aliases.h)
*/

//DEBUGS
//#define DEBUG	
//#define DEBUG_values	
//#define DEBUG_PID
//#define DEBUG_trace		


#define MAX_INTEGRAL_ERROR 	200

#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */
#define PI 			3.14

#define On 			1
#define Off			0

#define joy_max						127
#define joy_deadzone				7

#define True 		1
#define False		0

#define ARM_LENGTH_1 91/2
#define ARM_LENGTH_2 28
#define ARM_LENGTH_3 42/2

#define ARM_JOINT_1_CENTER 	265
#define ARM_JOINT_2_CENTER 	483

#define WRIST_JOINT_1_CENTER 	502
#define WRIST_JOINT_2_CENTER 	502

#define JOINT_1_TOP 	-5
#define JOINT_2_TOP 	167

#define JOINT_1_MIDDLE 	-149
#define JOINT_2_MIDDLE 	273

#define JOINT_1_BOTTOM 	-167
#define JOINT_2_BOTTOM 	212

#define JOINT_1_FLOOR 	-167
#define JOINT_2_FLOOR 	51

#define JOINT_1_HOME 	-167
#define JOINT_2_HOME 	406

#define TURRET_CENTER	500
#define TURRET_LEFT		-250
#define TURRET_RIGHT	250

#define OI_Arm_Green		Relay2_red
#define OI_Arm_Red			Relay1_red

//Port 1. Driver Button Box  ###### v v v v v v ### will probably have to be changed, don't remember wiring. =]
#define OI_Button_Place_Remove 		p1_sw_top
#define OI_Button_Light				p1_sw_trig
#define OI_Button_Ramp_Deploy		p1_sw_aux1
#define OI_Button_Driver_Override	p3_sw_aux1

//Port 2. Arm. This is the chicklet.
#define OI_Button_Arm_Top			p2_sw_aux1
#define OI_Button_Arm_Middle		((255-p2_aux)&128)
#define OI_Button_Arm_Bottom		((255-p2_aux)&32)
#define OI_Button_Arm_Floor			((255-p2_aux)&64)
#define OI_Button_Arm_Home			p2_sw_aux2
#define OI_Button_Arm_Shift			p2_sw_trig
#define OI_Button_Wrist_Shift		p3_sw_trig
#define OI_Button_Arm_Override		((255-p2_aux)&16)
//p2_sw_top

//#define OI_Button_Arm_Override		p1_sw_aux2

#define OI_Button_Hat_Up			(((signed int)p4_x+25)/50)==5
#define OI_Button_Hat_Right			(((signed int)p4_x+25)/50)==4
#define OI_Button_Hat_Down			(((signed int)p4_x+25)/50)==3
#define OI_Button_Hat_Left			(((signed int)p4_x+25)/50)==2

#define OI_p2_x						p2_x
#define OI_p2_y						p2_y
#define OI_p2_wheel					p2_wheel
#define OI_p2_aux					p2_aux

//Port 3. Arm (literally)
#define OI_Button_Arm_Activate		p3_sw_aux2
#define OI_Button_Wrist_Left		p3_sw_top
#define OI_Button_Wrist_Right		p3_sw_aux1
#define OI_Button_Turret_Activate	p3_sw_trig
#define OI_seg_1					255-p3_y
#define OI_seg_1_max				(4*177)
#define OI_seg_1_min				(4*85)
#define OI_seg_2					255-p3_aux
#define OI_seg_2_max				(4*231)
#define OI_seg_2_min				400
#define OI_seg_3					p3_wheel
#define OI_seg_3_max				951
#define OI_seg_3_min				72
#define OI_Turret					p3_x

#define RC_Wrist_Rot_Speed			255

//Port 4. Driver
#define OI_Button_Trans 			p1_sw_trig
#define OI_Drive_x						p1_x
#define OI_Drive_y						p1_y

//RC
#define RC_Camera1_Tilt				pwm01
#define RC_Camera1_Pan				pwm02
#define RC_Camera2_Tilt				pwm03
#define RC_Camera2_Pan				pwm04
#define RC_Trans					pwm05
#define RC_Motor_Drive_Left			pwm06
#define RC_Motor_Drive_Right		pwm07
#define RC_Motor_Turret				pwm08
#define RC_Motor_Arm_Joint_1		pwm09
#define RC_Motor_Arm_Joint_2		pwm10
#define RC_Motor_Wrist_Joint_1		pwm11
#define RC_Motor_Wrist_Joint_2		pwm12
#define RC_Motor_Manipulator		pwm13

#define RC_Pot_Drive_Left			rc_ana_in01_value
#define RC_Pot_Drive_Right			rc_ana_in02_value
#define RC_Pot_Turret				rc_ana_in03_value
#define RC_Pot_Arm_Joint_1			rc_ana_in04_value
#define RC_Pot_Arm_Joint_2			rc_ana_in05_value
#define RC_Pot_Arm_Joint_3			rc_ana_in06_value
#define RC_Pot_Wrist_Joint_2		rc_ana_in07_value
#define RC_Pot_Manipulator			rc_ana_in08_value
/*
#define RC_Pot_Drive_Left			Get_Analog_Value(rc_ana_in08)
#define RC_Pot_Drive_Right			Get_Analog_Value(rc_ana_in08)
#define RC_Pot_Turret				Get_Analog_Value(rc_ana_in08)
#define RC_Pot_Arm_Joint_1			Get_Analog_Value(rc_ana_in08)
#define RC_Pot_Arm_Joint_2			Get_Analog_Value(rc_ana_in08)
#define RC_Pot_Wrist_Joint_1		Get_Analog_Value(rc_ana_in08)
#define RC_Pot_Wrist_Joint_2		Get_Analog_Value(rc_ana_in08)
#define RC_Pot_Manipulator			Get_Analog_Value(rc_ana_in08)
*/
#define RC_dip_1			!rc_dig_in01
#define RC_dip_2			!rc_dig_in02
#define RC_dip_3			!rc_dig_in03
#define RC_dip_4			!rc_dig_in04
#define RC_dip_5			!rc_dig_in05
#define RC_dip_6			!rc_dig_in06
#define RC_dip_7			!rc_dig_in07
#define RC_dip_8			!rc_dig_in08
#define RC_lim_manipulator	!rc_dig_in09

#define OI_Button_Hand_Open 0

//#States
#define Unknown_State 				42

//State_Arm_Position
#define Arm_Bottom_State 			1
#define Arm_Middle_State 			2
#define Arm_Top_State 				4
#define Arm_Floor_State 			8
#define Arm_Home_State 				16

//State_Turret_Position
#define Turret_Center_State 		1
#define Turret_Left_State 			2
#define Turret_Right_State 			4

//State_Mode
#define Mode_Place_State			1
#define Mode_Remove_State			2

//State_Hand
#define Mode_Hand_Open_State		1
#define Mode_Hand_Close_State		2

//State_Light
#define Mode_Light_State			1
#define Mode_No_Light_State			2

//State_Ramp
#define Mode_Ramp_Deploy_Okay_State	1
#define Mode_Ramp_Deployed_State	2
#define Mode_Ramp_UnDeployed_State	4

//State_Trans
#define Mode_Trans_Low_State		0
#define Mode_Trans_High_State		1

//State_Error
#define No_Error					0
#define Error						1

//State_Move
#define Mode_Move_OI_State			0
#define Mode_Move_Auto_State		1
#define Mode_Move_Ramp_State		1

#define Camera_left					1
#define Camera_right				2

#define Circle_1_Distance			30

/*******************************************************************************
                            TYPEDEF DECLARATIONS
*******************************************************************************/
/* EXAMPLE DATA STRUCTURE */
/*
typedef struct
{
  unsigned int  NEW_CAPTURE_DATA:1;
  unsigned int  LAST_IN1:1;
  unsigned int  LAST_IN2:1;
  unsigned int  WHEEL_COUNTER_UP:1;
  unsigned int  :4;
  unsigned int wheel_left_counter;
  unsigned int wheel_right_counter;
} user_struct;
*/

typedef struct
{
	unsigned int count;
	unsigned int val;
	signed char dir;
	unsigned int zero;
	
}	rotcounter;

typedef struct{
	unsigned int *value;
	unsigned char *motor;
	unsigned char *motor2;
	unsigned char motor_sign;
	unsigned char motor2_sign;
	signed int prev_err;
	signed int integral_err;
	unsigned int hold_value;
//	unsigned int min_value;
//	unsigned int max_value;
//	signed int pot_correction;
	unsigned float KP;
    unsigned float KI;
    unsigned float KD;
} motor;
/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in user_routines.c */
void User_Initialization(void);
void Process_Data_From_Master_uP(void);
void Move_Bot(void);
void Handle_OI(void);
void Move_Turret(void);
void Move_Hand(void);
void Move_Arm(void);
void Move_Motor(motor mot);
void Set_Motor_Angle(motor mot, unsigned int angle);
unsigned int Angle_To_Pot(unsigned int angle);
unsigned char joy_condition(unsigned char in);
void out(motor mot, unsigned char val1, unsigned char val2);

/* These routines reside in user_routines_fast.c */
void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
void Process_Data_From_Local_IO(void);
void Get_Analog_Inputs(void);
void Auto_Out(unsigned char left, unsigned char right);
void debug_long(unsigned long v);
void debug(unsigned int v);
void LabView_Out();
void LabView();

#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
