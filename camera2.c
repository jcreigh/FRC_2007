/*******************************************************************************
*
*	TITLE:		camera.c
*
*	VERSION:	0.2 (Beta)                           
*
*	DATE:		16-Jan-2007
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This is the "streamlined" version of camera.c
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2005-2007 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	01-Jan-2006  0.1  RKW - Original code.
*	16-Jan-2007  0.2  RKW - Added Virtual2_Window() function.
*
*******************************************************************************/
#include <stdio.h>
#include "serial_ports.h"
#include "camera2.h"

// This variable, when equal to one, indicates that the
// camera has successfully initialized and should be
// sending data. You can also force a re-initialization
// by setting this variable to zero by calling the
// function Reinitialize_Camera().
unsigned char camera2_initialized = 0;

unsigned int camera2_t_packets = 0;
unsigned int camera2_acks = 0;
unsigned int camera2_ncks = 0;

// camera T packet structure
T_Packet_Data_Type T2_Packet_Data;

/*******************************************************************************
*
*	FUNCTION:		Camera2_Handler()
*
*	PURPOSE:		This function is responsable for camera initialization 
*					and camera serial data interpretation. Once the camera
*					is initialized and starts sending tracking data, this 
*					function will continuously update the global T_Packet_Data 
*					structure with the received tracking information.					
*
*	CALLED FROM:	user_routines.c/Process_Data_From_Master_uP()
*
*	PARAMETERS:		none
*
*	RETURNS:		nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Camera2_Handler(void)
{
	unsigned char return_value2;
	unsigned char byte2_count;
	unsigned char byte2;
	unsigned char i;

	// if needed, (re)initialize the camera and if the 
	// initialization process throws an error, retry 
	// until it's successfully initializes
	if(camera2_initialized == 0)
	{
		return_value2 = Initialize_Camera2();

		// is the camera done initializing and if so,
		// did it initialize without an error?
		if(return_value2 == 1)
		{
			camera2_initialized = 1;
			DEBUG(("\r\nCamera: Initialized normally\r\n"));
		}
		// is the camera done initializing and if so,
		// did it return an error?
		else if(return_value2 > 1)
		{
			DEBUG(("\r\nCamera: Initialized abnormally with code %u\r\n", (unsigned int)return_value2));
		}
	}

	// find out how much data, if any, is present in 
	// the camera serial port's received data queue?
	byte2_count = Camera2_Serial_Port_Byte_Count();

	// have we received any data?
	if(byte2_count > 0)
	{
		// we have fresh data, so read each received byte one
		// at a time and immediatly send it to the camera state
		// machine, which is responsable for parsing the camera
		// data packets
		for(i=0; i<byte2_count; i++)
		{
			// get the next data byte
			byte2 = Read_Camera2_Serial_Port();

			// send the byte to the camera state machine
			Camera2_State_Machine(byte2);
		}
	}
}

/*******************************************************************************
*
*	FUNCTION:		Camera2_State_Machine()
*
*	PURPOSE:		Parses the camera serial data stream looking for data
*					packets, ACKs and NCKS. When packets are complete the
*					individual packet counter variable is incremented, and 
*					in the case of packets, the global data structure is
*					updated with the new data.					
*
*	CALLED FROM:	Camera2_Handler(), above
*
*	PARAMETERS:		unsigned char of camera serial data
*
*	RETURNS:		nothing
*
*	COMMENTS:		Camera must be configured to output binary data, 
*					not ASCII. See Raw2_Mode() function.
*
*******************************************************************************/
void Camera2_State_Machine(unsigned char byte2)
{
	static unsigned char s2tate = UNSYNCHRONIZED;
	static unsigned char packet2_buffer[34];
	static unsigned char packet2_buffer_index;
	static unsigned char packet2_char_count; 

	switch(s2tate)
	{
		case UNSYNCHRONIZED:

			if(byte2 == 255) // start of a new data packet?
			{
				s2tate = DETERMINING_PACKET_TYPE;
			}
			else if(byte2 == 'A') // start of an ACK?
			{
				packet2_char_count = 2;
				s2tate = RECEIVING_ACK;
			}
			else if(byte2 == 'N') // start of a NCK?
			{
				packet2_char_count = 2;
				s2tate = RECEIVING_NCK;
			}
			break;

		case DETERMINING_PACKET_TYPE:

			if(byte2 == 'T') // are we receiving a "t packet"?
			{
				packet2_buffer_index = 0;
				s2tate = RECEIVING_T_PACKET;
			}
			else // unknown packet type; go back to the unsynchronized s2tate
			{
				s2tate = UNSYNCHRONIZED;
			}
			break;

		case RECEIVING_T_PACKET:

			if(packet2_buffer_index < sizeof(T_Packet_Data_Type)) // still building the packet?
			{
				// move packet character to our buffer
				packet2_buffer[packet2_buffer_index] = byte2;
				packet2_buffer_index++;
			}
			
			if(packet2_buffer_index == sizeof(T_Packet_Data_Type)) // complete packet?
			{
				T2_Packet_Data.mx = packet2_buffer[0];
				T2_Packet_Data.my = packet2_buffer[1];
				T2_Packet_Data.x1 = packet2_buffer[2];
				T2_Packet_Data.y1 = packet2_buffer[3];
				T2_Packet_Data.x2 = packet2_buffer[4];
				T2_Packet_Data.y2 = packet2_buffer[5];
				T2_Packet_Data.pixels = packet2_buffer[6];
				T2_Packet_Data.confidence = packet2_buffer[7];

				camera2_t_packets++;

				s2tate = UNSYNCHRONIZED; // we're done; go back to the unsynchronized s2tate
			}
			break;

		case RECEIVING_ACK:

			if(packet2_char_count == 2 && byte2 == 'C') // second character a C?
			{
				packet2_char_count++;
			}
			else if(packet2_char_count == 3 && byte2 == 'K') // third character a K?
			{
				packet2_char_count++;
			}
			else if(packet2_char_count == 4 && byte2 == '\r') // fourth character a return?
			{
				camera2_acks++;
				s2tate = UNSYNCHRONIZED;
			}
			else
			{
				s2tate = UNSYNCHRONIZED;
			}
			break;

		case RECEIVING_NCK:

			if(packet2_char_count == 2 && byte2 == 'C') // second character a C?
			{
				packet2_char_count++;
			}
			else if(packet2_char_count == 3 && byte2 == 'K') // third character a K?
			{
				packet2_char_count++;
			}
			else if(packet2_char_count == 4 && byte2 == '\r') // fourth character a return?
			{
				camera2_ncks++;
				s2tate = UNSYNCHRONIZED;
			}
			else
			{
				s2tate = UNSYNCHRONIZED;
			}
			break;	
	}
}

/*******************************************************************************
*
*	FUNCTION:		Initialize_Camera2()
*
*	PURPOSE:		This function is responsable for initializing the
*					camera.
*
*	CALLED FROM:	Camera2_Handler(), below.
*
*	PARAMETERS:		None.
*
*	RETURNS:		0: Initialization in progress.
*
*					1: Initialization has completed.
*
*					2-127: Camera returned a NCK and the returned value
*					indicates the s2tate that caused the NCK.
*
*					128-255: Camera didn't return a ACK or NCK within
*					the time allowed. The returned value is the value
*					128 added to the s2tate that failed. The amount of 
*					time allowed is set by the MAX_ACK_LOOP_COUNT 
*					parameter found in camera2.h.					
*
*	COMMENTS:		camera2_acks and camera2_ncks are incremented by the
*					function Camera2_State_Machine() which is called by
*					Process_Camera_Data() to process data sent by the
*					camera.
*					
*******************************************************************************/
unsigned char Initialize_Camera2(void)
{
	static unsigned char boot2_initialization_flag = 1;
	static unsigned char initialize2_flag = 1;
	static unsigned char s2tate;
	static unsigned char wait2_for_ack;
	static unsigned int loop2_count;
	static unsigned char return_value2;
	unsigned char returned_value;


	// stuff to do after the camera goes through a power-on reset
	if(boot2_initialization_flag == 1)
	{
		// get the camera's attention
		Camera2_Idle();
		// set the command & packet transfer mode
		Raw2_Mode(5);
		// don't execute this code again until the next power on reset
		boot2_initialization_flag = 0;
	}

	// do we need to (re)initialize the state machine?
	if(initialize2_flag == 1)
	{
		initialize2_flag = 0;
		wait2_for_ack = 0;
		s2tate = STATE_ONE;
		loop2_count = 0;
		return_value2 = 0;
		camera2_acks = 0;
		camera2_ncks = 0;
	}

	// do we need to wait for an ACK from the camera?
	if(wait2_for_ack == 1)
	{
		if(camera2_acks >= 1) // got ACK?
		{
			// we're no longer waiting for an ACK
			wait2_for_ack = 0;

			// reset the loop counter
			loop2_count = 0;
		}
		else if(camera2_ncks >= 1) // got NCK?
		{
			// return with a value that signals an error happened
			return_value2 = s2tate - 1;
		}
		else if(loop2_count >= MAX_ACK_LOOP_COUNT) // have we waited too long?
		{
			// return with a value that signals an error happened
			return_value2 = s2tate + 128 - 1;
		}
		else
		{
			// if we fall through to here it means that that we've sent a
			// command to the camera, and we're still waiting to receive
			// an ACK/NCK or time-out, so other than incrementing the loop
			// counter, we don't do anything and continue to wait...
			loop2_count++;
		}
	}
	else
	{

		// if debugging mode is on, send camera initialization information 
		// to the terminal (the DEBUG() macro is defined in camera2.h
		DEBUG(("Camera: Initialization state = %u\r\n", (unsigned int)s2tate));

		// reset the ACK/NCK counters
		camera2_acks = 0;
		camera2_ncks = 0;

		switch(s2tate)
		{
			case STATE_ONE:
				// get the camera's attention
				Camera2_Idle();
				// next state
				s2tate = STATE_TWO;
				// don't wait for an ACK before transitioning to the next state
				wait2_for_ack = 0;
				break;

			case STATE_TWO:
				// next state
				s2tate = STATE_THREE;
				// don't wait for an ACK before transitioning to the next state
				wait2_for_ack = 0;
				break;
	
			case STATE_THREE:
				// initialize the Common Control I register
				Write_Camera2_Module_Register(COMI_ADDRESS, COMI_DEFAULT);
				// next state
				s2tate = STATE_FOUR;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
	
			case STATE_FOUR:
				// initialize the Common Control B register
				Write_Camera2_Module_Register(COMB_ADDRESS, COMB_DEFAULT);
				// next state
				s2tate = STATE_FIVE;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;

			case STATE_FIVE:
				// initialize the Common Control J register to power-on state
				// to disable the banding filter, which must be done before
				// setting the Frame Rate Adjust register 2 (EHSL)
				Write_Camera2_Module_Register(COMJ_ADDRESS, COMJ_DEFAULT);
				// next state
				s2tate = STATE_SIX;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_SIX:
				// initialize the Frame Rate Adjust register 1
				Write_Camera2_Module_Register(EHSH_ADDRESS, EHSH_DEFAULT);
				// next state
				s2tate = STATE_SEVEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_SEVEN:
				// initialize the Frame Rate Adjust register 2
				Write_Camera2_Module_Register(EHSL_ADDRESS, EHSL_DEFAULT);
				// next state
				s2tate = STATE_EIGHT;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_EIGHT:
				// initialize the Common Control J register
				Write_Camera2_Module_Register(COMJ_ADDRESS, COMJ_DEFAULT);
				// next state
				s2tate = STATE_NINE;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_NINE:
				// initialize the Common Control A register
				Write_Camera2_Module_Register(COMA_ADDRESS, COMA_DEFAULT);
				// next state
				s2tate = STATE_TEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_TEN:
				// initialize the Automatic Gain Control register
				Write_Camera2_Module_Register(AGC_ADDRESS, AGC_DEFAULT);
				// next state
				s2tate = STATE_ELEVEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_ELEVEN:
				// initialize the Blue Gain Control register
				Write_Camera2_Module_Register(BLU_ADDRESS, BLU_DEFAULT);
				// next state
				s2tate = STATE_TWELVE;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_TWELVE:
				// initialize the Red Gain Control register
				Write_Camera2_Module_Register(RED_ADDRESS, RED_DEFAULT);
				// next state
				s2tate = STATE_THIRTEEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_THIRTEEN:
				// initialize the Saturation Control register
				Write_Camera2_Module_Register(SAT_ADDRESS, SAT_DEFAULT);
				// next state
				s2tate = STATE_FOURTEEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_FOURTEEN:
				// initialize the Brightness Control register
				Write_Camera2_Module_Register(BRT_ADDRESS, BRT_DEFAULT);
				// next state
				s2tate = STATE_FIFTEEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_FIFTEEN:
				// initialize the Automatic Exposure Control register
				Write_Camera2_Module_Register(AEC_ADDRESS, AEC_DEFAULT);
				// next state
				s2tate = STATE_SIXTEEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
		
			case STATE_SIXTEEN:
				// initialize the Noise Filter value
				Noise2_Filter(NF_DEFAULT);
				// next state
				s2tate = STATE_SEVENTEEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;

			case STATE_SEVENTEEN:
				// send the TC or Track Color command
				Track2_Color(R_MIN_DEFAULT, R_MAX_DEFAULT,
							G_MIN_DEFAULT, G_MAX_DEFAULT,
							B_MIN_DEFAULT, B_MAX_DEFAULT);
				// next state
				s2tate = STATE_EIGHTEEN;
				// wait for an ACK before transitioning to the next state
				wait2_for_ack = 1;
				break;
	
			case STATE_EIGHTEEN:
				// signal that we're done
				return_value2 = 1;
				break;
		}
	}

	// If we're returning from this function with a value greater
	// than zero, it means that configuration is complete or we've
	// generated an error. In either case we'll want to re-initialize
	// the state machine if this function is called again.
	if(return_value2 > 0)
	{
		initialize2_flag = 1;
	}

	return(return_value2);	
}

/*******************************************************************************
*
*	FUNCTION:		Track2_Color()
*
*	PURPOSE:		Properly formats and sends a TC (Track Color) command
*					to the camera.		
*
*	CALLED FROM:	Initialize_Camera2(), above.
*
*	PARAMETERS:		Minimum and maximum intensity values for the red, green 
*					and blue (or YCrCb) channels.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:		Camera must be configured to accept binary commands,
*					not ASCII. See Raw2_Mode() function.
*
*					See CMUCam2_commands.pdf for details.
*
*******************************************************************************/
void Track2_Color(unsigned char Rmin, unsigned char Rmax,
				 unsigned char Gmin, unsigned char Gmax,
				 unsigned char Bmin, unsigned char Bmax)
{
	Write_Camera2_Serial_Port('T');
	Write_Camera2_Serial_Port('C');
	Write_Camera2_Serial_Port(6);
	Write_Camera2_Serial_Port(Rmin);
	Write_Camera2_Serial_Port(Rmax);
	Write_Camera2_Serial_Port(Gmin);
	Write_Camera2_Serial_Port(Gmax);
	Write_Camera2_Serial_Port(Bmin);
	Write_Camera2_Serial_Port(Bmax);
}

/*******************************************************************************
*
*	FUNCTION:		Camera2_Idle()
*
*	PURPOSE:		If the camera is currently streaming data, this command
*					will stop the streaming and prepare it to receive commands.
*
*	CALLED FROM:	Initialize_Camera2(), above.
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Camera2_Idle(void)
{
	Write_Camera2_Serial_Port('\r');
}

/*******************************************************************************
*
*	FUNCTION:		Restart_Camera2()
*
*	PURPOSE:		This command will force a camera reinitialization		
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Restart_Camera2(void)
{
	camera2_initialized = 0;
}

/*******************************************************************************
*
*	FUNCTION:		Get_Camera2_State()
*
*	PURPOSE:		
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		0: camera is initializing.
*
*					1: camera is initialized and sending T (Tracking) packets.
*
*	COMMENTS:
*
*******************************************************************************/
unsigned char Get_Camera2_State(void)
{
	return(camera2_initialized);
}

/*******************************************************************************
*
*	FUNCTION:		Raw2_Mode()
*
*	PURPOSE:		Properly formats and sends a camera RM (Raw Mode) command
*					to the camera. 		
*
*	CALLED FROM:	Initialize_Camera2(), above.
*
*	PARAMETERS:		unsigned char
*
*	RETURNS:		nothing
*
*	COMMENTS:		This body of software assumes that raw serial communication 
*					mode five is invoked, meaning that camera input and output
*					is done in the raw binary format and that ACKs and NCKs
*					are not suppressed.
*
*					See CMUCam2_commands.pdf for details.
*
*******************************************************************************/
void Raw2_Mode(unsigned char mode)
{
	Write_Camera2_Serial_Port('R');
	Write_Camera2_Serial_Port('M');
	Write_Camera2_Serial_Port(' ');
	Write_Camera2_Serial_Port(48 + mode);
	Write_Camera2_Serial_Port('\r');
}

/*******************************************************************************
*
*	FUNCTION:		Noise2_Filter()
*
*	PURPOSE:		Properly formats and sends a camera NF (Noise Filter) 
*					command to the camera.
*
*	CALLED FROM:	Initialize_Camera2(), above.
*
*	PARAMETERS:		Noise filter threshold value.
*
*	RETURNS:		Nothing
*
*	COMMENTS:		Camera must be configured to accept binary commands,
*					not ASCII. See Raw2_Mode() function.
*
*					See CMUCam2_commands.pdf for details.
*
*******************************************************************************/
void Noise2_Filter(unsigned char threshold)
{
	Write_Camera2_Serial_Port('N');
	Write_Camera2_Serial_Port('F');
	Write_Camera2_Serial_Port(1);
	Write_Camera2_Serial_Port(threshold);
}

/*******************************************************************************
*
*	FUNCTION:		Virtual2_Window()
*
*	PURPOSE:		Properly formats and sends a VW (Virtual Window) command
*					to the camera.
*
*	CALLED FROM:
*
*	PARAMETERS:		Four unsigned chars specifying two corners of the 
*					virtual window.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:		Camera must be configured to accept binary commands,
*					not ASCII. See Raw2_Mode() function.
*
*					See CMUCam2_commands.pdf for details.
*
*******************************************************************************/
void Virtual2_Window(unsigned char x, unsigned char y, unsigned char x2, unsigned char y2)
{
	Write_Camera2_Serial_Port('V');
	Write_Camera2_Serial_Port('W');
	Write_Camera2_Serial_Port(4);
	Write_Camera2_Serial_Port(x);
	Write_Camera2_Serial_Port(y);
	Write_Camera2_Serial_Port(x2);
	Write_Camera2_Serial_Port(y2);
}

/*******************************************************************************
*
*	FUNCTION:		Write_Camera2_Module_Register()
*
*	PURPOSE:		Properly formats and sends a CR (Camera Register) command
*					to the camera.
*
*	CALLED FROM:	Initialize_Camera2(), above.
*
*	PARAMETERS:		Camera register number and new register value.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:		Camera must be configured to accept binary commands,
*					not ASCII. See Raw2_Mode() function.
*
*					See CMUCam2_commands.pdf for details.
*
*******************************************************************************/
void Write_Camera2_Module_Register(unsigned char reg, unsigned char value)
{
	Write_Camera2_Serial_Port('C');
	Write_Camera2_Serial_Port('R');
	Write_Camera2_Serial_Port(2);
	Write_Camera2_Serial_Port(reg);
	Write_Camera2_Serial_Port(value);
}

/*******************************************************************************
*
*	FUNCTION:		Camera2_Serial_Port_Byte_Count()
*
*	PURPOSE:		Returns the number of bytes in the camera serial port's
*					received data queue.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of bytes in the queue.
*
*	COMMENTS:		This code assumes that the camera serial port has been
*					properly set in camera2.h.		
*
*******************************************************************************/
unsigned char Camera2_Serial_Port_Byte_Count(void)
{
#ifdef CAMERA_SERIAL_PORT_1
	return(Serial_Port_One_Byte_Count());
#else
	return(Serial_Port_Two_Byte_Count());
#endif
}

/*******************************************************************************
*
*	FUNCTION:		Read_Camera2_Serial_Port()
*
*	PURPOSE:		Reads a byte of data from the camera serial port.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		If data is present in the camera serial port's received
*					data queue, the next byte in the queue is returned. If
*					data is not present in the camera serial port's received
*					data queue, zero is returned.
*
*	COMMENTS:		Camera2_Serial_Port_Byte_Count() should be called before
*					calling this function to make sure data is present.
*
*					This code assumes that the camera serial port has been
*					properly set in camera2.h.
*
*******************************************************************************/
unsigned char Read_Camera2_Serial_Port(void)
{
#ifdef CAMERA_SERIAL_PORT_1
	return(Read_Serial_Port_One());
#else
	return(Read_Serial_Port_Two());
#endif
}

/*******************************************************************************
*
*	FUNCTION:		Write_Camera2_Serial_Port()
*
*	PURPOSE:		Sends a byte of data to the camera serial port.
*
*	CALLED FROM:
*
*	PARAMETERS:		Byte of data to send to the camera serial port.
*
*	RETURNS:		nothing
*
*	COMMENTS:		This code assumes that the camera serial port has been
*					properly set in camera2.h.
*
*******************************************************************************/
void Write_Camera2_Serial_Port(unsigned char value)
{
#ifdef CAMERA_SERIAL_PORT_1
	Write_Serial_Port_One(value);
#else
	Write_Serial_Port_Two(value);
#endif
}

/*******************************************************************************
*
*	FUNCTION:		Terminal_Serial2_Port_Byte_Count()
*
*	PURPOSE:		Returns the number of bytes in the terminal serial port's
*					received data queue.
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of bytes in the queue.
*
*	COMMENTS:		This code assumes that the camera serial port has been
*					properly set in camera2.h.
*
*******************************************************************************/
unsigned char Terminal_Serial2_Port_Byte_Count(void)
{
#ifdef TERMINAL_SERIAL_PORT_1
	return(Serial_Port_One_Byte_Count());
#else
	return(Serial_Port_Two_Byte_Count());
#endif
}

/*******************************************************************************
*
*	FUNCTION:		Read_Terminal_Serial2_Port()
*
*	PURPOSE:		Reads a byte of data from the terminal serial port.
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		If data is present in the terminal serial port's received
*					data queue, the next byte in the queue is returned. If
*					data is not present in the terminal serial port's received
*					data queue, zero is returned.
*
*	COMMENTS:		Terminal_Serial2_Port_Byte_Count() should be called before
*					calling this function to make sure data is present.
*
*					This code assumes that the camera serial port has been
*					properly set in camera2.h.
*
*******************************************************************************/
unsigned char Read_Terminal_Serial2_Port(void)
{
#ifdef TERMINAL_SERIAL_PORT_1
	return(Read_Serial_Port_One());
#else
	return(Read_Serial_Port_Two());
#endif
}

/*******************************************************************************
*
*	FUNCTION:		Write_Terminal_Serial2_Port()
*
*	PURPOSE:		Sends a byte of data to the terminal serial port.		
*
*	CALLED FROM:
*
*	PARAMETERS:		Byte of data to send to the terminal serial port.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:		This code assumes that the camera serial port has been
*					properly set in camera2.h.
*
*******************************************************************************/
void Write_Terminal_Serial2_Port(unsigned char value)
{
#ifdef TERMINAL_SERIAL_PORT_1
	Write_Serial_Port_One(value);
#else
	Write_Serial_Port_Two(value);
#endif
}
