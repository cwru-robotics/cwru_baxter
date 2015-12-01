//##########################################################
//##                      R O B O T I S                   ##
//##          ReadWrite Example code for Dynamixel.       ##
//##                                           2009.11.10 ##
//##########################################################

//modified by wsn, 11/15 for use w/ ROS node
// modified by mak237, 11/23/15 to add further access to eeprom values (position min/max, torque max, etc.)
#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include "dynamixel.h"


// Control table address

// mak237, added addresses for cw and ccw limits
#define P_CW_LIMIT_L		6
#define P_CW_LIMIT_H		7 //not needed based on how dxl_write_word handles low/high bytes
#define P_CCW_LIMIT_L		8
#define P_CCW_LIMIT_H		9 //not needed based on how dxl_write_word handles low/high bytes

// mak237, added addresses for torque enable, setting, and limits
#define P_TORQUE_MAX_L		14
#define P_TORQUE_MAX_H		15
#define P_TORQUE_ENABLE		70
#define P_GOAL_TORQUE_L		71
#define P_GOAL_TORQUE_H		72

#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46

// Defualt setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define DEFAULT_ID		2



extern unsigned char gbStatusPacket[];

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

int open_dxl(int deviceIndex, int baudnum) {
	//int baudnum = DEFAULT_BAUDNUM;
	//printf( "\n\nRead/Write example for Linux\n\n" );
	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		//printf( "Failed to open USB2Dynamixel via /dev/ttyUSB%d!\n",deviceIndex );
		return 0;
	}
	else
		//printf( "Succeeded in opening /dev/ttyUSB2%d\n", deviceIndex);
	return 1;
}

int send_dynamixel_goal(short int motor_id, int goalval) 
{
		dxl_write_word( motor_id, P_GOAL_POSITION_L, goalval );
	return 0;
}


//mak237 added cw limit function
int set_dynamixel_CW_limit(short int motor_id, int CW_limit) 
{
		
		dxl_write_word( motor_id, P_CW_LIMIT_L, CW_limit );
 // mak237, not sure if this needs to be done as two seperate commands or just write the low with the total limit
		/*
		int CW_high_limit = CW_limit >> 8;
		int CW_low_limit = CW_limit & 0xFF;

		dxl_write_word( motor_id, P_CW_LIMIT_L, CW_high_limit );
		dxl_write_word( motor_id, P_CW_LIMIT_H, CW_low_limit );
*/
	return 0;
}

//mak237 added ccw limit function
int set_dynamixel_CCW_limit(short int motor_id, int CCW_limit) 
{
		
		dxl_write_word( motor_id, P_CCW_LIMIT_L, CCW_limit );
 // mak237, not sure if this needs to be done as two seperate commands or just write the low with the total limit
		/*
		int CCW_high_limit = CCW_limit >> 8;
		int CW_low_limit = CCW_limit & 0xFF;

		dxl_write_word( motor_id, P_CCW_LIMIT_L, CCW_high_limit );
		dxl_write_word( motor_id, P_CCW_LIMIT_H, CCW_low_limit );
*/
	return 0;
}

// mak237, added torque mode switch command
// torque_mode = 0 for joint mode, 1 for torque mode
int torque_control_toggle(short int motor_id, int torque_mode) 
{
		dxl_write_byte( motor_id, P_TORQUE_ENABLE, torque_mode );
	return 0;
}

// mak237, added set torque max
int set_torque_max(short int motor_id, int torque_max) 
{
		dxl_write_word( motor_id, P_TORQUE_MAX_L, torque_max );
	return 0;
}

// mak237, added set torque goal

int send_dynamixel_torque_goal(short int motor_id, int torquegoalval) 
{
	if (torquegoalval <= 1023)
	{ 
	  dxl_write_word( motor_id, P_GOAL_TORQUE_L, torquegoalval );
	}
	else 
	{
	  dxl_write_word( motor_id, P_GOAL_TORQUE_L, 0 );
	} 
	return 0;
}


// Read present position
short int read_position(short int motor_id) {
	//short int read_position_code = P_PRESENT_POSITION_L;
	short int PresentPos;
	short int CommStatus;
        PresentPos   = dxl_read_word( motor_id, P_PRESENT_POSITION_L );
			CommStatus = dxl_get_result();
		        if( CommStatus != COMM_RXSUCCESS )  {
		              PresentPos+=4096; // set bit to indicate fault
                              //printf("comm err\n");
			}
        return PresentPos;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}
