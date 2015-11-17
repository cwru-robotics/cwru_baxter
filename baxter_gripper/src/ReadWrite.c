//##########################################################
//##                      R O B O T I S                   ##
//##          ReadWrite Example code for Dynamixel.       ##
//##                                           2009.11.10 ##
//##########################################################

//modified by wsn, 11/15 for use w/ ROS node
#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include "dynamixel.h"


// Control table address
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
