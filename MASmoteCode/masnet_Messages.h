/*************************************************************************************
Mote Messages for Masnet Project  V 0.2.2   Aug, 23, 2004
Change log:
Aug, 23, 2004
	- Change AM_RESUME_MSG, AM_PAUSE_MSG, AM_STOP_ALLMOVEMENT_MSG to AM_ACTION_MSG
	- Add ExceptionMsg, including LowBatteryException
	- Add SensorSampleRateMsg
Aug, 22, 2004
	- Remove DiffDestinationMsg
	- Change AbsDestinationMsg to DestinationMsg
	- Change unit of angle to radian and use float to store
	- Change DebugMsg's format to 4 floats
	- Change PiControlMsg to ControlMsg, parameters: pi to wpi, pk to wpk, using float
	- Change parameters of VWControl to float 
Aug, 20, 2004
	- Remove pGPS info from ABS_DESTINATION_MSG
	- Change the name AM_MOTE_PHOTOSENSOR_MSG to AM_MOTE_SENSOR_MSG
Aug, 17, 2004
	- Add self-calculated position into "MoteSensorMsg" sent from mote to PC
Aug, 14, 2004:
	- Add Command Header to All Commands sent from PC to motes
	- Add sequence number to Command Header

Definition of MASNET Mote messages:

Position Information:
  - All position information in MASNET messages contains 
    (X, Y, ANGLE). X, Y are in millimeter and ANGLE
	is in radian (can be negative).
	When X, Y or Angle is undefined or not interested
    the value should be set as MASNET_UNSET_POSITION_X,
	MASNET_UNSET_POSITION_Y, MASNET_UNSET_ANGLE.

From Motes to PC: 
  - AM_MOTE_SENSOR_MSG (1)
    Motes periodically send sensor readings to PC
    along with it's own position.


  - AM_CMD_COMPLETE_MSG (2)
    Motes send this message to PC everytime they finish
    a single "COMMAND". This message contains a sequence
    number assigned with the command by PC.(see COMMAND HEADER)

  - AM_EXCEPTION_MSG (3)
    Motes notify PC its error including:
	  a. Low Battery: when code = EX_LOW_BATTERY, it's reporting low battery, param1 is the voltage.

From PC to Motes:
  - AM_GPS_CALIBRATION_MSG (51)
    PC periodically broadcast pGPS information to all Motes.
    In order to save bandwidth utilization, all motes' pGPS
    information will be packed together and broadcasted to
    all motes. For 10 motes, we will only need 3 packets
    to send all pGPS infomation of all motes. (4 pGPS messages
    in one packet)
    For motes, this means it will have to check the length
    of data, unpack pGPS messages from packet and
    check the "id" field of GPSCalibrationMsg to find its own
    pGPS message. Motes can choose to ignore pGPS information
    of other robots or keep track of that information, depends
    on our application.

From PC to Motes: (Movement/Control Commands)
   - COMMAND HEADER (MasnetCmdHeader)
     All of following messages are commands that will be sent by PC 
     to Motes. They all contain a  MasnetCmdHeader structure at the 
     beginning of message.
     MasnetCmdHeader has two fields:
     uint16_t seqno: 
     Sequence number for commands. PC will ensure
     that for a specific robot, there will be no two commands that 
     have the same sequence number. However, for one particular robot,
     sequence number will not necessarily increases by one and two robots
     can receive commands of the same sequnce number if that command
     is broadcasted.
	 Motes should send AM_CMD_COMPLETE_MSG with the sequence number
	 of the completed command to PC.
     
     uint8_t override: 
     If override is 0, it means this command will
     be append to current command queue which means this command
     is to be executed after all previous commands are finished.
     This is "APPENDING" behavior.
     If override is 1, it means robots should DISCARD all previous
     commands (including the one being executed) and execute this command
     immediately. This is "OVERRIDING" behavior.
     if override is 2, it means robot should execute this command 
     immediately and resume the current command queue after this command
     is finished. This is "INSERTING" behavior. This behavior is not yet
     implemented.

   - AM_DESTINATION_MSG (52)
     This commands robot to go to a specific position. It gives the
     absolute position of destination.
     
   - AM_CONTROL_MSG (53)
     Gives PI control parameters (wkP and wkI).
    
   - AM_VW_CONTROL_MSG (54)
     Give parameters for VW control.
     
From PC to Motes: (Status commands)
   The messages in this category are to change the behavior of robots.
   - AM_SENSOR_SAMPLE_RATE_MSG (101)
     Set the sampling rate of robot. When rate = 0, stop sampling.

   - AM_ACTION_MSG (102)
	 Set the execution behavior of robots.
	 "action" field indicates the behavior, including:
	 a. AC_STOP
        Immediately stop any command currently being executed and discard all 
        commands in command queue. The robot should stop any action and clear 
        all commands in command queue NOW.

	 b. AC_PAUSE
        Immediately stop any command currently being executed but DO NOT discard
        command queue. The robot should stop any action NOW but still remember
        what it was doing and keep the current command queue. Robot should 
        still be able to queue incoming commands and receive other messages
        (such as AM_GPS_CALIBRATION_MSG)
     c. AC_RESUME  
        Resume what robot was doing before receving AM_PAUSE_MSG. Robots
        should conitnue to subsequential commands in command queue after
        current command.
        
     Please note that PC may want to send an AC_PAUSE to all robots
     before any command has been sent and broadcast an AC_RESUME after sending out all 
     command sets for robots. This way all robots can start action at the
     same time. Mote should still be able to recognize commands with
	 "override" equals to 1 (OVERRIDING behavior) and clear command 
	 queue according to that.
     

*************************************************************************************/

#ifndef __MASNET_MESSAGES__
#define __MASNET_MESSAGES__

#ifndef __GNUC__
#  define  __attribute__(x)  /*NOTHING*/
typedef unsigned char uint8_t ;
typedef unsigned short uint16_t;
typedef unsigned int  uint32_t;
typedef char int8_t ;
typedef short int16_t;
typedef int  int32_t;
#endif
#ifndef __AM_H_NO_INSTANCE__
#define __AM_H_NO_INSTANCE__
#endif

/* include AM message types of tinyos */
/* If we're not using GNU C, elide __attribute__ */
#include "AM.h"

// the unset values, when these value are in command,
// it means doesn't care
#define MASNET_UNSET_POSITION_X		0
#define MASNET_UNSET_POSITION_Y		0
#define MASNET_UNSET_ANGLE			1000

/*********
 MASNET specific message IDs
 (names can be slightly different but id numbers 
  should be the same for each type)
 *********/
enum MASNET_MSG_TYPE {
	/* From mote to pc */
	AM_MOTE_SENSOR_MSG	=	1	/* sensor readings from motes*/
	,AM_CMD_COMPLETE_MSG=	2 /* sent from mote to pc once all current commands it has are completed */
	,AM_EXCEPTION_MSG	=   3 /* error alert */
	,AM_DEBUG_MSG		=	4	

	/* From pc to mote (Movement Commands) */
	,AM_GPS_CALIBRATION_MSG	=	51	/* position calibration from pGPS */
	,AM_DESTINATION_MSG		=	52	/* Movement destination command */
	,AM_CONTROL_MSG			=	53	
	,AM_VW_CONTROL_MSG		=   54 /* set the v, w value */

	/* From pc to mote (status commands) */
	,AM_SENSOR_SAMPLE_RATE_MSG	= 101
	,AM_ACTION_MSG				= 102
};

/*********************************************
	structure of messages
 ********************************************/
#pragma pack(push,old_align) //	to prevent the problem caused by memory align
#pragma pack(1)

/****************************
   Messages sent from Motes
 ****************************/

typedef struct CmdCompleteMsg_st{
	uint8_t mote;
	uint16_t seqno;/* sequence number */
} CmdCompleteMsg;

/* definitions for our application messagel */
typedef struct MoteSensorMsg_st{
	uint8_t mote;
	uint16_t	frontPhoto;	/* 16 bits */
	uint16_t	backPhoto;	/* 16 bits */
	uint16_t	leftEncode;	/* 16 bits */
	uint16_t	rightEncode;/* 16 bits */
	uint16_t	frontIR1;	/* 16 bits */
	uint16_t	frontIR2;	/* 16 bits */
	uint16_t	backIR;		/* 16 bits */
	uint16_t	x;			/* 16 bits self-calculated position X */
	uint16_t	y;			/* 16 bits self-calculated position Y */
	float		angle;		/* 16 bits signed self-calculated angle   */
} MoteSensorMsg;

/****************************
   Exception sent from Motes
 ****************************/

enum EXCEPTION_TYPE {
	EX_LOW_BATTERY = 1 /* Low battery exception, param1=voltage */
};

typedef struct ExceptionMsg_st{
	uint8_t mote;
	enum EXCEPTION_TYPE code;  /* see "enum EXCEPTION_TYPE" for definition of exception code */
	float param1;
	float param2;
	float param3;
	float param4;
} ExceptionMsg;


/* 
	To save broadbandth, calibration msg will be packed into one packet
	so mote will need to check the lenth of msg to see how many Calibration Msg 
	in a packet and check each one to see if there is a Calibratino msg for itself
*/
/****************************
   Messages sent from PC, 
 ****************************/

typedef struct GpsCalibrationMsg_st{ 
	uint8_t		id;			/*  8 bits  */
	uint16_t	x;			/* 16 bits */
	uint16_t	y;			/* 16 bits */
	float		angle;		/* 16 bits signed */
} GPSCalibrationMsg;

typedef struct DebugMsg_st{
	uint8_t mote;
	float debugv[6];
} DebugMsg;

/****************************
   Movement/Control Commands sent from PC, 
   !!! Sequence Number MUST be the first two bytes !!!
 ****************************/
typedef struct MasnetCmdHeader_st{
	uint16_t	seqno;		/* sequence number */
	uint8_t	override;		/* 0: append to the end of current commands, 
							   1: discrad all current command and execute this one 
							   2: insert, execute this command and then execute the current commands */
} MasnetCmdHeader;
typedef struct ControlMsg_st{
	MasnetCmdHeader	header;
	float	wkp;				/* 16 bits */
	float	wki;				/* 16 bits */
} ControlMsg;

typedef struct DestinationMsg_st{
	MasnetCmdHeader	header;
	uint16_t	new_x;		/* 16 bits */
	uint16_t	new_y;		/* 16 bits */
	float		new_angle;	/* 16 bits signed */
} DestinationMsg;


typedef struct VWControlMsg_st{ 
	MasnetCmdHeader	header;
	float	v;		/* 8 bits signed */
	float	w;		/* 8 bits signed */
} VWControlMsg;

/****************************
   Status Commands sent from PC, 
 ****************************/
typedef struct SensorSampleRateMsg_st{ 
	uint16_t	rate;		/* when rate = 0, stop sending sensor readings */
} SensorSampleRateMsg;

enum ACTION_TYPE{
	AC_STOP		=	1 /* emergency stop all movement */
	,AC_PAUSE	=	2 /* pause any execution */
	,AC_RESUME	=   3 /* resume command execution */
};

typedef struct ActionMsg_st{ 
	enum ACTION_TYPE	action;
	
} ActionMsg;

#pragma pack(pop,old_align)
#endif
