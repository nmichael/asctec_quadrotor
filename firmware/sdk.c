#include "main.h"
#include "sdk.h"
#include "LL_HL_comm.h"

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;

/* This function is triggered @ 1kHz.
 *
 * WO_(Write Only) data is written to the LL processor after
 * execution of this function.
 *
 * RO_(Read Only) data is updated before entering this function
 * and can be read to obtain information for supervision or control
 *
 * */

void SDK_mainloop(void)
{
#if 0
	WO_SDK.ctrl_mode=0x01;	//0x00: absolute angle and throttle control
							//0x01: direct motor control: WO_Direct_Motor_Control
							//		Please note that output mapping is done
							//		directly on the motor controllers

	WO_SDK.ctrl_enabled=0;  //0: disable control by HL processor
							//1: enable control by HL processor

/*	Example for WO_SDK.ctrl_mode=0x00; */
/*
	WO_CTRL_Input.ctrl=0x08;	//0x08:enable throttle control only

	WO_CTRL_Input.thrust=(RO_RC_Data.channel[2]/2);	//use R/C throttle stick input /2 to control thrust (just for testing)
*/

/*	Example for WO_SDK.ctrl_mode=0x01;
 *
 *  Stick commands directly mapped to motors, NO attitude control!
 * */

	WO_Direct_Motor_Control.pitch=RO_RC_Data.channel[0]/21;
	WO_Direct_Motor_Control.roll=(4095-RO_RC_Data.channel[1])/21;
	WO_Direct_Motor_Control.thrust=RO_RC_Data.channel[2]/21;
	WO_Direct_Motor_Control.yaw=RO_RC_Data.channel[3]/21;
#endif
}


