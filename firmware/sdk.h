#ifndef SDK_
#define SDK_

extern void SDK_mainloop(void);

struct WO_SDK_STRUCT {

	unsigned char ctrl_mode;
								//0x00: "standard scientific interface" => send R/C stick commands to LL
								//0x01:	direct motor control
								//0x02: waypoint control (not yet implemented)

	unsigned char ctrl_enabled; //0x00: Control commands are ignored by LL processor
								//0x01: Control commands are accepted by LL processor

};
extern struct WO_SDK_STRUCT WO_SDK;

struct RO_RC_DATA {

	unsigned short channel[8];
	/*
	 * channel[0]: Pitch
	 * channel[1]: Roll
	 * channel[2]: Thrust
	 * channel[3]: Yaw
	 * channel[4]: Serial interface enable/disable
	 * channel[5]: manual / height control / GPS + height control
	 *
	 * range of each channel: 0..4095
	 */
};
extern struct RO_RC_DATA RO_RC_Data;

struct WO_DIRECT_MOTOR_CONTROL
{
	unsigned char pitch;
	unsigned char roll;
	unsigned char yaw;
	unsigned char thrust;

	/*
	 * commands will be directly interpreted by the mixer
	 * running on each of the motor controllers
	 *
	 * range (pitch, roll, yaw commands): 0..200 = - 100..+100 %
	 * range of thrust command: 0..200 = 0..100 %
	 */

};
extern struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;

struct WO_CTRL_INPUT {	//serial commands (= Scientific Interface)
	short pitch;	//Pitch input: -2047..+2047 (0=neutral)
	short roll;		//Roll input: -2047..+2047	(0=neutral)
	short yaw;		//(=R/C Stick input) -2047..+2047 (0=neutral)
	short thrust;	//Collective: 0..4095 = 0..100%
	short ctrl;				/*control byte:
							bit 0: pitch control enabled
							bit 1: roll control enabled
							bit 2: yaw control enabled
							bit 3: thrust control enabled
							bit 4: Height control enabled
							bit 5: GPS position control enabled
							*/
};
extern struct WO_CTRL_INPUT WO_CTRL_Input;

#endif /*SDK_*/
