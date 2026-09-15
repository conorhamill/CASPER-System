/**
*	@file		SDK_Amplifier_DS402_StateMachine.mc
*	@brief		This file provides all the function used for DS402 PPM and PVM
*	@details	A DS402 slave can be operated in the different modes PPM and PVM
*				The amplifiers can be controlled via EtherCAT or CAN bus.
*
*	@example 	SDK_Amplifier_DS402_StateMachine.mc
*	@example 	CAN_1Ax_DS402_ProfilePositionMode-ppm-Test.mc
*	@example 	CAN_1Ax_DS402_ProfileVelocityMode-pvm-Test.mc
*	@example 	CAN_MultipleAx_DS402_ProfilePositionMode-ppm-Test.mc
*	@example 	CAN_MultipleAx_DS402_ProfilePositionMode-pvm-Test.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Amplifier_DS402_StateMachine.mh"


/***************************************************************************
*DS 402 State-Machine functions
***************************************************************************/


/**
*	@brief 		Read status word
*	@details	This function reads the status word form index @ref DS402_STATUSWORD(0x6041) and subindex 0
*				The bits have different meaning for Profile Position Mode and Profile Velocity Mode
* 	@param 		busId		Bus ID of the connected slave
*	@return 	value:	status word
*
*/
long sdkDS402_ReadStatusWord(long busId)
{
	return SdoRead(busId, DS402_STATUSWORD, 0);
}

/**
*	@brief 		Read control word
*	@details	This function reads the control word form index @ref DS402_CONTROLWORD(0x6040) and subindex 0
				The bits have different meaning for Profile Position Mode and Profile Velocity Mode
* 	@param 		busId		Bus ID of the connected slave
*	@return 	value:	control word
*
*/
long sdkDS402_ReadControlWord(long busId)
{
	return SdoRead(busId, DS402_CONTROLWORD, 0);
}

/**
*	@brief 		Write control word
*	@details	This function writes the control word to index @ref DS402_CONTROLWORD(0x6040) and subindex 0
*				The bits have different meaning for Profile Position Mode and Profile Velocity Mode
* 	@param 		busId		Bus ID of the connected slave
* 	@param 		value		Value of the control word
*	@return 	value:	control word
*
*/
void sdkDS402_WriteControlWord(long busId, long value)
{
	SdoWrite(busId, DS402_CONTROLWORD, 0, value);
}


/**
*	@brief 		Get the actual drive state
*	@details	This function gets the actual state in DS402 state machine
*				The state is depending on the folloeing bits in the status word.
*
*				Bit0: Ready to switch on\n
*				Bit1: Switched on\n
*				Bit2: Operation enabled\n
*				Bit3: Fault\n
*				Bit4: *(don't care)\n
*				Bit5: Quick stop (low active)\n
*				Bit6: switch on disabled\n
*
* 	@param 		busId Bus ID of the connected slave
*	@return 	value:	drive state (value mapping: in @ref SDK_Amplifier_DS402_StateMachine.mh)
*
*/
long sdkDS402_GetActDriveState(long busId)
{
	long status = sdkDS402_ReadStatusWord(busId);

    switch(status & 0x006F) //mask bits xxxx xxxx x11x 1111
	{

		case 0x0000: 	return(DS402_DRIVE_STATE_NOT_READY_TO_SWITCH_ON);	//xxxx xxxx x00x 0000
		case 0x0040: 	return(DS402_DRIVE_STATE_SWITCH_ON_DISABLED);		//xxxx xxxx x10x 0000
		case 0x0021: 	return(DS402_DRIVE_STATE_READY_TO_SWITCH_ON);		//xxxx xxxx x01x 0001
		case 0x0023: 	return(DS402_DRIVE_STATE_SWITCHED_ON);				//xxxx xxxx x01x 0011
		case 0x0027: 	return(DS402_DRIVE_STATE_OPERATION_ENABLED);		//xxxx xxxx x01x 0111
		case 0x0007: 	return(DS402_DRIVE_STATE_QUICK_STOP_ACTIVE);		//xxxx xxxx x01x 0001
		case 0x000F: 	return(DS402_DRIVE_STATE_FAULT_REACTION_ACTIVE);	//xxxx xxxx x00x 1111
		case 0x0008: 	return(DS402_DRIVE_STATE_FAULT);					//xxxx xxxx x00x 1000
		default:		return(-1);;
	}
}


/**
*	@brief 		Print the name of certain drive state
*	@details	This function prints the state name from a state vlaue.
*				It can be used to create human readable print outputs for debugging reason.
*
* 	@param 		state: value of the state to print (value mapping: in @ref SDK_Amplifier_DS402_StateMachine.mh)
*
*/
void sdkDS402_PrintState(long state)
{
	switch(state)
	{
		case DS402_DRIVE_STATE_NOT_READY_TO_SWITCH_ON: 	printf("DRIVE_STATE_NOT_READY_TO_SWITCH_ON"); break;
		case DS402_DRIVE_STATE_SWITCH_ON_DISABLED: 		printf("DRIVE_STATE_SWITCH_ON_DISABLED"); break;
		case DS402_DRIVE_STATE_READY_TO_SWITCH_ON: 		printf("DRIVE_STATE_READY_TO_SWITCH_ON"); break;
		case DS402_DRIVE_STATE_SWITCHED_ON: 			printf("DRIVE_STATE_SWITCHED_ON"); break;
		case DS402_DRIVE_STATE_OPERATION_ENABLED: 		printf("DRIVE_STATE_OPERATION_ENABLED"); break;
		case DS402_DRIVE_STATE_QUICK_STOP_ACTIVE: 		printf("DRIVE_STATE_QUICK_STOP_ACTIVE"); break;
		case DS402_DRIVE_STATE_FAULT_REACTION_ACTIVE:	printf("DRIVE_STATE_FAULT_REACTION_ACTIVE"); break;
		case DS402_DRIVE_STATE_FAULT: 					printf("DRIVE_STATE_FAULT"); break;
		default:									printf("%d", state);
	}
}

/**
*	@brief 		Execute a state transition
*	@details	This function executes a state transition within the DS402 state machine.
*				DO NOT USE this function for state changes. It is just a helper function for the function @ref sdkDS402_TransitionToState().
*				If you want to execute a state transition, use the function @ref sdkDS402_TransitionToState().
*
*				The function writes value to the control word, to set the bits for state changes.
*
*				Bit 0: Switched on\n
*				Bit 1: Enable voltage\n
*				Bit 2: Quick stop\n
*				Bit 3: Enable operation\n
*				..\n
*				Bit 7: Fault reset\n
*
*				T
*
*	@ref 		sdkDS402_TransitionToState()
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		transition		Value of the transition to execute
*	@return 	value:	Process value \n
*				value 	> 0 Transition successful 	\n
*				value 	< 0 Invalid transition 	\n
*
*/
long helperDS402_ExecuteTransition(long busId, long transition)
{
	long controlWord = sdkDS402_ReadControlWord(busId);
	long retVal = 1;

 	switch(transition) {
		case DS402_TRANSITION_SHUTDOWN:
        	//Transition Shutdown: CW 0xxx x110
        	printf("...: execute Transition: DS402_TRANSITION_SHUTDOWN\n", busId);
        	controlWord = (controlWord & 0xFF78) | 0x06;
            sdkDS402_WriteControlWord(busId, controlWord);
            break;
		case DS402_TRANSITION_SWITCH_ON:
			printf("...: execute Transition: DS402_TRANSITION_SWITCH_ON\n", busId);
        	//Transition Switch On: CW 0xxx x111
        	controlWord = (controlWord & 0xFF78) | 0x07;
            sdkDS402_WriteControlWord(busId, controlWord);
            break;
		case DS402_TRANSITION_ENABLE_OPERATION:
			printf("...: execute Transition: DS402_TRANSITION_ENABLE_OPERATION\n", busId);
        	//Transition Enable Operation: CW 0xxx 1111
        	controlWord = (controlWord & 0xFF70) | 0x0F;
            sdkDS402_WriteControlWord(busId, controlWord);
            break;
		case DS402_TRANSITION_DISABLE_VOLTAGE:
			printf("...: execute Transition: DS402_TRANSITION_DISABLE_VOLTAGE\n", busId);
        	//Transition Disable Voltage: CW 0xxx xx0x
        	controlWord = (controlWord & 0xFF7D);
            sdkDS402_WriteControlWord(busId, controlWord);
            break;
        case DS402_TRANSITION_QUICK_STOP:
        	printf("...: execute Transition: DS402_TRANSITION_QUICK_STOP\n", busId);
        	//Transition Quick Stop: CW 0xxx x01x
        	controlWord = (controlWord & 0xFF79) | 0x02;
            sdkDS402_WriteControlWord(busId, controlWord);
            break;
		case DS402_TRANSITION_DISABLE_OPERATION:
			printf("...: execute Transition: DS402_TRANSITION_DISABLE_OPERATION\n", busId);
        	//Transition Disable Voltage: CW 0xxx 0111
        	controlWord = (controlWord & 0xFF70) | 0x07;
            sdkDS402_WriteControlWord(busId, controlWord);
            break;
		case DS402_TRANSITION_FAULT_RESET:
			printf("...: execute Transition: DS402_TRANSITION_FAULT_RESET\n", busId);
        	//Transition Fault Resest: CW 0xxx xxxx -> 1xxx xxxx
        	controlWord = (controlWord & 0xFF7F);
            sdkDS402_WriteControlWord(busId, controlWord);
            //Delay(1);
            controlWord = controlWord | 0x80;
            sdkDS402_WriteControlWord(busId, controlWord);
            //Transition switch on disable
            controlWord = controlWord & 0xFF7F;
            sdkDS402_WriteControlWord(busId, controlWord);
            break;
        default:
            printf("Invalid state transition\n");
            retVal = -1;
            break;
	}

	return(retVal);
}


/**
*	@brief 		Transition to another state
*	@details	This function executes a state transition within the DS402 state machine.
*				It gets the actual state and performs a transition to a new state, if the transition between
*				those two states is allowed.
*				It is not checked whether the target state has been achieved.
*
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		newState		target state
*	@return 	value:	Process value \n
*				value 	> 0 Transition executed 	\n
*				value 	< 0 Invalid transition 	\n
*
*/
long sdkDS402_TransitionToState(long busId, long newState) {

    long transition = -1;
    long actState = sdkDS402_GetActDriveState(busId);

    printf("Bus Id %d, Transition from ", busId);
    sdkDS402_PrintState(actState);
    printf(" to ");
    sdkDS402_PrintState(newState);
    print();

    switch(actState) {
        case DS402_DRIVE_STATE_NOT_READY_TO_SWITCH_ON:
            if (newState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED) {
               // Automatic transition
            }
            break;
        case DS402_DRIVE_STATE_SWITCH_ON_DISABLED:
            if (newState == DS402_DRIVE_STATE_READY_TO_SWITCH_ON) {
            	transition = DS402_TRANSITION_SHUTDOWN;
            }
            break;
        case DS402_DRIVE_STATE_READY_TO_SWITCH_ON:
            if (newState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED) {
                transition = DS402_TRANSITION_DISABLE_VOLTAGE;
            }
            else if (newState == DS402_DRIVE_STATE_SWITCHED_ON) {
                transition = DS402_TRANSITION_SWITCH_ON;
            }
            else if (newState == DS402_DRIVE_STATE_OPERATION_ENABLED) {
                transition = DS402_TRANSITION_ENABLE_OPERATION;
            }
            break;
        case DS402_DRIVE_STATE_SWITCHED_ON:
            if (newState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED) {
                transition = DS402_TRANSITION_DISABLE_VOLTAGE;
            }
            else if (newState == DS402_DRIVE_STATE_READY_TO_SWITCH_ON) {
                transition = DS402_TRANSITION_SHUTDOWN;
            }
            else if (newState == DS402_DRIVE_STATE_OPERATION_ENABLED) {
                transition = DS402_TRANSITION_ENABLE_OPERATION;
            }
            break;
        case DS402_DRIVE_STATE_OPERATION_ENABLED:
            if (newState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED) {
                transition = DS402_TRANSITION_DISABLE_VOLTAGE;
            }
            else if (newState == DS402_DRIVE_STATE_READY_TO_SWITCH_ON) {
                transition = DS402_TRANSITION_SHUTDOWN;
            }
            else if (newState == DS402_DRIVE_STATE_SWITCHED_ON) {
                transition = DS402_TRANSITION_DISABLE_OPERATION;
            }
            else if (newState == DS402_DRIVE_STATE_QUICK_STOP_ACTIVE) {
                transition = DS402_TRANSITION_QUICK_STOP;
            }
            break;
        case DS402_DRIVE_STATE_QUICK_STOP_ACTIVE:
            if (newState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED) {
                transition = DS402_TRANSITION_SHUTDOWN;
            }
            else if (newState == DS402_DRIVE_STATE_OPERATION_ENABLED) {
                transition = DS402_TRANSITION_ENABLE_OPERATION;
            }
            break;
        case DS402_DRIVE_STATE_FAULT_REACTION_ACTIVE:
            if (newState == DS402_DRIVE_STATE_FAULT) {
                // Automatic transition
            }
            else if (newState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED) {
                transition = DS402_TRANSITION_FAULT_RESET;
            }
            break;
        case DS402_DRIVE_STATE_FAULT:
            if (newState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED) {
                transition = DS402_TRANSITION_FAULT_RESET;
            }
            break;
        default:
            printf("...: invalid actual drive state: %d\n", busId, actState);
            break;
    }

    if(transition != -1)
    {
    	helperDS402_ExecuteTransition(busId, transition);
    }
    else
    {
    	printf("...: invalid state transition\n", busId);
    }

    return(transition);
}

/**
*	@brief 		Wait transition to another state
*	@details	This function waits (block) until a target state in DS402 state machine has been reached or the timeout expierd.
*				It gets the actual state and performs a transition to a new state, if the transition between
*				those two states is allowed.
*				Because this function is blocking, just use it for a single axis. If you would like to change the state of several axis use @ref sdkDS402_TransitionToState()
*				funciton and verify the transition in the application.
*
*	@ref		sdkDS402_TransitionToState
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		newState		target state
*	@param		Timeout until the function is aborted
*				@b >=0: timeout in [ms] \n
*				@b < 0: infinite timeout\n
*	@return 	value:	Process value \n
*				value 	> 0 Transition executed (number of transition) 	\n
*				value 	-1  Invalid transition\n
*				value 	-2  Timeout expired\n
*				value 	-3  Invalid actual state\n
*
*/
long sdkDS402_WaitTransitionToState(long busId, long targetState, long timeout)
{
	long actState, transition;
	long time = Time();

	printf("Bus Id %d, sdkDS402_WaitTransitionToState\n" ,busId);

	transition = sdkDS402_TransitionToState(busId, targetState);
	if(transition < 0)
	{
		//Invalid transition
		return(-1);
	}

	actState = sdkDS402_GetActDriveState(busId);
	while(actState != targetState)
	{
		if(timeout >= 0 &&  (Time() - time) > timeout)
		{
			//Timeout elapsed
			printf("...: target state not reached within timeout (%d ms)\n", timeout);
			return(-2);
		}
		actState = sdkDS402_GetActDriveState(busId);
	}

	return(transition);
}


/***************************************************************************
*Profile Mode settings
***************************************************************************/


/**
*	@brief 		Set the profile acceleration @ref DS402_PROFILE_ACCELERATION(0x6083)
*	@details	Set the acceleration for PPM (Profile Position Mode) and PVM (Profile Velocity Mode)
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		acc				acceleration value [acceleration unit]
*
*/
void sdkDS402_SetProfileAcceleration(long busId, long acc)
{
	SdoWrite(busId, DS402_PROFILE_ACCELERATION, 0, acc);
}

/**
*	@brief 		Set the profile deceleration @ref DS402_PROFILE_DECELERATION(0x6084)
*	@details	Set the deceleration for PPM (Profile Position Mode) and PVM (Profile Velocity Mode)
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		acc				deceleration value [acceleration unit]
*
*/
void sdkDS402_SetProfileDeceleration(long busId, long dec)
{
	SdoWrite(busId, DS402_PROFILE_DECELERATION, 0, dec);
}

/**
*	@brief 		Set the profile velocity @ref DS402_PROFILE_VELOCITY(0x6081)
*	@details	Set the deceleration for PPM (Profile Position Mode)
				The target velocity for PVM has to be set with the function @ref sdkDS402_PVM_TargetReached()
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		vel				velocity value [velocity unit]
*
*/
void sdkDS402_SetProfileVelocity(long busId, long vel)
{
	SdoWrite(busId, DS402_PROFILE_VELOCITY, 0, vel);
}

/**
*	@brief 		Set the motion profile type @ref DS402_MOTION_PROFILE_TYPE(0x6086)
*	@details	Set the type of motion profile trajectory
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		type			motion profile type value  \n
								@b 0: linear ramp (trapezoidal profile)\n
*
*/
void sdkDS402_SetMotionProfileType(long busId, long type)
{
	SdoWrite(busId, DS402_MOTION_PROFILE_TYPE, 0, type);
}

/**
*	@brief 		Set the max profile velocity @ref DS402_MAX_PROFILE_VELOCITY(0x607F)
*	@details	Set the velocity limit in a PPM or PVM move.
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		vel				velocity value [velocity unit]
*
*/
void sdkDS402_SetMaxProfileVelocity(long busId, long vel)
{
	SdoWrite(busId, DS402_MAX_PROFILE_VELOCITY, 0, vel);
}

/**
*	@brief 		Set the max profile velocity @ref DS402_MAX_MOTOR_SPEED(0x6080)
*	@details	Set the maximum allowed speed for the motor.
*				It serves as protection for the motor. The value is given in [rpm].
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		vel				speed value [velocity unit]
*
*/
void sdkDS402_SetMaxMotorSpeed(long busId, long vel)
{
	SdoWrite(busId, DS402_MAX_MOTOR_SPEED, 0, vel);
}

/**
*	@brief 		Set the max acceleration @ref DS402_MAX_ACCELERATION(0x60C5)
*	@details	Set the maximum allowed acceleration to prevent mechanical damage.
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		acc				acceleration value
*
*/
void sdkDS402_SetMaxAcceleration(long busId, long acc)
{
	SdoWrite(busId, DS402_MAX_ACCELERATION, 0, acc);
}

/**
*	@brief 		Set quick stop deceleration @ref DS402_QUICK_STOP_DECELERATION(0x6085)
*	@details	Set the deceleration of the quick stop profile.
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		dec				deceleration value
*
*/
void sdkDS402_SetQuickStopDeceleration(long busId, long dec)
{
	SdoWrite(busId, DS402_QUICK_STOP_DECELERATION, 0, dec);
}

/**
*	@brief 		Set quick stop deceleration @ref DS402_QUICK_STOP_DECELERATION(0x6085)
*	@details	Set the deceleration of the quick stop profile.
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		dec				deceleration value
*
*/
void sdkDS402_SetProfileMovementParameter(long busId, long acc, long dec, long velocity)
{
	sdkDS402_SetProfileAcceleration(busId, acc);
    sdkDS402_SetProfileDeceleration(busId, dec);
    sdkDS402_SetProfileVelocity(busId, velocity);
}

/***************************************************************************
*Profile Mode functions
***************************************************************************/

/**
*	@brief 		Set the mode of operation @ref DS402_MODES_OF_OPERATION(0x6060)
*	@details	Set the mode of operation.
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		operationMode	Definition of the operation mode \n
*								@b DS402_OP_PPM (1): Profile Position Mode (PPM) \n
*								@b DS402_OP_PVM (3): Profile Velocity Mode (PVM)
*
*/
void sdkDS402_SetOpartionMode(long busId, long operationMode)
{
	if(	operationMode == DS402_OP_PPM)
	{
		printf("Bus Id %d, setup profile position mode\n" ,busId);
		SdoWrite( busId, DS402_MODES_OF_OPERATION, 0, DS402_OP_PPM);
	}
	else if(operationMode == DS402_OP_PVM)
	{
		printf("Bus Id %d, setup profile velocity mode\n" ,busId);
		SdoWrite( busId, DS402_MODES_OF_OPERATION, 0, DS402_OP_PVM);
	}
}

/**
*	@brief 		Get the mode of operation @ref DS402_MODES_OF_OPERATION(0x6060)
*	@details	Get the mode of operation.
* 	@param 		busId 			Bus ID of the connected slave
* 	@return		operationMode	Definition of the operation mode \n
*								@b DS402_OP_PPM (1): Profile Position Mode (PPM) \n
*								@b DS402_OP_PVM (3): Profile Velocity Mode (PVM)
*
*/
long sdkDS402_GetOpartionMode(long busId)
{
	long operationMode = SdoRead( busId, DS402_MODES_OF_OPERATION, 0);
	return(operationMode);
}

/**
*	@brief 		Execute quick stop
*	@details	This function executes a quick stop with the defined deceleration.
*				The deceleration can be set with method @ref sdkDS402_SetQuickStopDeceleration()
* 	@param 		busId 			Bus ID of the connected slave
*
*/
void sdkDS402_QuickStop(long busId)
{
	printf("Bus Id %d, Quick Stop\n", busId);
	sdkDS402_TransitionToState(busId, DS402_DRIVE_STATE_QUICK_STOP_ACTIVE);
}

/**
*	@brief 		Stop the motor
*	@details	This function stops the motor with the defined profile deceleration.
*				The deceleration can be set with method sdkDS402_SetProfileDeceleration
* 	@param 		busId 			Bus ID of the connected slave
*
*/
void sdkDS402_Halt(long busId)
{
	long controlWord;
	printf("Bus Id %d, Halt\n", busId);

	controlWord = sdkDS402_ReadControlWord(busId);
	controlWord.i[DS402_CW_BIT_HALT] = 1;

    sdkDS402_WriteControlWord(busId, controlWord);
}

/**
*	@brief 		Reset fault
*	@details	This function resets the fault execute a state transition to 'Switch On disabled'
* 	@param 		busId 			Bus ID of the connected slave
*
*/
void sdkDS402_ResetFault(long busId)
{
	printf("Bus Id %d, Reset Fault\n", busId);
	sdkDS402_TransitionToState(busId, DS402_DRIVE_STATE_SWITCH_ON_DISABLED);
}

/**
*	@brief 		Print the status word
*	@details	This function print the status word split into individual bits with their description.
*				It is depending on the actual modes of operation (PPM/PVM).
*				This function can be used for debugging purposes.
* 	@param 		busId 			Bus ID of the connected slave
*
*/
void sdkDS402_Print_StatusWord(long busId)
{
	long value;
	long status;
	long operationMode;
	operationMode = SdoRead(busId, DS402_MODES_OF_OPERATION, 0);
	status = sdkDS402_ReadStatusWord(busId);


	print("STATUS WORD");

	value = status.i[DS402_SW_BIT_READY_TO_SWITCH_ON];
	printf("\tREADY_TO_SWITCH_ON:\t%d\n", value);

	value = status.i[DS402_SW_BIT_SWITCHED_ON];
	printf("\tSWITCHED_ON:\t\t%d\n", value);

	value = status.i[DS402_SW_BIT_OPERATION_ENABLED];
	printf("\tOPERATION_ENABLED:\t%d\n", value);

	value = status.i[DS402_SW_BIT_FAULT];
	printf("\tFAULT:\t\t\t%d\n", value);

	value = status.i[DS402_SW_BIT_VOLTAGE_ENABLED];
	printf("\tVOLTAGE_ENABLED:\t%d\n", value);

	value = status.i[DS402_SW_BIT_QUICK_STOP];
	printf("\tQUICK_STOP:\t\t%d\n", value);

	value = status.i[DS402_SW_BIT_SWITCH_ON_DISABLED];
	printf("\tSWITCH_ON_DISABLED:\t%d\n", value);

	value = status.i[DS402_SW_BIT_WARNING];
	printf("\tWARNING:\t\t%d\n", value);

	value = status.i[DS402_SW_BIT_REMOTE];
	printf("\tREMOTE:\t\t\t%d\n", value);

	value = status.i[DS402_SW_BIT_TARGET_REACHED];
	printf("\tTARGET_REACHED:\t\t%d\n", value);

	value = status.i[DS402_SW_BIT_INTERNAL_LIMIT];
	printf("\tINTERNAL_LIMIT:\t\t%d\n", value);

	if(operationMode == DS402_OP_PPM)
	{
		value = status.i[DS402_SW_PPM_BIT_SETPOINT_ACK];
		printf("\tSETPOINT_ACK:\t\t%d\n", value);

		value = status.i[DS402_SW_PPM_BIT_FOLLWING_ERROR];
		printf("\tFOLLWING_ERROR:\t\t%d\n", value);
	}

	if(operationMode == DS402_OP_PVM)
	{
		value = status.i[DS402_SW_PVM_BIT_SPEED];
		printf("\tSPEED:\t\t\t%d\n", value);
	}
}

/**
*	@brief 		Print the control word
*	@details	This function writes the control word split into individual bits with their description.
*				It is depending on the actual modes of operation (PPM/PVM).
*				This function can be used for debugging purposes.
* 	@param 		busId 			Bus ID of the connected slave
*
*/
void sdkDS402_Print_ControlWord(long busId)
{
	long value;
	long controlWord;
	long operationMode;

	operationMode = SdoRead(busId, DS402_MODES_OF_OPERATION, 0);
	controlWord = sdkDS402_ReadControlWord(busId);

	print("CONTROL WORD");
	value = controlWord.i[DS402_CW_BIT_SWITCH_ON];
	printf("\tSWITCH_ON:\t\t%d\n", value);

	value = controlWord.i[DS402_CW_BIT_ENABLE_VOLTAGE];
	printf("\tENABLE_VOLTAGE:\t\t%d\n", value);

	value = controlWord.i[DS402_CW_BIT_QUICK_STOP];
	printf("\tQUICK_STOP:\t\t%d\n", value);

	value = controlWord.i[DS402_CW_BIT_ENABLE_OPERATION];
	printf("\tENABLE_OPERATION:\t%d\n", value);

	value = controlWord.i[DS402_CW_BIT_HALT];
	printf("\tHALT:\t\t\t%d\n", value);

	if(operationMode == DS402_OP_PPM)
	{
		value = controlWord.i[DS402_CW_PPM_BIT_NEW_SETPOINT];
		printf("\tNEW_SETPOINT:\t%d\n", value);

		value = controlWord.i[DS402_CW_PPM_BIT_CHANGE_IMMEDIATELY];
		printf("\tCHANGE_IMMEDIATELY:\t%d\n", value);

		value = controlWord.i[DS402_CW_PPM_BIT_RELATIVE_MOVEMENT];
		printf("\tRELATIVE_MOVEMENT:\t%d\n", value);

		value = controlWord.i[DS402_CW_PPM_BIT_ENDLESS_MOVEMENT];
		printf("\tENDLESS_MOVEMENT:\t%d\n", value);
	}
}

/***************************************************************************
*Profile Position Mode functions
***************************************************************************/

/**
*	@brief 		Start positioning relative or absolute
*	@details	This function starts positioning of an axis relative or absolute.
*				Do not use this funtion. Use @ref sdkDS402_PPM_PosAbsStart() or @ref sdkDS402_PPM_PosRelStart() instead.
*				This funtion is blocking until the setpoint acknowledge is received (max 2ms)
*
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		pos 			target position
* 	@param 		relative 		excute the positioning relative or absolute\n
*								@b 0: absolute\n
*								@b 1: relative\n
* 	@param 		startImmediately		start the movement immediately\n
*										@b 0: Finish actual positioning\n
*										@b 1: Abort actual positioning and start next positioning\n
*	@return 	value:	Process value \n
*				value 	= 1 	successful, new setpoint acknowlegded \n
*				value 	= -1 	failed, new setpoint not acknowlegded \n
*				value 	= -2 	failed, previous setpoint has been assumed and no additional setpoint may be accepted \n
*
*/
long helperDS402_PPM_PosStart(long busId, long pos, long relative, long startImmediately)
{
	long retVal = 1;
	long controlWord = 0;
	long setpointAcknowlege=1;
	long time = 0;
	long actState = -1;

	printf("...: new setpoint: %d\n", pos);

	if(sdkDS402_ReadStatusWord(busId).i[DS402_SW_PPM_BIT_SETPOINT_ACK]==1)
	{
		print("...: the previous setpoint has been assumed and no additional setpoint may be accepted");
		return(-2);
	}

	actState = sdkDS402_GetActDriveState(busId);
	if(actState != DS402_DRIVE_STATE_OPERATION_ENABLED)
	{
		print("...: drive state is not in operation enabled");
		return(-1);
	}

	SdoWrite(busId, DS402_TARGET_POSITION, 0 ,pos);

	controlWord = sdkDS402_ReadControlWord(busId);
	controlWord.i[DS402_CW_PPM_BIT_NEW_SETPOINT] = 1;
	controlWord.i[DS402_CW_PPM_BIT_CHANGE_IMMEDIATELY] = startImmediately;
	controlWord.i[DS402_CW_PPM_BIT_RELATIVE_MOVEMENT] = relative;
	controlWord.i[DS402_CW_PPM_BIT_ENDLESS_MOVEMENT] = 0;

	sdkDS402_WriteControlWord(busId, controlWord);

	time = Time();
	while(sdkDS402_ReadStatusWord(busId).i[DS402_SW_PPM_BIT_SETPOINT_ACK]!=1)
	{
		if((Time()-time) > 2) //is blocking until the new setpoint has been acknowledged
		{
			print("...: no setpoint acknowledge received");
			setpointAcknowlege = 0;
			retVal = -1;
			break;
		}
	}
	controlWord = sdkDS402_ReadControlWord(busId);
	controlWord.i[DS402_CW_PPM_BIT_NEW_SETPOINT] = 0;
	controlWord.i[DS402_CW_PPM_BIT_CHANGE_IMMEDIATELY] = 0;
	controlWord.i[DS402_CW_PPM_BIT_RELATIVE_MOVEMENT] = 0;
	controlWord.i[DS402_CW_PPM_BIT_ENDLESS_MOVEMENT] = 0;

	sdkDS402_WriteControlWord(busId, controlWord);

	if(setpointAcknowlege)
	{
		print("...: setpoint acknowleged");
	}

	return(retVal);
}


/**
*	@brief 		Start absolute positioning
*	@details	This function starts absolute positioning
*				This funtion is blocking until the setpoint acknowledge is received (max 2ms)
* 	@param 		busId 					Bus ID of the connected slave
* 	@param 		pos 					target position
* 	@param 		startImmediately		start the movement immediately\n
*										@b 0: Finish actual positioning\n
*										@b 1: Abort actual positioning and start next positioning\n
*	@return 	value:	Process value \n
*				value 	= 1 	successful, new setpoint acknowlegded \n
*				value 	= -1 	failed, new setpoint not acknowlegded \n
*				value 	= -2 	failed, previous setpoint has been assumed and no additional setpoint may be accepted \n
*
*/
long sdkDS402_PPM_PosAbsStart(long busId, long pos, long startImmediately)
{

	printf("Bus Id %d, sdkDS402_PPM_PosAbsStart:\n", busId);
	return helperDS402_PPM_PosStart(busId, pos, DS402_PPM_ABSOLUTE, startImmediately);
}

/**
*	@brief 		Start relative positioning
*	@details	This function starts relative positioning
*				This funtion is blocking until the setpoint acknowledge is received (max 2ms)
* 	@param 		busId 					Bus ID of the connected slave
* 	@param 		pos 					relative movement value
* 	@param 		startImmediately		start the movement immediately\n
*										@b 0: Finish actual positioning\n
*										@b 1: Abort actual positioning and start next positioning\n
*	@return 	value:	Process value \n
*				value 	= 1 	successful, new setpoint acknowlegded \n
*				value 	= -1 	failed, new setpoint not acknowlegded \n
*				value 	= -2 	failed, previous setpoint has been assumed and no additional setpoint may be accepted \n
*
*/
long sdkDS402_PPM_PosRelStart(long busId, long pos, long startImmediately)
{
	printf("Bus Id %d, sdkDS402_PPM_PosRelStart:\n", busId);
	return helperDS402_PPM_PosStart(busId, pos, DS402_PPM_RELATIVE, startImmediately);
}


/**
*	@brief 		Checks whether the target was reached
*	@details	This function verifies whether the axis reached the target position
*				This function is not blocking until the axis reaches the target.
* 	@param 		busId 			Bus ID of the connected slave
*	@return 	value:	Process value \n
*				value 	= 1 Target reached 	\n
*				value 	= 0 Target NOT reached 	\n
*
*/
long sdkDS402_PPM_TargetReached(long busId)
{
	return(sdkDS402_ReadStatusWord(busId).i[DS402_SW_BIT_TARGET_REACHED]);
}

/**
*	@brief 		Wait until target is reachd
*	@details	This function waits (blocks) until the axis has reached the target position whthin the defined timeout.
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		timeout			Timeout until the function is aborted \n
*								@b >=0: timeout in [ms] \n
*								@b < 0: infinite timeout
*	@return 	value:	Process value \n
*				value 	> 0 Target reached 	\n
*				value 	< 0 Timeout expired \n
*
*/
long sdkDS402_PPM_WaitTargetReached(long busId, long timeout)
{
	long time = Time();
	long targetReached = 1;

	printf("Bus Id %d, sdkDS402_PPM_WaitTargetReached:\n", busId);

	while(sdkDS402_PPM_TargetReached(busId) == 0 && targetReached)
	{
		if(timeout >= 0 &&  (Time() - time) > timeout)
		{
			targetReached = -1;
			printf("...: target position not reached within timeout (%d ms)\n", timeout);
		}
	}
	if(targetReached)
	{
		print("...: target position reached");
	}

	return(targetReached);
}

/***************************************************************************
*Profile Velocity Mode functions
***************************************************************************/


/**
*	@brief 		Set target velocity for PVM
*	@details	This function sets the target velocity for the Profile Position Mode (PVM)
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		targetVelocity	Target velocity
*
*/
void sdkDS402_PVM_SetTargetVelocity(long busId, long targetVelocity)
{
	printf("Bus Id %d, PVM: Set target velocity to %d\n", busId, targetVelocity);
	SdoWrite(busId, DS402_TARGET_VELOCITY, 0, targetVelocity);
}

/**
*	@brief 		Start continuous velocity (PVM)
*	@details	This function start to run the axis in continuous velocity
*				The velocity can be defined with the method sdkDS402_PVM_SetTargetVelocity.
* 	@param 		busId 			Bus ID of the connected slave
*	@return 	value:	Process value \n
*				value 	1  Successful \n
*				value 	-1 Failed, drive is not in 'Operation enabled' state \n
*
*/
long sdkDS402_PVM_CvelStart(long busId)
{
	long actState;
	long controlWord;

	printf("Bus Id %d, sdkDS402_PPM_CvelStart: \n", busId);

	actState = sdkDS402_GetActDriveState(busId);
	if(actState != DS402_DRIVE_STATE_OPERATION_ENABLED)
	{
		printf("...: drive state is not in operation enabled\n");
		return(-1);
	}

	//Reset the Halt bit to start the execution
	controlWord = sdkDS402_ReadControlWord(busId);
	controlWord.i[DS402_CW_BIT_HALT] = 0;
    sdkDS402_WriteControlWord(busId, controlWord);

   	printf("...: started\n");

    return(1);
}

/**
*	@brief 		Stop continuous velocity (PVM)
*	@details	This function stops the axis.
* 	@param 		busId 			Bus ID of the connected slave
*	@return 	value:	Process value \n
*				value 	1  Successful \n
*/
long sdkDS402_PVM_CvelStop(long busId)
{
	long controlWord;
	printf("Bus Id %d, sdkDS402_PPM_CvelStop:\n", busId);

	//Set the Halt bit, to stop the execution
	controlWord = sdkDS402_ReadControlWord(busId);
	controlWord.i[DS402_CW_BIT_HALT] = 1;

    sdkDS402_WriteControlWord(busId, controlWord);
    printf("...: stopped\n");

    return(1);
}

/**
*	@brief 		Checks whether the target velocity has reached
*	@details	This function verifies whether the axis reached the target velocity
* 	@param 		busId 			Bus ID of the connected slave
*	@return 	value:	Process value \n
*				value 	= 1 Target reached 	\n
*				value 	= 0 Target NOT reached 	\n
*
*/
long sdkDS402_PVM_TargetReached(long busId)
{
	return sdkDS402_ReadStatusWord(busId).i[DS402_SW_BIT_TARGET_REACHED];
}

/**
*	@brief 		Wait until target is reachd
*	@details	This function waits (blocks) until the axis has reached the target velocity whthin the defined timeout.
* 	@param 		busId 			Bus ID of the connected slave
* 	@param 		timeout			Timeout until the funcition returns \n
*								@b >=0: timeout in [ms] \n
*								@b < 0: infinite timeout\n
*	@return 	value:	Process value \n
*				value 	> 0 Target reached 	\n
*				value 	< 0 Timeout expired \n
*
*/
long sdkDS402_PVM_WaitTargetReached(long busId, long timeout)
{
	long time = Time();
	long targetReached = 1;

	printf("Bus Id %d, sdkDS402_PVM_WaitTargetReached:\n", busId);

	while(sdkDS402_PPM_TargetReached(busId) == 0 && targetReached)
	{
		if(timeout >= 0 &&  (Time() - time) > timeout)
		{
			targetReached = -1;
			printf("...: target velocity not reached within timeout (%d ms)\n", timeout);
		}
	}
	if(targetReached)
	{
		print("...: target velocity reached");
	}

	return(targetReached);
}