/**
*	@file		SDK_Error_Descriptions.mc
*	@brief		Functions to get description error messages.
*           	ApossIde Errors\n
*           	CiA 301 SDO Errors\n
*
*	@example 	CAN_1Ax_DS402_ProfilePositionMode-ppm-Test.mc
*	@example 	CAN_1Ax_DS402_ProfileVelocityMode-pvm-Test.mc
*	@example 	CAN_MultipleAx_DS402_ProfilePositionMode-ppm-Test.mc
*	@example 	CAN_MultipleAx_DS402_ProfilePositionMode-pvm-Test.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Error_Description.mh"


/**
*	@brief 		Prints a error description for ApossIDE errors
*	@details	This function prints a description for an error of ApossIDE
* 	@param 		errorCode		errorCode of ApossIDE
*
*/
void sdkErrorPrint_ApossIdeErrorDescription(long errorCode)
{
	switch(errorCode)
	{
		case F_CANMEM:
			printf("There are no more CAN objects available (CANINI).");
			break;
		case F_NOAXES:
			printf("Axis not in system.");
			break;
		case F_ERR:
			printf("Error not cleared.");
			break;
		case F_NOMNU:
			printf("Failed to move to HOME position.");
			break;
		case F_MNU:
			printf("Home velocity 0.");
			break;
		case F_POSERR:
			printf("Position error.");
			break;
		case F_NIO:
			printf("Index pulse (encoder) not found.");
			break;
		case F_UNBEK:
			printf("Unknown command.");
			break;
		case F_GRENZE:
			printf("Software end limit activated.");
			break;
		case F_PARNUM:
			printf("Illegal parameter number.");
			break;
		case F_NOCHZ:
			printf("Too many nested loops.");
			break;
		case F_NUMFORMAT:
			printf("INLONG command got an illegal string.");
			break;
		case F_CRCPAR:
			printf("Parameters in memory are corrupted.");
			break;
		case F_CRCPRG:
			printf("Programs in memory are corrupted.");
			break;
		case F_WDT:
			printf("Reset by CPU.");
			break;
		case F_ABBRUCH:
			printf("User abort.");
			break;
		case F_VLTCOM:
			printf("FC communication error.");
			break;
		case F_SDOCHN:
			printf("Number of SDO channels exceeded.");
			break;
		case F_FEATUREPROT:
			printf("Feature protection error.");
			break;
		case F_ARMCOM:
			printf("Communication with ARM lost.");
			break;
		case F_ENDSCHALT:
			printf("Limit switch activated.");
			break;
		case F_ILLGLSDO:
			printf("SDO access error in SYSVAR or LINK command.");
			break;
		case F_NOPROG:
			printf("Trial to execute a cleared or empty program.");
			break;
		case F_FPGA:
			printf("Wrong or no FPGA firmware loaded.");
			break;
		case F_AMP:
			printf("Amplifier error.");
			break;
		case F_ECAT_MASTER:
			printf("EtherCAT Master Error.");
			break;
		case F_NOINTLEFT:
			printf("Too many interrupt functions.");
			break;
		case F_UMIN:
			printf("Minimum voltage of the amplifier has been undershot.");
			break;
		case F_STACK:
			printf("Too many nested function/interruption calls.");
			break;
		case F_RETURN:
			printf("Too many return() commands.");
			break;
		case F_MATHERR:
			printf("A floating point function was called with an invalid argument.");
			break;
		case F_STATEMACHINE:
			printf("State machine error.");
			break;
		case F_INTPTR:
			printf("Interrupt happened, but interrupt address is no longer valid.");
			break;
		case F_MEEP:
			printf("Error in verifying.");
			break;
		case F_PATHERR:
			printf("Path control only: Path error.");
			break;
		case F_TIMEOUT:
			printf("Path control only: Time out in V24 path control.");
			break;
		case F_PATHINT:
			printf("Path control only.");
			break;
		case F_DIM:
			printf("Too many DIM arrays defined.");
			break;
		case F_ARRBDS:
			printf("Invalid array index.");
			break;
		case F_LTARRAY:
			printf("Array number does not exist.");
			break;
		case F_NOARRAY:
			printf("Array is empty.");
			break;
		case F_NOSPACE:
			printf("No more memory space for the new array defined by DIM.");
			break;
		case F_ARRSIZERR:
			printf("Array size does not correspond to the size of the existing array.");
			break;
		case F_TEMP:
			printf("Maximum temperature of the amplifier has been exceeded.");
			break;
		case F_UMAX:
			printf("Maximum voltage of the amplifier has been exceeded.");
			break;
		case F_TNDX:
			printf("Timeout while waiting for index.");
			break;
		case F_CMDERR:
			printf("Internal command error (illegal parameter or format or range).");
			break;
		case F_TIM:
			printf("Too many TIME or PERIOD interrupts.");
			break;
		case F_NOVARMEM:
			printf("Not enough memory for variables.");
			break;
		case F_CANGUARD:
			printf("CAN guarding error.");
			break;
		case F_CANIO:
			printf("CAN send or receive error.");
			break;
		case F_MEMLOCK:
			printf("Memory locked.");
			break;
		case F_CURARR:
			printf("Illegal curve array in SETCURVE.");
			break;
		case F_ENCERR:
			printf("Encoder error.");
			break;
		case F_DYNSTACK:
			printf("Stack overflow: Too many local variables or nested function calls.");
			break;
		case F_DYNMEM:
			printf("Out of dynamic memory.");
			break;
		case F_OPALINDX:
			printf("Too many test indices in data logging command.");
			break;
		case F_ILLGLCODE:
			printf("Code is too old for the current firmware.");
			break;
		case F_IMAX:
			printf("Internal overcurrent detection of power stage.");
			break;
		case F_LIMIT_VIOLATION:
			printf("Wrong direction after limit switch tripped and error reset.");
			break;
		case F_I2TLIMIT:
			printf("I²T Limit exceeded.");
			break;
		case F_AMPCOM:
			printf("Communication with amplifier interrupted.");
			break;
		case F_AMPPARAM:
			printf("Illegal access to amplifier parameter.");
			break;
		case F_NOTSUPPORTED:
			printf("Command is not supported.");
			break;
		case F_MEMDUMP:
			printf("MemoryDump error.");
			break;
		case F_DIVIDEBY0:
			printf("Divide by 0.");
			break;
		case F_ECAT_SLAVE:
			printf("EtherCAT Slave Error.");
			break;
		case F_PROFGEN:
			printf("Profile Generator Error.");
			break;
		case F_DIMTOOSHORT:
			printf("DIM array is too short.");
			break;
		case F_INTERNALERROR:
			printf("Internal firmware error.");
			break;
		case F_OPTIONBOOT:
			printf("Option boot failure.");
			break;
		case F_COMMAND_USAGE:
			printf("Wrong command usage.");
			break;
		case F_PROCESSDATAUSAGE:
			printf("Illegal write access to process data.");
			break;
		case F_ARGTOOLONG:
			printf("The size of string argument too long.");
			break;
		case F_AUTHFAILED:
			printf("Authentication failed.");
			break;
		case F_AMP_NOTREADY:
			printf("Amplifier not ready.");
			break;
		case F_LOGIC_UMIN:
			printf("Logic Supply Voltage too low.");
			break;
		case F_ZFCUPDATE:
			printf("ZFC file update failed.");
			break;
		case F_HWFAIL:
			printf("Hardware Failure.");
			break;
		case F_AMP_INIT_ORDER:
			printf("Amplifier's initialization order violated.");
			break;
		case F_HALL:
			printf("Hall sensor problem.");
			break;
		//case F_HALL_ANGLE:
			//printf("Hall angle detection error.");
			//break;
		//case F_STO:
			//printf("STO error.");
			//break;
		default:
			printf("Unknown error code (%d).", errorCode);
			break;
	}
}

/**
*	@brief 		Prints a error description for SDO abort codes
*	@details	This function prints a description for CiA 301 SDO abort codes
* 	@param 		errorCode		SDO abort code
*
*/
void sdkErrorPrint_SdoErrorDescription(long errorCode)
{
    switch (errorCode)
    {
        case SDO_ERR_ACCESS_RO:
            printf("Attempt to write a read only object.");
            break;
        case SDO_ERR_ACCESS_UNSUPPORTED:
            printf("Unsupported access to an object.");
            break;
        case SDO_ERR_ACCESS_WO:
            printf("Attempt to read a write only object.");
            break;
        case SDO_ERR_BLOCK_CRC:
            printf("Invalid block CRC value (block mode only).");
            break;
        case SDO_ERR_BLOCK_SEQUENCE:
            printf("Invalid block sequence number (block mode only).");
            break;
        case SDO_ERR_BLOCK_SIZE:
            printf("Invalid block size (block mode only).");
            break;
        case SDO_ERR_COMMAND:
            printf("Client/server command specifier not valid or unknown.");
            break;
        case SDO_ERR_DATA_STORE:
            printf("Data cannot be transferred or stored to the application.");
            break;
        case SDO_ERR_DATA_STORE_LOCAL:
            printf("Data cannot be transferred or stored to the application because of local control.");
            break;
        case SDO_ERR_DATA_STORE_STATE:
            printf("Data cannot be transferred or stored to the application because of the present device state.");
            break;
        case SDO_ERR_DATATYPE:
            printf("Data type does not match, length of service parameter does not match.");
            break;
        case SDO_ERR_DATATYPE_HIGH:
            printf("Data type does not match, length of service parameter too high.");
            break;
        case SDO_ERR_DATATYPE_LOW:
            printf("Data type does not match, length of service parameter too low.");
            break;
        case SDO_ERR_GENERAL:
            printf("General error.");
            break;
        case SDO_ERR_GENERAL_DEVICE:
            printf("General internal incompatibility in the device.");
            break;
        case SDO_ERR_GENERAL_PARAMETER:
            printf("General parameter incompatibility reason.");
            break;
        case SDO_ERR_HARDWARE:
            printf("Access failed due to a hardware error.");
            break;
        case SDO_ERR_MAPPING_LENGTH:
            printf("The number and length of the objects to be mapped would exceed PDO length.");
            break;
        case SDO_ERR_MAPPING_OBJECT:
            printf("Object cannot be mapped to the PDO.");
            break;
        case SDO_ERR_MEMORY:
            printf("Out of memory.");
            break;
        case SDO_ERR_NO_DATA:
            printf("No data available.");
            break;
        case SDO_ERR_NO_OBJECT:
            printf("Object does not exist in the object dictionary.");
            break;
        case SDO_ERR_NO_SUB_INDEX:
            printf("Sub-index does not exist.");
            break;
        case SDO_ERR_OBJECT_DICTIONARY:
            printf("Object dictionary dynamic generation fails or no object dictionary is present.");
            break;
        case SDO_ERR_SDO_CONNECTION:
            printf("Resource not available: SDO connection.");
            break;
        case SDO_ERR_TIMEOUT:
            printf("SDO protocol timed out.");
            break;
        case SDO_ERR_TOGGLE_BIT:
            printf("Toggle bit not altered.");
            break;
        case SDO_ERR_VALUE_RANGE:
            printf("Invalid value for parameter (download only).");
            break;
        case SDO_ERR_VALUE_HIGH:
            printf("Value of parameter written too high (download only).");
            break;
        case SDO_ERR_VALUE_LOW:
            printf("Value of parameter written too low (download only).");
            break;
        case SDO_ERR_VALUE_MIN_MAX:
            printf("Maximum value is less than minimum value.");
            break;
        default:
            printf("Unknown error code (0x%lX).", errorCode);
          	break;
    }
}

