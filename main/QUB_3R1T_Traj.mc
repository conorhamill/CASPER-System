/**
*	@brief		3R1T - homing, kinematic control and trajectory playback.
*
*	@detail		Four numbers in, one MOVE button, one HOME button.
*
*				  psi      tilt about the base X axis      [deg]
*				  phi      tilt about the base Y axis      [deg]
*				  theta_n  spin about the platform normal  [deg]
*				  tool     position along the stage        [mm]
*
*				Use with:  3R1T_traj_panel.zbm               the panel
*				           ../sensor_node/sensor_node_esp32  the sensors
*
*
*				HOW THE FILES FIT TOGETHER
*
*				  3R1T_Config.mh   every tunable number. Constants only.
*				  3R1T_Globals.mh  every variable, and the helpers that
*				                   only shuffle bits.
*				  3R1T_Kin_3R.mh   platform orientation -> 3 motor angles.
*				                   Complete.
*				  3R1T_Kin_1T.mh   tool position -> stage axis. A STUB -
*				                   linear pass-through, honest about it.
*				  QUB_3R1T_Traj.mc this file: what the rig DOES. The
*				                   state machine, homing, trajectory
*				                   playback, and the 2 ms ISR.
*
*				If you are changing a number, it is in Config. If you are
*				changing what the machine does next, it is here.
*
*
*				HOW MOTION WORKS
*
*				The commanded pose lives on four SIMULATED axes. MOVE
*				gives them an ordinary coordinated profiled move, and a
*				2 ms interrupt reads where they have got to, runs the
*				kinematics, and streams the answer to the real drives
*				through REG_USERREFPOS.
*
*				That indirection is the whole design:
*
*				  - the path is straight in POSE space, which is what you
*				    asked for, rather than straight in motor space, which
*				    you did not;
*				  - the IK continuity tracker sees the pose creep in tiny
*				    steps and never has to guess a branch. Handed a large
*				    jump it guesses wrong. See the warning in
*				    3R1T_Kin_3R.mh.
*
*				STOP stops the SIMULATED axes. The real drives follow the
*				kinematics to a standstill with them. Never AxisStop a
*				real axis while it is in USERREFPOS - it is not running a
*				profile, so there is nothing there to stop.
*
*
*				TELEOPERATION
*
*				A haptic device on a PC can drive the pose instead of the
*				MOVE button. It streams the same four targets over
*				Ethernet at about 100 Hz and the Teleop state retargets
*				the same four simulated axes - so the profile generator,
*				the kinematics and the 2 ms interrupt are all unchanged
*				and unaware. See SECTION 7c in Config for why it is done
*				that way rather than in the interrupt.
*
*				Control is handed over only while the operator holds an
*				enable, and taken back the moment the PC's heartbeat
*				stops. The PC side lives in ../haptic.
*
*
*				POWER
*
*				Everything energises at startup. There are no power
*				buttons. Anything pressed while the rig is booting is
*				thrown away, so a stale press cannot fire the instant the
*				drives come alive.
*
*
*				THE STATUS LED
*
*				  green  homed, streaming, will accept MOVE
*				  amber  working, or not ready yet
*				  red    error - drives are off, press CLR ERR
*/
#include <SysDef.mh>
#include "..\ApossC_SDK_V1-15\SDK\SDK_ApossC.mc"

#include "3R1T_Config.mh"		// constants
#include "3R1T_Globals.mh"		// variables and bit-shuffling helpers
#include "3R1T_Faults.mh"		// what error 40 actually was
#include "3R1T_Kin_3R.mh"		// platform orientation -> motor angles
#include "3R1T_Kin_1T.mh"		// tool position -> stage axis


#define ID_SM_MAIN		0

#pragma SmConfig { (SM_RUN_DELAY | SM_RUN_INTERRUPT | SM_RUN_DELDISABLE),
                   20, 5, 5, 25, 5, 5 }

SmEvent SIG_TOGGLE {}

// The ISR is armed in main(), before the axes are configured. This holds
// it off until SIG_INIT has finished setting them up.
long g_isr_ready   = 0;
long g_startup_ok  = 0;


///////////////////////////////////////////////////////////////////////////
// Drives
///////////////////////////////////////////////////////////////////////////
// AxisControl(ON) only STARTS the DS402 handshake, and at boot it races
// the CANopen slaves going OPERATIONAL. A drive left in "switch-on
// disabled" then ignores every setpoint in silence, which looks exactly
// like a mechanical problem. So the state is read back and retried.
long CheckOneDrive(long ax, long node)
{
#if (AXES_MODE != SIM_MODE)
	long sw, ec, retry, ok;

	sw = SdoRead(node, EPOS4_STATUSWORD, 0x00);
	ec = SdoRead(node, EPOS4_ERROR_CODE, 0x00);
	if ((sw & 0x6F) == 0x27) {
		if (g_verbose) print("Drive ",node," (axis ",ax,") ENABLED      sw=",radixstr(sw,16),"  err=",radixstr(ec,16));
		return(TRUE);
	}
	print("Drive ",node," (axis ",ax,") NOT ENABLED  sw=",radixstr(sw,16),"  err=",radixstr(ec,16)," -> clearing fault, re-enabling");

	// >>> REMEMBER IT BEFORE CLEARING IT. <<<
	// AmpErrorClear wipes 0x603F, so without this the reason a drive came
	// up faulted is gone by the time anything asks - which is exactly what
	// happened to the first start-up fault report on this rig.
	StashBootError(node, ec);
	PrintEpos4Fault(ec);

	if (sw & 0x08) { AmpErrorClear(ax); }
	AxisControl(ax, OFF);
	AxisControl(ax, ON);
	ok = FALSE;
	for (retry = 0; retry < 4; retry++) {
		Delay(500);
		sw = SdoRead(node, EPOS4_STATUSWORD, 0x00);
		if ((sw & 0x6F) == 0x27) {
			if (g_verbose) print("Drive ",node," (axis ",ax,") -> now ENABLED  sw=",radixstr(sw,16));
			ok = TRUE;
			retry = 99;
		}
	}
	if (ok == FALSE) {
		print("Drive ",node," (axis ",ax,") -> STILL NOT ENABLED  sw=",radixstr(sw,16),"  - check supply and wiring");
	}
	return(ok);
#else
	return(TRUE);
#endif
}

long CheckDrivesEnabled(void)
{
	long ok;

	ok = TRUE;
	if (CheckOneDrive(C_AXIS1,    C_DRIVE_BUSID1)    == FALSE) { ok = FALSE; }
	if (CheckOneDrive(C_AXIS2,    C_DRIVE_BUSID2)    == FALSE) { ok = FALSE; }
	if (CheckOneDrive(C_AXIS3,    C_DRIVE_BUSID3)    == FALSE) { ok = FALSE; }
	if (CheckOneDrive(C_AXIS_1T,  C_DRIVE_BUSID_1T)  == FALSE) { ok = FALSE; }
	return(ok);
}

// A drive with no current limit produces no torque.
void ApplyCurrentLimits(void)
{
#if (AXES_MODE != SIM_MODE)
	// 0x3001 sub 2 is the EPOS4's output current limit, in mA. One figure
	// for all four - see the note beside USR_CURRENT_LIMIT in Config.
	SdoWrite(C_DRIVE_BUSID1,   0x3001, 2, USER_PARAM(USR_CURRENT_LIMIT));
	SdoWrite(C_DRIVE_BUSID2,   0x3001, 2, USER_PARAM(USR_CURRENT_LIMIT));
	SdoWrite(C_DRIVE_BUSID3,   0x3001, 2, USER_PARAM(USR_CURRENT_LIMIT));
	SdoWrite(C_DRIVE_BUSID_1T, 0x3001, 2, USER_PARAM(USR_CURRENT_LIMIT));
	print("Current limit set to ",USER_PARAM(USR_CURRENT_LIMIT)," mA on all four drives");

	// >>> AND CHECK THE FOUR ARE THE SAME MOTOR BEFORE TRUSTING THAT. <<<
	// One limit only makes sense while they are. See CheckDriveMatch.
	CheckDriveMatch();

	// What each drive does when the controller stops talking - the reason
	// a drive can come up faulted after a program reload. Reported every
	// boot; only written if Config asks. See C_ABORT_CONN_OPTION.
	ApplyAbortConnOption();
#endif
}

// Hand the drives back to ordinary position control. Called before homing
// and on a fault: while armed they are following REG_USERREFPOS and will
// ignore AxisPosAbsStart entirely.
void LeaveStream(void)
{
	if (g_ik_armed == 0) { return; }
	g_ik_armed = 0;
	StaClr(C_STA_STREAM);
	AxisControl(C_ARM1_AXIS, ON, C_ARM2_AXIS, ON, C_ARM3_AXIS, ON, C_AXIS_1T, ON);
	if (g_verbose) print("Kinematics stream released - drives back in position control");
}


///////////////////////////////////////////////////////////////////////////
// CAN2 sensor frames
///////////////////////////////////////////////////////////////////////////
void CanArm(void)
{
	canObjTheta  = DefCanIn(CAN_BUS_OFFSET + CAN_ID_THETA,  8);
	canObjTheta2 = DefCanIn(CAN_BUS_OFFSET + CAN_ID_THETA2, 8);
	canObjPlat  = DefCanIn(CAN_BUS_OFFSET + CAN_ID_PLAT,  8);
	canObjTrans = DefCanIn(CAN_BUS_OFFSET + CAN_ID_TRANS, 8);
	canObjRef   = DefCanIn(CAN_BUS_OFFSET + CAN_ID_REF,   8);
	canLastRxTheta  = Time();
	canLastRxTheta2 = Time();
	canLastRxPlat   = Time();
	canLastRxEnc   = Time();
	if (g_verbose) print("CAN: receivers armed  theta=",canObjTheta," plat=",canObjPlat," trans=",canObjTrans," ref=",canObjRef);
}

void CanPoll(void)
{
	long b03, b47, now;

	// Valid DefCanIn handles include 0, so poll on >= 0.
	// >>> THE LIMB ANGLES ARE SIGNED int32 NOW, ACROSS TWO FRAMES. <<<
	//
	// They were three uint16 of 0..35999 packed into 0x6E4. The sensor node
	// unwraps them through the 360/0 seam, so they are continuous and
	// signed - which is what lets homing average and difference them
	// without a spike at the seam, and what makes theta read about zero at
	// home instead of flicking between 0 and 360.
	//
	// The slots are longs and the units are still centidegrees, so nothing
	// downstream of here changes. An OLD sensor node against this parser
	// reads nonsense, not stale values: flash both sides together.
	if (canObjTheta >= 0) {
		if (CanIn(canObjTheta, -1, 0, b03, b47) == 0) {
			USER_PARAM(USR_THETA1) = CanI32(b03, b47, 0);
			USER_PARAM(USR_THETA2) = CanI32(b03, b47, 4);
			g_frames = g_frames + 1;
			canLastRxTheta = Time();
		}
	}
	// theta3 carries the status for all three, so this is the frame that
	// completes a set.
	if (canObjTheta2 >= 0) {
		if (CanIn(canObjTheta2, -1, 0, b03, b47) == 0) {
			USER_PARAM(USR_THETA3)     = CanI32(b03, b47, 0);
			USER_PARAM(USR_IMU_STATUS) = CanBusByte(b03, b47, 4);
			canLastRxTheta2 = Time();
		}
	}
	if (canObjPlat >= 0) {
		if (CanIn(canObjPlat, -1, 0, b03, b47) == 0) {
			USER_PARAM(USR_PLAT_X)      = CanI16(b03, b47, 0);
			USER_PARAM(USR_PLAT_Y)      = CanI16(b03, b47, 2);
			USER_PARAM(USR_PLAT_Z)      = CanI16(b03, b47, 4);
			USER_PARAM(USR_PLAT_STATUS) = CanBusByte(b03, b47, 6);
			canLastRxPlat = Time();
		}
	}
	if (canObjTrans >= 0) {
		if (CanIn(canObjTrans, -1, 0, b03, b47) == 0) {
			USER_PARAM(USR_ENC_RAW_UM) = CanI32(b03, b47, 0);
			USER_PARAM(USR_ENC_VEL)    = CanI16(b03, b47, 4);
			USER_PARAM(USR_ENC_STATUS) = CanBusByte(b03, b47, 6);
			USER_PARAM(USR_ENC_UM)     = USER_PARAM(USR_ENC_RAW_UM) - g_zero_um;
			canLastRxEnc = Time();
		}
	}
	if (canObjRef >= 0) {
		if (CanIn(canObjRef, -1, 0, b03, b47) == 0) {
			USER_PARAM(USR_REF_RISE_UM) = CanI32(b03, b47, 0);
			USER_PARAM(USR_REF_FALL_UM) = CanI32(b03, b47, 4);
		}
	}

	now = Time();
	// Stale if EITHER limb frame has gone quiet - one without the other is
	// an incomplete set, not a usable one.
	USER_PARAM(USR_THETA_STALE) = (((now - canLastRxTheta)  > CAN_STALE_MS) ||
	                               ((now - canLastRxTheta2) > CAN_STALE_MS)) ? TRUE : FALSE;
	USER_PARAM(USR_PLAT_STALE)  = ((now - canLastRxPlat)  > CAN_STALE_MS) ? TRUE : FALSE;
	USER_PARAM(USR_ENC_STALE)   = ((now - canLastRxEnc)   > CAN_STALE_MS) ? TRUE : FALSE;

	if (USER_PARAM(USR_CAN_RESET) == 1) {
		USER_PARAM(USR_CAN_RESET) = 0;
		CanArm();
		Say(MSG_CAN_RESET);
	}
}

// Bits 4-7 of the stage status byte are a rolling count of completed
// crossings of the REF mark. Homing watches this rather than the level:
// a fast pass can start and finish between two CAN frames, so the level
// alone is missable but a counter never is.
long RefPasses(void)
{
	return((USER_PARAM(USR_ENC_STATUS) >> 4) & 0x0F);
}


///////////////////////////////////////////////////////////////////////////
// Motion helpers
///////////////////////////////////////////////////////////////////////////
// Homing speed in APOSS velocity units, converted from cdeg/s.
//
// Deliberately NOT SetVelAccDec: its Sysvar conversion is an error-8
// (track error) suspect, and small numbers through it command a crawl
// rather than a slow but real move.
long SetRealAxisVel(long axe, long vel_cdeg_s)
{
	long motorRpm, velUnits;

	motorRpm = (vel_cdeg_s * 60 * C_AXIS_POSFACT_Z) / C_AXIS_FEEDDIST;
	velUnits = (motorRpm * C_AXIS_VELRES) / C_AXIS_MAX_RPM;
	if (velUnits < 1)              { velUnits = 1; }
	if (velUnits > C_AXIS_VELRES)  { velUnits = C_AXIS_VELRES; }

	Cvel(axe, velUnits);
	Vel(axe, velUnits);
	Acc(axe, HOME_ACC_UNITS);
	Dec(axe, HOME_ACC_UNITS);
	return(0);
}

// TRUE when all three arm axes are back in plain position control AND
// within the window of the given targets. The profile-generator test
// matters: SM_STAT_POSREACHED only gives a rising edge, and gives none at
// all for a zero-length move, so polling is the only thing that cannot
// deadlock.
long ArmsAtTargets(long t1, long t2, long t3)
{
	// Both halves are needed. Without the profile-generator test an arm
	// still decelerating THROUGH the window counts as arrived, and the
	// next measurement is then taken while it is still moving.
	if (AXE_PROCESS(C_ARM1_AXIS, PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(C_ARM2_AXIS, PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(C_ARM3_AXIS, PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }

	if (AbsL(Cpos(C_ARM1_AXIS) - t1) > C_DONE_TOL_CDEG) { return(FALSE); }
	if (AbsL(Cpos(C_ARM2_AXIS) - t2) > C_DONE_TOL_CDEG) { return(FALSE); }
	if (AbsL(Cpos(C_ARM3_AXIS) - t3) > C_DONE_TOL_CDEG) { return(FALSE); }
	return(TRUE);
}

// The commanded-pose axes have STOPPED.
//
// >>> NOT THE SAME QUESTION AS PoseAtTarget. <<<
//
// This asks only whether the profile generators have finished. PoseAtTarget
// also asks whether they finished in the right PLACE, and the two come
// apart: a coordinated move that is superseded on its deceleration edge
// leaves a small residual on any axis whose share of the move was small,
// and that residual can sit outside C_DONE_TOL_CDEG for several waypoints.
//
// For a PAUSE, STOPPED is the question. The hold exists to put the rig at
// rest before the next move goes out, and a profile generator that has
// finished is at rest; that the platform finished a fraction of a degree
// from where it was asked to is a different complaint, and blocking on it
// stalls the run for the full settle timeout before the hold can even
// begin - which is what "step 65 did not come to rest" was.
long PoseStopped(void)
{
	if (AXE_PROCESS(EE_PSI,     PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(EE_PHI,     PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(EE_THETA_N, PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(EE_TOOL,    PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	return(TRUE);
}

// The commanded-pose axes have arrived. Same reasoning as above.
long PoseAtTarget(void)
{
	if (AXE_PROCESS(EE_PSI,     PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(EE_PHI,     PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(EE_THETA_N, PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }
	if (AXE_PROCESS(EE_TOOL,    PFG_AKTSTATE) != PGS_POSCTRL) { return(FALSE); }

	if (AbsL(Cpos(EE_PSI)     - USER_PARAM(USR_TGT_PSI))     > C_DONE_TOL_CDEG) { return(FALSE); }
	if (AbsL(Cpos(EE_PHI)     - USER_PARAM(USR_TGT_PHI))     > C_DONE_TOL_CDEG) { return(FALSE); }
	if (AbsL(Cpos(EE_THETA_N) - USER_PARAM(USR_TGT_THETA_N)) > C_DONE_TOL_CDEG) { return(FALSE); }
	if (AbsL(Cpos(EE_TOOL)    - USER_PARAM(USR_TGT_TOOL))    > C_DONE_TOL_TOOL) { return(FALSE); }
	return(TRUE);
}

// The cable-demand path scan lived here. It sampled the planned move,
// found the largest cable step between consecutive samples, and let
// StartPoseMove slow every axis to suit. Removed 2026-09-30 by request -
// see the note in StartPoseMove for what that means for the stage.
//
// ReportCableBudget below is NOT that scan and is still wanted: it prints
// once at boot what the tilt limits demand of the carriage, which is the
// figure LIM_TOOL_MM100 has to be sized against.

void ReportCableBudget(void)
{
	long a, b, c;
	double ps, ph, tn, v, lo, hi;

	lo =  1e9;
	hi = -1e9;
	for (a = -2; a <= 2; a++) {
		for (b = -2; b <= 2; b++) {
			for (c = 0; c < 8; c++) {
				ps = ((double)a / 2.0) * ((double)LIM_PSI_CDEG / 100.0);
				ph = ((double)b / 2.0) * ((double)LIM_PHI_CDEG / 100.0);
				tn = ((double)c) * 45.0 + IK_THETA_N_HOME_DEG;
				v  = CableSpan(ps, ph, tn) - g_L_home;
				if (v < lo) { lo = v; }
				if (v > hi) { hi = v; }
			}
		}
	}
	print("Cable budget over the configured tilt limits: ",lo," to ",hi," mm");
	print("Tool fence LIM_TOOL_MM100 is +/- ",(double)LIM_TOOL_MM100 / 100.0," mm - measure the stage travel and set it");
}


// Refuse a move that asks for something the mechanism cannot do, and say
// why. Much easier to understand than the alternative, which is the IK
// quietly holding the last pose while the machine appears to ignore you.
long PoseCommandOk(void)
{
	if (AbsL(USER_PARAM(USR_TGT_PSI)) > LIM_PSI_CDEG) {
		print("MOVE refused: psi ",USER_PARAM(USR_TGT_PSI)," cdeg is outside +/-",LIM_PSI_CDEG);
		Say(MSG_MOVE_PSI_LIM);
		return(FALSE);
	}
	if (AbsL(USER_PARAM(USR_TGT_PHI)) > LIM_PHI_CDEG) {
		print("MOVE refused: phi ",USER_PARAM(USR_TGT_PHI)," cdeg is outside +/-",LIM_PHI_CDEG);
		Say(MSG_MOVE_PHI_LIM);
		return(FALSE);
	}
	if (AbsL(USER_PARAM(USR_TGT_THETA_N)) > LIM_THETA_N_CDEG) {
		print("MOVE refused: theta_n ",USER_PARAM(USR_TGT_THETA_N)," cdeg is outside +/-",LIM_THETA_N_CDEG);
		Say(MSG_MOVE_THN_LIM);
		return(FALSE);
	}
	if (AbsL(USER_PARAM(USR_TGT_TOOL)) > LIM_TOOL_MM100) {
		print("MOVE refused: tool ",USER_PARAM(USR_TGT_TOOL)," (0.01 mm) is outside +/-",LIM_TOOL_MM100);
		Say(MSG_MOVE_TOOL_LIM);
		return(FALSE);
	}
	return(TRUE);
}

// Start, or blend into, a coordinated move of the commanded pose. All
// four axes are interpolated together so they arrive at the same moment.
void StartPoseMove(void)
{
	long vAng, aAng, vTool, aTool;

	// >>> THE CABLE-DEMAND SCAN AND THE SPEED SCALING ARE GONE. <<<
	//
	// 2026-09-30, by request: no path scan, no 75 % stage cap, no scaling.
	// A move now runs at exactly the speeds in the slots and nothing
	// works anything out.
	//
	// What that removed, so nobody has to reconstruct it from the git log:
	// ScanPosePath sampled the planned path, found the largest cable step
	// between consecutive samples, and slowed ALL FOUR axes by a common
	// factor if the stage could not pay cable out that fast. The cable
	// inlet sits 218 mm from the centre of rotation, so a degree of tilt
	// is about 3.8 mm of cable - a pose move that looks gentle can ask the
	// carriage for tens of mm/s. The table in 3R1T_Kin_1T.mh has the
	// measured figures.
	//
	// So the protection that remains is the drive's own following-error
	// trip, and there are no limit switches on the stage. Keep the pose
	// speeds modest until somebody has watched a fast tilt at full travel.
	vAng  = USER_PARAM(USR_MOVE_VEL);
	aAng  = USER_PARAM(USR_MOVE_ACC);
	vTool = USER_PARAM(USR_TOOL_VEL);
	aTool = USER_PARAM(USR_TOOL_ACC);
	if (vAng  < 1) { vAng  = 1; }
	if (aAng  < 1) { aAng  = 1; }
	if (vTool < 1) { vTool = 1; }
	if (aTool < 1) { aTool = 1; }

	SetVelAccDec(EE_PSI,     vAng,  aAng,  aAng);
	SetVelAccDec(EE_PHI,     vAng,  aAng,  aAng);
	SetVelAccDec(EE_THETA_N, vAng,  aAng,  aAng);
	SetVelAccDec(EE_TOOL,    vTool, aTool, aTool);

	if (g_verbose) print("MOVE to  psi=",USER_PARAM(USR_TGT_PSI)," phi=",USER_PARAM(USR_TGT_PHI)," theta_n=",USER_PARAM(USR_TGT_THETA_N)," tool=",USER_PARAM(USR_TGT_TOOL));
	if (g_verbose) print("MOVE at  ",vAng," cdeg/s / ",vTool," UU/s - no scaling applied");
	AxisLinAbsStart(EE_PSI, USER_PARAM(USR_TGT_PSI), EE_PHI, USER_PARAM(USR_TGT_PHI), EE_THETA_N, USER_PARAM(USR_TGT_THETA_N), EE_TOOL, USER_PARAM(USR_TGT_TOOL));
}


///////////////////////////////////////////////////////////////////////////
// Teleoperation
//
// See SECTION 7c in Config for why the pose still goes through the
// simulated axes rather than into the 2 ms interrupt.
///////////////////////////////////////////////////////////////////////////

// Track the PC's heartbeat. Called from the top-level SIG_IDLE so it runs
// in EVERY state, not just Teleop: the panel can then show whether the
// haptic link is alive before anyone asks to use it, and entering Teleop
// does not have to guess.
long TeleopFresh(void)
{
	long hb, now, age;

	hb  = USER_PARAM(USR_TELE_HEARTBEAT);
	now = Time();
	if (hb != g_tele_hb) {
		g_tele_hb    = hb;
		g_tele_hb_ms = now;
		USER_PARAM(USR_TELE_HB_SEEN) = hb;
	}

	age = now - g_tele_hb_ms;
	USER_PARAM(USR_TELE_HB_AGE) = age;

	// Only worth recording the worst gap while somebody is actually
	// streaming, or it fills up with the hours the PC was switched off.
	if (age < C_TELE_STALE_MS && age > USER_PARAM(USR_TELE_MAX_AGE)) {
		USER_PARAM(USR_TELE_MAX_AGE) = age;
	}

	return((age <= C_TELE_STALE_MS) ? TRUE : FALSE);
}


// Record both sides of the mapping as they are right now. See the note on
// g_tele_ref_* in 3R1T_Globals.mh - this is what makes taking control
// jump-free no matter what the PC is sending at the time.
void TeleopCapture(void)
{
	g_tele_ref_psi   = USER_PARAM(USR_TGT_PSI);
	g_tele_ref_phi   = USER_PARAM(USR_TGT_PHI);
	g_tele_ref_thn   = USER_PARAM(USR_TGT_THETA_N);
	g_tele_ref_tool  = USER_PARAM(USR_TGT_TOOL);

	g_tele_base_psi  = Cpos(EE_PSI);
	g_tele_base_phi  = Cpos(EE_PHI);
	g_tele_base_thn  = Cpos(EE_THETA_N);
	g_tele_base_tool = Cpos(EE_TOOL);

	g_tele_want_psi  = g_tele_base_psi;
	g_tele_want_phi  = g_tele_base_phi;
	g_tele_want_thn  = g_tele_base_thn;
	g_tele_want_tool = g_tele_base_tool;

	g_tele_next_ms   = 0;
}


// One retarget of the simulated axes towards where the operator's hand is.
void TeleopRetarget(void)
{
	long p, h, n, t, mask, moved, vRot, vTool, aRot, aTool;

	// The PC's demand, as a delta from what it was demanding when control
	// was taken, applied to where the pose was at that moment.
	p = g_tele_base_psi  + (USER_PARAM(USR_TGT_PSI)     - g_tele_ref_psi);
	h = g_tele_base_phi  + (USER_PARAM(USR_TGT_PHI)     - g_tele_ref_phi);
	n = g_tele_base_thn  + (USER_PARAM(USR_TGT_THETA_N) - g_tele_ref_thn);
	t = g_tele_base_tool + (USER_PARAM(USR_TGT_TOOL)    - g_tele_ref_tool);

	// >>> CLAMP HERE. DO NOT REFUSE. <<<
	//
	// PoseCommandOk refuses an out-of-range MOVE and says which axis, which
	// is right for a number somebody typed. Mid-stream a refusal is wrong:
	// the operator gets no explanation they can feel, because this machine
	// reflects no force to the haptic device, and a rig that silently stops
	// following looks broken. So the demand is clamped and the fact is
	// published for the PC to show on screen.
	//
	// The PC clamps too, against the same LIM_* figures parsed out of this
	// file. This is the backstop, not the fence.
	mask = 0;
	if (p >  LIM_PSI_CDEG)     { p =  LIM_PSI_CDEG;     mask = mask | 0x01; }
	if (p < -LIM_PSI_CDEG)     { p = -LIM_PSI_CDEG;     mask = mask | 0x01; }
	if (h >  LIM_PHI_CDEG)     { h =  LIM_PHI_CDEG;     mask = mask | 0x02; }
	if (h < -LIM_PHI_CDEG)     { h = -LIM_PHI_CDEG;     mask = mask | 0x02; }
	if (n >  LIM_THETA_N_CDEG) { n =  LIM_THETA_N_CDEG; mask = mask | 0x04; }
	if (n < -LIM_THETA_N_CDEG) { n = -LIM_THETA_N_CDEG; mask = mask | 0x04; }
	if (t >  LIM_TOOL_MM100)   { t =  LIM_TOOL_MM100;   mask = mask | 0x08; }
	if (t < -LIM_TOOL_MM100)   { t = -LIM_TOOL_MM100;   mask = mask | 0x08; }
	USER_PARAM(USR_TELE_CLAMPED) = mask;

	// Nothing has moved enough to be worth a new profile. Hand tremor and
	// the last digit of the device's own noise would otherwise re-kick the
	// profile generator on every single pass.
	moved = 0;
	if (AbsL(p - g_tele_want_psi)  > C_TELE_DEADBAND_CDEG) { moved = 1; }
	if (AbsL(h - g_tele_want_phi)  > C_TELE_DEADBAND_CDEG) { moved = 1; }
	if (AbsL(n - g_tele_want_thn)  > C_TELE_DEADBAND_CDEG) { moved = 1; }
	if (AbsL(t - g_tele_want_tool) > C_TELE_DEADBAND_TOOL) { moved = 1; }
	if (moved == 0) { return; }

	g_tele_want_psi  = p;
	g_tele_want_phi  = h;
	g_tele_want_thn  = n;
	g_tele_want_tool = t;

	// The follow speeds. Separate figures for the angles and the tool, for
	// the reason spelled out beside USR_MOVE_VEL: they are different units
	// on different drivetrains and must never share a number.
	vRot  = USER_PARAM(USR_TELE_VEL_ROT);
	vTool = USER_PARAM(USR_TELE_VEL_TOOL);
	if (vRot  < 1) { vRot  = 1; }
	if (vTool < 1) { vTool = 1; }

	aRot  = (USER_PARAM(USR_MOVE_ACC) * USER_PARAM(USR_TELE_ACC_SCALE)) / 100;
	aTool = (USER_PARAM(USR_TOOL_ACC) * USER_PARAM(USR_TELE_ACC_SCALE)) / 100;
	if (aRot  < 1) { aRot  = 1; }
	if (aTool < 1) { aTool = 1; }

	// >>> NO STAGE CEILING HERE EITHER. <<<
	// The 75 % cap went with the path scan on 2026-09-30. Teleop runs the
	// tool at exactly USR_TELE_VEL_TOOL, so that slot is now the only thing
	// bounding how fast a tilt can ask the carriage to pay cable out.
	SetVelAccDec(EE_PSI,     vRot,  aRot,  aRot);
	SetVelAccDec(EE_PHI,     vRot,  aRot,  aRot);
	SetVelAccDec(EE_THETA_N, vRot,  aRot,  aRot);
	SetVelAccDec(EE_TOOL,    vTool, aTool, aTool);

	AxisLinAbsStart(EE_PSI, p, EE_PHI, h, EE_THETA_N, n, EE_TOOL, t);
}


///////////////////////////////////////////////////////////////////////////
// TEMPORARY: the tool kinematics trace.
//
// Prints the quantities the reference notebook prints, so the rig can be
// checked against it directly instead of by inference, plus what the MSQS
// encoder actually did - which is the only line that says whether the
// model is right rather than merely plausible.
//
// With tool = 0 and the platform tilting, "compensation" is how much cable
// the tilt demands and "encoder" should stay put. If the encoder instead
// follows the compensation, the stage is moving when it should be holding.
//
// Turn g_tool_debug off in 3R1T_Globals.mh once the model is trusted.
///////////////////////////////////////////////////////////////////////////
void PrintToolDebug(void)
{
	print("1T  pose psi=",psi_deg," phi=",phi_deg," thn=",User_theta_n_deg," deg (offset included)");
#if (KIN_1T_MODEL == KIN_1T_COUPLED)
	print("1T  model COUPLED   span L=",cable_L," home=",g_L_home," -> compensation ",tool_corr_mm," mm");
#else
	print("1T  model PASSTHROUGH - TILT NOT COMPENSATED. span and compensation below are not computed.");
	print("1T  span L=",cable_L," home=",g_L_home," -> compensation ",tool_corr_mm," mm");
#endif
	print("1T  tool cmd ",tool_cmd_mm," + comp -> total ",tool_total_mm," mm");
	print("1T  drum ",tool_drum_turns," turns -> motor ",tool_motor_turns," turns");
	// The same move in the notebook's convention, so the rig and the
	// notebook can be compared number for number. These are exact
	// negatives of the line above, by construction.
	print("1T  notebook: total cable ",nb_total_cable_mm," mm -> drum ",nb_drum_turns," -> motor ",nb_motor_turns," turns");
	print("1T  axis setpoint ",USER_PARAM(USR_AXT_UU)," UU    encoder reads ",USER_PARAM(USR_ENC_UM)," um");
}


///////////////////////////////////////////////////////////////////////////
// Stage homing helpers
///////////////////////////////////////////////////////////////////////////
// There are no limit switches, so a STALL is the end stop: motor
// commanded to move, encoder not following.
// Stalled() lived here: the old end-stop detector, which judged a stop
// from the LINEAR ENCODER VELOCITY alone and so could not tell the
// carriage wound up against the end of travel from the drive not running
// at all. Phase A of stage homing now requires the motor to be turning
// as well - see Home1TSlack.

void StartSearch(long dir)
{
	sdkStartContinuousMove(C_AXIS_1T, dir * H_SEARCH_VEL, H_ACC);
	g_h_moveT0  = Time();
	g_h_stallT0 = Time();
}

// Re-arm the crossing detector so the edge just handled is not seen again
// on the next leg.
void RebaseRef(void)
{
	g_h_passes0 = RefPasses();
	g_h_fall0   = USER_PARAM(USR_REF_FALL_UM);
	g_h_rise0   = USER_PARAM(USR_REF_RISE_UM);
}

void StopEverything(void)
{
	sdkStopContinuousMove(C_AXIS_1T, H_DEC);
	AxisStop(C_ARM1_AXIS, C_ARM2_AXIS, C_ARM3_AXIS);
	AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
}


///////////////////////////////////////////////////////////////////////////
// Settling waits, used either side of a mode switch.
//
// Taking a drive OUT of USERREFPOS while it is still following a moving
// setpoint stops it dead rather than ramping it, and putting one IN while
// it still has speed on freezes it just as hard. Neither is fatal at
// these speeds, but both are avoidable by waiting a moment, and a track
// error puts the rig in red for no good reason.
//
// Blocking with Delay in SIG_ENTRY is the established pattern here - it
// is what the drive enable check does. The bound means a sensor that
// never settles cannot hang the sequence.
///////////////////////////////////////////////////////////////////////////
// Counted one axis per line rather than as one && chain: the ApossC
// preprocessor expands macros line by line, and AXE_PROCESS on a
// continuation line does not survive it.
void WaitPoseStopped(long ms)
{
	long t0, n;

	t0 = Time();
	while ((Time() - t0) < ms) {
		n = 0;
		if (AXE_PROCESS(EE_PSI,     PFG_AKTSTATE) == PGS_POSCTRL) { n = n + 1; }
		if (AXE_PROCESS(EE_PHI,     PFG_AKTSTATE) == PGS_POSCTRL) { n = n + 1; }
		if (AXE_PROCESS(EE_THETA_N, PFG_AKTSTATE) == PGS_POSCTRL) { n = n + 1; }
		if (AXE_PROCESS(EE_TOOL,    PFG_AKTSTATE) == PGS_POSCTRL) { n = n + 1; }
		if (n == 4) { return; }
		Delay(10);
	}
	print("Pose axes did not come to rest within ",ms," ms - continuing anyway");
}

// The stage, after a homing search. CanPoll runs in the parent SIG_IDLE
// and not while this blocks, so the encoder velocity here is the last
// frame received - which is why the axis profile state is checked too.
void WaitStageStopped(long ms)
{
	long t0;

	t0 = Time();
	while ((Time() - t0) < ms) {
		if (AXE_PROCESS(C_AXIS_1T, PFG_AKTSTATE) == PGS_POSCTRL) { return; }
		Delay(10);
	}
	print("Stage did not come to rest within ",ms," ms - continuing anyway");
}


///////////////////////////////////////////////////////////////////////////
// Trajectory playback
///////////////////////////////////////////////////////////////////////////
// TRUE once the profile generator has started slowing towards the current
// waypoint. That is the moment to queue the next one: issuing the new
// target while the axes are still moving makes them curve through the
// point instead of stopping on it.
long DecelStarted(void)
{
	if (AXE_PROCESS(EE_PSI,     PFG_AKTSTATE) == PGS_TPZDEC) { return(TRUE); }
	if (AXE_PROCESS(EE_PHI,     PFG_AKTSTATE) == PGS_TPZDEC) { return(TRUE); }
	if (AXE_PROCESS(EE_THETA_N, PFG_AKTSTATE) == PGS_TPZDEC) { return(TRUE); }
	if (AXE_PROCESS(EE_TOOL,    PFG_AKTSTATE) == PGS_TPZDEC) { return(TRUE); }
	return(FALSE);
}


// Copy the next row into the four command slots and step the read pointer.
// FALSE means the list ended - a STEP of zero is the terminator, and so is
// running off the end of the array.
//
// >>> AND PICK UP THE PAUSE FLAG THE ROW CARRIES. <<<
//
// Nothing here works out whether this waypoint ought to be a full stop.
// The file says so, in its own column, and the generator that wrote the
// file is what guarantees a rotation and a translation never overlap.
//
// The inference that used to live here is gone, and it is worth saying why
// so that nobody puts it back. It could only ever see the two rows either
// side of a boundary, so a file had no way to ask for a pause anywhere
// else; and what it produced was a suppressed blend rather than a hold, so
// the rig was released the instant the profile generators went quiet,
// which is not the same as the rig having come to rest.
//
// >>> ANY NON-ZERO IS A PAUSE. <<<
//
// The column is written by the generator and it writes 1, but a flag that
// read back as "flow on" because it was not exactly 1 would be a fault
// that only ever shows up on the rig, at speed, in the middle of a leg.
long LoadNextTrajPoint(void)
{
	long step;

	g_traj_pause_req = FALSE;
	g_traj_pausing   = FALSE;
	g_traj_pause_t0  = 0;
	g_traj_load_t0   = Time();
	USER_PARAM(USR_TRAJ_PAUSE) = 0;

	if (g_traj_row >= C_SEQUENCE_SIZE) { return(FALSE); }

	step = MoveSequenceData[g_traj_row][TRAJ_STEP];
	if (step == 0) { return(FALSE); }

	if (MoveSequenceData[g_traj_row][TRAJ_PAUSE] != 0) { g_traj_pause_req = TRUE; }

	USER_PARAM(USR_TGT_PSI)     = MoveSequenceData[g_traj_row][TRAJ_PSI];
	USER_PARAM(USR_TGT_PHI)     = MoveSequenceData[g_traj_row][TRAJ_PHI];
	USER_PARAM(USR_TGT_THETA_N) = MoveSequenceData[g_traj_row][TRAJ_THETA_N];
	USER_PARAM(USR_TGT_TOOL)    = MoveSequenceData[g_traj_row][TRAJ_TOOL];
	USER_PARAM(USR_TRAJ_STEP)   = step;
	USER_PARAM(USR_TRAJ_PAUSE)  = g_traj_pause_req;

	g_traj_row = g_traj_row + 1;
	return(TRUE);
}

// A file is loaded if its first row carries a step number.
long TrajLoaded(void)
{
	if (MoveSequenceData[0][TRAJ_STEP] == 0) { return(FALSE); }
	return(TRUE);
}

// Read the header the download wrote alongside the rows.
void TrajReadHeader(void)
{
	USER_PARAM(USR_TRAJ_LEN) = GeneralData[GD_SEQ_LENGTH];
	USER_PARAM(USR_TRAJ_NUM) = GeneralData[GD_SEQ_NUMBER];
}


// Put the panel's speeds aside and take the file's, where it has any.
//
// The four values are applied INDEPENDENTLY, because a file is allowed to
// pick a rotation profile and say nothing about the translation one. Zero
// means "say nothing", which is what every file written before the header
// had room for these carries - so this is a no-op on all of them.
//
// A file may not ask for more than C_TRAJ_*_MAX. Past that it is clamped
// and reported rather than refused: a clamped waypoint would be a
// different trajectory, but a clamped speed is the same trajectory run
// slower, and StartPoseMove already slows the whole move on its own
// whenever the cable would outrun the stage.
void TrajApplySpeeds(void)
{
	long v, a, any;

	// >>> A FILE THAT SAYS NOTHING MUST CHANGE NOTHING. <<<
	//
	// Saving unconditionally would make SIG_EXIT write the panel's slots
	// back on EVERY run, including the files that carry zeros - so a value
	// changed over SDO while a trajectory was running would be silently
	// reverted when it ended, which is not what happened before these
	// slots existed. Nothing is put aside unless something is going to be
	// put on top of it.
	any = 0;
	if (GeneralData[GD_ROT_VEL]  > 0) { any = 1; }
	if (GeneralData[GD_ROT_ACC]  > 0) { any = 1; }
	if (GeneralData[GD_TOOL_VEL] > 0) { any = 1; }
	if (GeneralData[GD_TOOL_ACC] > 0) { any = 1; }
	if (any == 0) { return; }

	g_traj_sv_rot_vel   = USER_PARAM(USR_MOVE_VEL);
	g_traj_sv_rot_acc   = USER_PARAM(USR_MOVE_ACC);
	g_traj_sv_tool_vel  = USER_PARAM(USR_TOOL_VEL);
	g_traj_sv_tool_acc  = USER_PARAM(USR_TOOL_ACC);
	g_traj_speeds_saved = TRUE;

	v = GeneralData[GD_ROT_VEL];
	if (v > C_TRAJ_VEL_ROT_MAX) {
		print("TRAJ: file asks ",v," cdeg/s of rotation, over the ",C_TRAJ_VEL_ROT_MAX," a file may ask for - clamped");
		v = C_TRAJ_VEL_ROT_MAX;
	}
	if (v > 0) { USER_PARAM(USR_MOVE_VEL) = v; }

	a = GeneralData[GD_ROT_ACC];
	if (a > C_TRAJ_ACC_ROT_MAX) {
		print("TRAJ: file asks ",a," cdeg/s^2 of rotation accel, over the ",C_TRAJ_ACC_ROT_MAX," a file may ask for - clamped");
		a = C_TRAJ_ACC_ROT_MAX;
	}
	if (a > 0) { USER_PARAM(USR_MOVE_ACC) = a; }

	v = GeneralData[GD_TOOL_VEL];
	if (v > C_TRAJ_VEL_TOOL_MAX) {
		print("TRAJ: file asks ",v," (0.01 mm/s) of translation, over the ",C_TRAJ_VEL_TOOL_MAX," a file may ask for - clamped");
		v = C_TRAJ_VEL_TOOL_MAX;
	}
	if (v > 0) { USER_PARAM(USR_TOOL_VEL) = v; }

	a = GeneralData[GD_TOOL_ACC];
	if (a > C_TRAJ_ACC_TOOL_MAX) {
		print("TRAJ: file asks ",a," (0.01 mm/s^2) of translation accel, over the ",C_TRAJ_ACC_TOOL_MAX," a file may ask for - clamped");
		a = C_TRAJ_ACC_TOOL_MAX;
	}
	if (a > 0) { USER_PARAM(USR_TOOL_ACC) = a; }

	// Reported whenever ANY of the four was applied. Gating this on the
	// velocities alone would let a hand-edited file change the whole run's
	// acceleration and leave nothing in the terminal to say so.
	print("TRAJECTORY: speeds from the file - rotation ",USER_PARAM(USR_MOVE_VEL),"/",USER_PARAM(USR_MOVE_ACC)," cdeg/s, translation ",USER_PARAM(USR_TOOL_VEL),"/",USER_PARAM(USR_TOOL_ACC)," (0.01 mm/s)");
}


// Give the panel its speeds back. Called from SIG_EXIT, so it runs on
// every way out of a trajectory - finished, stopped, HOME pressed, or a
// waypoint outside the limits.
void TrajRestoreSpeeds(void)
{
	if (g_traj_speeds_saved == FALSE) { return; }

	USER_PARAM(USR_MOVE_VEL) = g_traj_sv_rot_vel;
	USER_PARAM(USR_MOVE_ACC) = g_traj_sv_rot_acc;
	USER_PARAM(USR_TOOL_VEL) = g_traj_sv_tool_vel;
	USER_PARAM(USR_TOOL_ACC) = g_traj_sv_tool_acc;
	g_traj_speeds_saved = FALSE;
}

// Queue the next waypoint and start it.
//
//    1  a new waypoint is now running
//    0  the list ended - the last point is being allowed to finish
//   -1  the waypoint is outside the limits, stop the run
long TrajAdvance(void)
{
	if (LoadNextTrajPoint() == TRUE) {
		if (PoseCommandOk() == FALSE) { return(-1); }
		StartPoseMove();
		return(1);
	}

	if (g_traj_mode == TRJ_CONTINUOUS) {
		// Wrap. A generated file starts and ends at the same pose, so the
		// seam is just another waypoint.
		USER_PARAM(USR_TRAJ_LAPS) = USER_PARAM(USR_TRAJ_LAPS) + 1;
		g_traj_row = 0;
		if (LoadNextTrajPoint() == FALSE) { return(0); }
		StartPoseMove();
		return(1);
	}

	g_traj_last = TRUE;
	return(0);
}


// What actually arrived in the arrays. The download reports nothing back,
// so this is the only way to tell a file that did not land from one that
// landed somewhere unexpected. All zeros means it never reached array 1 -
// check the dim declaration order in 3R1T_Globals.mh.
//
// The pause column is printed for a second reason: a file generated before
// that column existed has a reserve zero sitting where the flag now lives,
// so it reads as "flow on" the whole way through and every boundary in it
// would run blended and silently. A dump whose pause figures are all zero
// on a file that should have them is a stale .zbc, not a player fault.
void TrajDump(void)
{
	print("TRAJ loaded: ",GeneralData[GD_SEQ_LENGTH]," points, number ",GeneralData[GD_SEQ_NUMBER]);
	print("TRAJ row 1: psi=",MoveSequenceData[0][TRAJ_PSI]," phi=",MoveSequenceData[0][TRAJ_PHI]," thn=",MoveSequenceData[0][TRAJ_THETA_N]," tool=",MoveSequenceData[0][TRAJ_TOOL]," pause=",MoveSequenceData[0][TRAJ_PAUSE]);
	print("TRAJ row 2: psi=",MoveSequenceData[1][TRAJ_PSI]," phi=",MoveSequenceData[1][TRAJ_PHI]," thn=",MoveSequenceData[1][TRAJ_THETA_N]," tool=",MoveSequenceData[1][TRAJ_TOOL]," pause=",MoveSequenceData[1][TRAJ_PAUSE]);
}


///////////////////////////////////////////////////////////////////////////
// State machine
///////////////////////////////////////////////////////////////////////////
SmState MainMachine {

	SIG_INIT = {
		long i;

		print("3R1T (id ",id,") - homing + kinematic control");
		if (g_verbose) print(" pose axes are SIMULATED: psi=",EE_PSI," phi=",EE_PHI," theta_n=",EE_THETA_N," tool=",EE_TOOL);
		if (g_verbose) print(" real drives: arms ",C_AXIS1,"/",C_AXIS2,"/",C_AXIS3," stage ",C_AXIS_1T);
		if (g_verbose) print(" sensor frame is ARM-ORDERED: theta1=arm1, theta2=arm2, theta3=arm3");
		if (g_verbose) print(" HOME_DIR arm1=",HOME_DIR_ARM1," arm2=",HOME_DIR_ARM2," arm3=",HOME_DIR_ARM3," (flip one if an arm homes the wrong way)");
		print(" stage limit ",C_TOOL_MAX_UU_S," UU/s, from ",C_AXIS_1T_MAX_RPM," rpm through ",C_AXIS_1T_POSFACT_Z,":",C_AXIS_1T_POSFACT_N," gearing - VERIFY that ratio");

		CanArm();
		USER_PARAM(USR_CAN_BAUD) = GLB_PARAM(CANBAUD);
		if (g_verbose) print(" GLB_PARAM(CANBAUD) = ",GLB_PARAM(CANBAUD)," (must be ",C_CAN_BAUDRATE,")");

		USER_PARAM(USR_COMMAND)          = 0;
		USER_PARAM(USR_STATE)            = 0;
		USER_PARAM(USR_LED)              = LED_AMBER;
		USER_PARAM(USR_MSG)              = MSG_BOOT;

		USER_PARAM(USR_TGT_PSI)          = 0;
		USER_PARAM(USR_TGT_PHI)          = 0;
		USER_PARAM(USR_TGT_THETA_N)      = 0;
		USER_PARAM(USR_TGT_TOOL)         = 0;

		// Teleop. The tunables get defaults here; the PC may overwrite them
		// at any time. The enable starts CLEAR so a stale 1 left in the
		// slot from a previous session cannot hand control straight over.
		USER_PARAM(USR_TELE_HEARTBEAT)   = 0;
		USER_PARAM(USR_TELE_ENABLE)      = 0;
		USER_PARAM(USR_TELE_VEL_ROT)     = C_TELE_VEL_ROT_DEF;
		USER_PARAM(USR_TELE_VEL_TOOL)    = C_TELE_VEL_TOOL_DEF;
		USER_PARAM(USR_TELE_ACC_SCALE)   = C_TELE_ACC_PCT_DEF;
		USER_PARAM(USR_TELE_TREMOR)      = C_TELE_TREMOR_DEF;
		USER_PARAM(USR_TELE_STATE)       = TELE_OFF;
		USER_PARAM(USR_TELE_HB_SEEN)     = 0;
		USER_PARAM(USR_TELE_HB_AGE)      = 0;
		USER_PARAM(USR_TELE_MAX_AGE)     = 0;
		USER_PARAM(USR_TELE_TRIPS)       = 0;
		USER_PARAM(USR_TELE_CLAMPED)     = 0;

		USER_PARAM(USR_TRAJ_STATE)       = TRJ_IDLE;
		USER_PARAM(USR_TRAJ_STEP)        = 0;
		USER_PARAM(USR_TRAJ_LEN)         = 0;
		USER_PARAM(USR_TRAJ_NUM)         = 0;
		USER_PARAM(USR_TRAJ_LAPS)        = 0;
		USER_PARAM(USR_TRAJ_PAUSE)       = 0;

		USER_PARAM(USR_HOME_STATE)       = H_IDLE;
		USER_PARAM(USR_HOME_PASS)        = 0;
		USER_PARAM(USR_ENC_ZERO_UM)      = 0;
		USER_PARAM(USR_HOME_EDGE_UM)     = 0;

		USER_PARAM(USR_IK_SINGULAR)      = 0;
		USER_PARAM(USR_IK_SINGULAR_COUNT)= 0;
		USER_PARAM(USR_IK_LINK_SURPASS)  = 0;
		USER_PARAM(USR_IK_RESET_CONT)    = 0;

		USER_PARAM(USR_MOVE_VEL)         = 1000;	// [cdeg/s]  10 deg/s
		USER_PARAM(USR_MOVE_ACC)         = 8000;	// [cdeg/s^2]
		// The stage now runs the SAME MOTOR PROFILE as the three arms, which
		// is the only profile on this rig with a track record.
		//
		//   arms   1000 cdeg/s, 8000 cdeg/s^2  ->  530 rpm, 4240 rpm/s
		//   stage   870 UU/s,   7000 UU/s^2    ->  530 rpm, 4240 rpm/s
		//
		// The two look nothing alike because the units and drivetrains are
		// nothing alike - which is exactly why they must not share a number.
		// At the drum that works out at 8.7 mm/s. Lower it if the tool
		// wants to go in more gently; the motor has plenty in hand either
		// way, since 530 rpm is well under the 7000 it can do.
		USER_PARAM(USR_TOOL_VEL)         = 1000;	// [0.01 mm/s]    10 mm/s
		USER_PARAM(USR_TOOL_ACC)         = 8000;	// [0.01 mm/s^2]  80 mm/s^2
		USER_PARAM(USR_HOME_VEL)         = 1000;	// [cdeg/s]  10 deg/s
		// >>> ONE FIGURE FOR ALL FOUR DRIVES, AND IT IS 68% OF NOMINAL. <<<
		//
		// The history, because the number is only defensible with it:
		//
		//   1000 arms / 500 stage   arm 1 tripped a following error 105 ms
		//                           after hitting 1000; the stage sat on
		//                           its 500 constantly. Both too tight.
		//   1200 all four           held +/-19 deg. Still tripped arm 1,
		//                           but at that point arm 1's drive was
		//                           commissioned for a different motor -
		//                           1490 mA nominal against the others'
		//                           2660 - so 1200 was 81% of ITS rating
		//                           and there was nothing to give.
		//   1800 all four           <- here. With the four drives matched
		//                           at 2660 mA nominal, 1200 held 20 deg
		//                           and let go at 24.74 with bit 11,
		//                           internal limit, set at the instant.
		//
		// The gravity moment goes roughly as sin(tilt), so 20 -> 25 deg
		// wants about 1.24x the torque and 20 -> 30 about 1.46x. 1800 mA
		// clears 25 comfortably and leaves margin at 30.
		//
		// >>> AND IT STAYS BELOW NOMINAL ON PURPOSE. <<<
		//
		// 1800 is 68% of the 2660 mA continuous rating, so the drive's
		// thermal model never engages and the 1 s winding time constant
		// stays irrelevant. Push this past 2660 and the drives start
		// derating themselves after a second or two of real work, which
		// looks exactly like raising the limit and nothing happening.
		USER_PARAM(USR_CURRENT_LIMIT)    = 2660;	// [mA] all four drives
		USER_PARAM(USR_CAN_RESET)        = 0;
		USER_PARAM(USR_ERROR_NO)         = 0;
		USER_PARAM(USR_ERROR_INFO)       = 0;
		USER_PARAM(USR_FAULT_COUNT)      = 0;
		USER_PARAM(USR_FAULT_AXIS)       = 0;
		USER_PARAM(USR_FAULT_NODE)       = 0;
		USER_PARAM(USR_FAULT_EPOS)       = 0;
		USER_PARAM(USR_FAULT_SW)         = 0;
		USER_PARAM(USR_FAULT_STATE)      = 0;
		USER_PARAM(USR_FAULT_STEP)       = 0;
		USER_PARAM(USR_FAULT_MS)         = 0;

		ErrorClear();
		AmpErrorClear(C_AXIS1, C_AXIS2, C_AXIS3, C_AXIS_1T);

		// ---- the commanded-pose axes -------------------------------
		// Always simulated. They exist only so the ISR has something
		// smooth to read.
		sdkSetupAxisSimulation(EE_PSI);
		sdkSetupAxisSimulation(EE_PHI);
		sdkSetupAxisSimulation(EE_THETA_N);
		sdkSetupAxisSimulation(EE_TOOL);
		DoSettingsUserUnits(EE_PSI);
		DoSettingsUserUnits(EE_PHI);
		DoSettingsUserUnits(EE_THETA_N);
		DoSettingsUserUnits(EE_TOOL);
		AXE_PARAM(EE_PSI,     ERRCOND) = 5;		// no "motor off" after an error
		AXE_PARAM(EE_PHI,     ERRCOND) = 5;
		AXE_PARAM(EE_THETA_N, ERRCOND) = 5;
		AXE_PARAM(EE_TOOL,    ERRCOND) = 5;
		DefOrigin(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
		AxisControl(EE_PSI, ON, EE_PHI, ON, EE_THETA_N, ON, EE_TOOL, ON);

		// ---- the real drives ---------------------------------------
#if (AXES_MODE == SIM_MODE)
		print(" *** SIM_MODE - THE MOTORS WILL NOT POWER UP ***");
		sdkSetupAxisSimulation(C_AXIS1);
		sdkSetupAxisSimulation(C_AXIS2);
		sdkSetupAxisSimulation(C_AXIS3);
		sdkSetupAxisSimulation(C_AXIS_1T);
		DoSettingsUserUnits(C_AXIS1);
		DoSettingsUserUnits(C_AXIS2);
		DoSettingsUserUnits(C_AXIS3);
		DoSettingsUserUnits(C_AXIS_1T);
#else
		if (GLB_PARAM(CANBAUD) != C_CAN_BAUDRATE) {
			print("Set new Baudrate and save global parameters");
			GLB_PARAM(CANBAUD) = C_CAN_BAUDRATE;
			CanOpenRestart();
			Save(GLBPARS);
		}
		// Must be set before sdkEpos4_SetupCanSdoParam().
		GLB_PARAM(CANSYNCTIMER) = 2;
		SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 0;		// all slaves PREOPERATIONAL

		for (i = 0; i < 3; i++) {
			sdkEpos4_SetupCanSdoParam(i + 1, C_PDO_NUMBER, C_AXISPOLARITY, EPOS4_OP_CSP);
			sdkEpos4_SetupCanBusModule(i, i + 1, C_PDO_NUMBER, EPOS4_OP_CSP);
			sdkEpos4_SetupCanVirtAmp(i, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
			sdkEpos4_SetupCanVirtCntin(i, EPOS4_OP_CSP);
			sdkSetupAxisMovementParam(i, C_AXIS_VELRES, C_AXIS_MAX_RPM,
			                          C_AXIS_RAMPTYPE, C_AXIS_RAMPMIN,
			                          C_AXIS_JERKMIN, 0);
			sdkSetupAxisUserUnits(i, C_AXIS_POSENCREV, C_AXIS_POSENCQC,
			                      C_AXIS_POSFACT_Z, C_AXIS_POSFACT_N,
			                      C_AXIS_FEEDREV, C_AXIS_FEEDDIST);
			AXE_PARAM(i, POSERR) = C_AXIS_TRACKERR;
		}

		sdkEpos4_SetupCanSdoParam(C_DRIVE_BUSID_1T, C_PDO_NUMBER, C_AXISPOLARITY, EPOS4_OP_CSP);
		sdkEpos4_SetupCanBusModule(C_AXIS_1T, C_DRIVE_BUSID_1T, C_PDO_NUMBER, EPOS4_OP_CSP);
		sdkEpos4_SetupCanVirtAmp(C_AXIS_1T, C_AXIS_1T_MAX_RPM, EPOS4_OP_CSP);
		sdkEpos4_SetupCanVirtCntin(C_AXIS_1T, EPOS4_OP_CSP);
		sdkSetupAxisMovementParam(C_AXIS_1T, C_AXIS_VELRES, C_AXIS_1T_MAX_RPM,
		                          C_AXIS_RAMPTYPE, C_AXIS_RAMPMIN,
		                          C_AXIS_JERKMIN, 0);
		sdkSetupAxisUserUnits(C_AXIS_1T, C_AXIS_1T_POSENCREV, C_AXIS_1T_POSENCQC,
		                      C_AXIS_1T_POSFACT_Z, C_AXIS_1T_POSFACT_N,
		                      C_AXIS_1T_FEEDREV, C_AXIS_1T_FEEDDIST);
		AXE_PARAM(C_AXIS_1T, POSERR) = C_AXIS_1T_TRACKERR;

		SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 1;		// all slaves OPERATIONAL
#endif

		// The cable model needs its home span before anything can move,
		// and it is computed rather than written down so that editing the
		// geometry cannot leave a stale number behind.
		InitToolKinematics();
		ReportCableBudget();

		SmSubscribe(id, SIG_ERROR);
		SmPeriod(1000, id, SIG_TOGGLE);

		// The axes exist now, so the kinematics interrupt may run.
		g_isr_ready = 1;

		return(SmTrans(->Startup));
	}


	SIG_TOGGLE = {
		if ((USER_PARAM(USR_STATE) & C_STA_TOGGLE) == C_STA_TOGGLE) {
			StaClr(C_STA_TOGGLE);
		} else {
			StaSet(C_STA_TOGGLE);
		}
		// TEMPORARY - see PrintToolDebug. One line per print: the ApossC
		// preprocessor expands macros line by line, so USER_PARAM() on a
		// continuation line will not compile.
		// Nothing on the panel reports a download, so say it here - once,
		// when a new file arrives, not every second.
		TrajReadHeader();

		if (USER_PARAM(USR_TRAJ_NUM) != g_traj_num_seen) {
			g_traj_num_seen = USER_PARAM(USR_TRAJ_NUM);
			print("TRAJ loaded: ",USER_PARAM(USR_TRAJ_LEN)," points, number ",USER_PARAM(USR_TRAJ_NUM));
		}
		if (g_tool_debug) { PrintToolDebug(); }

		// A drive that is about to fault usually sets its warning bit
		// first. Nothing else in this program would ever mention it.
		// Turn it off with C_DRIVE_HEALTH_POLL if the bus is the suspect.
		PollDriveHealth();
	}


	SIG_IDLE = {
		CanPoll();
		// Tracked in every state, not just Teleop, so the panel can show
		// whether the haptic PC is alive before anyone asks to use it.
		TeleopFresh();
		USER_PARAM(USR_POSE_PSI)     = Cpos(EE_PSI);
		USER_PARAM(USR_POSE_PHI)     = Cpos(EE_PHI);
		USER_PARAM(USR_POSE_THETA_N) = Cpos(EE_THETA_N);
		USER_PARAM(USR_POSE_TOOL)    = Cpos(EE_TOOL);
		USER_PARAM(USR_ERROR_NO)   = ErrorNo();
		USER_PARAM(USR_ERROR_INFO) = ErrorInfo();

		// Warnings announce themselves once per event. Posted every pass
		// they would fill all four message lines within a millisecond and
		// wipe out the history they are supposed to sit alongside.
		if (USER_PARAM(USR_THETA_STALE) == TRUE || USER_PARAM(USR_ENC_STALE) == TRUE) {
			if (g_warned_stale == 0) {
				g_warned_stale = 1;
				Say(MSG_WARN_STALE);
				print("WARNING: sensor CAN stale - theta=",USER_PARAM(USR_THETA_STALE)," enc=",USER_PARAM(USR_ENC_STALE)," - is the sensor node running?");
			}
		} else {
			g_warned_stale = 0;
		}

		if (USER_PARAM(USR_IK_SINGULAR) == 1) {
			if (g_warned_sing == 0) {
				g_warned_sing = 1;
				Say(MSG_WARN_SING);
				print("WARNING: IK has no real solution - pose held. Count=",USER_PARAM(USR_IK_SINGULAR_COUNT));
			}
		} else {
			g_warned_sing = 0;
		}

		if (USER_PARAM(USR_IK_LINK_SURPASS) == 1) {
			if (g_warned_surpass == 0) {
				g_warned_surpass = 1;
				Say(MSG_WARN_SURPASS);
				print("WARNING: two legs are more than a full turn apart - a proximal link may surpass its neighbour");
			}
		} else {
			g_warned_surpass = 0;
		}

		return(SmNotHandled);
	}


	///////////////////////////////////////////////////////////////////////
	// >>> THE SNAPSHOT HAPPENS HERE, BEFORE ANYTHING ELSE RUNS. <<<
	//
	// ErrorNo(), ErrorAxis() and ErrorInfo() report the LAST error, so any
	// command issued between the fault and reading them can overwrite
	// them - including the AxisControl(OFF) that Fault does on its way in.
	// That is why the capture is here and not in Fault's SIG_ENTRY, where
	// it used to be.
	//
	// CaptureFault touches no CAN. The drives are interrogated in Fault,
	// after they have been made safe.
	///////////////////////////////////////////////////////////////////////
	SIG_ERROR = {
		CaptureFault();
		print(" !!!!! ERROR ",g_errNo," on axis ",g_errAxis," at ",g_fault_ms," ms - fault ",g_fault_count," this power-up");

		// A fault raised while ALREADY handling one must not transition
		// again: Fault's SIG_ENTRY turns drives off and talks to them over
		// CAN, and if the bus is the problem either can fail. Re-entering
		// would re-run all of it and fault again, for ever.
		if (g_in_fault == TRUE) {
			print(" (already in Fault - not re-entering. Second error was ",g_errNo,")");
			return(SmNotHandled);
		}
		return(SmTrans(->Fault));
	}


	///////////////////////////////////////////////////////////////////////
	// Startup - everything energises here. There are no power buttons.
	///////////////////////////////////////////////////////////////////////
	SmState Startup {
		SIG_ENTRY = {
			if (g_verbose) print("3R1T -> Startup");
			g_state_id = ST_STARTUP;
			SetLed(LED_AMBER);
			Say(MSG_CFG_DRIVES);
			Say(MSG_PWR_ON);

			// >>> BEFORE AxisControl(ON), NOT AFTER. <<<
			//
			// A program reload leaves the drives faulted, and enabling
			// them in that state makes the controller raise error 40 and
			// fail the whole startup - which is why the first execute
			// after a reload used to fail and the second one worked. See
			// ClearLeftoverDriveFaults.
			ClearLeftoverDriveFaults();

			// Outside the #if on purpose: simulated axes need enabling
			// too, or nothing moves in SIM_MODE either.
			AxisControl(C_AXIS1, ON, C_AXIS2, ON, C_AXIS3, ON, C_AXIS_1T, ON);

#if (AXES_MODE == SIM_MODE)
			Say(MSG_SIM);
			g_startup_ok = TRUE;
#else
			// AxisControl(ON) only starts the DS402 handshake, so give the
			// slaves time to get to OPERATIONAL before reading them back.
			Delay(1000);
			g_startup_ok = CheckDrivesEnabled();
			ApplyCurrentLimits();
#endif

			// Anything pressed while the rig was booting is thrown away,
			// so a stale press cannot fire the moment the drives wake up.
			USER_PARAM(USR_COMMAND) = 0;
		}

		SIG_IDLE = {
			if (g_startup_ok == TRUE) {
				Say(MSG_DRV_OK);
				return(SmTrans(NotHomed));
			}
			Say(MSG_DRV_FAIL);
			return(SmTrans(Fault));
		}
	}


	///////////////////////////////////////////////////////////////////////
	// Fault - drives off until the operator clears it.
	//
	// HOME is the way out. There is no separate clear button, because
	// there was never anything useful to do after clearing except home
	// again: whatever the fault was, the axes may have moved while it was
	// being cleared, so the datum cannot be trusted either way.
	///////////////////////////////////////////////////////////////////////
	SmState Fault {
		SIG_ENTRY = {
			g_in_fault = TRUE;
			g_state_id = ST_FAULT;
			SetLed(LED_RED);
			Say(MSG_ERROR);
			StaSet(C_STA_ERROR);
			StaClr(C_STA_READY | C_STA_MOVING | C_STA_HOMED | C_STA_STREAM);

			// >>> SAFE FIRST, DIAGNOSE SECOND. <<<
			//
			// The numbers that matter were latched by CaptureFault() in
			// SIG_ERROR, so nothing is lost by making the rig safe before
			// reading anything. The drive's own 0x603F and 0x1003 persist
			// until a fault reset, so they read the same after the drives
			// are disabled as before - and the report below is a few dozen
			// CAN transactions, which is not something to leave three
			// working drives energised through.
			g_ik_armed = 0;
			AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
			AxisControl(C_AXIS1, OFF, C_AXIS2, OFF, C_AXIS3, OFF, C_AXIS_1T, OFF);

			// Which drive is actually complaining, into the SDO slots, so
			// the cause is readable without the terminal.
			FindFaultingDrive();
			ReportFault();
		}

		SIG_IDLE = {
			long cmd;

			cmd = USER_PARAM(USR_COMMAND);
			if (cmd == 0) { return(SmNotHandled); }
			USER_PARAM(USR_COMMAND) = 0;

			// HOME clears the fault and re-homes in one press. C_CMD_ERROR_CLR
			// still works for anyone driving the slots over SDO, but it only
			// clears - it cannot leave the rig usable on its own, because an
			// un-homed rig will not move anyway.
			if (cmd == C_CMD_HOME || cmd == C_CMD_ERROR_CLR) {
				print("Fault: clearing");
				ErrorClear();
				AmpErrorClear(C_AXIS1, C_AXIS2, C_AXIS3, C_AXIS_1T);
				USER_PARAM(USR_ERROR_NO)   = 0;
				USER_PARAM(USR_ERROR_INFO) = 0;
				AxisControl(C_AXIS1, ON, C_AXIS2, ON, C_AXIS3, ON, C_AXIS_1T, ON);
				Delay(500);
				Say(MSG_ERR_CLR);
				// Always via NotHomed. Whatever the fault was, the axes may
				// have moved while it was being cleared, so the datum is no
				// longer trustworthy - NotHomed then picks up the HOME press
				// and runs the sequence.
				if (cmd == C_CMD_HOME) { USER_PARAM(USR_COMMAND) = C_CMD_HOME; }
				return(SmTrans(NotHomed));
			}

			// Anything else is discarded, so nothing can linger and fire later.
			return(SmNotHandled);
		}

		SIG_EXIT = {
			StaClr(C_STA_ERROR);
			g_in_fault = FALSE;
		}
	}


	///////////////////////////////////////////////////////////////////////
	// NotHomed - powered, but there is no datum yet, so no MOVE.
	///////////////////////////////////////////////////////////////////////
	SmState NotHomed {
		SIG_ENTRY = {
			if (g_verbose) print("3R1T -> NotHomed  (press HOME)");
			g_state_id = ST_NOTHOMED;
			SetLed(LED_AMBER);
			Say(MSG_NEED_HOME);
			StaClr(C_STA_READY | C_STA_MOVING | C_STA_HOMED);
			LeaveStream();
			USER_PARAM(USR_HOME_STATE) = H_IDLE;
		}

		SIG_IDLE = {
			long cmd;

			cmd = USER_PARAM(USR_COMMAND);
			if (cmd == 0) { return(SmNotHandled); }
			USER_PARAM(USR_COMMAND) = 0;

			if (cmd == C_CMD_HOME) {
				if (USER_PARAM(USR_THETA_STALE) == TRUE) {
					print("HOME refused: no limb angles on CAN - is the sensor node running?");
					Say(MSG_HOME_NO_IMU);
					return(SmNotHandled);
				}
				// >>> AND THE FILTERS MUST HAVE SETTLED. <<<
				// Before they do, the sensor node has not seeded its
				// continuity trackers and every theta reads 0 - which is
				// indistinguishable from three limbs genuinely at zero,
				// so homing would measure no error and declare success
				// without having moved anything.
				if ((USER_PARAM(USR_IMU_STATUS) & IMU_ST_CONVERGED) == 0) {
					print("HOME refused: limb IMUs still settling - wait a second and press again");
					Say(MSG_HOME_IMU_SETTLE);
					return(SmNotHandled);
				}
#if (HOME_1T_ENABLE == 1)
				if (USER_PARAM(USR_ENC_STALE) == TRUE) {
					print("HOME refused: no stage encoder data on CAN");
					Say(MSG_HOME_NO_ENC);
					return(SmNotHandled);
				}
#endif
				return(SmTrans(Homing->Home3RMeasure));
			}

			if (cmd == C_CMD_MOVE) {
				print("MOVE refused: not homed");
				Say(MSG_MOVE_NO_HOME);
			}
			if (cmd == C_CMD_TELEOP) {
				// The pose the device streams is measured from the home
				// pose. Without a datum there is nothing for it to mean.
				print("TELEOP refused: not homed");
				Say(MSG_TELE_NOT_HOMED);
			}
			if (cmd == C_CMD_STOP) {
				StopEverything();
				Say(MSG_STOP);
			}
			if (cmd == C_CMD_ERROR_CLR) {
				ErrorClear();
				AmpErrorClear(C_AXIS1, C_AXIS2, C_AXIS3, C_AXIS_1T);
				Say(MSG_ERR_CLR);
			}
			return(SmNotHandled);
		}
	}


	///////////////////////////////////////////////////////////////////////
	// Homing - limbs first, then the stage, then capture the datum.
	//
	// Limbs before stage on purpose: homing the limbs swings the platform
	// the tool passes through, so the platform is put somewhere known
	// before the stage is asked to travel.
	///////////////////////////////////////////////////////////////////////
	SmState Homing {
		SIG_ENTRY = {
			if (g_verbose) print("3R1T -> Homing");
			g_state_id = ST_HOMING;
			SetLed(LED_AMBER);
			Say(MSG_HOME_START);
			StaSet(C_STA_HOMING);
			StaClr(C_STA_READY | C_STA_HOMED | C_STA_MOVING);

			// Homing drives the axes directly, so the stream has to let go
			// of them first - but not while they are still moving. HOME
			// can be pressed mid-move, and releasing the stream then would
			// stop the drives dead instead of letting them ramp down with
			// the pose they are following.
			AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
			WaitPoseStopped(2000);
			LeaveStream();

			// Clear the command boxes the moment HOME is pressed, not just when
			// it succeeds - a failed run should not leave a stale pose sitting
			// in them for the next MOVE to act on.
			USER_PARAM(USR_TGT_PSI)     = 0;
			USER_PARAM(USR_TGT_PHI)     = 0;
			USER_PARAM(USR_TGT_THETA_N) = 0;
			USER_PARAM(USR_TGT_TOOL)    = 0;

			g_home_failed = FALSE;
			g_home_done   = FALSE;
			g_pass = 0;
			g_moved1 = 0;
			g_moved2 = 0;
			g_moved3 = 0;
			// Seeded huge so the first pass cannot fail the shrink test -
			// there is nothing yet to have shrunk from.
			g_err_prev = 999999;
			USER_PARAM(USR_HOME_PASS) = 0;
		}

		// Only reached when a child returns SmNotHandled, so a child that
		// wants to keep a command for itself simply consumes it first.
		//
		// >>> LEAVING Homing IS THIS STATE'S JOB - NOT ITS CHILDREN'S. <<<
		//
		// SmTrans resolves a bare name among SIBLINGS, and a leading arrow
		// means "descend into my child". Neither of those can reach
		// NotHomed or Ready from inside Home3RMeasure, because those are
		// siblings of Homing, not of the child - there is no notation for
		// an uncle. Trying it is a compile error, not a runtime surprise.
		//
		// So a child that is finished says so with a flag and returns
		// SmNotHandled, which hands this pass straight here. The exit
		// happens in the same idle pass, from the one level that can
		// actually see NotHomed and Ready.
		SIG_IDLE = {
			if (g_home_failed == TRUE) {
				g_home_failed = FALSE;
				return(SmTrans(NotHomed));
			}
			if (g_home_done == TRUE) {
				g_home_done = FALSE;
				return(SmTrans(Ready));
			}

			if (USER_PARAM(USR_COMMAND) == C_CMD_STOP) {
				USER_PARAM(USR_COMMAND) = 0;
				StopEverything();
				print("Homing: aborted by STOP");
				Say(MSG_HOME_ABORT);
				return(SmTrans(NotHomed));
			}
			// A second HOME press also aborts, so a hung sequence never
			// needs a power cycle to escape.
			if (USER_PARAM(USR_COMMAND) == C_CMD_HOME) {
				USER_PARAM(USR_COMMAND) = 0;
				StopEverything();
				print("Homing: aborted - HOME pressed again");
				Say(MSG_HOME_ABORT);
				return(SmTrans(NotHomed));
			}
			if (USER_PARAM(USR_COMMAND) != 0) { USER_PARAM(USR_COMMAND) = 0; }
			return(SmNotHandled);
		}

		SIG_EXIT = {
			StaClr(C_STA_HOMING);
		}


		///////////////////////////////////////////////////////////////////
		// Measure the limb angles from the IMUs and work out the error.
		///////////////////////////////////////////////////////////////////
		SmState Home3RMeasure {
			SIG_ENTRY = {
				USER_PARAM(USR_HOME_STATE) = H_3R_MEASURE;
				Say(MSG_HOME_MEAS);
				g_sum1 = 0; g_sum2 = 0; g_sum3 = 0; g_count = 0;
				g_ref1 = 0; g_ref2 = 0; g_ref3 = 0;
				g_lastFrame = g_frames;
				if (g_verbose) print("Homing: measuring (",HOME_AVG_SAMPLES," frames, about 0.3 s)  pass ",g_pass);
			}

			SIG_IDLE = {
				long m1, m2, m3, d1, d2, d3, moved;
				long errMax, s1, s2, s3;

				if (USER_PARAM(USR_THETA_STALE) == TRUE) {
					print("Homing: CAN went stale while measuring");
					Say(MSG_HOME_STALE);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}

				// One sample per NEW frame. The idle loop runs far faster
				// than 50 Hz, so sampling per pass would add the same
				// frame up 16 times and average nothing at all.
				if (g_frames == g_lastFrame) { return(SmNotHandled); }
				g_lastFrame = g_frames;

				// >>> AVERAGED RELATIVE TO THE FIRST SAMPLE. <<<
				//
				// A plain mean of angles is wrong across a seam: 359 and 1
				// average to 180, which is the opposite side of the limb.
				// The sensor node now sends a CONTINUOUS angle so there is
				// no seam in normal running - but a node restart, a garbled
				// frame or a future change to that packing would put one
				// back, and a mean that is only correct while nothing goes
				// wrong is not worth having in a homing routine.
				//
				// So the first sample is the reference and only WRAPPED
				// differences from it are accumulated. Correct either way,
				// and the same cost.
				if (g_count == 0) {
					g_ref1 = USER_PARAM(USR_THETA1);
					g_ref2 = USER_PARAM(USR_THETA2);
					g_ref3 = USER_PARAM(USR_THETA3);
				}
				g_sum1 = g_sum1 + WrapCDeg(USER_PARAM(USR_THETA1) - g_ref1);
				g_sum2 = g_sum2 + WrapCDeg(USER_PARAM(USR_THETA2) - g_ref2);
				g_sum3 = g_sum3 + WrapCDeg(USER_PARAM(USR_THETA3) - g_ref3);
				g_count = g_count + 1;
				if (g_count < HOME_AVG_SAMPLES) { return(SmNotHandled); }

				m1 = g_ref1 + g_sum1 / g_count;
				m2 = g_ref2 + g_sum2 / g_count;
				m3 = g_ref3 + g_sum3 / g_count;
				USER_PARAM(USR_MEAS1) = m1;
				USER_PARAM(USR_MEAS2) = m2;
				USER_PARAM(USR_MEAS3) = m3;

				d1 = WrapCDeg(HOME_ARM1_CDEG - m1);
				d2 = WrapCDeg(HOME_ARM2_CDEG - m2);
				d3 = WrapCDeg(HOME_ARM3_CDEG - m3);
				USER_PARAM(USR_ERR1) = d1;
				USER_PARAM(USR_ERR2) = d2;
				USER_PARAM(USR_ERR3) = d3;

				if (g_verbose) print("Homing: measured [cdeg] arm1=",m1," arm2=",m2," arm3=",m3);
				print("Homing: error    [cdeg] arm1=",d1," arm2=",d2," arm3=",d3,"  (tolerance ",HOME_TOL_CDEG,")");

				if (AbsL(d1) <= HOME_TOL_CDEG && AbsL(d2) <= HOME_TOL_CDEG && AbsL(d3) <= HOME_TOL_CDEG) {
					print("Homing: all limbs within tolerance after ",g_pass," pass(es)");
					Say(MSG_HOME_LIMBS_OK);
					return(SmTrans(Home1TSlack));
				}

				// ---- RUNAWAY GUARD 1: the error must be shrinking ----
				// This loop only converges if moving an arm by
				// HOME_DIR_ARMn * error actually reduces that error. With
				// the sign wrong it does the opposite, and the error
				// DOUBLES every pass. Waiting for the pass counter to run
				// out would mean waiting for 2^8 times the original error.
				//
				// So every pass has to beat the one before it. One strike
				// and homing stops, because the failure is exponential and
				// there is no second chance worth having.
				errMax = AbsL(d1);
				if (AbsL(d2) > errMax) { errMax = AbsL(d2); }
				if (AbsL(d3) > errMax) { errMax = AbsL(d3); }

				if (errMax >= g_err_prev) {
					print("Homing: STOPPED - the error grew instead of shrinking.");
					print("Homing: was ",g_err_prev," cdeg, now ",errMax," cdeg, after pass ",g_pass);
					print("Homing: an arm driving AWAY from its target means HOME_DIR_ARMn has the");
					print("Homing: wrong sign. Flip the one for the arm whose error grew, in");
					print("Homing: 3R1T_Config.mh, and home again.");
					AxisStop(C_ARM1_AXIS, C_ARM2_AXIS, C_ARM3_AXIS);
					Say(MSG_HOME_DIVERGE);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}
				g_err_prev = errMax;

				g_pass = g_pass + 1;
				USER_PARAM(USR_HOME_PASS) = g_pass;
				if (g_pass > HOME_MAX_PASSES) {
					print("Homing: gave up after ",HOME_MAX_PASSES," passes - still ",errMax," cdeg out.");
					print("Homing: the error was shrinking but too slowly. The loop gain is low -");
					print("Homing: check the arm gearing against C_AXIS_POSFACT_Z.");
					Say(MSG_HOME_NO_CONV);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}

				// Per arm: only the ones outside tolerance are given a new
				// target. The rest hold their current position, so a limb
				// already at home is not nudged by rounding.
				//
				// ---- RUNAWAY GUARD 2: bound every single pass ----
				// One bad measurement - a stale frame, an IMU glitch, a
				// wrapped angle - can ask for a move of up to 180 deg. The
				// clamp turns that into a bounded nudge that the next pass
				// simply corrects. It costs nothing when things are right,
				// because a real error is small by pass two.
				s1 = HOME_DIR_ARM1 * d1;
				s2 = HOME_DIR_ARM2 * d2;
				s3 = HOME_DIR_ARM3 * d3;
				if (s1 >  HOME_MAX_STEP_CDEG) { s1 =  HOME_MAX_STEP_CDEG; }
				if (s1 < -HOME_MAX_STEP_CDEG) { s1 = -HOME_MAX_STEP_CDEG; }
				if (s2 >  HOME_MAX_STEP_CDEG) { s2 =  HOME_MAX_STEP_CDEG; }
				if (s2 < -HOME_MAX_STEP_CDEG) { s2 = -HOME_MAX_STEP_CDEG; }
				if (s3 >  HOME_MAX_STEP_CDEG) { s3 =  HOME_MAX_STEP_CDEG; }
				if (s3 < -HOME_MAX_STEP_CDEG) { s3 = -HOME_MAX_STEP_CDEG; }

				g_tgt1 = Cpos(C_ARM1_AXIS);
				g_tgt2 = Cpos(C_ARM2_AXIS);
				g_tgt3 = Cpos(C_ARM3_AXIS);
				moved = FALSE;
				if (AbsL(d1) > HOME_TOL_CDEG) { g_tgt1 = g_tgt1 + s1; g_moved1 = g_moved1 + AbsL(s1); moved = TRUE; }
				if (AbsL(d2) > HOME_TOL_CDEG) { g_tgt2 = g_tgt2 + s2; g_moved2 = g_moved2 + AbsL(s2); moved = TRUE; }
				if (AbsL(d3) > HOME_TOL_CDEG) { g_tgt3 = g_tgt3 + s3; g_moved3 = g_moved3 + AbsL(s3); moved = TRUE; }

				// ---- RUNAWAY GUARD 3: a budget for the whole run ----
				// The per-pass clamp bounds one bad pass. Eight of them in
				// a row are each individually reasonable and add up to a
				// runaway, so the total is bounded too.
				if (g_moved1 > HOME_MAX_TOTAL_CDEG || g_moved2 > HOME_MAX_TOTAL_CDEG || g_moved3 > HOME_MAX_TOTAL_CDEG) {
					print("Homing: STOPPED - an arm has travelled too far for one homing run.");
					print("Homing: travelled [cdeg] arm1=",g_moved1," arm2=",g_moved2," arm3=",g_moved3," budget ",HOME_MAX_TOTAL_CDEG);
					print("Homing: remaining error [cdeg] arm1=",d1," arm2=",d2," arm3=",d3);
					print("Homing: the arms were not where the IMUs said, or an arm is being");
					print("Homing: dragged by its neighbours through the platform.");
					AxisStop(C_ARM1_AXIS, C_ARM2_AXIS, C_ARM3_AXIS);
					Say(MSG_HOME_FAR);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}

				if (moved == FALSE) { return(SmTrans(Home1TSlack)); }

				if (g_verbose) print("Homing: moving to [cdeg] ax1=",g_tgt1," ax2=",g_tgt2," ax3=",g_tgt3);
				Say(MSG_HOME_MOVE);
				SetRealAxisVel(C_ARM1_AXIS, USER_PARAM(USR_HOME_VEL));
				SetRealAxisVel(C_ARM2_AXIS, USER_PARAM(USR_HOME_VEL));
				SetRealAxisVel(C_ARM3_AXIS, USER_PARAM(USR_HOME_VEL));
				AxisPosAbsStart(C_ARM1_AXIS, g_tgt1, C_ARM2_AXIS, g_tgt2, C_ARM3_AXIS, g_tgt3);
				return(SmTrans(Home3RMove));
			}
		}


		///////////////////////////////////////////////////////////////////
		// Run the limb move, then go back and measure again. Repeating
		// the cycle is what makes one press of HOME enough.
		///////////////////////////////////////////////////////////////////
		SmState Home3RMove {
			SIG_ENTRY = {
				USER_PARAM(USR_HOME_STATE) = H_3R_MOVE;
				StaSet(C_STA_MOVING);
				g_move_t0 = Time();
			}

			SIG_IDLE = {
				if (ArmsAtTargets(g_tgt1, g_tgt2, g_tgt3) == TRUE) {
					Say(MSG_HOME_PASS);
					return(SmTrans(Home3RMeasure));
				}
				if ((Time() - g_move_t0) > HOME_MOVE_TIMEOUT) {
					print("Homing: TIMEOUT - limbs did not settle. Stopping.");
					print("Homing: cpos ",Cpos(C_ARM1_AXIS)," ",Cpos(C_ARM2_AXIS)," ",Cpos(C_ARM3_AXIS)," target ",g_tgt1," ",g_tgt2," ",g_tgt3);
					print("Homing: if cpos never moved at all, the drives are not enabled.");
					AxisStop(C_ARM1_AXIS, C_ARM2_AXIS, C_ARM3_AXIS);
					Say(MSG_HOME_MV_TMO);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}
				return(SmNotHandled);
			}

			SIG_EXIT = {
				StaClr(C_STA_MOVING);
			}
		}


		///////////////////////////////////////////////////////////////////
		// Find the stage reference mark.
		//
		// Drive slowly until ANY change appears on REF - rising edge,
		// falling edge, sitting on the mark, or the pass counter moving -
		// then stop. That edge is the datum.
		//
		// No width check, no midpoint, no second pass - each would be a
		// way to throw away a perfectly good crossing. The only question
		// asked is "did REF change", which is the same question you answer
		// by eye pushing the stage across by hand.
		//
		// If the end stop comes first, reverse and keep looking. Two end
		// stops with no REF change means the mark is not on the rail.
		///////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////
		// Phase A - find the end of travel, by CABLE SLACK.
		//
		// The REF mark is ignored entirely here. The stage drives in
		// C_1T_HOME_DIR until the motor is still turning while the
		// carriage has stopped - which is what winding up against the end
		// of travel looks like on a cable drive.
		//
		// >>> BOTH CONDITIONS, TOGETHER, OR IT PROVES NOTHING. <<<
		//
		// The motor turning on its own is the normal state of affairs. The
		// carriage being still on its own happens whenever the drive has
		// faulted, tripped or never started - and calling that an end stop
		// would set the datum in mid-rail with nothing to say it was
		// wrong. Only the pair, held for H_SLACK_TIME_MS, means slack.
		///////////////////////////////////////////////////////////////
		SmState Home1TSlack {
			SIG_ENTRY = {
				USER_PARAM(USR_HOME_STATE) = H_1T_SLACK;
#if (HOME_1T_ENABLE == 1)
				Say(MSG_HOME_1T_START);
				g_h_dir     = C_1T_HOME_DIR;
				g_h_t0      = Time();
				g_h_slackT0 = Time();
				g_h_encRef  = USER_PARAM(USR_ENC_RAW_UM);
				if (g_verbose) print("Homing: stage phase A - driving ",g_h_dir," at ",H_SEARCH_VEL," to find the end of travel. REF ignored.");
				StartSearch(g_h_dir);
#else
				Say(MSG_HOME_1T_SKIP);
				if (g_verbose) print("Homing: stage homing disabled (HOME_1T_ENABLE = 0) - skipped");
#endif
			}

			SIG_IDLE = {
				long mv, d;

#if (HOME_1T_ENABLE == 0)
				return(SmTrans(HomeFinish));
#else
				if (USER_PARAM(USR_ENC_STALE) == TRUE) {
					sdkStopContinuousMove(C_AXIS_1T, H_DEC);
					print("Homing: stage encoder went stale in phase A - stopping");
					Say(MSG_HOME_STALE);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}

				if ((Time() - g_h_t0) > H_SLACK_TIMEOUT_MS) {
					sdkStopContinuousMove(C_AXIS_1T, H_DEC);
					print("Homing: FAILED - no cable slack in ",H_SLACK_TIMEOUT_MS," ms.");
					print("Homing: the carriage never stopped moving, so the end of travel");
					print("Homing: was never reached. Check C_1T_HOME_DIR is driving towards");
					print("Homing: an end, and that H_SLACK_ENC_UM is not larger than the");
					print("Homing: movement per ",H_SLACK_TIME_MS," ms at this search speed.");
					Say(MSG_HOME_1T_NOSLK);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}

				// Held off while the drive takes up the load, exactly as
				// the old stall detector was: straight after starting, the
				// carriage genuinely has not moved yet.
				if ((Time() - g_h_moveT0) < H_SLACK_ARM_MS) {
					g_h_slackT0 = Time();
					g_h_encRef  = USER_PARAM(USR_ENC_RAW_UM);
					return(SmNotHandled);
				}

				mv = AbsL(AXE_PROCESS(C_AXIS_1T, REG_AVEL));
				d  = AbsL(USER_PARAM(USR_ENC_RAW_UM) - g_h_encRef);

				// Either condition failing reopens the window. The encoder
				// reference moves with it, so the test is always "how far
				// in the last H_SLACK_TIME_MS", never "since we started".
				if (mv < H_SLACK_MOTOR_UU_S || d > H_SLACK_ENC_UM) {
					g_h_slackT0 = Time();
					g_h_encRef  = USER_PARAM(USR_ENC_RAW_UM);
					return(SmNotHandled);
				}

				if ((Time() - g_h_slackT0) > H_SLACK_TIME_MS) {
					sdkStopContinuousMove(C_AXIS_1T, H_DEC);
					print("Homing: end of travel - motor at ",mv," UU/s, carriage moved ",d," um in ",H_SLACK_TIME_MS," ms");
					Say(MSG_HOME_1T_SLACK);
					return(SmTrans(Home1TRef));
				}
				return(SmNotHandled);
#endif
			}
		}


		///////////////////////////////////////////////////////////////
		// Phase B - reverse and take the datum from the REF edge.
		//
		// The baseline is taken AFTER the stage has stopped, so an edge
		// latched during phase A's deceleration is counted as old and
		// cannot be mistaken for the one we are looking for.
		//
		// The datum is the LATCHED edge position. The sensor node now
		// records the encoder count inside the REF interrupt, so that
		// figure no longer depends on a speed estimate and is good
		// whatever the stage was doing when it crossed.
		//
		// Note there is no "already on the mark" shortcut here, unlike
		// the old single-phase search: phase A has just driven to the end
		// of travel, so the mark is behind us and a genuine new edge will
		// arrive on the way back.
		///////////////////////////////////////////////////////////////
		SmState Home1TRef {
			SIG_ENTRY = {
				USER_PARAM(USR_HOME_STATE) = H_1T_REF;
				WaitStageStopped(3000);
				RebaseRef();
				g_h_t0     = Time();
				g_h_encRef = USER_PARAM(USR_ENC_RAW_UM);
				if (g_verbose) print("Homing: stage phase B - reversing to ",(0 - C_1T_HOME_DIR)," to find REF");
				if (g_verbose) print("Homing: baseline rise=",g_h_rise0," fall=",g_h_fall0," passes=",g_h_passes0);
				StartSearch(0 - C_1T_HOME_DIR);
			}

			SIG_IDLE = {
				long edgeUm, found, travelled;

				if (USER_PARAM(USR_ENC_STALE) == TRUE) {
					sdkStopContinuousMove(C_AXIS_1T, H_DEC);
					print("Homing: stage encoder went stale in phase B - stopping");
					Say(MSG_HOME_STALE);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}

				travelled = AbsL(USER_PARAM(USR_ENC_RAW_UM) - g_h_encRef);

				if (travelled > H_REF_MAX_TRAVEL_UM || (Time() - g_h_t0) > H_REF_TIMEOUT_MS) {
					sdkStopContinuousMove(C_AXIS_1T, H_DEC);
					print("Homing: FAILED - no new REF edge in ",travelled," um / ",(Time() - g_h_t0)," ms");
					print("Homing: rise=",USER_PARAM(USR_REF_RISE_UM)," fall=",USER_PARAM(USR_REF_FALL_UM)," passes=",RefPasses());
					print("Homing: baseline was rise=",g_h_rise0," fall=",g_h_fall0," passes=",g_h_passes0);
					print("Homing: if none of those moved while the stage crossed the mark, REF");
					print("Homing: is not reaching the ESP32 while the motor is running.");
					Say(MSG_HOME_1T_NOREF);
					g_home_failed = TRUE;
					return(SmNotHandled);
				}

				// A NEW latched edge, of either polarity. The pass counter
				// is the backstop: a crossing fast enough to latch both
				// edges between two CAN frames still bumps it.
				found  = FALSE;
				edgeUm = 0;

				if (USER_PARAM(USR_REF_RISE_UM) != g_h_rise0) {
					edgeUm = USER_PARAM(USR_REF_RISE_UM);
					found  = TRUE;
					if (g_verbose) print("Homing: RISING edge latched at ",edgeUm," um");
				} else if (USER_PARAM(USR_REF_FALL_UM) != g_h_fall0) {
					edgeUm = USER_PARAM(USR_REF_FALL_UM);
					found  = TRUE;
					if (g_verbose) print("Homing: FALLING edge latched at ",edgeUm," um");
				} else if (RefPasses() != g_h_passes0) {
					edgeUm = USER_PARAM(USR_REF_FALL_UM);
					found  = TRUE;
					if (g_verbose) print("Homing: pass counted, edge at ",edgeUm," um");
				}

				if (found == TRUE) {
					sdkStopContinuousMove(C_AXIS_1T, H_DEC);
					// That edge is the datum. Zero is set there, so the
					// position straight afterwards shows how far the stage
					// coasted past it.
					g_zero_um = edgeUm;
					USER_PARAM(USR_ENC_ZERO_UM)  = g_zero_um;
					USER_PARAM(USR_HOME_EDGE_UM) = edgeUm;
					USER_PARAM(USR_ENC_UM)       = USER_PARAM(USR_ENC_RAW_UM) - g_zero_um;
					print("Homing: stage datum set at ",edgeUm," um");
					Say(MSG_HOME_1T_FOUND);
					return(SmTrans(HomeFinish));
				}
				return(SmNotHandled);
			}
		}


		SmState HomeFinish {
			SIG_ENTRY = {
				USER_PARAM(USR_HOME_STATE) = H_FINISH;
				Say(MSG_HOME_ALIGN);
				if (g_verbose) print("Homing: capturing the datum and aligning the kinematics");

				// 0. the stage was still decelerating out of its search
				//    when the REF edge was seen. Let it finish: arming the
				//    stream while it still has speed on would freeze it
				//    where it was rather than where it settles, and the
				//    datum would move with the coast.
				WaitStageStopped(3000);

				// 1. the tracker's zero is the home pose.
				USER_PARAM(USR_IK_RESET_CONT) = 1;

				// 2. the commanded pose is zero, and zero is right here.
				AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
				DefOrigin(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
				USER_PARAM(USR_TGT_PSI)     = 0;
				USER_PARAM(USR_TGT_PHI)     = 0;
				USER_PARAM(USR_TGT_THETA_N) = 0;
				USER_PARAM(USR_TGT_TOOL)    = 0;

				// 3. about 25 ISR cycles with the new zero. The solver
				//    returns zero, the offsets pin to the encoders, and
				//    REG_USERREFPOS settles on the present position.
				Delay(50);

				// 4. hand over. g_ik_armed is set AFTER the switch, not
				//    before: the axes are briefly OFF between these two
				//    calls, and if an arm sags in that window the ISR is
				//    still pinning the offsets to it, so the register
				//    follows the sag instead of yanking it back. Setting
				//    the flag first would freeze the offsets at the
				//    pre-sag value and turn that sag into a step.
				AxisControl(C_AXIS1, OFF, C_AXIS2, OFF, C_AXIS3, OFF, C_AXIS_1T, OFF);
				AxisControl(C_AXIS1, USERREFPOS, C_AXIS2, USERREFPOS, C_AXIS3, USERREFPOS, C_AXIS_1T, USERREFPOS);
				g_ik_armed = 1;
				StaSet(C_STA_STREAM | C_STA_HOMED);

				// The stage coasted this far past the mark before it could
				// stop. Tool zero is where it ACTUALLY ended up, not the
				// mark - see the note in 3R1T_Kin_1T.mh.
				print("Homing: stage coasted ",USER_PARAM(USR_ENC_UM)," um past the REF mark - that position is tool zero");
				print("Homing: datum [cdeg] arm1=",USER_PARAM(USR_AX1_CDEG)," arm2=",USER_PARAM(USR_AX2_CDEG)," arm3=",USER_PARAM(USR_AX3_CDEG)," tool=",USER_PARAM(USR_AXT_UU));
				print("Homing: DONE - the drives are now following the kinematics");
				Say(MSG_HOME_DONE);
			}

			SIG_IDLE = {
				// Leaving Homing altogether is the parent's job - see the
				// note on its SIG_IDLE.
				g_home_done = TRUE;
				return(SmNotHandled);
			}
		}
	}


	///////////////////////////////////////////////////////////////////////
	// Ready - homed, streaming, standing still. The green state.
	///////////////////////////////////////////////////////////////////////
	SmState Ready {
		SIG_ENTRY = {
			if (g_verbose) print("3R1T -> Ready");
			g_state_id = ST_READY;
			SetLed(LED_GREEN);
			StaSet(C_STA_READY);
			StaClr(C_STA_MOVING);
			USER_PARAM(USR_HOME_STATE) = H_DONE;
		}

		SIG_IDLE = {
			long cmd;

			cmd = USER_PARAM(USR_COMMAND);
			if (cmd == 0) { return(SmNotHandled); }
			USER_PARAM(USR_COMMAND) = 0;

			if (cmd == C_CMD_MOVE) {
				if (PoseCommandOk() == FALSE) { return(SmNotHandled); }
				Say(MSG_MOVE_CMD);
				StartPoseMove();
				return(SmTrans(Moving));
			}

			if (cmd == C_CMD_HOME) {
				if (USER_PARAM(USR_THETA_STALE) == TRUE) {
					print("HOME refused: no limb angles on CAN");
					Say(MSG_HOME_NO_IMU);
					return(SmNotHandled);
				}
				// See the note at the other HOME entry point.
				if ((USER_PARAM(USR_IMU_STATUS) & IMU_ST_CONVERGED) == 0) {
					print("HOME refused: limb IMUs still settling - wait a second and press again");
					Say(MSG_HOME_IMU_SETTLE);
					return(SmNotHandled);
				}
#if (HOME_1T_ENABLE == 1)
				if (USER_PARAM(USR_ENC_STALE) == TRUE) {
					print("HOME refused: no stage encoder data on CAN");
					Say(MSG_HOME_NO_ENC);
					return(SmNotHandled);
				}
#endif
				return(SmTrans(Homing->Home3RMeasure));
			}

			if (cmd == C_CMD_TRAJ_SINGLE || cmd == C_CMD_TRAJ_CONT) {
				TrajDump();
				if (TrajLoaded() == FALSE) {
					print("TRAJECTORY refused: no file loaded - press LOAD first");
					Say(MSG_TRAJ_EMPTY);
					return(SmNotHandled);
				}
				if (cmd == C_CMD_TRAJ_CONT) {
					g_traj_mode = TRJ_CONTINUOUS;
				} else {
					g_traj_mode = TRJ_SINGLE;
				}
				return(SmTrans(TrajRun));
			}

			if (cmd == C_CMD_TELEOP) {
				// Refuse rather than enter a state that cannot do anything.
				// Teleop with no heartbeat would sit in TELE_WAIT_ENABLE
				// for ever and look like a hang.
				if (USER_PARAM(USR_TELE_HB_AGE) > C_TELE_STALE_MS) {
					print("TELEOP refused: no heartbeat from the PC - is the haptic program running?");
					Say(MSG_TELE_NO_LINK);
					return(SmNotHandled);
				}
				return(SmTrans(Teleop));
			}

			if (cmd == C_CMD_STOP) {
				// Nothing is moving, but say so anyway - a button that
				// appears to do nothing is worse than one that reports.
				Say(MSG_STOP);
			}

			if (cmd == C_CMD_ERROR_CLR) {
				ErrorClear();
				AmpErrorClear(C_AXIS1, C_AXIS2, C_AXIS3, C_AXIS_1T);
				Say(MSG_ERR_CLR);
			}
			return(SmNotHandled);
		}
	}


	///////////////////////////////////////////////////////////////////////
	// Moving - the commanded pose is running its profile and the drives
	// are following the kinematics.
	///////////////////////////////////////////////////////////////////////
	SmState Moving {
		SIG_ENTRY = {
			g_state_id = ST_MOVING;
			SetLed(LED_AMBER);
			Say(MSG_MOVE_RUN);
			StaSet(C_STA_MOVING);
			StaClr(C_STA_READY);
		}

		SIG_IDLE = {
			long cmd;

			cmd = USER_PARAM(USR_COMMAND);
			if (cmd != 0) {
				USER_PARAM(USR_COMMAND) = 0;

				if (cmd == C_CMD_STOP) {
					// Stop the COMMANDED POSE. The real drives follow the
					// kinematics to a standstill with it. Do not AxisStop
					// a real axis here - in USERREFPOS it is not running a
					// profile, so there is nothing to stop, and it would
					// simply leave the stream and the drive behind.
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					if (g_verbose) print("STOP - pose held at psi=",Cpos(EE_PSI)," phi=",Cpos(EE_PHI)," theta_n=",Cpos(EE_THETA_N)," tool=",Cpos(EE_TOOL));
					Say(MSG_STOP);
					return(SmTrans(Ready));
				}

				if (cmd == C_CMD_MOVE) {
					// Blend straight into the new target rather than
					// bouncing through Ready.
					if (PoseCommandOk() == TRUE) {
						Say(MSG_MOVE_CMD);
						StartPoseMove();
					}
				}

				if (cmd == C_CMD_HOME) {
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					return(SmTrans(Homing->Home3RMeasure));
				}
			}

			if (PoseAtTarget() == TRUE) {
				if (g_verbose) print("MOVE complete  psi=",Cpos(EE_PSI)," phi=",Cpos(EE_PHI)," theta_n=",Cpos(EE_THETA_N)," tool=",Cpos(EE_TOOL));
				Say(MSG_MOVE_DONE);
				return(SmTrans(Ready));
			}
			return(SmNotHandled);
		}

		SIG_EXIT = {
			StaClr(C_STA_MOVING);
		}
	}


	///////////////////////////////////////////////////////////////////////
	// Teleop - the haptic device has the pose.
	//
	// The state is entered with the enable NOT held. Taking control is a
	// separate, deliberate act: hold the enable on the device and the rig
	// starts following; let go and it stops where it is. So there are two
	// ways out of following - releasing the enable, and the watchdog - and
	// only one way in.
	//
	// >>> RELEASING THE ENABLE IS NOT AN EMERGENCY STOP. <<<
	//
	// It is a button on the far side of a USB cable, a Python program and
	// an Ethernet link. It stops the rig following in the ordinary case and
	// that is all it is for. The watchdog is what covers the PC dying, and
	// the hardware E-stop is what covers everything else.
	//
	// >>> AND NEITHER IS THE CLAMP. <<<
	//
	// TeleopRetarget clamps the demand to LIM_* and publishes the fact, but
	// the operator feels nothing when it bites - this machine reflects no
	// force back to the device. The clamp keeps the rig inside its fence;
	// it does not tell the hand to stop. That is the PC's job, on screen.
	///////////////////////////////////////////////////////////////////////
	SmState Teleop {
		SIG_ENTRY = {
			if (g_verbose) print("3R1T -> Teleop");
			g_state_id = ST_TELEOP;
			SetLed(LED_AMBER);
			StaSet(C_STA_TELEOP);
			StaClr(C_STA_READY);

			g_tele_following = 0;
			USER_PARAM(USR_TELE_STATE) = TELE_WAIT_ENABLE;

			// Start the freshness clock from now. The heartbeat was checked
			// before the transition, but priming it here means a slow first
			// cycle cannot look like a dropped link.
			g_tele_hb    = USER_PARAM(USR_TELE_HEARTBEAT);
			g_tele_hb_ms = Time();

			TeleopCapture();
			Say(MSG_TELE_ENTER);
			print("TELEOP: entered. Hold the enable on the haptic device to take control.");
			print("TELEOP: follow speeds ",USER_PARAM(USR_TELE_VEL_ROT)," cdeg/s and ",USER_PARAM(USR_TELE_VEL_TOOL)," UU/s, accel ",USER_PARAM(USR_TELE_ACC_SCALE)," %");
		}

		SIG_IDLE = {
			long cmd, fresh, enable;

			cmd = USER_PARAM(USR_COMMAND);
			if (cmd != 0) {
				USER_PARAM(USR_COMMAND) = 0;

				// STOP and a second TELEOP both leave. Stopping the pose
				// axes is enough - the drives follow the kinematics to a
				// standstill with them, exactly as in Moving. Do NOT
				// AxisStop a real axis here.
				if (cmd == C_CMD_STOP || cmd == C_CMD_TELEOP) {
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					if (g_verbose) print("TELEOP: left at psi=",Cpos(EE_PSI)," phi=",Cpos(EE_PHI)," theta_n=",Cpos(EE_THETA_N)," tool=",Cpos(EE_TOOL));
					Say(MSG_TELE_EXIT);
					return(SmTrans(Ready));
				}

				if (cmd == C_CMD_HOME) {
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					return(SmTrans(Homing->Home3RMeasure));
				}
			}

			// The parent's SIG_IDLE has already refreshed the heartbeat
			// figures this pass; this only reads the verdict.
			fresh  = (USER_PARAM(USR_TELE_HB_AGE) <= C_TELE_STALE_MS) ? TRUE : FALSE;
			enable = (USER_PARAM(USR_TELE_ENABLE) == 1) ? TRUE : FALSE;

			// ---- the watchdog ------------------------------------------
			if (fresh == FALSE) {
				if (g_tele_following == 1) {
					g_tele_following = 0;
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					USER_PARAM(USR_TELE_TRIPS) = USER_PARAM(USR_TELE_TRIPS) + 1;
					USER_PARAM(USR_TELE_STATE) = TELE_DROPPED;
					Say(MSG_TELE_LOST);
					print("TELEOP: no heartbeat for ",USER_PARAM(USR_TELE_HB_AGE)," ms - pose held. Drop-out ",USER_PARAM(USR_TELE_TRIPS)," this power-up");
				}
				// Stay in the state. The link may come back, and the
				// operator still has to press STOP to leave - a rig that
				// silently returned to Ready would be one where nobody
				// could tell whether the link had ever worked.
				return(SmNotHandled);
			}

			// ---- the enable --------------------------------------------
			if (enable == FALSE) {
				if (g_tele_following == 1) {
					g_tele_following = 0;
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					USER_PARAM(USR_TELE_STATE) = TELE_WAIT_ENABLE;
					Say(MSG_TELE_RELEASE);
					if (g_verbose) print("TELEOP: enable released - pose held");
				}
				// Re-reference on every idle pass while not following, so
				// however far the hand wanders in the meantime, taking
				// control again starts from where the pose actually is.
				TeleopCapture();
				return(SmNotHandled);
			}

			if (g_tele_following == 0) {
				g_tele_following = 1;
				TeleopCapture();
				USER_PARAM(USR_TELE_STATE) = TELE_FOLLOW;
				Say(MSG_TELE_FOLLOW);
				if (g_verbose) print("TELEOP: following from psi=",Cpos(EE_PSI)," phi=",Cpos(EE_PHI)," theta_n=",Cpos(EE_THETA_N)," tool=",Cpos(EE_TOOL));
			}

			// ---- follow -------------------------------------------------
			// Rate-limited on purpose. See C_TELE_RETARGET_MS.
			if (Time() >= g_tele_next_ms) {
				g_tele_next_ms = Time() + C_TELE_RETARGET_MS;
				TeleopRetarget();
			}
			return(SmNotHandled);
		}

		SIG_EXIT = {
			g_tele_following = 0;
			StaClr(C_STA_TELEOP);
			USER_PARAM(USR_TELE_STATE)   = TELE_OFF;
			USER_PARAM(USR_TELE_CLAMPED) = 0;
		}
	}


	///////////////////////////////////////////////////////////////////////
	// TrajRun - play the loaded trajectory.
	//
	// The path flows THROUGH the waypoints rather than stopping at each
	// one. The moment the profile generator starts decelerating towards
	// the current target, the next is loaded and a fresh coordinated move
	// is issued over the top of it - the axes never reach zero speed, so
	// the corner is rounded instead of squared.
	//
	// That means a trajectory is not a series of moves; it is one
	// continuous move whose target keeps being replaced.
	//
	// >>> UNLESS THE ROW ASKS FOR A PAUSE. <<<
	//
	// A row carrying PAUSE = 1 is not blended into the one after it. The
	// axes are brought to a standstill and then held there for
	// C_TRAJ_PAUSE_MS before the next waypoint goes out, which is what
	// keeps a rotation from overlapping a translation. That is DECLARED
	// BY THE FILE - the player reads the column and works nothing out for
	// itself. The last waypoint runs to a standstill too, whatever its
	// flag says, because there is nothing left to blend into.
	//
	// Every waypoint is limit-checked as it is loaded. A file that strays
	// outside the fence stops the run there rather than clamping, because
	// a silently clamped trajectory is a different trajectory.
	///////////////////////////////////////////////////////////////////////
	SmState TrajRun {
		SIG_ENTRY = {
			g_state_id = ST_TRAJ;
			SetLed(LED_AMBER);
			StaSet(C_STA_MOVING | C_STA_TRAJ);
			StaClr(C_STA_READY);

			TrajReadHeader();
			USER_PARAM(USR_TRAJ_STATE) = g_traj_mode;
			USER_PARAM(USR_TRAJ_LAPS)  = 0;
			g_traj_row       = 0;
			g_traj_last      = FALSE;
			g_traj_pending   = FALSE;
			g_traj_pause_req = FALSE;
			g_traj_pausing   = FALSE;
			g_traj_pause_t0  = 0;

			// Before the first StartPoseMove below, which reads them.
			TrajApplySpeeds();

			print("TRAJECTORY: file ",USER_PARAM(USR_TRAJ_NUM),", ",USER_PARAM(USR_TRAJ_LEN)," points");
			if (g_traj_mode == TRJ_CONTINUOUS) {
				Say(MSG_TRAJ_CONT);
			} else {
				Say(MSG_TRAJ_SINGLE);
			}

			LoadNextTrajPoint();
			if (g_verbose) print("TRAJ: step ",USER_PARAM(USR_TRAJ_STEP)," -> psi=",USER_PARAM(USR_TGT_PSI)," phi=",USER_PARAM(USR_TGT_PHI)," thn=",USER_PARAM(USR_TGT_THETA_N)," tool=",USER_PARAM(USR_TGT_TOOL)," pause=",USER_PARAM(USR_TRAJ_PAUSE));
			StartPoseMove();
		}

		SIG_IDLE = {
			long cmd, advance;

			cmd = USER_PARAM(USR_COMMAND);
			if (cmd != 0) {
				USER_PARAM(USR_COMMAND) = 0;

				if (cmd == C_CMD_STOP) {
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					Say(MSG_TRAJ_STOP);
					return(SmTrans(Ready));
				}
				if (cmd == C_CMD_HOME) {
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					return(SmTrans(Homing->Home3RMeasure));
				}
				// MOVE and the trajectory buttons are ignored while one is
				// already running - STOP first.
			}

			// ---- may the next waypoint be queued yet? ----
			//
			// ONE question, and the ROW ITSELF answers it. The pause flag
			// was read out of the file when this waypoint was loaded.
			// Nothing here compares one row with the next, and nothing
			// here decides where a leg boundary is; a file that wants the
			// rig to stand still says so in its own column.
			//
			// >>> A PAUSE IS TWO STAGES, AND IT NEEDS BOTH OF THEM. <<<
			//
			// Stage one waits for the axes to reach a genuine standstill.
			// Stage two holds them there for C_TRAJ_PAUSE_MS. Stamping the
			// hold timer at load time instead would spend most of it on
			// the tail of a move that is still running, and the rig would
			// be let go at the moment it stopped - a pause that measures
			// the move rather than the rest, which is the whole reason the
			// old suppressed blend never behaved.
			//
			// >>> STOPPED, NOT AT TARGET. <<< PoseStopped asks only
			// whether the profile generators have finished. PoseAtTarget
			// also asks whether they finished inside the tolerance window,
			// and a blended move leaves a small residual on any axis whose
			// share of it was small - psi sitting 0.46 deg off while phi
			// did the work. Gating stage one on that sat out the entire
			// settle timeout before the hold could even begin, which is
			// what a "pause of a few seconds" after a rotation was. The
			// first waypoint of a file never showed it, because nothing
			// has moved yet to leave a residual.
			//
			// A ROW THAT ASKS FOR NO PAUSE is queued on the DECELERATION
			// edge, which is what rounds the corner instead of squaring
			// it. A waypoint the pose is already standing on produces no
			// move and so no edge, which is why arrival counts too - every
			// file starts at all-zeros, exactly where HOME leaves the rig.
			advance = FALSE;

			if (g_traj_pause_req == TRUE) {
				// The two tests below are sequential and NOT an else-if,
				// so the hold starts on the same pass that finds the axes
				// stopped rather than a state machine tick later.
				if (g_traj_pausing == FALSE) {
					g_traj_pending = FALSE;
					if (PoseStopped() == TRUE) {
						g_traj_pausing  = TRUE;
						g_traj_pause_t0 = Time();
					} else if ((Time() - g_traj_load_t0) >= C_TRAJ_SETTLE_MS) {
						// Bounded: a paused waypoint has no deceleration
						// edge to fall back on, so an axis that never
						// reports standstill must not be able to hold the
						// run up for ever. The hold still happens - it is
						// just timed from here instead of from rest.
						print("TRAJ: step ",USER_PARAM(USR_TRAJ_STEP)," not at rest after ",C_TRAJ_SETTLE_MS," ms - psi/phi/thn/tool profile states ",AXE_PROCESS(EE_PSI,PFG_AKTSTATE),"/",AXE_PROCESS(EE_PHI,PFG_AKTSTATE),"/",AXE_PROCESS(EE_THETA_N,PFG_AKTSTATE),"/",AXE_PROCESS(EE_TOOL,PFG_AKTSTATE)," (",PGS_POSCTRL," = stopped) - holding anyway");
						g_traj_pausing  = TRUE;
						g_traj_pause_t0 = Time();
					}
				}
				if (g_traj_pausing == TRUE) {
					if ((Time() - g_traj_pause_t0) >= C_TRAJ_PAUSE_MS) { advance = TRUE; }
				}
			} else {
				if (DecelStarted() == TRUE) {
					if (g_traj_pending == FALSE) {
						g_traj_pending = TRUE;
						advance = TRUE;
					}
				} else {
					g_traj_pending = FALSE;
					if (PoseAtTarget() == TRUE) { advance = TRUE; }
				}
			}

			// The one place a waypoint is queued. TrajAdvance reloads
			// g_traj_pause_req from the next row and clears the hold that
			// has just expired, so nothing above has to.
			if (advance == TRUE && g_traj_last == FALSE) {
				if (TrajAdvance() < 0) {
					AxisStop(EE_PSI, EE_PHI, EE_THETA_N, EE_TOOL);
					print("TRAJECTORY: stopped at step ",USER_PARAM(USR_TRAJ_STEP)," - outside the limits");
					Say(MSG_TRAJ_LIM);
					return(SmTrans(Ready));
				}
			}

			if (g_traj_last == TRUE && PoseAtTarget() == TRUE) {
				Say(MSG_TRAJ_DONE);
				return(SmTrans(Ready));
			}
			return(SmNotHandled);
		}

		SIG_EXIT = {
			// The step number the run got to is the whole diagnosis when a
			// trajectory ends early: 1 means it never advanced off the
			// first waypoint, 50 is the sweep's phase boundary, 73 is a
			// normal finish.
			print("TRAJECTORY ended at step ",USER_PARAM(USR_TRAJ_STEP)," of ",USER_PARAM(USR_TRAJ_LEN),", laps ",USER_PARAM(USR_TRAJ_LAPS));
			StaClr(C_STA_MOVING | C_STA_TRAJ);
			USER_PARAM(USR_TRAJ_STATE) = TRJ_IDLE;
			g_traj_mode = TRJ_IDLE;

			// The file's speeds were only ever for the length of the run.
			// A MOVE from the panel afterwards gets the panel's back.
			TrajRestoreSpeeds();
			g_traj_pause_req = FALSE;
			g_traj_pausing   = FALSE;
			g_traj_pause_t0  = 0;
			g_traj_load_t0   = 0;
			USER_PARAM(USR_TRAJ_PAUSE) = 0;
		}
	}
}


SmMachine Main3R1T { ID_SM_MAIN, * , MainMachine, 5, 2 }


/*********************************************************************
** Main
*********************************************************************/
long main(void)
{
	print("\n****************************************");
	print("|  QUB 3R1T - homing + kinematics      |");
	print("****************************************");
	ErrorClear();
	print("App Version: ",C_APP_VERSION,"\n");

	// >>> A PAUSE BEFORE THE 2 ms INTERRUPT STARTS FIRING. <<<
	//
	// InterruptSetup arms PeriodRoutine before SmRun has run a single
	// state, so without this the interrupt is already running while the
	// CANopen slaves are still coming up - on the same bus it will be
	// streaming setpoints over. g_isr_ready guards the axis access but
	// does not slow the interrupt down.
	//
	// This is a HYPOTHESIS about a start-up race, not a proved fix. If the
	// fault turns out to happen mid-run, this is not what cured it - set
	// C_STARTUP_SETTLE_MS back to 0 rather than leave it as a lucky charm.
	if (C_STARTUP_SETTLE_MS > 0) {
		print("Settling for ",C_STARTUP_SETTLE_MS," ms before arming the kinematics interrupt");
		Delay(C_STARTUP_SETTLE_MS);
	}

	InterruptSetup(PERIOD, PeriodRoutine, C_PERIOD_TIME);

	SmRun(Main3R1T);
	return(0);
}


/*********************************************************************
** The 2 ms kinematics interrupt.
**
** Read where the commanded pose has got to, solve both kinematics, and
** stream the answer to the drives.
**
** This runs 500 times a second and is the only thing that writes
** REG_USERREFPOS. It writes it whether or not the drives are listening,
** which is deliberate: see the note on g_ik_armed in 3R1T_Globals.mh and
** the HomeFinish state above.
*********************************************************************/
void PeriodRoutine(void)
{
	double ik1, ik2, ik3, toolUU;

	// main() arms the interrupt before SIG_INIT has configured the axes.
	if (g_isr_ready == 0) { return; }

	g_isr_t0 = TimeHw();

	// The commanded pose. Cpos on these axes reads back exactly what was
	// commanded: centidegrees for the three angles, 0.01 mm for the tool.
	psi_deg          = (double)(Cpos(EE_PSI))     / 100.0;
	phi_deg          = (double)(Cpos(EE_PHI))     / 100.0;
	User_tool_mm     = (double)(Cpos(EE_TOOL))    / 100.0;

	// The 3R solver wants theta_n measured from the mechanism's own zero;
	// the operator measures it from the home pose. The offset is added
	// here, once, and NOT again inside the solver.
	User_theta_n_deg = (double)(Cpos(EE_THETA_N)) / 100.0 + IK_THETA_N_HOME_DEG;

	// Both solvers want the sine and cosine of the same three angles, so
	// they are computed once here rather than 80-odd times between them.
	UpdatePoseTrig();

	InverseKinematicsSPM();		// 3R1T_Kin_3R.mh  -> final_theta_1/2/3 [rad]
	InverseKinematics1T();		// 3R1T_Kin_1T.mh  -> final_tool_uu [0.01 mm]

	ik1 = grad(final_theta_1) * 100.0;		// [cdeg]
	ik2 = grad(final_theta_2) * 100.0;
	ik3 = grad(final_theta_3) * 100.0;

	// The tool setpoint arrives FINISHED from InverseKinematics1T, which has
	// already summed the tool command and the tilt correction the way the
	// notebook sums them:
	//
	//     total_cable = delta_L_co_ci + delta_L_ci_th
	//
	// Nothing is added to it here. This line is the exact counterpart of the
	// three ik1/ik2/ik3 lines above - each solver hands back a finished axis
	// setpoint and the ISR only applies the unit factor and streams it.
	toolUU = final_tool_uu;

	// Not armed: keep the offsets pinned to where the axes actually are.
	// REG_USERREFPOS then always says "stay exactly here", so the moment
	// AxisControl(USERREFPOS) takes effect there is nothing to jump to.
	// Once armed the offsets are frozen, and they are what makes the
	// solver's zero mean the homed position.
	if (g_ik_armed == 0) {
		g_off1 = (double)(Cpos(C_ARM1_AXIS)) - ik1;
		g_off2 = (double)(Cpos(C_ARM2_AXIS)) - ik2;
		g_off3 = (double)(Cpos(C_ARM3_AXIS)) - ik3;
		g_offT = (double)(Cpos(C_AXIS_1T))   - toolUU;
	}

	USER_PARAM(USR_AX1_CDEG) = (long)(g_off1 + ik1);
	USER_PARAM(USR_AX2_CDEG) = (long)(g_off2 + ik2);
	USER_PARAM(USR_AX3_CDEG) = (long)(g_off3 + ik3);
	USER_PARAM(USR_AXT_UU)   = (long)(g_offT + toolUU);

	// UU -> quadcounts. REG_USERREFPOS is in qc, not user units.
	AXE_PROCESS(C_ARM1_AXIS, REG_USERREFPOS) = (g_off1 + ik1) * factor_UU_QC;
	AXE_PROCESS(C_ARM2_AXIS, REG_USERREFPOS) = (g_off2 + ik2) * factor_UU_QC;
	AXE_PROCESS(C_ARM3_AXIS, REG_USERREFPOS) = (g_off3 + ik3) * factor_UU_QC;
	AXE_PROCESS(C_AXIS_1T,   REG_USERREFPOS) = (g_offT + toolUU) * factor_1T_UU_QC;

	USER_PARAM(USR_ISR_DURATION) = (TimeHw() - g_isr_t0);
}


/*********************************************************************
** Profile velocity / acceleration for a profiled move, in user units.
*********************************************************************/
long SetVelAccDec(long axe, long vel_uu, long acc_uu, long dec_uu)
{
	SYS_COS_POS_SPEED(axe)  = (vel_uu);		// [uu/s]    0x6081
	SYS_COS_PROFIL_ACC(axe) = (acc_uu);		// [uu/s^2]  0x6083
	SYS_COS_PROFIL_DEC(axe) = (dec_uu);		// [uu/s^2]  0x6084
	Cvel(axe, SYS_INT_PROFIL_VEL(axe));
	Vel(axe,  SYS_INT_PROFIL_VEL(axe));
	Acc(axe,  SYS_INT_PROFIL_ACC(axe));
	Dec(axe,  SYS_INT_PROFIL_DEC(axe));
	return(0);
}


/*********************************************************************
** User units per axis.
**
** Arms:   1 UU = 1 centidegree.
** Stage:  1 UU = one C_AXIS_1T_FEEDDIST-th of an output revolution,
**         which is what 3R1T_Kin_1T.mh calls 0.01 mm and flags VERIFY.
** Pose:   3600 UU per revolution, so Cpos reads back exactly what was
**         commanded and no conversion is needed anywhere.
*********************************************************************/
void DoSettingsUserUnits(long axis_no)
{
	switch (axis_no) {
		case C_AXIS1:
		case C_AXIS2:
		case C_AXIS3:
			AXE_PARAM(axis_no, VELMAX) = 2000;
			AXE_PARAM(axis_no, VELRES) = 1000;
			SYS_COS_POS_ENC_QC(axis_no)   = C_AXIS_ENCRES;
			SYS_COS_POS_ENC_REV(axis_no)  = 1;
			SYS_COS_G1_MOTOR_REV(axis_no) = C_AXIS_POSFACT_Z;
			SYS_COS_G1_SHAFT_REV(axis_no) = C_AXIS_POSFACT_N;
			SYS_COS_G2_FEED(axis_no)      = C_AXIS_FEEDDIST;
			SYS_COS_G2_SHAFT_REV(axis_no) = C_AXIS_FEEDREV;
			SYS_COS_MAX_SPEED(axis_no)    = 5000;
#if (AXES_MODE == SIM_MODE)
			AXE_PARAM(axis_no, POSERR) = 0;
#endif
			break;

		case C_AXIS_1T:
			AXE_PARAM(axis_no, VELMAX) = 2000;
			AXE_PARAM(axis_no, VELRES) = 1000;
			SYS_COS_POS_ENC_QC(axis_no)   = C_AXIS_1T_POSENCQC;
			SYS_COS_POS_ENC_REV(axis_no)  = C_AXIS_1T_POSENCREV;
			SYS_COS_G1_MOTOR_REV(axis_no) = C_AXIS_1T_POSFACT_Z;
			SYS_COS_G1_SHAFT_REV(axis_no) = C_AXIS_1T_POSFACT_N;
			SYS_COS_G2_FEED(axis_no)      = C_AXIS_1T_FEEDDIST;
			SYS_COS_G2_SHAFT_REV(axis_no) = C_AXIS_1T_FEEDREV;
			SYS_COS_MAX_SPEED(axis_no)    = 5000;
#if (AXES_MODE == SIM_MODE)
			AXE_PARAM(axis_no, POSERR) = 0;
#endif
			break;

		// >>> THE THREE POSE AXES AND THE TOOL NO LONGER SHARE A CEILING. <<<
		//
		// They did while both were 1000. The pose axes went to 1500 and then
		// to 2000 on 2026-08-27, so that 20 deg/s can be run - a
		// viscous-friction sweep is a straight-line fit of torque against
		// speed, and the range of speed IS the leverage on the answer.
		// C_TRAJ_VEL_ROT_MAX in Config.mh moves with this; the two are one
		// thing in two places and a file fenced above the axis would run
		// slower than its own header claimed.
		//
		// >>> 2000 IS NOT A MEASURED CEILING. IT IS A CHOSEN ONE. <<<
		//
		// 20 deg/s of pose is about 1060 rpm at an arm, against the 5000 the
		// drives are commissioned for, so the motors are nowhere near it.
		// What has actually been run is 14. Anything above that is new
		// ground: watch for drive warnings and for the cable message, and
		// note that a FLAT theta_n sweep asks the stage for nothing at all
		// while a tilted move at this speed asks for a great deal.
		//
		// The TOOL was left at 1000. Nothing wants a faster needle, and the
		// carriage is the part of this rig with the least margin - it also
		// pays out whatever a tilt swallows, on top of whatever the tool
		// asks for, and StartPoseMove is already rationing it.
		case EE_PSI:
		case EE_PHI:
		case EE_THETA_N:
			AXE_PARAM(axis_no, VELMAX)   = 2000;
			AXE_PARAM(axis_no, VELRES)   = 1000;
			AXE_PARAM(axis_no, RAMPTYPE) = 0;
			AXE_PARAM(axis_no, RAMPMIN)  = 100;
			AXE_PARAM(axis_no, POSERR)   = 0;	// simulated: nothing to track
			SYS_COS_POS_ENC_QC(axis_no)   = EE_AXES_ENCRES;
			SYS_COS_POS_ENC_REV(axis_no)  = 1;
			SYS_COS_G1_MOTOR_REV(axis_no) = EE_AXES_POSFACT_Z;
			SYS_COS_G1_SHAFT_REV(axis_no) = EE_AXES_POSFACT_N;
			SYS_COS_G2_FEED(axis_no)      = EE_AXES_FEEDDIST;
			SYS_COS_G2_SHAFT_REV(axis_no) = EE_AXES_FEEDREV;
			SYS_COS_MAX_SPEED(axis_no)    = 2000;	// 20.00 deg/s
			break;

		case EE_TOOL:
			AXE_PARAM(axis_no, VELMAX)   = 2000;
			AXE_PARAM(axis_no, VELRES)   = 1000;
			AXE_PARAM(axis_no, RAMPTYPE) = 0;
			AXE_PARAM(axis_no, RAMPMIN)  = 100;
			AXE_PARAM(axis_no, POSERR)   = 0;	// simulated: nothing to track
			SYS_COS_POS_ENC_QC(axis_no)   = EE_AXES_ENCRES;
			SYS_COS_POS_ENC_REV(axis_no)  = 1;
			SYS_COS_G1_MOTOR_REV(axis_no) = EE_AXES_POSFACT_Z;
			SYS_COS_G1_SHAFT_REV(axis_no) = EE_AXES_POSFACT_N;
			SYS_COS_G2_FEED(axis_no)      = EE_AXES_FEEDDIST;
			SYS_COS_G2_SHAFT_REV(axis_no) = EE_AXES_FEEDREV;
			SYS_COS_MAX_SPEED(axis_no)    = 1000;	// 10.00 mm/s - held back
			break;
	}
}
