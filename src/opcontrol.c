/*
 * 	Copyright 2016 Jacob Knaup
 *
 * 	This file is part of the firmware for ALBA's Open Source Starstruck Robot and is built upon the PURDUE ROBOTICS OS (PROS).
 *
 * 	The firmware for ALBA's Open Source Starstruck Robot is free software: you can redistribute it and/or modify
 * 	it under the terms of the GNU General Public License as published by
 * 	the Free Software Foundation, either version 3 of the License, or
 * 	(at your option) any later version, while keeping with the terms of the license for PROS.
 *
 * 	The firmware for ALBA's Open Source Starstruck Robot is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 * 	You should have received a copy of the GNU General Public License
 * 	along with The firmware for ALBA's Open Source Starstruck Robot.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * File for operator control code.
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Purdue University ACM SIG BOTS nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 ********************************************************************************/

#include "main.h"

//functions to set our subsystem motors to control values. Some motors need to be reversed
void lDriveSet(int control) {
	motorSet(flDrive, control);
	motorSet(alDrive, -control);
}
void rDriveSet(int control) {
	motorSet(frDrive, -control);
	motorSet(arDrive, control);
}
void armSet(int control) {
	motorSet(lArm, control);
	motorSet(rArm, -control);
}

void tlDriveSet(int control) {
	motorSet(tlDrive, control);
}
void trDriveSet(int control) {
	motorSet(trDrive, -control);
}
void tArmSet(int control) {
	motorSet(tArm, control);
}

#define recordLength 1500
typedef struct motors {
	char leftDrive[recordLength];
	char rightDrive[recordLength];
	char lift[recordLength];
	char claw[recordLength];
} motors;

motors phil;
motors lil;

void rerunSave() {
	FILE *my_auto = fopen("rer", "w");
	for (int i = 0; i < recordLength; i++) {
		fputc(phil.leftDrive[i], my_auto);
		//fprint(", ", my_auto);
	}
	//fprint("\n rightDrive \n", my_auto);
	for (int i = 0; i < recordLength; i++) {
		fputc(phil.rightDrive[i], my_auto);
	}
	//fprint("\n lift \n", my_auto);
	for (int i = 0; i < recordLength; i++) {
		fputc(phil.lift[i], my_auto);
	}
	//fprint("\n claw \n", my_auto);
	for (int i = 0; i < recordLength; i++) {
		fputc(phil.claw[i], my_auto);
	}
	fclose(my_auto);
}

void rerunSaveLil(){
	FILE *my_auto = fopen("rerl", "w");
	for (int i = 0; i < recordLength; i++) {
		fputc(lil.leftDrive[i], my_auto);
		//fprint(", ", my_auto);
	}
	//fprint("\n rightDrive \n", my_auto);
	for (int i = 0; i < recordLength; i++) {
		fputc(lil.rightDrive[i], my_auto);
	}
	//fprint("\n lift \n", my_auto);
	for (int i = 0; i < recordLength; i++) {
		fputc(lil.lift[i], my_auto);
	}
	//fprint("\n claw \n", my_auto);
	for (int i = 0; i < recordLength; i++) {
		fputc(lil.claw[i], my_auto);
	}
	fclose(my_auto);
}

void rerunRecord(void * parameter) {
	while (true) {
		for (int i = 0; i < recordLength; i++) {
			phil.leftDrive[i] = motorGet(flDrive);
			phil.rightDrive[i] = motorGet(arDrive);
			phil.lift[i] = motorGet(lArm);
			phil.claw[i] = digitalRead(RPneuAssist);
			delay(20);
		}
		print("start");
		rerunSave();
		print("done");
		taskDelete(rerunRecordHandle);
	}
}

void rerunRecordLil(void * parameter) {
	while (true) {
		for (int i = 0; i < recordLength; i++) {
			lil.leftDrive[i] = motorGet(tlDrive);
			lil.rightDrive[i] = -motorGet(trDrive);
			lil.lift[i] = motorGet(tArm);
			lil.claw[i] = digitalRead(TPneuAssist);
			delay(20);
		}
		print("start");
		rerunSaveLil();
		print("done");
		taskDelete(rerunRecordLilHandle);
	}
}

char loadedLeft[recordLength];
char loadedRight[recordLength];
char loadedLift[recordLength];
char loadedClaw[recordLength];

void rerunLoad() {
	FILE *my_auto = fopen("rer", "r");
	for (int i = 0; i < recordLength; i++) {
		loadedLeft[i] = (signed char) fgetc(my_auto);
	}
	for (int i = 0; i < recordLength; i++) {
		loadedRight[i] = (signed char) fgetc(my_auto);
	}
	for (int i = 0; i < recordLength; i++) {
		loadedLift[i] = (signed char) fgetc(my_auto);
	}
	for (int i = 0; i < recordLength; i++) {
		loadedClaw[i] = (signed char) fgetc(my_auto);
	}
	fclose(my_auto);
}

char loadedLLeft[recordLength];
char loadedLRight[recordLength];
char loadedLLift[recordLength];
char loadedLClaw[recordLength];

void rerunLoadLil() {
	FILE *my_auto = fopen("rerl", "r");
	for (int i = 0; i < recordLength; i++) {
		loadedLLeft[i] = (signed char) fgetc(my_auto);
	}
	for (int i = 0; i < recordLength; i++) {
		loadedLRight[i] = (signed char) fgetc(my_auto);
	}
	for (int i = 0; i < recordLength; i++) {
		loadedLLift[i] = (signed char) fgetc(my_auto);
	}
	for (int i = 0; i < recordLength; i++) {
		loadedLClaw[i] = (signed char) fgetc(my_auto);
	}
	fclose(my_auto);
}

void rerunReplay() {
	for (int i = 0; i < recordLength; i++) {
		lDriveSet(loadedLeft[i]);
		rDriveSet(loadedRight[i]);
		armSet(loadedLift[i]);
		clawSet(loadedClaw[i]);
		delay(20);
	}
}

void rerunReplayLil() {
	for (int i = 0; i < recordLength; i++) {
		tlDriveSet(loadedLLeft[i]);
		trDriveSet(loadedLRight[i]);
		tArmSet(loadedLLift[i]);
		tClawSet(loadedLClaw[i]);
		delay(20);
	}
}

/**
 * Runs the user operator control code.
 *
 * This function will be started in its own task with the default priority and stack size whenever the robot is enabled via the Field Management System or the VEX Competition Switch in the operator control mode. If the robot is disabled or communications is lost, the operator control task will be stopped by the kernel. Re-enabling the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will run the operator control task. Be warned that this will also occur if the VEX Cortex is tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
//********AUTON PRACTICE************
//	while(powerLevelBackup() < 100){
//		delay(20);
//	}
//	delay(2000);
//	autonomous();
//	while(1)
//		delay(20);
//variables to hold previous arm directions

	short armLast = 0;
	short armTLast = 0;

//	while (1) {
//		if (V) {
//			rerunRecordHandle = taskCreate(rerunRecord, TASK_DEFAULT_STACK_SIZE,
//					NULL,
//					TASK_PRIORITY_DEFAULT);
//			break;
//		} else if (X) {
//			rerunLoad();
//			rerunReplay();
//		} else if (P_V) {
//			rerunRecordLilHandle = taskCreate(rerunRecordLil, TASK_DEFAULT_STACK_SIZE,
//					NULL,
//					TASK_PRIORITY_DEFAULT);
//			break;
//		}
//		else if(P_X){
//			rerunLoadLil();
//			rerunReplayLil();
//		}
//		else if(O){
//			rerunLoad();
//			rerunLoadLil();
//			autonomous();
//		}
//	}

//begin tele-op loop
	while (true) {
		//set drive motors with a deadband of 5
		if (abs(L_JOY) > 5)
			lDriveSet(L_JOY);
		else
			lDriveSet(0);
		if (abs(R_JOY) > 5)
			rDriveSet(R_JOY);
		else
			rDriveSet(0);

		//set lift motors, 2 different speeds; apply holding power of 12
		if (L1)
			armSet(127);
		else if (L2)
			armSet(-127);
		else if (armLast < 0)
			armSet(-10);
		else if (armLast > 0)
			armSet(10);
		else
			armSet(0);

		//Phil pneu claw
		if (R2) {
			clawSet(OPEN);
		} else if (R1) {
			clawSet(CLOSE);
		}

		//save previous direction of arm
		armLast = motorGet(lArm);

//---------------------------------------------------------------------------------------

//base control for second base; deadband of 5 (same as above)
		if (abs(P_L_JOY) > 10)
			tlDriveSet(P_L_JOY);
		else
			tlDriveSet(0);
		if (abs(P_R_JOY) > 10)
			trDriveSet(P_R_JOY);
		else
			trDriveSet(0);

		//arm control of second base (same as above)
		if (P_L1)
			tArmSet(127);
		else if (P_L2)
			tArmSet(-127);
		else if (P_X)
			tArmSet(0);
		else if (armTLast == -127 || armTLast == -10)
			tArmSet(-10);
		else if (armTLast == 127 || armTLast == 10)
			tArmSet(10);
		else
			tArmSet(0);

		//pneu claw for lil
		if (P_R2)
			tClawSet(OPEN);
		else if (P_R1)
			tClawSet(CLOSE);

		//record last direction
		armTLast = motorGet(tArm);

		//debugging/auton practice

		//printf("%d, ",encoderGet(tBaseEnc));

		//auton practice without competition switch
		//if(joystickGetDigital(1,8,JOY_UP)){
		//	encoderReset(baseEnc);
		//	autonomous();
		//}
//		if(joystickGetDigital(1,8,JOY_DOWN)){
//			encoderReset(baseEnc);
//			encoderReset(tBaseEnc);
//		}

		delay(20);
	}
}
