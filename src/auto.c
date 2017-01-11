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
* File for autonomous code.
*
* This file should contain the user autonomous() function and any functions related to it.
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

//set both sides of drive
void driveSet(int left, int right){
	lDriveSet(left);
	rDriveSet(right);
}

//set both sides of lil's drive
void tDriveSet(int left, int right){
	tlDriveSet(left);
	trDriveSet(right);
}

int stop=0;
//P controller for drving straight
void straight(long target, int faltDetect){
	long timeout = abs(target) * 4;
	long time = 0;
	int threshold = 20; //how close is good enough
	int driveCommand;
	float kP = 1.5;	//scale error for reasonable motor control values
	//calculate error
	long error = target - encoderGet(baseEnc);
	long initialError=error;
	//set drive proportional to error while greater than threshold
	while(abs(error) > threshold && time < timeout && stop==0){
		error = target - encoderGet(baseEnc);
		driveCommand = error*kP+117;
		if(driveCommand > 234)
			driveCommand = 234;
		else if(driveCommand < 0)
			driveCommand = 0;
		driveSet(trueBaseSpeed[driveCommand],trueBaseSpeed[driveCommand]);
		time += 20;
		printf("%d, ",encoderGet(baseEnc));
		delay(20);
	} //apply braking power
	if(time>=timeout && faltDetect == 1)
		stop=1;
	driveSet(-1*sign(velocityGet(0))*11, -1*sign(velocityGet(0))*11);
	encoderReset(baseEnc);
	delay(100);
}

//P controller for turning
void turn(long target){
	int threshold = 12;	//how close is good enough
	float kP = 2; //proportional constant
	long timeout = abs(target) * 3;
	long time = 0;
	int driveCommand;
	//calculate error
	long error = target - encoderGet(baseEnc);
	long initialError = error;
	//set drive proportional to error while above threshold
	while(abs(error) > threshold && time < timeout && !stop){
		error = target - encoderGet(baseEnc);
		driveCommand = kP*error+117;
		if(driveCommand > 234)
			driveCommand = 234;
		else if(driveCommand < 0)
			driveCommand = 0;
		driveSet(trueBaseSpeed[driveCommand],-trueBaseSpeed[driveCommand]);
		time += 20;
		printf("%d, ",encoderGet(baseEnc));
		delay(20);
	}
	//brake!
	driveSet(-1*sign(initialError)*11, 1*sign(initialError)*11);
	encoderReset(baseEnc);
	delay(100);
}

int stopt=0;
//P controller for drving straight
void tStraight(long target, int faltDetect){
	long timeout = abs(target) * 4;
	long time = 0;
	int threshold = 20; //how close is good enough
	int driveCommand;
	float kP = 1.5;	//scale error for reasonable motor control values
	//calculate error
	long error = target - encoderGet(tBaseEnc);
	long initialError=error;
	//set drive proportional to error while greater than threshold
	while(abs(error) > threshold && time < timeout && stop==0){
		error = target - encoderGet(tBaseEnc);
		driveCommand = error*kP+117;
		if(driveCommand > 234)
			driveCommand = 234;
		else if(driveCommand < 0)
			driveCommand = 0;
		tDriveSet(trueBaseSpeed[driveCommand],trueBaseSpeed[driveCommand]);
		time += 20;
		printf("%d, ",encoderGet(tBaseEnc));
		delay(20);
	} //apply braking power
	if(time>=timeout && faltDetect == 1)
		stop=1;
	driveSet(-1*sign(velocityGet(1))*11, -1*sign(velocityGet(1))*11);
	encoderReset(tBaseEnc);
	delay(100);
}

//P controller for turning
void tTurn(long target){
	int threshold = 12;	//how close is good enough
	float kP = 2; //proportional constant
	long timeout = abs(target) * 3;
	long time = 0;
	int driveCommand;
	//calculate error
	long error = target - encoderGet(tBaseEnc);
	long initialError = error;
	//set drive proportional to error while above threshold
	while(abs(error) > threshold && time < timeout && !stop){
		error = target - encoderGet(tBaseEnc);
		driveCommand = kP*error+117;
		if(driveCommand > 234)
			driveCommand = 234;
		else if(driveCommand < 0)
			driveCommand = 0;
		tDriveSet(trueBaseSpeed[driveCommand],-trueBaseSpeed[driveCommand]);
		time += 20;
		printf("%d, ",encoderGet(tBaseEnc));
		delay(20);
	}
	//brake!
	tDriveSet(-1*sign(initialError)*11, 1*sign(initialError)*11);
	encoderReset(tBaseEnc);
	delay(100);
}

//subroutine for destacking
void destack(int color){
	int time=0;
	//release bottom scoop
	tArmSet(80);
	delay(500);
	tArmSet(-80);
	delay(700);
	tArmSet(80);
	armSet(-80);
	delay(120);
	tArmSet(-8);
	armSet(-8);
	delay(100);
	//back off to destack
	driveSet(-127,-127);
	tDriveSet(-127,-127);
	delay(600);
	driveSet(0,0);
	tDriveSet(-127,-100);
	delay(2200);
	//turn 90 degrees away from each other
	tDriveSet(0,0);
	encoderReset(tBaseEnc);
	encoderReset(baseEnc);
	time=0;
	while(abs(encoderGet(baseEnc)) < 380 && time < 1200){
		driveSet(color*-127,color*127);
		time+=20;
		delay(20);
	}
	driveSet(color*12,color*-12);
	tArmSet(-100);
	delay(250);
	tArmSet(-12);
	time=0;
	while(abs(encoderGet(tBaseEnc)) < 600 && time < 1500){
			tDriveSet(color*127,color*-127);
			time+=20;
			delay(20);
		}
	tDriveSet(color*-12,color*12);
	//delay(500);
	//lil turns 90 degrees more
	driveSet(0,0);
	//tDriveSet(color*127,color*-127);
	//delay(500);
	//Lil drives away
	tDriveSet(127,127);
	delay(2200);
	tDriveSet(0,0);
//	tDriveSet(-127,-127);
//	delay(500);
//	//shake Lil's scoop down
//	tDriveSet(0,0);
//	tArmSet(-127);
//	delay(500);
//	tArmSet(-12);
//	tDriveSet(-127,-127);
//	delay(800);
//	tDriveSet(127,127);
//	delay(500);
//	tDriveSet(-127,-127);
//	delay(500);
//	tDriveSet(127,127);
//	delay(700);
//	tArmSet(0);
//	tDriveSet(0,0);
}

void philScore(int color){
//	driveSet(-127,-127);
//	delay(400);
//	driveSet(0,0);
	encoderReset(baseEnc);
	//raise arm to fence height
	armSet(100);
	delay(350);
	armSet(5);
	//drive into fence
	straight(1100,0);
	//backup, turn, and repeat
	straight(-300,1);
	turn(color*380);
	straight(500,1);
	turn(color*-360);
	straight(300,0);
	straight(-200,1);
	turn(color*360);
	straight(700,1);
	turn(color*-360);
	armSet(-100);
	delay(175);
	armSet(5);
	straight(200,0);
	straight(-100,1);
	turn(color*360);
	straight(700,1);
	turn(color*-360);
	straight(200,0);
}

//deploy intake, raise lift & drive to fence, outtake
//only a framework; will need to be adjusted on actual field
void standardAuton(){

}

void calBase(){
	for(int i = 0; i < 128; i++){
		driveSet(i,i);
		delay(2000);
		printf("%d,%f;", motorGet(flDrive), velocityGet(0));
		driveSet(-7,-7);
		delay(1000);
		driveSet(-i,-i);
		delay(2000);
		printf("%d,%f;", motorGet(flDrive), velocityGet(0));
		driveSet(7,7);
		delay(1000);
	}
}


/**
* Runs the user autonomous code.
*
* This function will be started in its own task with the default priority and stack size whenever the robot is enabled via the Field Management System or the VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart the task, not re-start it from where it left off.
*
* Code running in the autonomous task cannot access information from the VEX Joystick. However, the autonomous function can be invoked from another task if a VEX Competition Switch is not available, and it can access joystick information if called in this way.
*
* The autonomous task may exit, unlike operatorControl() which should never exit. If it does so, the robot will await a switch to another mode or disable/enable cycle.
*/
void autonomous() {
	delay(5000);
	tStraight(600,0);
	delay(5000);
	tStraight(-600,0);
	delay(5000);
}

