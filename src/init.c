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
 * File for initialization code.
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * Copyright(c) 2011 - 2014, Purdue University ACM SIG BOTS.All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met :
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and / or other materials provided with the distribution.
 * Neither the name of Purdue University ACM SIG BOTS nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS(http://www.freertos.org) whose source code may be obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/**
 * Runs pre-initialization code.
 *
 * This function will be started in kernel mode one time while the VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
	//tether pneumatic solenoids
	digitalWrite(1, LOW);
	pinMode(1, OUTPUT);
	//left and right pneumatic solenoids
	digitalWrite(6, LOW);
	pinMode(6, OUTPUT);
	digitalWrite(7, LOW);
	pinMode(7, OUTPUT);
	//jumper positions
	pinMode(11, INPUT);
	pinMode(12, INPUT);

}

/**
 * Runs user initialization code.
 *
 * This function will be started in its own task with the default priority and stack size once when the robot is starting up. It is possible that the VEXnet communication link may not be fully established at this time, so reading from the VEX Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks will not start. An autonomous mode selection menu like the pre_auton() in other environments can be implemented in this task if desired.
 */

//initialize encoder
Encoder baseEnc;
Encoder tBaseEnc;

void initialize() {
	baseEnc = encoderInit(4, 5, false);
	tBaseEnc = encoderInit(2, 3, true);

	//TaskHandle velocityTaskHandle = taskCreate(velocityUpdate,
			//TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
}

float bVelocity = 0, tVelocity = 0;
void velocityUpdate(void * parameter) {
	int bEncLast, tEncLast, bEncDelta, tEncDelta;
	int bTimeLast, tTimeLast, bTimeDelta, tTimeDelta;
	while (true) {
		bEncLast = encoderGet(baseEnc);
		bTimeLast = micros();
		tEncLast = encoderGet(tBaseEnc);
		tTimeLast = micros();
		delay(200);
		bEncDelta = encoderGet(baseEnc) - bEncLast;
		bTimeDelta = micros() - bTimeLast;
		tEncDelta = encoderGet(tBaseEnc) - tEncLast;
		tTimeDelta = micros() - tTimeLast;
		bVelocity = bVelocity * .5 + .5 * bEncDelta / bTimeDelta;
		tVelocity = tVelocity * .5 + .5 * tEncDelta / tTimeDelta;
		//printf("%f, ", bVelocity);
	}
}

float velocityGet(bool tether) {
	if (tether)
		return tVelocity;
	else
		return bVelocity;
}

//return sign of a variable
int sign(int var) {
	return var / abs(var);
}

const int trueBaseSpeed[235] = { -110, -108, -106, -104, -102, -101, -99, -97,
		-95, -93, -91, -90, -88, -86, -85, -83, -81, -80, -78, -77, -75, -74,
		-72, -71, -69, -68, -66, -65, -64, -62, -61, -60, -58, -57, -56, -55,
		-53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40,
		-39, -38, -37, -36, -35, -34, -33, -32, -32, -31, -30, -29, -28, -28,
		-27, -26, -25, -25, -24, -23, -23, -22, -21, -21, -20, -19, -19, -18,
		-18, -17, -16, -16, -15, -15, -14, 14, -13, -13, -12, -12, -11, -11,
		-10, -10, -9, -9, -8, -8, -7, -7, -7, -6, 6, -5, -5, -4, -4, -4, -3, -3,
		-2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6,
		6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15,
		16, 16, 17, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 26,
		26, 27, 28, 28, 29, 30, 31, 31, 32, 33, 34, 35, 35, 36, 37, 38, 39, 40,
		41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 56, 57, 58, 59,
		61, 62, 63, 64, 66, 67, 68, 70, 71, 73, 74, 76, 77, 79, 80, 82, 83, 85,
		87, 88, 90, 92, 93, 95, 97, 99, 101, 103, 104 };
