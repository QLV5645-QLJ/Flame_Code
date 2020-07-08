/*+==========================================================================
  File:      MTComm Example.cpp

  Summary:   Example code for reading MTi/MTx Data

             This code uses the Xsens MTComm Class v1.2
			 
  Functions: main
			   Gets user inputs, initializes MTComm class, reads MTi/MTx data from
			   COM port and writes to screen.

----------------------------------------------------------------------------
  This file is part of the Xsens SDK Code Samples.

  Copyright (C) Xsens Technologies B.V., 2005-2006.  All rights reserved.

  This source code is intended only as a supplement to Xsens
  Development Tools and/or documentation.  See these other
  materials for detailed information regarding Xsens code samples.

  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
  KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  PARTICULAR PURPOSE.
==========================================================================+*/

#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include "MTComm.h"

CMTComm mtcomm;
int portNumber;
char deviceName[15];
int outputMode;
int outputSettings;
unsigned short numDevices;
int screenSensorOffset = 0;

//////////////////////////////////////////////////////////////////////////
// gotoxy
//
// Sets the cursor position at the specified console position
//
// Input
//	 x	: New horizontal cursor position
//   y	: New vertical cursor position
void gotoxy(int x, int y)
{
#ifdef WIN32
	COORD coord;
	coord.X = x;
	coord.Y = y;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
#else
	char essq[100];		// String variable to hold the escape sequence
    char xstr[100];		// Strings to hold the x and y coordinates
    char ystr[100];		// Escape sequences must be built with characters

    /*
    ** Convert the screen coordinates to strings
    */
    sprintf(xstr, "%d", x);
    sprintf(ystr, "%d", y);

    /*
    ** Build the escape sequence (vertical move)
    */
    essq[0] = '\0';
    strcat(essq, "\033[");
    strcat(essq, ystr);

    /*
    ** Described in man terminfo as vpa=\E[%p1%dd
    ** Vertical position absolute
    */
    strcat(essq, "d");

    /*
    ** Horizontal move
    ** Horizontal position absolute
    */
    strcat(essq, "\033[");
    strcat(essq, xstr);
    // Described in man terminfo as hpa=\E[%p1%dG
    strcat(essq, "G");

    /*
    ** Execute the escape sequence
    ** This will move the cursor to x, y
    */
    printf("%s", essq);
#endif
}

//////////////////////////////////////////////////////////////////////////
// clrscr
//
// Clear console screen
void clrscr() 
{
#ifdef WIN32
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coord = {0, 0};
	DWORD count;
	
	GetConsoleScreenBufferInfo(hStdOut, &csbi);
	FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y, coord, &count);
	SetConsoleCursorPosition(hStdOut, coord);
#else
	int i;

    for (i = 0; i < 100; i++)
    // A bunch of new lines for now. It's blank, hey!
		putchar('\n');
	gotoxy(0,0);
#endif
}

#ifndef WIN32
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif
//////////////////////////////////////////////////////////////////////////
// getUserInputs
//
// Request user for output data
void getUserInputs()
{
	clrscr();

#ifdef WIN32
	printf("Enter COM port: ");
	scanf("%d", &portNumber);
#else
	printf("Enter device name (eg /dev/ttyS0 or /dev/ttyUSB0): ");
	scanf("%s", deviceName);
#endif
	clrscr();

	do{
		printf("Select desired output:\n");
		printf("1 - Calibrated data\n");
		printf("2 - Orientation data\n");
		printf("3 - Both Calibrated and Orientation data\n");
		printf("Enter your choice: ");
		scanf("%d", &outputMode);
		// flush stdin
		while (getchar() != '\n') continue;

		if (outputMode < 1 || outputMode > 3) {
			printf("\n\nPlease enter a valid output mode\n");
		}
	}while(outputMode < 1 || outputMode > 3);
	clrscr();

	// Update outputMode to match data specs of SetOutputMode
	outputMode <<= 1;

	if ((outputMode & OUTPUTMODE_ORIENT) != 0) {
		do{
			printf("Select desired output format\n");
			printf("1 - Quaternions\n");
			printf("2 - Euler angles\n");
			printf("3 - Matrix\n");
			printf("Enter your choice: ");
			scanf("%d", &outputSettings);
			// flush stdin
			while (getchar() != '\n') continue;

			if (outputSettings < 1  || outputSettings > 3) {
				printf("\n\nPlease enter a valid choice\n");
			}
		}while(outputSettings < 1 || outputSettings > 3);

		// Update outputSettings to match data specs of SetOutputSettings
		switch(outputSettings) {
		case 1:
			outputSettings = OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
			break;
		case 2:
			outputSettings = OUTPUTSETTINGS_ORIENTMODE_EULER;
			break;
		case 3:
			outputSettings = OUTPUTSETTINGS_ORIENTMODE_MATRIX;
			break;
		}
	}
	else{
		outputSettings = 0;
	}
	outputSettings |= OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
	clrscr();
}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
bool doMtSettings(void) 
{
	unsigned long tmpOutputMode, tmpOutputSettings;
	unsigned short tmpDataLength;

	// Put MTi/MTx in Config State
	if(mtcomm.writeMessage(MID_GOTOCONFIG) != MTRV_OK){
		printf("No device connected\n");
		return false;
	}

	// Get current settings and check if Xbus Master is connected
	if (mtcomm.getDeviceMode(&numDevices) != MTRV_OK) {
		if (numDevices == 1)
			printf("MTi / MTx has not been detected\nCould not get device mode\n");
		else
			printf("Not just MTi / MTx connected to Xbus\nCould not get all device modes\n");
		return false;
	}
	
	// Check if Xbus Master is connected
	mtcomm.getMode(tmpOutputMode, tmpOutputSettings, tmpDataLength, BID_MASTER);
	if (tmpOutputMode == OUTPUTMODE_XM)	{
		// If Xbus Master is connected, attached Motion Trackers should not send sample counter
		outputSettings &= 0xFFFFFFFF - OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
	}
	
	// Set output mode and output settings for each attached MTi/MTx
	for (int i = 0; i < numDevices; i++) {
		if (mtcomm.setDeviceMode(outputMode, outputSettings, BID_MT + i) != MTRV_OK) {
			printf("Could not set (all) device mode(s)\n");
			return false;
		}
	}

	// Put MTi/MTx in Measurement State
	mtcomm.writeMessage(MID_GOTOMEASUREMENT);

	return true;
}

//////////////////////////////////////////////////////////////////////////
// writeHeaders
//
// Write appropriate headers to screen
void writeHeaders()
{
	for (int i = 0; i < numDevices; i++) {	
		gotoxy(0, 2 + i * screenSensorOffset);
		printf("Sensor %d\n", i + 1);
		if ((outputMode & OUTPUTMODE_CALIB) != 0) {
			gotoxy(0,3 + i * screenSensorOffset);
			printf("Calibrated sensor data");
			gotoxy(0,4 + i * screenSensorOffset);
			printf(" Acc X\t Acc Y\t Acc Z");
			gotoxy(23, 5 + i * screenSensorOffset);
			printf("(m/s^2)");
			gotoxy(0,6 + i * screenSensorOffset);
			printf(" Gyr X\t Gyr Y\t Gyr Z");
			gotoxy(23, 7 + i * screenSensorOffset);
			printf("(rad/s)");
			gotoxy(0,8 + i * screenSensorOffset);
			printf(" Mag X\t Mag Y\t Mag Z");
			gotoxy(23, 9 + i * screenSensorOffset);
			printf("(a.u.)");
			gotoxy(0,11 + i * screenSensorOffset);
		}

		if ((outputMode & OUTPUTMODE_ORIENT) != 0) {
			printf("Orientation data\n");
			switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK) {
			case OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				printf("    q0\t    q1\t    q2\t    q3\n");
				break;
			case OUTPUTSETTINGS_ORIENTMODE_EULER:
				printf("  Roll\t Pitch\t   Yaw\n");
				printf("                       degrees\n");
				break;
			case OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				printf(" Matrix\n");
				break;
			default:
				;
			}			
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// calcScreenOffset
//
// Calculates offset for screen data with multiple MTx on Xbus Master
void calcScreenOffset()
{
	// 1 line for "Sensor ..."
	screenSensorOffset += 1;
	if ((outputMode & OUTPUTMODE_CALIB) != 0)
		screenSensorOffset += 8;
	if ((outputMode & OUTPUTMODE_ORIENT) != 0) {
		switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK) {
		case OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
			screenSensorOffset += 4;
			break;
		case OUTPUTSETTINGS_ORIENTMODE_EULER:
			screenSensorOffset += 4;
			break;
		case OUTPUTSETTINGS_ORIENTMODE_MATRIX:
			screenSensorOffset += 6;
			break;
		default:
			;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// main
// 
// Example program for setting MTi/MTx settings and reading MTi/MTx data
//
int main(int argc, char *argv[]) 
{
	unsigned char data[MAXMSGLEN];
	short datalen;
	float fdata[18] = {0};
	unsigned short samplecounter;

	// Skip factor for writing data to screen, make screen seem a bit smoother
	short screenSkipFactor = 10;
	short screenSkipFactorCnt = screenSkipFactor;

	getUserInputs();	

	// Open and initialize serial port
#ifdef WIN32
	if (mtcomm.openPort(portNumber) != MTRV_OK) {
		printf("Cannot open COM port %d\n", portNumber);
#else
	if (mtcomm.openPort(deviceName) != MTRV_OK) {
		printf("Cannot open COM port %s\n", deviceName);
#endif
		return MTRV_INPUTCANNOTBEOPENED;
	}	

	if(doMtSettings() == false)
		return MTRV_UNEXPECTEDMSG;

	clrscr();

	calcScreenOffset();

	writeHeaders();
	
	while(mtcomm.readDataMessage(data, datalen) == MTRV_OK && !_kbhit()) {
		mtcomm.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);
		
		gotoxy(0,0);
		printf("Sample Counter %05d\n", samplecounter);
		
		if (screenSkipFactorCnt++ == screenSkipFactor) {
			screenSkipFactorCnt = 0;
			for (int i = 0; i < numDevices; i++) {
				gotoxy(0,5 + i * screenSensorOffset);			
				if ((outputMode & OUTPUTMODE_CALIB) != 0) {
					// Output Calibrated data
					mtcomm.getValue(VALUE_CALIB_ACC, fdata, data, BID_MT + i);
					printf("%6.2f\t%6.2f\t%6.2f", fdata[0], 
												  fdata[1], 
												  fdata[2]);
					gotoxy(0,7 + i * screenSensorOffset);
					mtcomm.getValue(VALUE_CALIB_GYR, fdata, data, BID_MT + i);
					printf("%6.2f\t%6.2f\t%6.2f", fdata[0], 
												  fdata[1], 
												  fdata[2]);
					gotoxy(0,9 + i * screenSensorOffset);
					mtcomm.getValue(VALUE_CALIB_MAG, fdata, data, BID_MT + i);
					printf("%6.2f\t%6.2f\t%6.2f", fdata[0], 
												  fdata[1], 
												  fdata[2]);
					gotoxy(0,13 + i * screenSensorOffset);
				}

				if ((outputMode & OUTPUTMODE_ORIENT) != 0) {
					switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK) {
					case OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
						// Output: quaternion
						mtcomm.getValue(VALUE_ORIENT_QUAT, fdata, data, BID_MT + i);
						printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n",
								fdata[0],
								fdata[1], 
								fdata[2], 
								fdata[3]);
						break;
					case OUTPUTSETTINGS_ORIENTMODE_EULER:
						// Output: Euler
						mtcomm.getValue(VALUE_ORIENT_EULER, fdata, data, BID_MT + i);
						printf("%6.1f\t%6.1f\t%6.1f\n",
								fdata[0],
								fdata[1], 
								fdata[2]);
						break;
					case OUTPUTSETTINGS_ORIENTMODE_MATRIX:
						// Output: Cosine Matrix
						mtcomm.getValue(VALUE_ORIENT_MATRIX, fdata, data, BID_MT + i);
						printf("%6.3f\t%6.3f\t%6.3f\n",fdata[0], 
													   fdata[1], 
													   fdata[2]);
						printf("%6.3f\t%6.3f\t%6.3f\n",fdata[3],
													   fdata[4], 
													   fdata[5]);
						printf("%6.3f\t%6.3f\t%6.3f\n",fdata[6], 
													   fdata[7], 
													   fdata[8]);
						break;
					default:
						;
					}
				}
			}
		}
	}

	// When done, close the serial port
	mtcomm.close();

	return MTRV_OK;
}
