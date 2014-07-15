/*
*  Copyright (c) 2012, Robotnik Automation, SLL
* 
*   This file is part of sick-s3000-ros-pkg.
*
*   sick-s3000-ros-pkg is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/** \file SerialDevice.h
 * \author Robotnik Automation S.L.L.
 * \version 1.1
 * \date    2009
 *
 * \brief SerialDevice Class 
 * (C) 2009 Robotnik Automation, SLL
 * Class to manage a serial port connection
*/

#ifndef __SERIALDEV_H
#define __SERIALDEV_H
	
#include <stdio.h>

#define SERIAL_OK				0
#define SERIAL_ERROR				-2

#define BUFFER					254
#define DEFAULT_PORT 				"/dev/ttyS1"
#define DEFAULT_TRANSFERRATE 			19200
#define DEFAULT_PARITY 				"odd" //"even" "odd" "none"
#define DEFAULT_SIZE_ARRAY			128
#define DEFAULT_DATA_SIZE			7

class SerialDevice{

private:	
	/**
		* File descriptor
	*/
	int fd;
	//
	bool bReady;	
	//
	unsigned char SendBuf[BUFFER];		
	//
	unsigned char RecBuf[BUFFER];
	//Device's name
	char cDevice[DEFAULT_SIZE_ARRAY];	
	//Parity for input and output: EVEN, ODD, NONE
	char cParity[DEFAULT_SIZE_ARRAY];	
	//BaudRate: 9600, 19200, 38400, 115200
	int iBaudRate;
	//Character size mask. Values are CS5, CS6, CS7, or CS8.
	int iBitDataSize;
	//Entrada canónica-> La entrada canónica es orientada a línea. Los caracteres se meten en un buffer hasta recibir un CR o LF.
	bool bCanon;
	
public:
	// Public Constructor
	SerialDevice(void);
	// Public Constructor
	SerialDevice(const char *device, int baudrate,const char *parity, int datasize);	
	// Public Destructor
	~SerialDevice(void);	
	int OpenPort1(void);
	int OpenPort2(void);
	// Closes serial port
	int ClosePort();
	// Receive commands from SerialDevice
	// @return number of read bytes
	int ReceiveMessage(char *msg);
	// Send commands to the SerialDevice
	// @return number of sent bytes 
 	int SendMessage(char *msg, int length);
	//int SendMessage(char *msg);	
	int WritePort(char *chars, int length);
	int WritePort(char *chars, int *written_bytes, int length);	// Importat del SerialDevice amb Component	
	int ReadPort(char *result);	
	int ReadPort(char *result, int num_bytes);
	int ReadPort(char *result, int *read_bytes, int num_bytes);  	// Importat del SerialDevice amb Component
	//! Clean port buffer
	int Flush();	
	//! Returns opened device
	char *GetDevice();
	//! Sets the Canonical input. False by default
	void SetCanonicalInput(bool value);
	
private:
	
	/**
		* Set serial communication speed.
		* Valid values: 9600, 19200, 38400, 115200
		* @return SERIALDEV_OK
		* @return SERIALDEV_ERROR
	*/	
	int InitPort(void);
	int GetBaud(void);
	//!	Set serial communication speed.
	int SetTermSpeed(int speed);

};

#endif
