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
#include <string>

class SerialDevice
{
private:	
	//Device's name
	const std::string device_;	
	//Parity for input and output: EVEN, ODD, NONE
	const std::string parity_;	
	//BaudRate: 9600, 19200, 38400, 115200, 500000
	const int baudrate_;
	//Character size mask. Values are CS5, CS6, CS7, or CS8.
	const int datasize_;

	int serial_port_; // File descriptor

public:
	SerialDevice(const char *device, int baudrate,const char *parity, int datasize);	
	virtual ~SerialDevice();
	
	bool OpenPort();
	bool ClosePort();
	bool ReadPort( char *result, int bytes_to_read, int &bytes_read );

	const char* GetDevice() const { return device_.c_str(); }
	
private:
	//!	Set serial communication speed.
	bool SetTermSpeed(int speed);

};

#endif
