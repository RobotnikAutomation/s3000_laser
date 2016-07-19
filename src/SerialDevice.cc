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
/** \file SerialDevice.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.1
 * \date    2009
 *
 * \brief SerialDevice Class 
 * (C) 2009 Robotnik Automation, SLL
 * Class to manage a serial connection
*/

#include <s3000_laser/SerialDevice.h>
#include <time.h>
#include <string.h> 
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/stat.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <iostream>
#include <errno.h>   /* Error number definitions */
#include <stropts.h>
#include <ros/ros.h>

/*! \fn SerialDevice::SerialDevice()
 	* Constructor by default
*/
SerialDevice::SerialDevice(const char *device, int baudrate, const char *parity, int datasize)
: device_( device )
, parity_( parity )
, baudrate_( baudrate )
, datasize_( datasize )
{		
	ROS_INFO_STREAM( "SerialDevice: " << device 
	    << " Parity= " << parity 
	    << " DataSize=" << datasize 
	    << " BaudRate=" << baudrate );
}

/*! \fn int SerialDevice::OpenPort2(char *dev)
 	* Opens serial port for communication
*/
bool SerialDevice::OpenPort()
{
	fd_ = open(device_.c_str(),  O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	
	if( fd_ == -1 )
	{
		ROS_WARN( "Error opening serial device=%s", device_.c_str() );
		return false;  // invalid device file	
	}
	
	// set up comm flags
	struct termios comms_flags;
	memset(&comms_flags, 0,sizeof(termios));
	
	switch(datasize_)
	{
		case 5:
			comms_flags.c_cflag = CS5 | CREAD;
		    break;
		case 6:
			comms_flags.c_cflag = CS6 | CREAD;
		    break;
		case 7:
			comms_flags.c_cflag = CS7 | CREAD;
		    break;
		case 8:
			comms_flags.c_cflag = CS8 | CREAD;
		    break;
		default:
			comms_flags.c_cflag = CS8 | CREAD;
		    break;
	}
	
	comms_flags.c_iflag = INPCK;
	comms_flags.c_oflag = 0;
	comms_flags.c_lflag = 0;
	
	if( parity_ == "even" )
	{
		comms_flags.c_cflag |= PARENB;
	}
	else if( parity_ == "odd" )
	{
		comms_flags.c_cflag |= PARODD;
	}
	else if( parity_ == "none" )
	{
		comms_flags.c_cflag &= ~PARODD;
		comms_flags.c_cflag &= ~PARENB;
	}
	
	comms_flags.c_lflag &= ~ICANON;	// TEST
	
	tcsetattr(fd_, TCSANOW, &comms_flags);
	tcflush(fd_, TCIOFLUSH);
		
	if (SetTermSpeed(baudrate_) == false)
	    return false;
	
	// Make sure queue is empty
	tcflush(fd_, TCIOFLUSH);
	usleep(1000);
	tcflush(fd_, TCIFLUSH);
	
	return true;
	
}

/*! \fn int SerialDevice::ClosePort()
 	* Closes serial port
*/
bool SerialDevice::ClosePort()
{
	return close(fd_) == 0;
}

/*!	\fn int SerialDevice::ReadPort(char *result, int *bytes_read, int bytes_to_read)
 * @brief Reads serial port
 * @param result as char *, output buffer
 * @param bytes_read as int, number of read bytes
 * @param bytes_to_read as *int, number of desired bytes to read
 * @return OK
 * @return NOT_INITIALIZED
 * @return ERROR
*/
bool SerialDevice::ReadPort(char *result, int bytes_to_read, int &bytes_read ) 
{
	bytes_read = read( fd_, result, bytes_to_read );

    if( bytes_read >= 0 ) 
    {
        return true;
    }
    else if ( errno == EAGAIN ) 
    {
        return true; // nothing to read
    }
    else
    {
        ROS_WARN("SERIAL read error %d %s", errno, strerror(errno) );
        return false;
    }
}

/*!	\fn int SerialDevice::SetTermSpeed(int speed)
	* Set serial communication speed.
	* Valid values: 9600, 19200, 38400, 115200
	* @return false if error occured
*/
bool SerialDevice::SetTermSpeed(int baudrate)
{
	int term_speed;
	
	switch(baudrate)
	{
		case 9600:
			term_speed = B9600;
			break;
		case 19200:
			term_speed = B19200;
			break;
		case 38400:
			term_speed = B38400;
			break;
		case 115200:
			term_speed = B115200;
			break;
		case 500000:
			term_speed = B500000;
			break;
		default:
		    ROS_WARN("unsupported baudrate=%d", baudrate );
			return false;
	}
	
	struct termios term;

	if( tcgetattr( fd_, &term ) < 0 )
		return false;
	
	if( cfsetispeed( &term, term_speed ) < 0 || cfsetospeed( &term, term_speed ) < 0)
		return false;
	
	if( tcsetattr( fd_, TCSAFLUSH, &term ) < 0 )
		return false;

	return true;
}

