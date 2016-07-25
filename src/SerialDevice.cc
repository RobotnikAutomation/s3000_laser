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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SerialDevice::SerialDevice( const std::string& port, int baudrate, const std::string& parity, int datasize)
: port_( port )
, parity_( parity )
, baudrate_( baudrate )
, datasize_( datasize )
{		
	ROS_INFO_STREAM( "SerialDevice: " << port 
	    << " Parity= " << parity 
	    << " DataSize=" << datasize 
	    << " BaudRate=" << baudrate );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SerialDevice::~SerialDevice() 
{
    ClosePort();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SerialDevice::OpenPort()
{
	serial_port_ = open( port_.c_str(),  O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK );
	
	if( serial_port_ == -1 )
	{
		ROS_WARN( "Error opening serial port=%s", port_.c_str() );
		return false;  // invalid port file	
	}
	
	// set up comm flags
	struct termios comms_flags;
	memset( &comms_flags, 0, sizeof(termios) );
	
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
			ROS_WARN( "unsupported datasize=%d", datasize_ );
		    return false;
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
	
	tcsetattr( serial_port_, TCSANOW, &comms_flags );
	tcflush( serial_port_, TCIOFLUSH );
		
	if ( SetTermSpeed(baudrate_) == false )
	    return false;
	
	// Make sure queue is empty
	tcflush( serial_port_, TCIOFLUSH );
	return true;
	
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SerialDevice::ClosePort()
{
	return close(serial_port_) == 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SerialDevice::ReadPort( char *buffer, int bytes_to_read, int &bytes_read ) 
{
	bytes_read = read( serial_port_, buffer, bytes_to_read );

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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SerialDevice::SetTermSpeed(int baudrate)
{
	int baudrate_flag;
	
	switch(baudrate)
	{
		case 9600:
			baudrate_flag = B9600;
			break;
		case 19200:
			baudrate_flag = B19200;
			break;
		case 38400:
			baudrate_flag = B38400;
			break;
		case 115200:
			baudrate_flag = B115200;
			break;
		case 500000:
			baudrate_flag = B500000;
			break;
		default:
		    ROS_WARN("unsupported baudrate=%d", baudrate );
			return false;
	}
	
	struct termios comms_flags;

	if( tcgetattr( serial_port_, &comms_flags ) < 0 )
		return false;
	
	if( cfsetispeed( &comms_flags, baudrate_flag ) < 0 || cfsetospeed( &comms_flags, baudrate_flag ) < 0)
		return false;
	
	if( tcsetattr( serial_port_, TCSAFLUSH, &comms_flags ) < 0 )
		return false;

	return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

