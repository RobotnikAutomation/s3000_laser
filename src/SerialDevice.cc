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

/*! \fn SerialDevice::SerialDevice(void)
 	* Constructor by default
*/
SerialDevice::SerialDevice(void){	
	
	strcpy(cDevice,DEFAULT_PORT);
	strcpy(cParity,DEFAULT_PARITY);
	iBaudRate=DEFAULT_TRANSFERRATE;
	iBitDataSize = DEFAULT_DATA_SIZE;
	bReady = false;	
	bCanon = false;
}


/*! \fn SerialDevice::SerialDevice(void)
 	* Constructor by default
*/
SerialDevice::SerialDevice(const char *device, int baudrate, const char *parity, int datasize){	
	
	strcpy(cDevice,device);
	std::cout << "SerialDevice::SerialDevice: " << cDevice << " Parity= " << parity 
	<< " DataSize=" << datasize <<" BaudRate=" << baudrate << std::endl;
	strcpy(cParity,parity);
	iBaudRate=baudrate;
	iBitDataSize = datasize;
	bReady = false;	
	bCanon = false;
}

/*! \fn SerialDevice::~SerialDevice(void)
 	* Destructor by default
*/
SerialDevice::~SerialDevice(void){
	//std::cout << "SerialDevice::~SerialDevice" << std::endl;
}

/*! \fn SerialDevice::InitPort()
 	* Initialize port
*/

int SerialDevice::InitPort() {
	struct termios options;

	// set up comm flags
	//memset(&options, 0,sizeof(options));
	// Get the current options for the port...
	if ( tcgetattr(fd, &options) < 0) {
		//ROS_ERROR("unable to get device attributes");
		return -1;
		}

	// Set the baud rates to X
	switch(iBaudRate){
		case 9600:
			cfsetispeed(&options, B9600);
			cfsetospeed(&options, B9600);
		break;
		case 19200:
			cfsetispeed(&options, B19200);
			cfsetospeed(&options, B19200);
		break;
		case 38400:
			cfsetispeed(&options, B38400);
			cfsetospeed(&options, B38400);
		break;
		case 115200:
			cfsetispeed(&options, B115200);
			cfsetospeed(&options, B115200);
		break;
		case 500000:
			ROS_INFO("SerialDevice::InitPort - Setting speed to 500000");
	 		if(cfsetispeed( &options, iBaudRate ) < 0 || cfsetospeed( &options, iBaudRate ) < 0)
			  {
				// ROS_ERROR("failed to set serial baud rate");
		  		return -1; 
	  		  }
                break;
		default:
			cfsetispeed(&options, B19200);
			cfsetospeed(&options, B19200);
		break;
	}	
	// Enable the receiver and set local mode...
	options.c_cflag |= CLOCAL | CREAD; 
	options.c_cflag &= ~HUPCL;
	// 
	// PARITY
	if(!strcmp(cParity, "none")){
		//parity = NONE 
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~PARODD;  
		printf("SerialDevice::InitPort: Parity = none\n");
	} else if(!strcmp(cParity, "even")){
		//parity = EVEN 
		options.c_cflag |= PARENB;  
		printf("SerialDevice::InitPort: Parity = even\n");
	} else if(!strcmp(cParity, "odd")){
		options.c_cflag |= PARENB;
		options.c_cflag |= PARODD; 
		printf("SerialDevice::InitPort: Parity = odd\n");
	} else {
		//parity = NONE 
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~PARODD; // NONE 
		printf("SerialDevice::InitPort: Parity = none\n");
	}
	
	options.c_cflag &= ~CSTOPB;// 1 Stop bit	
	options.c_cflag &= ~CSIZE;
	//
	// Character size
	switch(iBitDataSize){
		case 5:
			options.c_cflag |= CS5;	
			printf("SerialDevice::InitPort: Data size = CS5\n");
		break;
		case 6:
			options.c_cflag |= CS6;	
			printf("SerialDevice::InitPort: Data size = CS6\n");
		break;
		case 7:
			options.c_cflag |= CS7;	
			printf("SerialDevice::InitPort: Data size = CS7\n");
		break;
		case 8:
			options.c_cflag |= CS8;	
			printf("SerialDevice::InitPort: Data size = CS8\n");
		break;
		default:
			options.c_cflag |= CS7;	
			printf("SerialDevice::InitPort: Data size = CS7\n");
		break;
	}
	/*
	-parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts
ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
-isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke
	 */
	options.c_cflag &= ~CRTSCTS;	//Disables hardware flow control	

	options.c_iflag |= IGNBRK;
	options.c_iflag &= ~ICRNL;
	options.c_iflag &= ~IXON;
	options.c_oflag &= ~OPOST;
	options.c_oflag &= ~ONLCR;
	options.c_lflag &= ~ISIG;
	options.c_lflag &= ~IEXTEN;
	options.c_lflag &= ~ECHOK;
	options.c_lflag &= ~ECHOCTL;
	options.c_lflag &= ~ECHOKE;
	options.c_lflag &= ~ECHO;
	options.c_lflag &= ~ECHOE;
	// 
	// Entrada canónica-> La entrada canónica es orientada a línea. Los caracteres se meten en un buffer hasta recibir un CR o LF.
	if(bCanon)
		options.c_lflag |= ICANON;
	else
		options.c_lflag &= ~ICANON;

	// carácteres de control
	//options.c_cc[VMIN] = (cc_t)1;
	options.c_cc[VMIN] = (cc_t)1;
	options.c_cc[VTIME] = (cc_t)5;

	tcflush(fd, TCIFLUSH);
	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);

	printf("SerialDevice::InitPort: Port %s initialized\n",cDevice);
	return SERIAL_OK;
}

/*! \fn int SerialDevice::OpenPort1(char *dev)
 	* Opens serial port for communication
*/
int SerialDevice::OpenPort1(void){

	fd = open(cDevice,  O_RDWR | O_NOCTTY);// | O_NDELAY);// | O_NONBLOCK)
	
	if(fd==-1){
		std::cout << "SerialDevice::OpenPort: Error opening " << cDevice << " port" << std::endl;
		return SERIAL_ERROR;  // invalid device file	
	}else{
		std::cout << "SerialDevice::OpenPort: " << cDevice << " opened properly" << std::endl;
		//return SERIAL_OK;
	}
	
	InitPort();		//TEST
	
	fcntl(fd,F_SETFL, FNDELAY);	//TEST
	
	bReady = true;
	
	return SERIAL_OK;
}


/*! \fn int SerialDevice::OpenPort2(char *dev)
 	* Opens serial port for communication
*/
int SerialDevice::OpenPort2(void){
	struct termios newtio;
		
	fd = open(cDevice,  O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if(fd==-1){
		std::cout << "SerialDevice::OpenPort2: Error opening " << cDevice << " port" << std::endl;
		return SERIAL_ERROR;  // invalid device file	
	}//else{
		//std::cout << "SerialDevice::OpenPort: " << cDevice << " opened properly" << std::endl;
		//return SERIAL_OK;
	//}
	
	// set up comm flags
	memset(&newtio, 0,sizeof(newtio));
	
	switch(iBitDataSize)
	{
		case 5:
			newtio.c_cflag = CS5 | CREAD;
		break;
		case 6:
			newtio.c_cflag = CS6 | CREAD;
		break;
		case 7:
			newtio.c_cflag = CS7 | CREAD;
		break;
		case 8:
			newtio.c_cflag = CS8 | CREAD;
		break;
		default:
			//std::cout << "SerialDevice::OpenPort: unknown datasize (" << iBitDataSize << " bits) : setting default datasize ("<< DEFAULT_DATA_SIZE <<" bits)" << std::endl; 
			iBitDataSize = 8;
			newtio.c_cflag = CS8 | CREAD;
		break;
	}
	
	newtio.c_iflag = INPCK;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	
	if(!strcmp(cParity,"even"))
		newtio.c_cflag |= PARENB;//Even parity
	else if(!strcmp(cParity,"odd"))
		newtio.c_cflag |= PARODD;//Odd parity
	else if(!strcmp(cParity,"none")){	//No parity
		newtio.c_cflag &= ~PARODD;
		newtio.c_cflag &= ~PARENB;
	}
	
	newtio.c_lflag &= ~ICANON;	// TEST
	
	tcsetattr(fd, TCSANOW, &newtio);
	tcflush(fd, TCIOFLUSH);
		
	if (SetTermSpeed(iBaudRate) == SERIAL_ERROR)
	    return SERIAL_ERROR;
	
	// Make sure queue is empty
	tcflush(fd, TCIOFLUSH);
	usleep(1000);
	tcflush(fd, TCIFLUSH);
	
	bReady = true;
	
	return SERIAL_OK;
	
}

/*! \fn int SerialDevice::ClosePort(void)
 	* Closes serial port
*/
int SerialDevice::ClosePort(void){
	bReady = false;
	
	printf("SerialDevice::ClosePort: closing port %s\n", cDevice);
	return close(fd);
	//return SERIAL_OK;
}


/*!	\fn void SerialDevice::ReceiveMessage(char *msg)
	* Receives commands from SerialDevice
	* @return number of read bytes
	* @return SERIAL_ERROR if device is not ready
*/
int SerialDevice::ReceiveMessage(char *msg)
{	
	int n=0;
	if(bReady) {		
		// n= read(fd, msg, BUFFER);
		n= ReadPort(msg);
		//if(n>0)
		//	printf("SerialDevice::ReceiveMessage: read %d bytes -> %s \n", n, msg);
		return n;
	} else {
		std::cout << "SerialDevice::ReceiveMessage: Device %s "<< cDevice << " is not ready" << std::endl;
		return SERIAL_ERROR;
	}
}


/*!	\fn int SerialDevice::WritePort(char *chars, int length)
	* @brief Sends commands to the Serial port
	* @param chars as char*, string for sending
	* @param length as int, length of the string	
	* @return number of sent bytes
*/
int SerialDevice::WritePort(char *chars, int length) {

	//int len = strlen(chars);	
	//chars[len] = 0x0d; // stick a <CR> after the command	
	//chars[len+1] = 0x00; // terminate the string properly

	int n = write(fd, chars, length);//strlen(chars));
	if (n < 0) {
		//printf("SerialDevice::WritePort: write failed!: %s\n", stderr(errno));
		perror("SerialDevice::WritePort: Error\n");
		return SERIAL_ERROR;
	}
	return n;
}

/*!	\fn int SerialDevice::WritePort(char *chars, int *written_bytes, int length)
 * @brief Sends data to the serial port
 * @param chars as char*, string for sending
 * @param written_bytes as int*, number of written bytes
 * @param length as int, length of the string
 * @return operation result
*/
int SerialDevice::WritePort(char *chars, int *written_bytes, int length) {

	//if(bInitialized){
        *written_bytes = write(fd, chars, length);//strlen(chars));

        if (written_bytes < 0) {
	    perror("SerialDevice::WritePort: Error writting on the port");
            return SERIAL_ERROR;
        }
	/*
	}else{
        sprintf(cAux, "%s::WritePort: Device %s not ready", sComponentName.c_str(), cDevice);
        rlcLog->AddEvent((char*)cAux);
        return NOT_INITIALIZED;
	}
	*/
	return SERIAL_OK;
}

/*!	\fn int SerialDevice::ReadPort(char *result)
	* @brief Reads serial port 
	* @param result as char *, output buffer
	* @return number of read bytes
*/
int SerialDevice::ReadPort(char *result) {
	int iIn = read(fd, result, 254);
	//result[iIn-1] = 0x00;
	
	if (iIn < 0) {
		if (errno == EAGAIN) {
			//printf("SerialDevice::ReadPort: Read= %d, SERIAL EAGAIN ERROR, errno= %d\n",iIn, errno);
			return -1;
		} else {
			printf("SerialDevice::ReadPort: SERIAL read error %d %s\n", errno, strerror(errno));
			return -1;
		}
	}
	//printf("SerialDevice::ReadPort: read %d bytes -> %s \n", iIn, result);	
	return iIn;
}

/*!	\fn int SerialDevice::ReadPort(char *result, int num_bytes)
	* @brief Reads serial port 
	* @param result as char *, output buffer
	* @return number of read bytes
*/
int SerialDevice::ReadPort(char *result, int num_bytes) {
	int iIn = read(fd, result, num_bytes);
	//result[iIn-1] = 0x00;
	
	if (iIn < 0) {
		if (errno == EAGAIN) {
			//printf("SerialDevice::ReadPort: Read= %d, SERIAL EAGAIN ERROR, errno= %d\n",iIn, errno);
			return -1;
		} else {
			printf("SerialDevice::ReadPort: SERIAL read error %d %s\n", errno, strerror(errno));
			return -1;
		}
	}
	//printf("SerialDevice::ReadPort: read %d bytes -> %s \n", iIn, result);	
	return iIn;
}

/*!	\fn int SerialDevice::ReadPort(char *result, int *read_bytes, int num_bytes)
 * @brief Reads serial port
 * @param result as char *, output buffer
 * @param read_bytes as int, number of read bytes
 * @param num_bytes as *int, number of desired bytes to read
 * @return OK
 * @return NOT_INITIALIZED
 * @return ERROR
*/
int SerialDevice::ReadPort(char *result, int *read_bytes, int num_bytes) {
	int n = 0;

        n =  read(fd, result, num_bytes);
        *read_bytes = n;

        if(n < 0){
            if (errno == EAGAIN) {
                return SERIAL_OK;
            }else{
                printf("SerialDevice::ReadPort: SERIAL read error %d %s\n", errno, strerror(errno));
                return SERIAL_ERROR;
            }
	}
	return SERIAL_OK;
}

/*!	\fn int SerialDevice::GetBaud(void)
	*
*/
int SerialDevice::GetBaud(void) {
	
	struct termios termAttr;
	int inputSpeed = -1;
	speed_t baudRate;
	tcgetattr(fd, &termAttr);
	/* Get the input speed. */
	baudRate = cfgetispeed(&termAttr);
	switch (baudRate) {
		case B0:      inputSpeed = 0; break;
		case B50:     inputSpeed = 50; break;
		case B110:    inputSpeed = 110; break;
		case B134:    inputSpeed = 134; break;
		case B150:    inputSpeed = 150; break;
		case B200:    inputSpeed = 200; break;
		case B300:    inputSpeed = 300; break;
		case B600:    inputSpeed = 600; break;
		case B1200:   inputSpeed = 1200; break;
		case B1800:   inputSpeed = 1800; break;
		case B2400:   inputSpeed = 2400; break;
		case B4800:   inputSpeed = 4800; break;
		case B9600:   inputSpeed = 9600; break;
		case B19200:  inputSpeed = 19200; break;
		case B38400:  inputSpeed = 38400; break;
	}
	return inputSpeed;
}

/*!	\fn int SerialDevice::SendMessage(char *msg, int length)
	* Sends commands to the SerialDevice
*/
int SerialDevice::SendMessage(char *msg, int length){
	int n=0;
	
	//n=write(fd, SendBuf, length);
	if(bReady){
	//printf("SerialDevice::SendMessage: strlen=%d\n", strlen(msg));
		n = WritePort(msg, length);		
		return n;
		//if(n==0){
		//	printf("SerialDevice::SendMessage: Could not send command ");
		//	return SERIAL_ERROR;
		//}else
		//	printf("SerialDevice::SendMessage: Transmited %d", n);
		
		//return SERIAL_OK;
	}else	{
		std::cout << "SerialDevice::SendMessage: Device is not ready" << std::endl;
		return SERIAL_ERROR;
	}
}

/*!	\fn int SerialDevice::SetTermSpeed(int speed)
	* Set serial communication speed.
	* Valid values: 9600, 19200, 38400, 115200
	* @return SERIAL_OK
	* @return SERIAL_ERROR
*/
int SerialDevice::SetTermSpeed(int speed){
	struct termios term;
	int term_speed;
	
	switch(speed){
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
			term_speed = B19200;
			//std::cout << "SerialDevice::SetTermSpeed: Unknown speed ("<< speed <<") speedL Setting default value ("<< term_speed <<")"<< std::endl;
			break;
	}
	
	switch(term_speed)
	{
		case B9600:
		case B19200:
		case B38400:
		case B115200:
		case B500000:
			if( tcgetattr( fd, &term ) < 0 )
			{
				//std::cout << "SerialDevice::SetTermSpeed: Unable to get device attributes" << std::endl;
				return SERIAL_ERROR;
			}
			
		  	//cfmakeraw( &term );
			if(cfsetispeed( &term, term_speed ) < 0 || cfsetospeed( &term, term_speed ) < 0)
			{
				///std::cout << "SerialDevice::SetTermSpeed: Failed to set serial baud rate" << std::endl;
				return SERIAL_ERROR;
			}
			
			if( tcsetattr( fd, TCSAFLUSH, &term ) < 0 )
			{
				//std::cout << "SerialDevice::SetTermSpeed: Unable to set device attributes" << std::endl;			
				return SERIAL_ERROR;
			}
			//std::cout << "SerialDevice::SetTermSpeed: Communication rate changed to " << speed << std::endl;
			return SERIAL_OK;
		break;
	
	default:
		//std::cout << "SerialDevice::SetTermSpeed: Unknown speed "<< speed << std::endl;
		return SERIAL_ERROR;
		
	}
	return SERIAL_OK;
}

/*!	\fn int SerialDevice::Flush()
	* Clean port buffer
	* @return SERIAL_OK
	* @return SERIAL_ERROR
*/
int SerialDevice::Flush(){
	if (tcflush( fd, TCIOFLUSH ) < 0 ) return SERIAL_OK;
	else return SERIAL_ERROR;
}


char *SerialDevice::GetDevice(){
	return cDevice;
}
//! Sets the Canonical input. False by default
void SerialDevice::SetCanonicalInput(bool value){
	bCanon = value;
}

/*	
agvservicios:/robotrans# stty -F /dev/ttyS0 -a 

speed 19200 baud; rows 0; columns 0; line = 0;
intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^D; eol = <undef>;
eol2 = <undef>; swtch = <undef>; start = ^Q; stop = ^S; susp = ^Z; rprnt = ^R;
werase = ^W; lnext = ^V; flush = ^O; min = 1; time = 5;
parenb parodd cs7 -hupcl -cstopb cread clocal -crtscts

ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff
-iuclc -ixany -imaxbel -iutf8
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
-isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt
-echoctl -echoke


agvservicios:/robotrans/src# stty -F /dev/ttyS1 -a
speed 19200 baud; rows 0; columns 0; line = 0;
intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^D; eol = <undef>; 
eol2 = <undef>; swtch = <undef>; start = ^Q; stop = ^S; susp = ^Z; rprnt = ^R; 

werase = ^W; lnext = ^V; flush = ^O; min = 1; time = 5;
parenb parodd cs7 -hupcl -cstopb cread clocal -crtscts
ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff 
-iuclc -ixany -imaxbel -iutf8 
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0

-isig icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt 
-echoctl -echoke

*/

