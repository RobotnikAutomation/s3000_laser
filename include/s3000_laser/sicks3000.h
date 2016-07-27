/*
*  Copyright (c) 2012, Robotnik Automation, SLL
*
*
*   This program is free software: you can redistribute it and/or modify
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
/*
 Desc: Driver for the SICK S3000 laser
 Author: Robotnik Automation SLL (based on sicks3000 by Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard for Player/Stage)
 Date: 1 Sept 2012

 The sicks3000 driver controls the SICK S 3000 safety laser scanner interpreting its data output.
 The driver is very basic and assumes the S3000 has already been configured to continuously output
 its measured data on the RS422 data lines.
*/
#include <assert.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <s3000_laser/SerialDevice.h>

//! Converts degrees to radians
inline double deg_to_rad(double val) { return val*M_PI/180.0; }

// The laser device class.
class SickS3000
{
  public:

    // Constructor
    SickS3000( std::string port, int baudrate, std::string parity, int datasize );

    // Destructor
    ~SickS3000();

    //! Open the port
    bool Open();

    //! Close the port
    bool Close();

    //! Read and process data
    bool ReadLaser( sensor_msgs::LaserScan& scan );
  
  private:

    // Process range data from laser
    // int ProcessLaserData();
    int ProcessLaserData( sensor_msgs::LaserScan& scan_msg, bool& bValidData ); // public periodic function

    // Calculates CRC for a telegram
    static unsigned short CreateCRC(const uint8_t *data, ssize_t len);

    static bool SetScannerParams(sensor_msgs::LaserScan& scan, int data_count);

  protected:

    // serial port
    SerialDevice serial_;

    bool recognisedScanner;

    // rx buffer
    uint8_t* rx_buffer;
    unsigned int rx_buffer_size;
    unsigned int rx_count;
    
    static const size_t READ_BUFFER_SIZE=2000;
    char read_buffer_[READ_BUFFER_SIZE];
};


