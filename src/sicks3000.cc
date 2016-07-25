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

#include <netinet/in.h>	// htons
#include "s3000_laser/sicks3000.h"

// 1 second of data at 500kbaud
#define DEFAULT_RX_BUFFER_SIZE 500 * 1024 / 8

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SickS3000::SickS3000( const std::string port, int baudrate, const std::string parity, int datasize )
: serial_( port, baudrate, parity, datasize )
{
    rx_buffer_.reserve( DEFAULT_RX_BUFFER_SIZE );
	recognisedScanner = false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SickS3000::~SickS3000()
{
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SickS3000::Open()
{
	return serial_.OpenPort();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SickS3000::Close()
{
	return serial_.ClosePort();
}

////////////////////////////////////////////////////////////////////////////////
// Set up scanner parameters based on number of results per scan
bool SickS3000::SetScannerParams(sensor_msgs::LaserScan& scan, int data_count)
{
	if (data_count == 761)	// sicks3000
	{
		float freq_hz = 16.6;

		scan.angle_min          = deg_to_rad(-95);
		scan.angle_max          = deg_to_rad(95);
		scan.angle_increment    = deg_to_rad(0.25);
		scan.scan_time          = 1.0 / freq_hz;
		scan.time_increment     = scan.scan_time / data_count;
		scan.range_min          = 0;
		scan.range_max          = 49;	// check ?

		return true;
	}

	if (data_count == 381)	// sicks3000 0.5deg resolution
	{
		float freq_hz = 20.0;

		scan.angle_min          = deg_to_rad(-95);
		scan.angle_max          = deg_to_rad(95);
		scan.angle_increment    = deg_to_rad(0.5);
		scan.scan_time          = 1.0 / freq_hz;
		scan.time_increment     = scan.scan_time / data_count;
		scan.range_min          = 0;
		scan.range_max          = 49;	// check ?

		return true;
	}

	if (data_count == 541)	// sicks30b
	{
		float freq_hz = 12.7;

		scan.angle_min          = deg_to_rad(-135);
		scan.angle_max          = deg_to_rad(135);
		scan.angle_increment    = deg_to_rad(0.5);
		scan.scan_time          = 1.0 / freq_hz;
		scan.time_increment     = scan.scan_time / data_count;
		scan.range_min          = 0;
		scan.range_max          = 40;	// check ? 30m in datasheet

		return true;
	}

	return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SickS3000::ReadLaser( sensor_msgs::LaserScan& scan )
{
	int recv_bytes = 0;				// Number of received bytes

	if ( !serial_.ReadPort( recv_buffer_, READ_BUFFER_SIZE, recv_bytes ) )
	{
		ROS_ERROR("SickS3000::ReadLaser: Error reading port");
		return false;
	}

	if ( rx_buffer_.size() + recv_bytes > rx_buffer_.capacity() )
	{
		ROS_WARN("S3000 Buffer Full");
		rx_buffer_.clear();
		return false;
	}

    rx_buffer_ += std::string( recv_buffer_, recv_bytes );
	return ParseLaserData(scan);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct Telegram
{
	uint8_t     header[4];
	uint8_t     admin_data[2];
	uint16_t    size;	// from admin data to CRC included
	uint8_t     coordination_flag;
	uint8_t     device_address;
	uint16_t    protocol_version;
	uint16_t    status;
	uint32_t    scan_number;
	uint16_t    telegram_number;
	uint8_t     packet_type;
	uint8_t     packet_type_confirm;

	bool has_valid_header() const
    	{ return memcmp( header, "\0\0\0\0", 4 ) == 0 && memcmp( admin_data, "\0\0", 2 ) == 0; }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool SickS3000::ParseLaserData( sensor_msgs::LaserScan& scan )
{
	while ( rx_buffer_.size() >= sizeof(Telegram) )
	{
		unsigned int telegram_pos;
		const Telegram* telegram = NULL;

		for ( telegram_pos = 0; telegram_pos < rx_buffer_.size()-sizeof(Telegram); ++telegram_pos )
		{
			telegram = reinterpret_cast<Telegram*>( &rx_buffer_[telegram_pos] );
			if ( telegram->has_valid_header() ) break;
			telegram = NULL;
		}

        rx_buffer_.erase(0,telegram_pos);
		if (!telegram) return false;

		ROS_INFO("telegram #%u: scan=%u size=%u protocol=%u address=%u status=%u",
			telegram->telegram_number,
			telegram->scan_number,
			telegram->size,
			telegram->protocol_version,
			telegram->device_address,
			telegram->status );

		// check if we have enough data yet
		if ( rx_buffer_.size() < sizeof(telegram->header) + telegram->size )
		{
			ROS_WARN("S3000: more data needed to parse telegram #%u", telegram->telegram_number );
			return false;
		}

		int checksum_pos = sizeof(telegram->header) + telegram->size - sizeof(uint16_t);

		uint16_t packet_checksum = *reinterpret_cast<uint16_t*> ( &rx_buffer_[checksum_pos] );
		uint16_t calc_checksum = CreateCRC( &rx_buffer_[sizeof(telegram->header)], telegram->size - 2);

		if ( packet_checksum != calc_checksum )
		{
			ROS_WARN("S3000: Invalid checksum, skipping telegram #%u", telegram->telegram_number);
	        rx_buffer_.erase(0,telegram->size);
			continue;
		}

		if ( telegram->packet_type != telegram->packet_type_confirm )
		{
			ROS_WARN("S3000: mismatched packet type header bytes in telegram #%u", telegram->telegram_number );
	        rx_buffer_.erase(0,telegram->size);
			continue;
		}

		if ( telegram->packet_type == 0xAA ) // I/O data
		{
			ROS_WARN("S3000: We got a I/O data packet we dont know what to do with it\n");
	        rx_buffer_.erase(0,telegram->size);
			continue;
		}
		else if ( telegram->packet_type == 0xBB ) // range data
		{
			int data_count = (telegram->size - 22) / 2;

			if (data_count < 0)
			{
				ROS_WARN("S3000: bad data count (%d)", data_count);
    	        rx_buffer_.erase(0,telegram->size);
				continue;
			}

			if (!recognisedScanner)
				SetScannerParams( scan, data_count );

			scan.header.stamp = ros::Time::now();
			scan.ranges.resize(data_count);
			scan.intensities.clear();	// not used

			int read_pos = sizeof(Telegram);

			for ( int i=0; i < data_count; ++i, read_pos += sizeof(uint16_t) )
			{
				uint16_t Distance_CM = *reinterpret_cast<uint16_t*>( &rx_buffer_[read_pos] );
				Distance_CM &= 0x1fff;	// remove status bits
				scan.ranges[i] = Distance_CM / 100.0;
			}

	        rx_buffer_.erase(0,telegram->size);
			return true;
		}
		else if ( telegram->packet_type == 0xCC ) // reflector data
		{
			int read_pos = sizeof(Telegram);

			uint16_t reflector_count = *reinterpret_cast<uint16_t*>( &rx_buffer_[read_pos] );
			read_pos += sizeof(reflector_count);
			ROS_DEBUG( "reflectors=%u", reflector_count );

			uint32_t reflector_data, num_ranges = ( scan.angle_max - scan.angle_min ) / scan.angle_increment;

			scan.header.stamp = ros::Time::now();
			scan.ranges.resize( num_ranges );
			std::fill( scan.ranges.begin(), scan.ranges.end(), scan.range_min - 1 );
			scan.intensities.clear();	// not used

			for ( size_t i = 0; i < reflector_count; ++i )
			{
				reflector_data = *reinterpret_cast<uint32_t*>( &rx_buffer_[read_pos] );
				read_pos += sizeof(reflector_data);

				int raw_angle = reflector_data & 0xFFFF;// read 16 bits
				int raw_dist = ( reflector_data >> 16 ) & 0x1FFF;	// read 13 bits
				ROS_DEBUG("reflector %lu: angle=%.2fdeg distance=%.2fm", i, raw_angle * 0.01, raw_dist * 0.01 );

				float scan_angle = scan.angle_max - deg_to_rad(raw_angle * 0.01);
				int range_idx = ( scan_angle - scan.angle_min ) / scan.angle_increment;

				if ( range_idx >= 0 && range_idx < scan.ranges.size() ) scan.ranges[range_idx] = raw_dist * 0.01;
				else ROS_WARN("invalid reflector angle=%.2frad, idx=%d/%lu", scan_angle, range_idx, scan.ranges.size() );
			}

	        rx_buffer_.erase(0,telegram->size);
			if ( reflector_count > 0 ) return true;
		}
		else
		{
			ROS_WARN("We got an unknown packet\n");
	        rx_buffer_.erase(0,telegram->size);
			continue;
		}
	}

	return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static const unsigned short crc_table[256] =
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

unsigned short SickS3000::CreateCRC(const char *Data, ssize_t length)
{
	unsigned short CRC_16 = 0xFFFF;
	unsigned short i;
	for (i = 0; i < length; i++)
	{
		CRC_16 = (CRC_16 << 8) ^ (crc_table[(CRC_16 >> 8) ^ (Data[i])]);
	}
	return CRC_16;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

