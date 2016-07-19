/*********************************************************************
*
*  This program uses the sicks3000  component to get laser scans, and then
*  publishes them as ROS messages
*
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
*********************************************************************/

#include <assert.h>
#include <math.h>
#include <iostream>
#include <boost/format.hpp>

#include "s3000_laser/sicks3000.h"   // s3000 driver from player (Toby Collet / Andrew Howard)
#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "sensor_msgs/LaserScan.h"

#include "tf/transform_broadcaster.h"
#include "s3000_laser/enable_disable.h"

using namespace std;

class s3000node {

private:
	SickS3000 *laser; 
	sensor_msgs::LaserScan reading;

	string port;
	int baud_rate;

	self_test::TestRunner self_test_;
	diagnostic_updater::Updater diagnostic_;

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	ros::Publisher laser_data_pub_;

	//! publish its transform to base_link
	bool publish_tf_;
	tf::TransformBroadcaster laser_broadcaster;
	//! Flag to enable/disable the scan publication
	bool publish_scan_;	

	bool running;

	int error_count_;
	int slow_count_;
	std::string was_slow_;
	std::string error_status_;
	//! Name of the frame associated
	string frame_id_;
	string topic_name;
	double desired_freq_;
	diagnostic_updater::FrequencyStatus freq_diag_;
	//! Enables/disables the scan publication
	ros::ServiceServer enable_disable_srv_;

public:
	
	s3000node(ros::NodeHandle h) : self_test_(), diagnostic_(), 
	node_handle_(h), private_node_handle_("~"), 
	error_count_(0), 
	slow_count_(0), 
	desired_freq_(20), 
	freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
	{
	    float angle_min_deg, angle_max_deg, angle_increment_deg;
	    
		ros::NodeHandle laser_node_handle(node_handle_, "s3000_laser");
		private_node_handle_.param("port", port, string("/dev/ttyUSB1"));
		private_node_handle_.param("baud_rate", baud_rate, 500000);
		private_node_handle_.param<float>("range_min", reading.range_min, 0 );
		private_node_handle_.param<float>("range_max", reading.range_max, 40 );
		private_node_handle_.param<float>("angle_min", angle_min_deg, -100 );
		private_node_handle_.param<float>("angle_max", angle_max_deg, 100 );
		private_node_handle_.param<float>("angle_increment", angle_increment_deg, 0.25 );
		private_node_handle_.param("topic_name", topic_name, string("/scan"));
		private_node_handle_.param<bool> ("publish_tf", publish_tf_, false);
		private_node_handle_.param<bool> ("publish_scan", publish_scan_, true);
		laser_data_pub_ = laser_node_handle.advertise<sensor_msgs::LaserScan>(topic_name, 1);
		running = false;
		private_node_handle_.param("frame_id", frame_id_, string("/laser"));
		
		reading.angle_min = DTOR(angle_min_deg);
		reading.angle_max = DTOR(angle_max_deg);
		reading.angle_increment = DTOR(angle_increment_deg);
		reading.header.frame_id = frame_id_;
			
		self_test_.add("Connect Test", this, &s3000node::ConnectTest);

		diagnostic_.add( freq_diag_ );
		diagnostic_.add( "Laser S3000 Status", this, &s3000node::deviceStatus );
		ROS_INFO("s3000node: Port = %s", port.c_str());
		ROS_INFO("s3000node: Baudrate = %d", baud_rate);
		
		// Advertises new service to enable/disable the scan publication
		enable_disable_srv_ = private_node_handle_.advertiseService("enable_disable",  &s3000node::EnableDisable, this);
		
		// Create SickS3000 in the given port
		laser = new SickS3000( port, baud_rate );
	}

	~s3000node()
	{
		stop();
	}
	
	bool spin(){

		// sensor_msgs::LaserScan data;
		ros::Rate r(desired_freq_);

		while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
		{
			if (start() == 0)
			{
				while(node_handle_.ok()) {
					getData( reading );           

					self_test_.checkTest();
					diagnostic_.update();
					ros::spinOnce();
					r.sleep();          
				}
			} else {
				// No need for diagnostic here since a broadcast occurs in start
				// when there is an error.
				usleep(1000000);
				self_test_.checkTest();
				ros::spinOnce();
			}
		}

		stop();

		return true;
	}

private:

	int start()
	{
		stop();
		laser->Open();
		diagnostic_.setHardwareID("Laser Ranger");
		ROS_INFO("Laser Ranger sensor initialized.");
		freq_diag_.clear();
		running = true;
		return(0);
	}


	int stop()
	{
		if(running)
		{
			laser->Close();
			running = false;
		}

		return(0);
	}


	void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
	{
		laser->Open();
		status.summary(0, "Connected successfully.");
	}
	
	//! Gets the sensor data and publishes 
	void getData(sensor_msgs::LaserScan& data)
	{
		bool bValidData = false;

		laser->ReadLaser( data, bValidData );

		//// If valid data, publish it
		if (bValidData) {
			data.header.stamp = ros::Time::now();
			data.header.frame_id = frame_id_;
			if(publish_scan_)	// Publishes only if enabled
				laser_data_pub_.publish( data );
			freq_diag_.tick();
		}

		/*
		// Robotnik - to test unit
		if (publish_tf_) {
		// create a tf message
		geometry_msgs::TransformStamped imu_trans;
		imu_trans.header.stamp = data.header.stamp;
		imu_trans.header.frame_id = "laser";   //odom_frame_id.c_str();
		imu_trans.child_frame_id = "base_link";
		imu_trans.transform.translation.x = 0.0;
		imu_trans.transform.translation.y = 0.0;
		imu_trans.transform.translation.z = 0.0;
		imu_trans.transform.rotation = data.orientation;
		//send the transform
		imu_broadcaster.sendTransform(imu_trans);
		}
		*/  

	}

	//! Updates the diagnostic status
	void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
	{
		if (!running)
			status.summary(2, "Laser Ranger is stopped");
		else if (!was_slow_.empty())
		{
			status.summary(1, "Excessive delay");
			was_slow_.clear();
		}
		else{
			
			status.summary(0, "Laser Ranger is running");
			
		}
		status.add("Device", port);
		status.add("TF frame", frame_id_);
		status.add("Error count", error_count_);
		status.add("Excessive delay", slow_count_);
		status.add("Scan publication", publish_scan_);
	}

    //! Service callback to enable/disable scan publication
	bool EnableDisable(s3000_laser::enable_disable::Request &req, s3000_laser::enable_disable::Response &res ){
		publish_scan_ = req.value;

		ROS_INFO("s3000_node::EnablaDisable: Setting scan publish to %d", req.value);
		res.ret = true;
		
		return true;
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "s3000_node");

	ros::NodeHandle nh;

	s3000node s3000n(nh);
	s3000n.spin();

	return(0);
}
