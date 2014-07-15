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
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"
#include "tf/transform_broadcaster.h"

using namespace std;

class s3000node {

public:
  SickS3000 *laser; 
  sensor_msgs::LaserScan reading;

  string port;

  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher laser_data_pub_;

  // publish its transform to base_link
  bool publish_tf;
  tf::TransformBroadcaster laser_broadcaster;

  bool running;
  
  int error_count_;
  int slow_count_;
  std::string was_slow_;
  std::string error_status_;

  string frameid_;
  
  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;

  s3000node(ros::NodeHandle h) : self_test_(), diagnostic_(), 
  node_handle_(h), private_node_handle_("~"), 
  error_count_(0), 
  slow_count_(0), 
  desired_freq_(20), 
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
  {
    ros::NodeHandle laser_node_handle(node_handle_, "s3000_laser");
        private_node_handle_.param("port", port, string("/dev/ttyUSB0"));

    private_node_handle_.param<bool> ("publish_tf", publish_tf, false);
    laser_data_pub_ = laser_node_handle.advertise<sensor_msgs::LaserScan>("scan", 100);
    running = false;
    private_node_handle_.param("frame_id", frameid_, string("laser"));
    reading.header.frame_id = frameid_;
        
    self_test_.add("Connect Test", this, &s3000node::ConnectTest);

    diagnostic_.add( freq_diag_ );
    diagnostic_.add( "Laser S3000 Status", this, &s3000node::deviceStatus );
    
    // Create SickS3000 in the given port
    laser = new SickS3000( port );
   }

  ~s3000node()
  {
    stop();
  }


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

  void getData(sensor_msgs::LaserScan& data)
  {
    bool bValidData = false;

    laser->ReadLaser( data, bValidData );

    //// If valid data, publish it
    if (bValidData) {
    	data.header.stamp = ros::Time::now();
	laser_data_pub_.publish( data );
	}

/*
    // Robotnik - to test unit
    if (publish_tf) {
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

  bool spin()
  {

    // sensor_msgs::LaserScan data;
    ros::Rate r(desired_freq_);

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (start() == 0)
      {
        while(node_handle_.ok()) {
          //if(publish_datum() < 0)	
          //  break;
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


  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (!running)
      status.summary(2, "Laser Ranger is stopped");
    else if (!was_slow_.empty())
    {
      status.summary(1, "Excessive delay");
      was_slow_.clear();
    }
    else
      status.summary(0, "Laser Ranger is running");

    status.add("Device", port);
    status.add("TF frame", frameid_);
    status.add("Error count", error_count_);
    status.add("Excessive delay", slow_count_);
  }

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "s3000_node");

  ros::NodeHandle nh;

  s3000node s3000n(nh);
  s3000n.spin();

  return(0);
}
