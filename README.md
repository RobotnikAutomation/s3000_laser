# s3000_laser

<h2>Description</h2>
Driver for ROS to read the scan data from the device SICK S3000. This driver is based on the original driver developed for Player/Stage by Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard.

This driver controls the SICK S3000 safety laser scanner interpreting its data output. The driver is very basic and assumes the S3000 has already been configured to continuously output its measured data on the RS422 data lines.

<h2>Supported Hardware</h2>
This package should work with SICK S3000 safety laser scanners. 

![Image of Sick s3000](http://wiki.ros.org/s3000_laser?action=AttachFile&do=get&target=SICK_S3000.jpg)

<h2>Nodes
S3000 laser driver.

<h3>Published Topics:</h3>
s3000_laser/scan (sensor_msgs/LaserScan)
    Scan data from the laser, containing either range data or reflector detections. 

<h3>Parameters:</h3>
~port (string, default: "/dev/ttyUSB0")
    Serial port. 

~frame_id (string, default: "/laser")
    Relative frame for the publication of the laser measures. 

~baud_rate (int, default: 500000)
    Serial port baudrate (options: 9600, 19200, 38400, 115200, 500000). 

~serial_parity (string, default: "none")
    Serial port parity (options: "even", "odd", "none"). 

~serial_datasize (int, default: 8)
    Character size mask for serial link (options: 5,6,7,8). 

~range_min (float, default: 0m)
    Min valid range (m) for S3000 data produced. 

~range_max (float, default: 40m)
    Max valid range (m) for S3000 data. 
