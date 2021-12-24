#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <string>


//Forward Declaration
serial::Serial ros_serial;

class joystick_driver
{
	public:
	joystick_driver(ros::NodeHandle *nh);
	void init_serial_driver();
	void send_command();
	void serial_read();
	void set_speed(double x, double y , double z);
	
	
	private:

	ros::NodeHandle *_nh;
	ros::Publisher _pub;
	ros::Subscriber _sub;
	
	const std::string _SDK_MODE = "command;";

	//help function -> get controller class

	geometry_msgs::Twist cmd_vel;

	double _vel_x = 0; //Speed X 1ms
	double _vel_y = 0; //Speed Y 1ms
	double _vel_z = 0; //SPeed Pivot

	double _max_spd_x = 0.25;
	double _max_spd_y = 0.25;
	double _max_spd_z = 0.0;

	std::string _spd_x = "0.00";
	std::string _spd_y = "0.00";
	std::string _spd_z = "0.00";



	//Subscriber
	void joy_callback(const sensor_msgs::Joy & joy_);
	
	
	
};


void joystick_driver::init_serial_driver()
{
	
	//Hardware driver init
	try{

		ros_serial.setPort("/dev/ttyUSB0");
		ros_serial.setBaudrate(115200);
		serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
		ros_serial.setTimeout(time_out);
		ros_serial.open();

		

	}

	catch(serial::IOException& e)
	{
         ROS_ERROR_STREAM("Unable to open port ");
         
    }
	if(ros_serial.isOpen())
	{
		ROS_INFO_STREAM("Port is opened");
	}
	

	ros_serial.write(_SDK_MODE);
	ROS_INFO_STREAM("Waiting for reply");
	
}

void joystick_driver::send_command()
{



	std::string speed_command = std::string("chassis speed")
					+ " x " + _spd_x
					+ " y " + _spd_y
					+ " z " + "1.15"
					+ ";";

	ROS_INFO_STREAM(speed_command);
	ros_serial.write(speed_command);

}



void joystick_driver::set_speed(double x, double y, double z)
{

	//conversion of axes to max speed (1m/s)
	_vel_x = _max_spd_x * x;
	_vel_y = _max_spd_y * y;
	_vel_z = 0.00;
	


	//For future use, convert to cmd_vel
	cmd_vel.linear.x = _vel_x;
	cmd_vel.linear.y = _vel_y;
	cmd_vel.linear.z = 0.00;

	_pub.publish(cmd_vel);

	if(_vel_x == -0)
	{
		_vel_x =0.00;
	}

	if(_vel_y == -0)
	{
		_vel_y =0.00;
	}
	_spd_x = std::to_string(_vel_x);
	_spd_y = std::to_string(_vel_y);
	_spd_z = std::to_string(_vel_z);

}

void joystick_driver::serial_read()
{

	std::string data;

	if(ros_serial.available())
		{
			ros_serial.read(data,20);
			ROS_INFO_STREAM(data);
		}
}



joystick_driver::joystick_driver(ros::NodeHandle *nh) : _nh(nh)
{
	//init all publisher and Subscriber
	//_pub = _nh->advertise<std_msgs::Float32>("/joystick_feedback",1);
	_pub = _nh->advertise<geometry_msgs::Twist>("/cmd_vel",1);
	_sub = _nh->subscribe("/joy",10,&joystick_driver::joy_callback,this);
}


void joystick_driver::joy_callback(const sensor_msgs::Joy & joy_)
{

	//joy to low_level controller parsing
	float x = joy_.axes[1]; // -1 to 1
	float y = joy_.axes[0]; // -1 to 1
	float z = 0; //TODO find a way to do pivoting

	//set speed
	set_speed(x,y,z);

}





int main(int argc, char ** argv)
{
	
	
	ros::init(argc,argv,"joystick_driver");
	ros::NodeHandle nh;
	joystick_driver driver(&nh);

	driver.init_serial_driver();
	//driver.send_command();

	ros::Rate r(1.2);
	while(ros::ok())
	{
		driver.serial_read();
		r.sleep();
		driver.send_command();	
		ros::spinOnce();
	}
	

	return 0;
	
	
}
