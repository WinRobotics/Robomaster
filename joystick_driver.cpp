#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"


//Forward Declaration
serial::Serial ros_serial;

class joystick_driver
{
	public:
	joystick_driver(ros::NodeHandle *nh);
	void init_serial_driver();
	void drive();
	void serial_read();
	
	
	private:

	ros::NodeHandle *_nh;
	ros::Publisher _pub;
	ros::Subscriber _sub;
	
	const std::string _SDK_MODE = "command;";

	//help function -> get controller class
	void set_speed(float x, float y, float z);
	float _vel_x; //Speed X
	float _vel_y; //Speed Y
	float _vel_z; //SPeed Pivot

	


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

void joystick_driver::drive()
{

}

void joystick_driver::set_speed(float x, float y, float z)
{

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
	_pub = _nh->advertise<std_msgs::Float32>("/joystick_feedback",1);
	_sub = _nh->subscribe("/joy",10,&joystick_driver::joy_callback,this);
}


void joystick_driver::joy_callback(const sensor_msgs::Joy & joy_)
{

	//joy to low_level controller parsing
	float x = joy_.axes[1];
	float y = joy_.axes[0];
	float z =0; //TODO find a way to do pivoting

	//set speed
	set_speed(x,y,z);
	
	

}





int main(int argc, char ** argv)
{
	
	
	ros::init(argc,argv,"joystick_driver");
	ros::NodeHandle nh;
	joystick_driver driver(&nh);

	driver.init_serial_driver();
		
	while(ros::ok())
	{
		driver.serial_read();
		ros::spinOnce();
	}
	

	return 0;
	
	
}
