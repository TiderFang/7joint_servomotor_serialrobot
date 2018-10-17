#include <imu_jy61_hw/imu_jy61_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
/********************************
 * set up a robot hw node for imu
 * jy61.
 */

int main(int argc, char ** argv)
{
	//first init a ros node
	ros::init(argc,argv,"imu_jy61_hw_node");
	//get the node handle
	ros::NodeHandle nh;
	ros::console::set_logger_level("my debug level", ros::console::levels::Debug);
	ros::CallbackQueue queue;
	nh.setCallbackQueue(&queue);

	//define the hw object
	std::string portname = "/dev/ttyUSB0";
	int baudrate = 9600;
	imu_jy61_hw imu(portname, baudrate);
	controller_manager::ControllerManager cm(&imu,nh);
	ros::AsyncSpinner spinner(4,&queue);
	spinner.start();
	ros::Rate rate(50);
	ros::Time recordtime;
	ros::Time currenttime;
	while(ros::ok()){
		//ROS_INFO("In the loop");
		imu.read();
		currenttime = ros::Time::now();
		ros::Duration periodtime = currenttime - recordtime;
		cm.update(currenttime, periodtime);
		recordtime = currenttime;
		imu.write();
		ros::spinOnce();
		rate.sleep();
		//sleep();
	}
	spinner.stop();
	return 0;

}
