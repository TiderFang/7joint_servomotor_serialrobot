#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <serial/serial.h>
#include <sstream>
#include <string.h>
#include <ros/callback_queue.h>
using namespace std;
class RobotSerialHw:public hardware_interface::RobotHW
{
public:
        RobotSerialHw();
        void read();
        void write();
        //~RobotSerialHw();
        ros::Time get_time();
        ros::Duration get_period();
        //void update();
        //bool checkForConflict(const std::list<ControllerInfo>& info) const

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];
    serial::Serial ser;
    int jnt_number;
    ros::Time laststamp;
public:
    std::string readbuff;
};

RobotSerialHw::RobotSerialHw()
{
    // connect and register the joint state interface
    
    jnt_number = 6;
    for(int i = 0; i < jnt_number; i++)
    {

    	ostringstream temp;
        temp<<i;
        std::string jnt_name = "joint_";
        jnt_name = jnt_name + temp.str();
        hardware_interface::JointStateHandle state_handle(jnt_name.c_str(),&pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle);
        //ROS_INFO("joint name is :");
        //ROS_INFO(jnt_name.c_str());


        pos[i] = 0;
        cmd[i] = 0;
        vel[i] = 0;
        eff[i] = 0;
    }
    registerInterface(&jnt_state_interface);
    
    for(int i = 0; i < jnt_number; i++)
    {
        ostringstream temp;
        temp<<i;
        std::string jnt_name = "joint_";
        jnt_name = jnt_name + temp.str();
        ROS_INFO(jnt_name.c_str());
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(jnt_name.c_str()), &cmd[i]);
        jnt_pos_interface.registerHandle(pos_handle);

    }
    registerInterface(&jnt_pos_interface);
    
    // open the serial port
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
        ser.setTimeout(timeout);
        ser.open();
    }
    catch(serial::IOException &e)
    {
        ROS_ERROR_STREAM("unable to open port");
        exit(-1);
    }
    ros::Time::init();
    laststamp = ros::Time::now();
}

void RobotSerialHw::read()
{
    // data format: jnt[number:1-7]pos[0-180]done+eof
    int jntnumber;
    int posnumber;
    int donenumber;
    int availablesize = ser.available();
    if(ser.available()>0)
    {
    	readbuff += ser.read(availablesize);
    }
    if(readbuff.length()>20)
    {
    	jntnumber = readbuff.find("jnt");
    	posnumber = readbuff.find("pos");
    	donenumber = readbuff.find("done");
    	int jnt_now = atoi(readbuff.substr(jntnumber+3,posnumber-jntnumber).c_str());
    	int pos_now = atoi(readbuff.substr(posnumber+3,donenumber-posnumber).c_str());
    	pos[jnt_now] = pos_now;
    	readbuff = readbuff.substr(donenumber+4,readbuff.length());
    	ROS_INFO("joint %i now is in pos %f",jnt_now,pos[jnt_now]);
    }
/*
    if(ser.available()>10)
    {
    	printf("ser.available %i\n",int(ser.available()));
        readbuff = ser.readline();
        printf("\n");
        printf(readbuff.c_str());
        jntnumber = readbuff.find("jnt");
        posnumber = readbuff.find("pos");
        donenumber = readbuff.find("done");
        int jnt_now = atoi(readbuff.substr(jntnumber+3,posnumber-jntnumber).c_str());
        int pos_now = atoi(readbuff.substr(posnumber+3,donenumber-posnumber).c_str());
        pos[jnt_now] = pos_now;
        printf("jnt %i pos %d",jnt_now,pos[jnt_now]);
    }
    */
}

void RobotSerialHw::write()
{
    string buff;
    for(int i=0; i<jnt_number; i++)
    {
        ostringstream temp;
        temp<<i;
        buff = "jnt" + temp.str();
        ostringstream temp2;
        temp2<<cmd[i];
        buff = buff + "pos" + temp2.str() + "done"+'\n';
        ser.write(buff);
    }
}
 ros::Time RobotSerialHw::get_time()
 {
 	return ros::Time::now();
 }
 
 ros::Duration RobotSerialHw::get_period()
 {
 	ros::Time newstamp = ros::Time::now();
 	ros::Duration  duration = newstamp - laststamp;
 	laststamp = newstamp;
 	return duration;
 }

int main(int argc,char** argv)
{
	ros::init(argc,argv,"robot_serialhw_node");
	ros::NodeHandle nh;
    RobotSerialHw robot;
    controller_manager::ControllerManager cm(&robot,nh);

	//ros::CallbackQueue queue;
	//nh.setCallbackQueue(&queue);
	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate rate(50);

    while (ros::ok())
    {
    	ROS_INFO("in the loop");
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
        //ros::spinOnce();
        rate.sleep();
    }
    spinner.stop();
    return 0;
}
