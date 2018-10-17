#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include <string.h>
#include <sstream>
using namespace std;

class imu_jy61_hw: public hardware_interface::RobotHW
{
public:
	imu_jy61_hw(std::string portname , int baudrate );
	void read();
	void write();
	ros::Time get_time();
	ros::Duration get_period();
	bool closeport();
	bool openport();
    void setDTR(bool value);
    std::string get_frame(int &head, std::string& buff);
    void deal_frame(std::string frame);
	
private:
	hardware_interface::ImuSensorInterface imu;
	std::string name_;
	std::string frame_id_;
public:
	double orientation_[4];
	double orientation_covariance_[9];
	double angular_velocity_[3];
	double angular_velocity_covariance_[9];
	double linear_acceleration_[3];
	double linear_acceleration_covariance_[9];
	double temprature_;

	std::string portname_;
	int baudrate_;	
    std::string readbuff;
    int printed = 0;
    
public:
	serial::Serial imu_serial_port_;

};

void printchar(std::string content)
{
	for(int i =0; i< content.length();i++)
	{
		printf("%2x ",(unsigned char)content[i]);
	}
	printf("\n");
}
