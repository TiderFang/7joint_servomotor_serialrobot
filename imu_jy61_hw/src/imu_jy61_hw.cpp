#include <imu_jy61_hw/imu_jy61_hw.h>
#include <stdio.h>
imu_jy61_hw::imu_jy61_hw(std::string portname, int baudrate):portname_(portname),baudrate_(baudrate)
{
	//私有變量初始化
    name_ = "imu_jy61_serial";
    frame_id_ = "imu_jy61_link";
    temprature_ = 0;
    //printf("temprature %f  ", temprature_);
    /*
    orientation_ = 0;
    orientation_covariance_ = 0;
    angular_veloctiy_ = 0;
    angular_veloctiy_covariance_ = 0;
    linear_acceleration_ = 0;
    linear_acceleration_covariance_ = 0;
    */
	//初始化imu_sensor_handle
    for(int i=0;i<3;i++)
    {
    	angular_velocity_[i] = 0;
    	linear_acceleration_[i] = 0;
    	orientation_[i] = 0;
    	//printf("angular_velocity %f  ", temprature_);
    }
    orientation_[3] = 0;
    hardware_interface::ImuSensorHandle imuhandle = hardware_interface::ImuSensorHandle(name_, frame_id_, orientation_, orientation_covariance_, angular_velocity_, angular_velocity_covariance_, linear_acceleration_, linear_acceleration_covariance_);
    
    //注冊handle到imu_sensor_interface
    imu.registerHandle(imuhandle);
    
    // conect and register hardware_interface
    registerInterface(&imu);
    try
    {
        imu_serial_port_.setPort(portname_.c_str());
        imu_serial_port_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
        imu_serial_port_.setTimeout(timeout);
        imu_serial_port_.open();
        setDTR(false);
    }
    catch(serial::IOException &e)
    {
        ROS_ERROR_STREAM("unable to open port");
        exit(-1);
    }
}

bool imu_jy61_hw::openport()
{
    try
    {
        imu_serial_port_.open();
        return true;
    }catch(serial::IOException &e)
    {
        printf("port open failed");
        return false;
    }
}

bool imu_jy61_hw::closeport()
{
    try
    {
        imu_serial_port_.close();
        return true;
    }
    catch(serial::IOException &e)
    {
        return false;
    }
    
}

void imu_jy61_hw::setDTR(bool value)
{
    imu_serial_port_.setDTR(value);
}

std::string imu_jy61_hw::get_frame(int &head, std::string &buff)
{
    size_t charsize = 0;
    charsize = buff.length();
    int headpointer = head;
    //get qualified string
    std::string frame;
    while(charsize>=12)
    {
    	//printf("buff:  ");
    	//printchar(buff);
    	//printf(buff.c_str());
    	if( buff[head] == 0x55 )
    	{
			charsize = buff.length();
			int sum = 0;
			int checksum = short(buff[headpointer+10]) & 0xFF ;
			for(int i = 0; i < 10; i++)
			{
				sum = (sum + short(buff[headpointer+i]));
			}
			sum = sum & 0xFF;
			if(sum == checksum)
			{
				//ROS_INFO("headpointer is %i",headpointer+11);
				//ROS_INFO("charsize is %i",charsize-11);
				frame = buff.substr(headpointer,headpointer+11);
				buff = buff.substr(headpointer+12,charsize);
				head = 0;
				charsize = buff.length();
				//ROS_INFO("get a frame!");
				return frame;
			}else{
				buff = buff.substr(head+1,charsize);
				head = 0;
				charsize = buff.length();
			}
    	}else
    	{
			buff = buff.substr(head+1,charsize);
			head = 0;
			charsize = buff.length();
    	}
    }
    return "";
}

void imu_jy61_hw::deal_frame(std::string frame)
{
    double angle[]={0, 0, 0};
    switch(frame[1])
    {
        case 0x51:
            linear_acceleration_[0] = double(short(frame[3]<<8 | frame[2])) / 32768 * 16 * 9.8;
            linear_acceleration_[1] = double((short(frame[5]<<8 | frame[4]))) / 32768 * 16 * 9.8;
            linear_acceleration_[2] = double((short(frame[7]<<8 | frame[6]))) / 32768 * 16 * 9.8;
            temprature_ = double((short(frame[9]<<8 | frame[8]))) / 340.0 + 36.25;
            //printf("get linear acceleration!\n");
            break;
        case 0x52:
        	angular_velocity_[0] = double((short(frame[3]<<8 | frame[2]))) / 32768 * 2000 / 57.3;
        	angular_velocity_[1] = double((short(frame[5]<<8 | frame[4]))) / 32768 * 2000 / 57.3;
        	angular_velocity_[2] = double((short(frame[7]<<8 | frame[6]))) / 32768 * 2000 / 57.3;
        	temprature_ = double((short(frame[9]<<8 | frame[8]))) / 340.0 + 36.25;
        	break;
        case 0x53:
            angle[0] = double((short(frame[3]<<8 | frame[2]))) / 32768 * 180 / 57.3;
            angle[1] = double((short(frame[5]<<8 | frame[4]))) / 32768 * 180 / 57.3;
            angle[2] = double((short(frame[7]<<8 | frame[6]))) / 32768 * 180 / 57.3;
            temprature_ = double((short(frame[9]<<8 | frame[8])))/ 340.0 + 36.25;
            orientation_[0] = cos(angle[0]/2)*cos(angle[1]/2)*cos(angle[2]/2) + sin(angle[0]/2)*sin(angle[1]/2)*sin(angle[2]/2);
            orientation_[1] = sin(angle[0]/2)*cos(angle[1]/2)*cos(angle[2]/2) - cos(angle[0]/2)*sin(angle[1]/2)*sin(angle[2]/2);
            orientation_[2] = cos(angle[0]/2)*sin(angle[1]/2)*cos(angle[2]/2) + sin(angle[0]/2)*cos(angle[1]/2)*sin(angle[2]/2);
            orientation_[3] = cos(angle[0]/2)*cos(angle[1]/2)*sin(angle[2]/2) - sin(angle[0]/2)*sin(angle[1]/2)*cos(angle[2]/2);
            //printf("get angle!\n");
            break;
        default:
            break;
    }
}

void imu_jy61_hw::read()
{
	//ROS_INFO("in the read function");
    //get the size of char in buffer
    size_t hascharsize = imu_serial_port_.available();
    size_t charsize = 0;
    //ROS_INFO("hascharsize %i",hascharsize);
    //ROS_INFO("buffer size %i",readbuff.length());
    if(hascharsize != 0)
    {
       readbuff += imu_serial_port_.read(hascharsize);
       charsize = readbuff.length();
    }
    int head = 0;
    //get qualified string
    while((charsize>=12))
    {
    	charsize = readbuff.length();
    	//ROS_INFO("in the read loop %i",charsize);
        std::string frame = get_frame(head, readbuff);
        if(frame != "")
        {
            deal_frame(frame);
        }else
        {
            head ++ ;
        }
    }
}

void imu_jy61_hw::write()
{

#define PRINT_INFO
#ifdef PRINT_INFO
	//do some info print work
	/*
	for(int i =0;i<3;i++){
		fputs("\033[A\033[2K\033[A\033[2k",stdout);
	}
	rewind(stdout);
	ftruncate(1,0);
	*/
	//rewind(stdout);
	if(printed == 1){
		for(int i =0; i<6; i++)
		{
			printf("\033[1A\r\033[2K");
		}
	}
	std::string dir[]= {"x:  ", "y:  ","z:  "};
	// @orientation
	std::string orientation_message = std::string("orientation:\n") + std::string("[  ");
	for(int i=0;i<4;i++){
		std::ostringstream os;
		os<<orientation_[i];
		orientation_message += os.str() + std::string("  ");
	}
	orientation_message += std::string("]\n");
	//fputs(orientation_message.c_str(),stdout);
	printf(orientation_message.c_str());

	// @angular_velocity_
	std::string angular_velocity_message = std::string("angular_velocity:\n");
	for(int i=0;i<3;i++){
		std::ostringstream os;
		os<<angular_velocity_[i];
		angular_velocity_message += dir[i] + os.str() + std::string("  ");
	}
	angular_velocity_message += std::string("\n");
	//fputs(angular_velocity_message.c_str(),stdout);
	printf(angular_velocity_message.c_str());

	// @linear_acceleration_
	std::string linear_acceleration_message = std::string("linear acceleration:\n");

	for(int i=0;i<3;i++){
		std::ostringstream os;
		os<<linear_acceleration_[i];
		linear_acceleration_message += dir[i] + os.str() + std::string("  ");
	}
	linear_acceleration_message += std::string("\n");
	//fputs(linear_acceleration_message.c_str(),stdout);
	printf(linear_acceleration_message.c_str());
	printed = 1;
	//printf("0x33[1A\r\033[2K");
	//printf("\033[6A\r\033[2K");
	ROS_DEBUG("in the write function");
	//fflush(stdout);
#endif
}

