# give authority to /dev/ttyUSB0  imu device
`$ sudo chmod 777 /dev/ttyUSB0`

# run the imu_jy61_hw_node
`rosrun imu_jy61_hw imu_jy61_hw_node`

# launch the imu_jy61_hw.launch
`roslaunch imu_jy61_hw imu_jy61_hw.launch`

# start the imu_sensor_controller
`rosservice call /controller_manager/switch_controller '{start_controllers:["imu_sensor_controller"], stop_controllers:[]}'`
