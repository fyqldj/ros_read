  #include <iostream>
  #include <ros/ros.h>
  #include <sensor_msgs/Imu.h>

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
      // 处理IMU数据，这里只是简单地打印数据到命令行
      std::cout<<"Received Header data: "<<std::endl;
      std::cout << "    uint32 seq: " << msg->header.seq << std::endl;
      std::cout << "    time stamp: " << msg->header.stamp << std::endl;
      std::cout << "    string frame_id: " << msg->header.frame_id << std::endl;
      std::cout<<"Received IMU data: orientation (x, y, z)"<<std::endl;
      std::cout << "    float64 x: " << msg->orientation.x << std::endl;
      std::cout << "    float64 y: " << msg->orientation.y << std::endl;
      std::cout << "    float64 z: " << msg->orientation.z << std::endl;
      std::cout << "    float64 w: " << msg->orientation.w << std::endl;
      std::cout<<"Received IMU data: angular velocity (x, y, z)"<<std::endl;
      std::cout << "    float64 x: " << msg->angular_velocity.x << std::endl;
      std::cout << "    float64 y: " << msg->angular_velocity.y << std::endl;
      std::cout << "    float64 z: " << msg->angular_velocity.z << std::endl;
      std::cout<<"Received IMU data: linear acceleration (x, y, z)"<<std::endl;
      std::cout << "    float64 x: " << msg->linear_acceleration.x << std::endl;
      std::cout << "    float64 y: " << msg->linear_acceleration.y << std::endl;
      std::cout << "    float64 z: " << msg->linear_acceleration.z << std::endl;
  }

  int main(int argc, char** argv)
  {
      ros::init(argc, argv, "imu_topic");  // 初始化ROS节点
      ros::NodeHandle nh;  // 创建节点句柄

      // 创建订阅器，订阅IMU和里程计主题
      ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 1000, imuCallback);

      ros::spin();  // 循环等待接收ROS消息

      return 0;
  }