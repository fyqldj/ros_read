#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
cv::Mat depthImage;  // 存储深度相机图像的全局变量


void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // 将ROS图像消息转换为OpenCV图像
  try
  {
    depthImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image");
  ros::NodeHandle nh;

  ros::Subscriber depthImageSub = nh.subscribe("/camera/depth/image_rect_raw", 1000, depthImageCallback);

  cv::namedWindow("Depth Image");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    // 显示深度相机图像
    if (!depthImage.empty())
    {
      cv::imshow("Depth Image", depthImage);
    }

    cv::waitKey(1);
    loop_rate.sleep();
  }

  return 0;
}
