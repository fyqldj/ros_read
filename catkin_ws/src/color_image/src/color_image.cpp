#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat colorImage;  // 存储颜色相机图像的全局变量

void colorImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // 将ROS图像消息转换为OpenCV图像
  try
  {
    colorImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_image");
  ros::NodeHandle nh;

  ros::Subscriber colorImageSub = nh.subscribe("/camera/color/image_raw", 1000, colorImageCallback);

  cv::namedWindow("Color Image");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    // 显示颜色相机图像
    if (!colorImage.empty())
    {
      cv::imshow("Color Image", colorImage);
    }

    cv::waitKey(1);
    loop_rate.sleep();
  }

  return 0;
}
