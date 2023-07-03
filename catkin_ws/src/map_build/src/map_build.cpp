#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

void callback(const nav_msgs::OccupancyGrid::ConstPtr& ptr)
{   
    std::cout << "resolution: " << ptr->info.resolution << std::endl;
    std::cout << "width: " << ptr->info.width << std::endl;
    std::cout << "height: " << ptr->info.height << std::endl;
    double scale = 1.0;
    int width = 1200;
    int height = 1200;
    cv::Point offset = {-1600, -1600};
    cv::Mat map = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
    for (int i = 0; i < ptr->info.width * ptr->info.height; ++i) {
        int x = (i % ptr->info.width + offset.x) * scale, y = (i / ptr->info.width + offset.y) * scale;
        if (ptr->data[i] == -1) {
            cv::circle(map, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
        } else if (ptr->data[i] >= 80) {
            cv::circle(map, cv::Point(x, y), 3, cv::Scalar(0, 0, 0), -1);
        } else {
            cv::circle(map, cv::Point(x, y), 3, cv::Scalar(255, 0, 255), -1);
        }    
    }
    cv::imshow("map", map);
    cv::waitKey(10);
    return ;
}

void LaunchGMapping()
{
    system("rosrun gmapping slam_gmapping");
}

int main(int argc, char** argv)
{
    std::thread gm(LaunchGMapping);
    ros::init(argc, argv, "show_map");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/map", 1000, callback);
    
    // 创建可移动和可调整大小的窗口
    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::resizeWindow("map", 800, 600);

    ros::spin();

    // 销毁窗口
    cv::destroyWindow("map");

    return 0;
}
