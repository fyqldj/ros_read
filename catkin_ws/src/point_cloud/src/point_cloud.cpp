#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 将ROS消息的点云数据转换为PCL格式
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 更新点云数据并刷新可视化界面
    viewer->updatePointCloud(cloud, "cloud");
    viewer->spinOnce();
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pcl_display_node");
    ros::NodeHandle nh;

    // 创建PCL可视化器
    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    // 创建一个空的点云，并添加到可视化器中进行显示
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    // 订阅点云数据的话题，并设置回调函数
    ros::Subscriber sub = nh.subscribe("/rslidar_points", 1000, cloudCallback);

    // 开始自循环（等待和处理ROS消息）
    ros::spin();

    return 0;
}
