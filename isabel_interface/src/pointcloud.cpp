#include "rclcpp/rclcpp.hpp"
#include "pcl_ros/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <string>
//#include <iostream>
//#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tf_pub;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
//int actual_floor;

void pointcloudcallback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
       sensor_msgs::msg::PointCloud2 pcl_out;
       char s1[20]= "bodyy";
 	//char s2[10]="number";
	//std::stringstream ss;
	//ss<<actual_floor;
	//ss>>s2;
	//std::strcat(s1,s2);

       //tf_listener->waitForTransform(s1 ,(*msg).header.frame_id, (*msg).header.stamp, ros::Duration(5.0));
       geometry_msgs::msg::TransformStamped transform_stamped;
       transform_stamped = tf_buffer->lookupTransform(s1, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(5.0));

       //pcl_ros::transformPointCloud(s1, *msg, pcl_out , *tf_listener);
       pcl_ros::transformPointCloud(s1, transform_stamped, *msg, pcl_out);

       tf_pub->publish(pcl_out);
}

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
auto nh = rclcpp::Node::make_shared("mapconversion");
//ros::NodeHandle nh;
//ros::NodeHandle n1;

//ros::param::get("~floor", actual_floor);

//std::cout << actual_floor << std::endl;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
//sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/points2", 1000, std::bind(&pointcloudcallback, nh, std::placeholders::_1));
sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/points2", 1000, [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pointcloudcallback(msg); });

//tf_pub = nh.advertise<sensor_msgs::PointCloud2> ("tf_points2", 1000);

tf_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("tf_points2", 1000);

//tf_buffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
//tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

//tf_listener    = new tf::TransformListener();

rclcpp::spin(nh);
rclcpp::shutdown();

}
