#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <string>

class MapConversionNode : public rclcpp::Node
{
public:
    MapConversionNode() : Node("mapconversion")
    {
        tf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tf_points2", 1000);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points2", 1000, std::bind(&MapConversionNode::pointcloudCallback, this, std::placeholders::_1));
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

       sensor_msgs::msg::PointCloud2 pcl_out;
       char s1[20]= "bodyy";

       try
        {
            // Check if the transform is available
            if (tf_buffer_->canTransform(s1, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(5.0)))
            {
                // Lookup the transform and apply it
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform(s1, msg->header.frame_id, msg->header.stamp);

                pcl_ros::transformPointCloud(s1, *msg, pcl_out, *tf_buffer_);
                tf_pub_->publish(pcl_out);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Transform from %s to %s not available after waiting for 5 seconds.", 
                            msg->header.frame_id.c_str(), s1);
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", 
                        msg->header.frame_id.c_str(), s1, ex.what());
        }


/*
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                s1, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(5.0));
//            pcl_ros::transformPointCloud(s1, transform_stamped, *msg, pcl_out);
//            tf_pub_->publish(pcl_out);
            std::cout << "AAAA" << std::endl;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform");
        }
*/
    }
    int actual_floor_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tf_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapConversionNode>());
    rclcpp::shutdown();
    return 0;
}
