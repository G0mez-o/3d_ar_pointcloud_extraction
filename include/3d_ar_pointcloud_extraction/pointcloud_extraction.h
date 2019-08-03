#ifndef POINTCLOUD_EXTRACTION_H
#define POINTCLOUD_EXTRACTION_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace pointcloud_extraction
{
    class PointCloud_Extraction
    {
        public:
        PointCloud_Extraction(ros::NodeHandle& nh);
        void publish(void);
        void extraction(const sensor_msgs::PointCloud2::ConstPtr& phoxi_point);
        void tflistener(std::string target_frame, std::string source_frame);

        public:
        pcl::PointCloud<pcl::PointXYZ> extracted_points;

        private:
        void updateARTransform(void);

        private:
        ros::NodeHandle nh_;
        ros::Subscriber point_sub_;
        std::vector<std::string> ar_link_name_vector_;
        ros::Publisher extracted_pc_pub_;
        std::vector<tf::StampedTransform> ar_tf_vector_;
        tf::TransformListener tf_listener_;
        tf::TransformListener tf_listener__;
        tf::StampedTransform transform_;
    };
}

#endif