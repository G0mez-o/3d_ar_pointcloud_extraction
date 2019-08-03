#include <3d_ar_pointcloud_extraction/pointcloud_extraction.h>

using pointcloud_extraction::PointCloud_Extraction;

PointCloud_Extraction::PointCloud_Extraction(ros::NodeHandle& nh) : nh_(nh), ar_link_name_vector_({"/ar_marker_0", "/ar_marker_2", "/ar_marker_3", "/ar_marker_5"})
{
    extracted_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/extracted_pointcloud", 10);
    point_sub_ = nh.subscribe("/photoneo_center/pointcloud", 1, &PointCloud_Extraction::extraction, this); 
}

void PointCloud_Extraction::extraction(const sensor_msgs::PointCloud2::ConstPtr& phoxi_points)
{
    PointCloud_Extraction::updateARTransform();
    PointCloud_Extraction::tflistener("photoneo_center_optical_frame", "base_link");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud;
    tf::Transform tf;
    tf.setOrigin(transform_.getOrigin());
    tf.setRotation(transform_.getRotation());
    pcl::fromROSMsg(*phoxi_points, *cloud);
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
    extracted_points.clear();
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = trans_cloud->points.begin(); pt < trans_cloud->points.end(); ++pt)
    {
        for (unsigned int i = 0; i < ar_tf_vector_.size(); ++i)
        {
            if ((*pt).x < ar_tf_vector_[i].getOrigin().x() + 0.04 && (*pt).x > ar_tf_vector_[i].getOrigin().x() - 0.04 
            && (*pt).y < ar_tf_vector_[i].getOrigin().y() + 0.04 && (*pt).y > ar_tf_vector_[i].getOrigin().y() - 0.04
            && (*pt).z < ar_tf_vector_[i].getOrigin().z() && (*pt).z > ar_tf_vector_[i].getOrigin().z() - 0.07)
            {
                pcl::PointXYZ buffer_point;
                buffer_point.x = (*pt).x;
                buffer_point.y = (*pt).y;
                buffer_point.z = (*pt).z;
                extracted_points.push_back(buffer_point);
            }
        }
    }

}

void PointCloud_Extraction::publish(void)
{
    auto msg = extracted_points.makeShared();
    msg->header.frame_id = "base_link";
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    // &extracted_points->header.frame_id = "base_link";
    // pcl_conversions::toPCL(ros::Time::now(), &extracted_points->header.stamp);
    extracted_pc_pub_.publish(msg);
}

void PointCloud_Extraction::tflistener(std::string target_frame, std::string source_frame)
{
    ros::Time time = ros::Time(0);
    try
    {
        tf_listener__.waitForTransform(target_frame, source_frame, time, ros::Duration(4.0));
        tf_listener__.lookupTransform(target_frame, source_frame, time, transform_);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
        ros::Duration(4.0).sleep();
    }
    
}

void PointCloud_Extraction::updateARTransform(void)
{
    while (ros::ok())
    {
        ar_tf_vector_.clear();
        try
        {
            for (unsigned int i = 0; i < ar_link_name_vector_.size(); ++i)
            {
                tf::StampedTransform ar_tf;
                tf_listener_.waitForTransform("/base_link", ar_link_name_vector_[i], ros::Time(0), ros::Duration(8.0));
                tf_listener_.lookupTransform("/base_link", ar_link_name_vector_[i], ros::Time(0), ar_tf);
                ar_tf_vector_.push_back(ar_tf);
            }
            break;
        }
        catch(tf::TransformException ex)
        {
            ROS_WARN("%s Retry...", ex.what());
            ros::Duration(1.0).sleep();
        }
        
    }
}