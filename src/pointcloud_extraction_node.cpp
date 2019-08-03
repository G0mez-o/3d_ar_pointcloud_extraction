#include <3d_ar_pointcloud_extraction/pointcloud_extraction.h>

using pointcloud_extraction::PointCloud_Extraction;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_extraction");
    ros::NodeHandle nh;

    PointCloud_Extraction extractor(nh);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        extractor.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}