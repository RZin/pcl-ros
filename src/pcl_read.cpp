#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <ros/console.h>
//using namespace ros;
//using namespace std;
//std::string PATH = "/home/mobrob/lechuga/mnt/Vol0/datasets/L_CAS_3D_Point_Cloud_People_Dataset/3D Point Cloud People Dataset/LCAS_20160523_1200_1218_pcd";
//std::string PATH = "/home/mobrob/catkin_ws/src/pcl_tut/data/LCAS_vsevolod.pcd";
std::string PATH = "/home/mobrob/catkin_ws/src/pcl_tut/data/test_pcd.pcd";
//std::string PATH = "/home/mobrob/data/sk/vlp_blue_hall_2019-12-12-13-18-30/1576145910.881324530.pcd";
//std::string PATH = "/home/mobrob/cpp/pcl_practice/tutorials_flat/pointclouds/Statues_4_clustered.pcd";

main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_read");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::PointCloud<pcl::PointXYZI> cloud_i;


    pcl::io::loadPCDFile (PATH, cloud); // "test_pcd.pcd"
//    pcl::io::loadPCDFile (PATH, cloud_i); // "test_pcd.pcd"

    pcl::toROSMsg(cloud, output);
//    pcl::toROSMsg(cloud_i, output);

    output.header.frame_id = "odom";

    ros::Rate loop_rate(0.01);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_DEBUG_STREAM("cloud size is/ :" << cloud.points.size());
//        ROS_DEBUG_STREAM("cloud_i size is/ :" << cloud_i.points.size());
    }

    return 0;
}

