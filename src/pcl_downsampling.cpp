#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <stdio.h>                              // sscanf
#include <iostream>
using namespace std;

float LEAFSIZE_X = 0.01f; // 0.5f
float LEAFSIZE_Y = 0.01f; // 0.5f
float LEAFSIZE_Z = 0.01f;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_filtered", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud.makeShared());
        voxelSampler.setLeafSize(LEAFSIZE_X, LEAFSIZE_Y, LEAFSIZE_Z);
//        voxelSampler.setLeafSize(LEAFSIZE, LEAFSIZE, LEAFSIZE);
//        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(cloud_downsampled);

        pcl::toROSMsg(cloud_downsampled, output);
        pcl_pub.publish(output);

    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    if(argc < 1) {
        printf("no LEAFSIZE given");
    }

//    sscanf(argv[1], "%f", &LEAFSIZE);   // Argument 1 is LeafSize of voxelgrid filter

//    cout << "argc:" << argc << endl;
//    cout << "LEAFSIZE:" << LEAFSIZE << endl;

    ros::init(argc, argv, "pcl_downsampling");

    ROS_DEBUG_STREAM("LEAFSIZE_X: " << LEAFSIZE_X);
    ROS_DEBUG_STREAM("LEAFSIZE_Y: " << LEAFSIZE_Y);
    ROS_DEBUG_STREAM("LEAFSIZE_Z: " << LEAFSIZE_Z);

    cloudHandler handler;

    ros::spin();

    return 0;
}

