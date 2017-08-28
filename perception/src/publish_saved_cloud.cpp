//
// Created by timadamson on 7/12/17.
//

#include <iostream>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

void print_usage() {
    std::cout << "Publishes a point cloud from a bag file located NAME " << std::endl;
    std::cout << "Usage: rosrun perception publish_saved_cloud NAME" << std::endl;
}

sensor_msgs::PointCloud2ConstPtr readCloud(std::string filePath){
    rosbag::Bag bag(filePath);
    rosbag::View view(bag);

    foreach(rosbag::MessageInstance const m, view){
                    sensor_msgs::PointCloud2ConstPtr cloudPtr = m.instantiate<sensor_msgs::PointCloud2>();
                    if(cloudPtr != NULL){
                        return cloudPtr;
                    }
                }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "publish_saved_cloud");
    if(argc < 2){
        print_usage();
        return 1;
    }

    std::string fileName(argv[1]);
    std::cout << "Getting point cloud from " << fileName << std::endl ;
    sensor_msgs::PointCloud2ConstPtr cloudPtr = readCloud(fileName);

    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("odom", cloudPtr->header.frame_id, ros::Time(0), ros::Duration(5.0));
    tf::StampedTransform transform;
    try {
        std::cout << "trying" << std::endl;
        tf_listener.lookupTransform("odom", cloudPtr->header.frame_id,
                                    ros::Time(0), transform);
    } catch (tf::LookupException& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    } catch (tf::ExtrapolationException& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    std::cout << "Making progress" << std::endl;
    sensor_msgs::PointCloud2 cloud_out;
    pcl_ros::transformPointCloud("odom", transform, *cloudPtr, cloud_out);

    ros::NodeHandle n;
    ros::Publisher cloudPub = n.advertise<sensor_msgs::PointCloud2>("saved_points", 2);
    ros::Rate loop_rate(1);

    while(ros::ok()){
        std::cout << "publishing" << std::endl;
        cloudPub.publish(cloud_out);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

