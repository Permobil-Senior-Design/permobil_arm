//Converts published pointclouds to the baselink transform, as the GPD does not have direct access to TF

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sstream>


sensor_msgs::PointCloud2 cloud_msg;
sensor_msgs::PointCloud2 out_msg;
void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  cloud_msg=*msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_converter");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    sleep(5);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, callback);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/gpd_pcd", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok()){
        
        //for some reason this doesn't work
        //pcl_ros::transformPointCloud("link_base",cloud_msg,out_msg,tfBuffer);
        
        geometry_msgs::TransformStamped transform;
        
        try{
          transform = tfBuffer.lookupTransform ("link_base", "camera_depth_optical_frame", cloud_msg.header.stamp, ros::Duration(3.0));
          pcl_ros::transformPointCloud("link_base", transform.transform, cloud_msg, out_msg);
          pub.publish(out_msg);
        } catch(...){
          ROS_INFO_NAMED("pcl_converter", "no transform found");
        }
        ros::spinOnce();

        loop_rate.sleep();
    }
}