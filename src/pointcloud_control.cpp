//
// Created by alexander on 29.09.19.
//
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>

#include "webots_arm/pointcloud_control.h"

namespace webots_arm {
    PointCloudControl::PointCloudControl(std::string model_name)
            : model_name_(std::move(model_name)), subscriber_num_(0) {
        ros::NodeHandle nh("~");
        ros::NodeHandle n;
        const auto image_topic = nh.param<std::string>("image_topic", "/camera/point_cloud");
        const auto camera_name = nh.param<std::string>("camera_name", "camera");
        frame_id_ = nh.param<std::string>("frame_id", "camera_link");

        // Setting tcpNoNelay tells the subscriber to ask publishers that connect
        // to set TCP_NODELAY on their side. This prevents some joint_state messages
        // from being bundled together, increasing the latency of one of the messages.
        ros::TransportHints transport_hints;
        transport_hints.tcpNoDelay(true);
        pcl_sub_ = nh.subscribe(model_name_ + "/" + camera_name + "/point_cloud", 5,
                &PointCloudControl::callbackPointCloud, this, transport_hints);

        pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>(image_topic, 5);

        camera_enable_client_ = n.serviceClient<webots_ros::set_int>(model_name_ + "/" + camera_name + "/enable");
        point_cloud_enable_client_ = n.serviceClient<webots_ros::set_bool>(model_name_ + "/"  + camera_name + "/enable_point_cloud");

        webots_ros::set_int enableReq;
        enableReq.request.value = 1;
        camera_enable_client_.call(enableReq);
        webots_ros::set_bool enableReq2;
        enableReq2.request.value = true;
        point_cloud_enable_client_.call(enableReq2);
    }

    void PointCloudControl::callbackPointCloud(const sensor_msgs::PointCloud &pcl) {
        sensor_msgs::PointCloud2 pcl2;
        sensor_msgs::convertPointCloudToPointCloud2(pcl, pcl2);
        pcl2.header.frame_id = frame_id_;
        pcl_pub_.publish(pcl2);
    }

    PointCloudControl::~PointCloudControl() = default;
}

////////////////////////////////////////////
// Main
int main(int argc, char **argv) {
    std::string controllerName;
    ros::init(argc, argv, "depth_camera_control");

    const auto &msg = ros::topic::waitForMessage<std_msgs::String>("model_name", ros::Duration(5));
    std::string modelName = msg->data;
    ROS_DEBUG_STREAM("Using robot model " << modelName);

    webots_arm::PointCloudControl cameraControl(modelName);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
