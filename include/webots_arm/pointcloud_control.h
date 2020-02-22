//
// Created by alexander on 29.09.19.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>

#ifndef WEBOTS_ARM_POINTCLOUD_CONTROL_H
#define WEBOTS_ARM_POINTCLOUD_CONTROL_H
namespace webots_arm {
    class PointCloudControl {
    public:
        explicit PointCloudControl(std::string model_name);

        ~PointCloudControl();

    protected:
        virtual void callbackPointCloud(const sensor_msgs::PointCloud &pcl);

        ros::Subscriber pcl_sub_;
        ros::Publisher pcl_pub_;
        std::string model_name_;
        std::string frame_id_;
        int subscriber_num_;

        ros::ServiceClient camera_enable_client_;
        ros::ServiceClient point_cloud_enable_client_;

    };
}
#endif //WEBOTS_ARM_POINTCLOUD_CONTROL_H
