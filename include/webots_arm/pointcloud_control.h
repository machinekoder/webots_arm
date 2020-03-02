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
#include <queue>

#ifndef WEBOTS_ARM_POINTCLOUD_CONTROL_H
#define WEBOTS_ARM_POINTCLOUD_CONTROL_H
namespace webots_arm {
    class PointCloudControl {
    public:
        explicit PointCloudControl(std::string model_name);

        ~PointCloudControl();

    private:
        boost::shared_ptr<ros::NodeHandle> nh_;
        boost::shared_ptr<ros::NodeHandle> pnh_;

        ros::Subscriber pcl_sub_;
        ros::Subscriber image_sub_;
        ros::Timer publish_timer_;
        ros::ServiceClient camera_enable_client_;
        ros::ServiceClient pcl_enable_client_;
        ros::Publisher pcl_pub_;
        image_transport::CameraPublisher cam_pub_;
        sensor_msgs::CameraInfoPtr cam_info_msg_;

        std::string model_name_;

        // config
        std::string frame_id_;
        std::string camera_name_;
        int fps_;
        int buffer_queue_size_;
        float clip_distance_;
        bool enable_cloud_;

        int subscriber_num_;
        std::queue<sensor_msgs::ImagePtr> image_queue_;
        sensor_msgs::ImagePtr image_;
        std::queue<sensor_msgs::PointCloud2Ptr> pcl_queue_;
        sensor_msgs::PointCloud2Ptr pcl_;

        std::mutex s_mutex_;
        std::mutex img_q_mutex_;
        std::mutex pcl_q_mutex_;

        void pointCloudCallback(const sensor_msgs::PointCloudPtr &pcl);
        void imageCallback(const sensor_msgs::ImagePtr &image);
        void doPublish(const ros::TimerEvent& event);

        bool enableCamera(bool enable);
        bool enablePointCloud(bool enable);

        // subscribe/unsubscribe handling
        bool subscribe();
        bool unsubscribe();
        void connectionCallbackImpl();
        void disconnectionCallbackImpl();
    };
}
#endif //WEBOTS_ARM_POINTCLOUD_CONTROL_H
