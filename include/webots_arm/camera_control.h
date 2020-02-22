//
// Created by alexander on 29.09.19.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mutex>
#include <queue>

#ifndef WEBOTS_ARM_CAMERA_CONTROL_H
#define WEBOTS_ARM_CAMERA_CONTROL_H
namespace webots_arm {
    class CameraControl {
    public:
        explicit CameraControl(std::string model_name);

        ~CameraControl();

        //protected:
    private:
        ros::Subscriber image_sub_;
        ros::Timer publish_timer_;
        ros::ServiceClient camera_enable_client_;
        image_transport::CameraPublisher pub_;
        sensor_msgs::CameraInfoPtr cam_info_msg_;

        std::string model_name_;

        // config
        std::string frame_id_;
        std::string camera_name_;
        int fps_;
        int buffer_queue_size_;

        int subscriber_num_;
        std::queue<sensor_msgs::ImagePtr> image_queue_;
        sensor_msgs::ImagePtr image_;

        std::mutex s_mutex;
        std::mutex q_mutex;

        sensor_msgs::CameraInfoPtr get_default_camera_info_from_image(const sensor_msgs::ImageConstPtr &img) const;

        void imageCallback(const sensor_msgs::ImagePtr &image);
        void doPublish(const ros::TimerEvent& event);

        // subscribe/unsubscribe handling
        bool subscribe();
        void unsubscribe();
        void connectionCallbackImpl();
        void disconnectionCallbackImpl();
        void connectionCallback(const image_transport::SingleSubscriberPublisher&);
        void infoConnectionCallback(const ros::SingleSubscriberPublisher&);
        void disconnectionCallback(const image_transport::SingleSubscriberPublisher&);
        void infoDisconnectionCallback(const ros::SingleSubscriberPublisher&);
    };
}
#endif //WEBOTS_ARM_CAMERA_CONTROL_H
