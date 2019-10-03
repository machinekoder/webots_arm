//
// Created by alexander on 29.09.19.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mutex>

#ifndef WEBOTS_ARM_CAMERA_CONTROL_H
#define WEBOTS_ARM_CAMERA_CONTROL_H
namespace webots_arm {
    class CameraControl {
    public:
        explicit CameraControl(std::string model_name);

        ~CameraControl();

    protected:
        virtual void callbackImage(const sensor_msgs::ImagePtr &image);

        ros::Subscriber image_sub_;
        image_transport::CameraPublisher pub_;
        sensor_msgs::CameraInfoPtr cam_info_msg_;
        std::string model_name_;
        std::string frame_id_;
        int subscriber_num_;

        ros::ServiceClient camera_enable_client_;

    private:
        std::mutex s_mutex;

        sensor_msgs::CameraInfoPtr get_default_camera_info_from_image(const sensor_msgs::ImageConstPtr &img) const;

        // subscribe/unsubscribe handling
        void subscribe();
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
