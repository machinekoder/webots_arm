//
// Created by alexander on 29.09.19.
//
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>

#include <webots_ros/set_int.h>

#include "webots_arm/camera_control.h"

namespace webots_arm {
    CameraControl::CameraControl(std::string model_name)
            : model_name_(std::move(model_name)), subscriber_num_(0) {
        ros::NodeHandle nh("~");
        ros::NodeHandle n;
        const auto image_topic = nh.param<std::string>("image_topic", "/camera/image_raw");
        const auto camera_name = nh.param<std::string>("camera_name", "camera");
        const auto camera_info_url = nh.param<std::string>("camera_info_url", "");
        frame_id_ = nh.param<std::string>("frame_id", "camera_link");

        // initialize camera info publisher
        camera_info_manager::CameraInfoManager cam_info_manager(n, camera_name, camera_info_url);
        // Get the saved camera info if any
        cam_info_msg_ = boost::make_shared<sensor_msgs::CameraInfo>(cam_info_manager.getCameraInfo());
        cam_info_msg_->header.frame_id = frame_id_;

        image_transport::SubscriberStatusCallback connect_cb =
                boost::bind(&CameraControl::connectionCallback, this, _1);
        ros::SubscriberStatusCallback info_connect_cb =
                boost::bind(&CameraControl::infoConnectionCallback, this, _1);
        image_transport::SubscriberStatusCallback disconnect_cb =
                boost::bind(&CameraControl::disconnectionCallback, this, _1);
        ros::SubscriberStatusCallback info_disconnect_cb =
                boost::bind(&CameraControl::infoDisconnectionCallback, this, _1);
        pub_ = image_transport::ImageTransport(n).advertiseCamera(image_topic, 1, connect_cb, disconnect_cb,
                                                                  info_connect_cb, info_disconnect_cb);

        // Setting tcpNoNelay tells the subscriber to ask publishers that connect
        // to set TCP_NODELAY on their side. This prevents some joint_state messages
        // from being bundled together, increasing the latency of one of the messages.
        ros::TransportHints transport_hints;
        transport_hints.tcpNoDelay(true);
        image_sub_ = n.subscribe(model_name_ + "/camera/image", 1, &CameraControl::callbackImage, this,
                                 transport_hints);

        camera_enable_client_ = n.serviceClient<webots_ros::set_int>(model_name_ + "/camera/enable");
    }

    void CameraControl::subscribe() {
        ROS_INFO("Starting Camera");
        camera_enable_client_.waitForExistence();
        webots_ros::set_int enable_srv;
        enable_srv.request.value = true;
        camera_enable_client_.call(enable_srv);
    }

    void CameraControl::unsubscribe() {
        ROS_INFO("Stopping Camera");
        camera_enable_client_.waitForExistence();
        webots_ros::set_int enable_srv;
        enable_srv.request.value = false;
        camera_enable_client_.call(enable_srv);
    }

    void CameraControl::connectionCallbackImpl() {
        std::lock_guard<std::mutex> lock(s_mutex);
        subscriber_num_++;
        if (subscriber_num_ == 1) {
            subscribe();
        }
    }

    void CameraControl::disconnectionCallbackImpl() {
        std::lock_guard<std::mutex> lock(s_mutex);
        subscriber_num_--;
        if (subscriber_num_ == 0) {
            unsubscribe();
        }
    }

    void CameraControl::connectionCallback(const image_transport::SingleSubscriberPublisher &) {
        connectionCallbackImpl();
    }

    void CameraControl::infoConnectionCallback(const ros::SingleSubscriberPublisher &) {
        connectionCallbackImpl();
    }

    void CameraControl::disconnectionCallback(const image_transport::SingleSubscriberPublisher &) {
        disconnectionCallbackImpl();
    }

    void CameraControl::infoDisconnectionCallback(const ros::SingleSubscriberPublisher &) {
        disconnectionCallbackImpl();
    }

    sensor_msgs::CameraInfoPtr
    CameraControl::get_default_camera_info_from_image(const sensor_msgs::ImageConstPtr &img) const {
        auto cam_info_msg = boost::make_shared<sensor_msgs::CameraInfo>();
        cam_info_msg->header.frame_id = img->header.frame_id;
        // Fill image size
        cam_info_msg->height = img->height;
        cam_info_msg->width = img->width;
        ROS_INFO_STREAM("The image width is: " << img->width);
        ROS_INFO_STREAM("The image height is: " << img->height);
        // Add the most common distortion model as sensor_msgs/CameraInfo says
        cam_info_msg->distortion_model = "plumb_bob";
        // Don't let distorsion matrix be empty
        cam_info_msg->D.resize(5, 0.0);
        // Give a reasonable default intrinsic camera matrix
        cam_info_msg->K = boost::assign::list_of(1.0)(0.0)(img->width / 2.0)
                (0.0)(1.0)(img->height / 2.0)
                (0.0)(0.0)(1.0);
        // Give a reasonable default rectification matrix
        cam_info_msg->R = boost::assign::list_of(1.0)(0.0)(0.0)
                (0.0)(1.0)(0.0)
                (0.0)(0.0)(1.0);
        // Give a reasonable default projection matrix
        cam_info_msg->P = boost::assign::list_of(1.0)(0.0)(img->width / 2.0)(0.0)
                (0.0)(1.0)(img->height / 2.0)(0.0)
                (0.0)(0.0)(1.0)(0.0);
        return cam_info_msg;
    }

    void CameraControl::callbackImage(const sensor_msgs::ImagePtr &image) {
        image->header.frame_id = frame_id_;

        // Create a default camera info if we didn't get a stored one on initialization
        if (cam_info_msg_->distortion_model.empty()) {
            ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
            cam_info_msg_ = get_default_camera_info_from_image(image);
        }
        // The timestamps are in sync thanks to this publisher
        cam_info_msg_->header.stamp = image->header.stamp;
        pub_.publish(image, cam_info_msg_);
    }

    CameraControl::~CameraControl() = default;
}
