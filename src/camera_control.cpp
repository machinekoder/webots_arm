//
// Created by alexander on 29.09.19.
//
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <camera_info_manager/camera_info_manager.h>

#include <webots_ros/set_int.h>

#include "webots_arm/camera_control.h"

namespace webots_arm {
    CameraControl::CameraControl(std::string model_name)
            : model_name_(std::move(model_name)), subscriber_num_(0) {
        ros::NodeHandle nh("~");
        ros::NodeHandle n;
        camera_name_ = nh.param<std::string>("camera_name", "camera");
        const auto camera_info_url = nh.param<std::string>("camera_info_url", "");
        frame_id_ = nh.param<std::string>("frame_id", "camera_link");
        fps_ = nh.param<int>("fps", 8);
        buffer_queue_size_ = nh.param<int>("buffer_queue_size", 1);

        // initialize camera info publisher
        camera_info_manager::CameraInfoManager cam_info_manager(n, camera_name_, camera_info_url);
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
        pub_ = image_transport::ImageTransport(nh).advertiseCamera("image_raw", 1, connect_cb, disconnect_cb,
                                                                  info_connect_cb, info_disconnect_cb);

        camera_enable_client_ = n.serviceClient<webots_ros::set_int>(model_name_ + "/" + camera_name_ + "/enable");
        camera_enable_client_.waitForExistence();
    }

    bool CameraControl::subscribe() {
        ROS_INFO("Starting Camera");
        std::cout << "Starting Camera" << std::endl;
        webots_ros::set_int enable_srv;
        enable_srv.request.value = 1;
        if (camera_enable_client_.call(enable_srv)) {
            bool success = enable_srv.response.success;
            if (!success) {
                ROS_WARN("Calling service to start camera was not successful");
                return false;
            }
            else {
                ros::NodeHandle n;
                // Setting tcpNoNelay tells the subscriber to ask publishers that connect
                // to set TCP_NODELAY on their side. This prevents some joint_state messages
                // from being bundled together, increasing the latency of one of the messages.
                ros::TransportHints transport_hints;
                transport_hints.tcpNoDelay(true);
                image_sub_ = n.subscribe(model_name_ + "/" + camera_name_ + "/image", 1, &CameraControl::imageCallback, this,
                                         transport_hints);

                publish_timer_ = n.createTimer(
                ros::Duration(1.0 / fps_), &CameraControl::doPublish, this);
            }
        }
        else {
            ROS_ERROR("Failed to call service to start camera.");
            return false;
        }
        return true;
    }

    void CameraControl::unsubscribe() {
        ROS_INFO("Stopping Camera");
        std::cout << "Stopping Camera" << std::endl;
        publish_timer_.stop();
        image_sub_.shutdown();
        webots_ros::set_int enable_srv;
        enable_srv.request.value = 0;
        if (camera_enable_client_.call(enable_srv)) {
            bool success = enable_srv.response.success;
            if (!success) {
                ROS_WARN("Calling service to stop camera was not successful");
            }
        }
        else {
            ROS_ERROR("Failed to call service to stop camera.");
        }
        while (!image_queue_.empty()) {
            image_queue_.pop();
        }
        image_.reset();
    }

    void CameraControl::connectionCallbackImpl() {
        std::lock_guard<std::mutex> lock(s_mutex);
          subscriber_num_++;
          if (subscriber_num_ == 1) {
              if (!subscribe()) {
                  subscriber_num_--;
                  ROS_DEBUG("not subscribed");
                  std::cout << "not subscribed" << std::endl;
              }
              else {
                  ROS_DEBUG("subscribed");
                  std::cout << "subscribed" << std::endl;
              }
          }
    }

    void CameraControl::disconnectionCallbackImpl() {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (subscriber_num_ > 0) {
            subscriber_num_--;
            if (subscriber_num_ == 0) {
                unsubscribe();
            }
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
        // Don't let distortion matrix be empty
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

    void CameraControl::imageCallback(const sensor_msgs::ImagePtr &image) {
        std::lock_guard<std::mutex> guard(q_mutex);

        while (image_queue_.size() > buffer_queue_size_) {
            image_queue_.pop();
        }
        image_queue_.push(image);
    }

    void CameraControl::doPublish(const ros::TimerEvent &event) {
        {
            std::lock_guard<std::mutex> guard(q_mutex);
            if (!image_queue_.empty()) {
                image_ = image_queue_.front();
                image_queue_.pop();
            }
        }

        if (image_) {
            image_->header.frame_id = frame_id_;

            // Create a default camera info if we didn't get a stored one on initialization
            if (cam_info_msg_->distortion_model.empty()) {
                ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
                cam_info_msg_ = get_default_camera_info_from_image(image_);
            }
            // The timestamps are in sync thanks to this publisher
            cam_info_msg_->header.stamp = image_->header.stamp;
            pub_.publish(image_, cam_info_msg_);
        }
    }

    CameraControl::~CameraControl() = default;
}

////////////////////////////////////////////
// Main
int main(int argc, char **argv) {
    std::string controllerName;
    ros::init(argc, argv, "camera_control");

    const auto &msg = ros::topic::waitForMessage<std_msgs::String>("model_name", ros::Duration(5));
    std::string modelName = msg->data;
    ROS_INFO_STREAM("Using robot model " << modelName);

    webots_arm::CameraControl cameraControl(modelName);
    ros::spin();
    //ros::AsyncSpinner spinner(2);
    //spinner.start();
    //ros::waitForShutdown();

    return 0;
}
