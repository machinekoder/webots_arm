//
// Created by alexander on 29.09.19.
//
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv_bridge/cv_bridge.h>

#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>

#include "webots_arm/pointcloud_control.h"

namespace webots_arm {
    PointCloudControl::PointCloudControl(std::string model_name)
            : model_name_(std::move(model_name)), subscriber_num_(0) {
        pnh_.reset(new ros::NodeHandle("~"));
        nh_.reset(new ros::NodeHandle());
        camera_name_ = pnh_->param<std::string>("camera_name", "camera");
        const auto camera_info_url = pnh_->param<std::string>("camera_info_url", "");
        frame_id_ = pnh_->param<std::string>("frame_id", "camera_link");
        fps_ = pnh_->param<int>("fps", 8);
        buffer_queue_size_ = pnh_->param<int>("buffer_queue_size", 1);
        clip_distance_ = pnh_->param<float>("clip_distance", 4.0);
        enable_cloud_ = pnh_->param<bool>("enable_cloud", false);

        ros::SubscriberStatusCallback connection_cb = boost::bind(&PointCloudControl::connectionCallbackImpl, this);
        ros::SubscriberStatusCallback disconnection_cb = boost::bind(&PointCloudControl::disconnectionCallbackImpl, this);
        image_transport::SubscriberStatusCallback img_connection_cb = boost::bind(&PointCloudControl::connectionCallbackImpl, this);
        image_transport::SubscriberStatusCallback img_disconnection_cb = boost::bind(&PointCloudControl::disconnectionCallbackImpl, this);

        camera_info_manager::CameraInfoManager cam_info_manager(*nh_, camera_name_, camera_info_url);
        // Get the saved camera info if any
        cam_info_msg_ = boost::make_shared<sensor_msgs::CameraInfo>(cam_info_manager.getCameraInfo());
        cam_info_msg_->header.frame_id = frame_id_;
        cam_pub_ = image_transport::ImageTransport(*pnh_).advertiseCamera("image_raw", 1, img_connection_cb, img_disconnection_cb,
                                                                         connection_cb, disconnection_cb);
        if (enable_cloud_) {
          pcl_pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("points", 1, connection_cb, disconnection_cb);
        }

        camera_enable_client_ = nh_->serviceClient<webots_ros::set_int>(model_name_ + "/" + camera_name_ + "/enable");
        if (enable_cloud_) {
          pcl_enable_client_ = nh_->serviceClient<webots_ros::set_bool>(model_name_ + "/"  + camera_name_ + "/enable_point_cloud");
        }
    }

    void PointCloudControl::pointCloudCallback(const sensor_msgs::PointCloudPtr &pcl) {
        std::lock_guard<std::mutex> guard(pcl_q_mutex_);

        sensor_msgs::PointCloud2Ptr pcl2(new sensor_msgs::PointCloud2);
        sensor_msgs::convertPointCloudToPointCloud2(*pcl, *pcl2);
        while (pcl_queue_.size() > buffer_queue_size_) {
            pcl_queue_.pop();
        }
        pcl_queue_.push(pcl2);
    }

    void PointCloudControl::imageCallback(const sensor_msgs::ImagePtr &image) {
        std::lock_guard<std::mutex> guard(img_q_mutex_);

        while (image_queue_.size() > buffer_queue_size_) {
            image_queue_.pop();
        }
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
        auto roi = cv_ptr->image;
        auto pixel = roi.ptr<float>(0, 0);
        const auto endPixel = pixel + roi.cols * roi.rows;
        for (; pixel != endPixel; pixel++)
        {
          if (*pixel >= clip_distance_)
          {
              *pixel = 0.0f;
          }
        }
        image_queue_.push(cv_ptr->toImageMsg());
    }

    void PointCloudControl::doPublish(const ros::TimerEvent &event) {
        {
            std::lock_guard<std::mutex> guard(img_q_mutex_);
            if (!image_queue_.empty()) {
                image_ = image_queue_.front();
                image_queue_.pop();
            }
        }
        if (enable_cloud_) {
            std::lock_guard<std::mutex> guard(pcl_q_mutex_);
            if (!pcl_queue_.empty()) {
                pcl_ = pcl_queue_.front();
                pcl_queue_.pop();
            }
        }
        if (image_) {
            image_->header.frame_id = frame_id_;

            // Create a default camera info if we didn't get a stored one on initialization
            if (cam_info_msg_->distortion_model.empty()) {
                ROS_WARN_STREAM("No calibration file given.");
            }
            // The timestamps are in sync thanks to this publisher
            cam_info_msg_->header.stamp = image_->header.stamp;
            cam_pub_.publish(image_, cam_info_msg_);
        }
        if (enable_cloud_ && pcl_) {
            pcl_->header.frame_id = frame_id_;
            pcl_pub_.publish(pcl_);
        }
    }

    bool PointCloudControl::enableCamera(bool enable) {
        webots_ros::set_int enable_srv;
        enable_srv.request.value = static_cast<int>(enable);
        if (!camera_enable_client_.call(enable_srv)) {
            return false;
        }
        return enable_srv.response.success;
    }

    bool PointCloudControl::enablePointCloud(bool enable) {
        webots_ros::set_bool enable_srv;
        enable_srv.request.value = enable;
        if (!pcl_enable_client_.call(enable_srv)) {
            return false;
        }
        return enable_srv.response.success;
    }

    bool PointCloudControl::subscribe() {
        ROS_INFO("Starting Depth Camera");
        std::cout << "Starting Depth Camera" << std::endl;

        if (!enableCamera(true)) {
            ROS_WARN("Starting camera was not successful.");
            return false;
        }

        if (enable_cloud_ && !enablePointCloud(true)) {
            ROS_WARN("Enabling point cloud was not successful.");
            return false;
        }

        ros::TransportHints transport_hints;
        transport_hints.tcpNoDelay(true);
        image_sub_ = nh_->subscribe(model_name_ + "/" + camera_name_ + "/range_image", 1, &PointCloudControl::imageCallback, this,
                                 transport_hints);
        if (enable_cloud_) {
            pcl_sub_ = nh_->subscribe(model_name_ + "/" + camera_name_ + "/point_cloud", 1,
                                      &PointCloudControl::pointCloudCallback, this, transport_hints);
        }

        publish_timer_ = nh_->createTimer(ros::Duration(1.0 / fps_), &PointCloudControl::doPublish, this);

        return true;
    }

    bool PointCloudControl::unsubscribe() {
        ROS_INFO("Stopping Depth Camera");
        std::cout << "Stopping Depth Camera" << std::endl;
        publish_timer_.stop();
        image_sub_.shutdown();
        if (enable_cloud_) {
            pcl_sub_.shutdown();
        }
        if (enable_cloud_ && !enablePointCloud(false)) {
            ROS_WARN("Stopping point cloud as not successful.");
        }
        if (!enableCamera(false)) {
            ROS_WARN("Stopping camera was not successful.");
            return false;
        }
        while (!image_queue_.empty()) {
            image_queue_.pop();
        }
        image_.reset();
        if (enable_cloud_) {
            while (!pcl_queue_.empty()) {
                pcl_queue_.pop();
            }
            pcl_.reset();
        }
        return true;
    }

    void PointCloudControl::connectionCallbackImpl() {
      std::lock_guard<std::mutex> lock(s_mutex_);
      subscriber_num_++;
      if (subscriber_num_ == 1) {
        ROS_DEBUG("subscribe");
        if (!subscribe()) {
          subscriber_num_--;
          ROS_DEBUG("not subscribed");
        }
        else {
          ROS_DEBUG("subscribed");
        }
      }
    }

    void PointCloudControl::disconnectionCallbackImpl() {
      std::lock_guard<std::mutex> lock(s_mutex_);
      ROS_DEBUG("disconnection callback");
      bool always_subscribe;
      if (!pnh_->getParamCached("always_subscribe", always_subscribe)) {
        always_subscribe = false;
      }
      if (always_subscribe) {
        return;
      }

      if (subscriber_num_ > 0) {
        ROS_DEBUG("unsubscribe");
        subscriber_num_--;
        if (subscriber_num_ == 0) {
          unsubscribe();
          ROS_DEBUG("unsubscribed");
        }
      }
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

#if ASYNC
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
#else
    webots_arm::PointCloudControl cameraControl(modelName);
    ros::spin();
#endif

    return 0;
}
