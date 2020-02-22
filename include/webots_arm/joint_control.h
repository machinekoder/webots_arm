//
// Created by alexander on 29.09.19.
//

#ifndef ROS_WS_JOINT_CONTROL_H
#define ROS_WS_JOINT_CONTROL_H

#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace webots_arm {
    class JointControl {
    public:
        explicit JointControl(std::string model_name);

        ~JointControl();

    private:
        virtual void jointStateCallback(const sensor_msgs::JointStateConstPtr &state);

        ros::Subscriber joint_state_sub_;
        ros::ServiceClient joint_position_client_;
        std::string model_name_;
        std::string name_;

        std::mutex s_mutex_;
    };
}

#endif //ROS_WS_JOINT_CONTROL_H
