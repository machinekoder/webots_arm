//
// Created by alexander on 29.09.19.
//

#ifndef ROS_WS_JOINT_CONTROL_H
#define ROS_WS_JOINT_CONTROL_H

#include <urdf/model.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace ros;

typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;
typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

namespace webots_arm {
    class JointControl {
    public:
        explicit JointControl(std::string model_name,
                              const urdf::Model &model = urdf::Model());

        ~JointControl();

    protected:
        virtual void callbackJointState(const JointStateConstPtr &state, const std::string &name);

        std::vector<Subscriber> joint_state_subs_{};
        std::string model_name_;

        std::map<std::string, ros::ServiceClient> joint_position_clients_{};
    };
}

#endif //ROS_WS_JOINT_CONTROL_H
