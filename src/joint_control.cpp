#include <utility>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <webots_ros/set_float.h>

#include "webots_arm/joint_control.h"

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

namespace webots_arm {
    JointControl::JointControl(std::string model_name)
            : model_name_(std::move(model_name)) {
        ros::NodeHandle nh("~");
        ros::NodeHandle n;

        name_ = nh.param<std::string>("name", "joint_1");

        // Setting tcpNoNelay tells the subscriber to ask publishers that connect
        // to set TCP_NODELAY on their side. This prevents some joint_state messages
        // from being bundled together, increasing the latency of one of the messages.
        ros::TransportHints transport_hints;
        transport_hints.tcpNoDelay(true);
        // subscribe to joint state
        joint_state_sub_ = n.subscribe("joint_states", 1, &JointControl::jointStateCallback, this, transport_hints);
        // create service client
        joint_position_client_ = n.serviceClient<webots_ros::set_float>(
            model_name_ + "/" + name_ + "/set_position");
        joint_position_client_.waitForExistence();
        ROS_INFO_STREAM("created joint position client " << name_);
    }

    JointControl::~JointControl() = default;

    void JointControl::jointStateCallback(const sensor_msgs::JointStateConstPtr &state) {
        std::unique_lock<std::mutex> lock(s_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return;

        if (state->name.size() != state->position.size()) {
            if (state->position.empty()) {
                const int throttleSeconds = 300;
                ROS_WARN_THROTTLE(throttleSeconds,
                                  "Ignored a JointState message about joint(s) "
                                  "\"%s\"(,...) whose position member was empty. This message will "
                                  "not reappear for %d seconds.", state->name[0].c_str(),
                                  throttleSeconds);
            } else {
                ROS_ERROR("Ignored an invalid JointState message");
            }
            return;
        }

        webots_ros::set_float jointSrv;
        for (size_t i = 0; i < state->name.size(); i++) {
            if (state->name[i] == name_) {
                jointSrv.request.value = state->position[i];
                joint_position_client_.call(jointSrv);
                break;
            }
        }
    }
}

////////////////////////////////////////////
// Main
int main(int argc, char **argv) {
    std::string controllerName;
    ros::init(argc, argv, "joint_control");

    const auto &msg = ros::topic::waitForMessage<std_msgs::String>("model_name", ros::Duration(5));
    std::string modelName = msg->data;
    ROS_INFO_STREAM("Using robot model " << modelName);

    webots_arm::JointControl joint_control(modelName);
    ros::spin();

    return 0;
}
