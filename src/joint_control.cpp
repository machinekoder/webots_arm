#include <utility>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <urdf/model.h>

#include <webots_ros/set_float.h>

#include "webots_arm/joint_control.h"

using namespace std;
using namespace ros;

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

namespace webots_arm {
    JointControl::JointControl(std::string model_name,
                               const urdf::Model &model)
            : model_name_(std::move(model_name)) {
        ros::NodeHandle n;

        // Setting tcpNoNelay tells the subscriber to ask publishers that connect
        // to set TCP_NODELAY on their side. This prevents some joint_state messages
        // from being bundled together, increasing the latency of one of the messages.
        ros::TransportHints transport_hints;
        transport_hints.tcpNoDelay(true);


        for (auto &joint : model.joints_) {
            if (joint.second->type == urdf::Joint::FIXED) {
                continue;
            }
            // subscribe to joint state
            boost::function<void (const JointStateConstPtr&)> f (boost::bind(&JointControl::callbackJointState, this, _1, joint.first));
            joint_state_subs_.push_back(n.subscribe("joint_states", 1, f, VoidConstPtr(), transport_hints));
            // create service client
            joint_position_clients_[joint.first] = n.serviceClient<webots_ros::set_float>(
                    model_name_ + "/" + joint.first + "/set_position");
            ROS_INFO_STREAM("created joint position client " << joint.first);
        }
    }

    JointControl::~JointControl() = default;

    void JointControl::callbackJointState(const JointStateConstPtr &state, const std::string &name) {
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
            if (state->name[i] == name) {
                jointSrv.request.value = state->position[i];
                joint_position_clients_.at(name).call(jointSrv);
                break;
            }
        }
    }
}

////////////////////////////////////////////
// Main
int main(int argc, char **argv) {
    std::string controllerName;
    ros::init(argc, argv, "joint_control", ros::init_options::AnonymousName);

    const auto &msg = ros::topic::waitForMessage<std_msgs::String>("model_name", ros::Duration(5));
    std::string modelName = msg->data;
    ROS_DEBUG_STREAM("Using robot model " << modelName);

    // gets the location of the robot description on the parameter server
    urdf::Model model;
    if (!model.initParam("robot_description"))
        return 1;

    webots_arm::JointControl joint_control(modelName, model);
    ros::AsyncSpinner spinner(6);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
