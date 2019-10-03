// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// line_following example
// this example reproduce the e-puck_line_demo example
// but uses the ROS controller on the e-puck instead.
// The node connect to an e-puck and then uses values from its sensors
// to follow and line and get around obstacles.
// the duration of the example is given as argument to the node.

#include <ros/ros.h>

#include <csignal>
#include <utility>

#include <std_msgs/String.h>

#include <webots_ros/set_float.h>

#include "webots_arm/joint_control.h"

using namespace std;
using namespace ros;

#define TIME_STEP = 32

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
        // subscribe to joint state
        joint_state_sub_ = n.subscribe("joint_states", 1, &JointControl::callbackJointState, this, transport_hints);

        for (auto &joint : model.joints_) {
            if (joint.second->type == urdf::Joint::FIXED) {
                continue;
            }
            joint_position_clients_[joint.first] = n.serviceClient<webots_ros::set_float>(
                    model_name_ + "/" + joint.first + "/set_position");
            ROS_INFO_STREAM("created joint position client " << joint.first);
        }
    }

    JointControl::~JointControl() = default;

    void JointControl::callbackJointState(const JointStateConstPtr &state) {
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
            const auto &name = state->name[i];
            if (joint_position_clients_.count(name)) {
                jointSrv.request.value = state->position[i];
                joint_position_clients_.at(name).call(jointSrv);
            }
        }
    }
}

