//
// Created by alexander on 29.09.19.
//
#include <csignal>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <urdf/model.h>

#include "webots_arm/joint_control.h"
#include "webots_arm/camera_control.h"

using namespace std;
using namespace ros;

void quit(int sig) {
    //setTimeStepSrv.request.value = 0;
    //setTimeStepClient.call(setTimeStepSrv);
    ROS_INFO("User stopped the 'joint_control' node.");
    ros::shutdown();
    exit(0);
}

////////////////////////////////////////////
// Main
int main(int argc, char **argv) {
    std::string controllerName;
    ros::init(argc, argv, "joint_control", ros::init_options::AnonymousName);
    ros::NodeHandle node;

    signal(SIGINT, quit);

    std::vector<std::string> names;
    std::string modelName;

    ros::Subscriber nameSub = node.subscribe<std_msgs::String>("model_name", 100,
                                                               [&names](const std_msgs::String::ConstPtr &name) {
                                                                   names.push_back(name->data);
                                                               });
    while (names.empty() || names.size() < nameSub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    ros::spinOnce();
    if (names.size() == 1)
        modelName = names.at(0);
    else {
        int wantedModel = 0;
        std::cout << "Choose the # of the model you want to use:\n";
        std::cin >> wantedModel;
        if (1 <= wantedModel && wantedModel <= names.size())
            modelName = names.at(wantedModel);
        else {
            ROS_ERROR("Invalid choice.");
            return 1;
        }
    }
    nameSub.shutdown();

    // gets the location of the robot description on the parameter server
    urdf::Model model;
    if (!model.initParam("robot_description"))
        return 1;

    webots_arm::JointControl joint_control(modelName, model);
    webots_arm::CameraControl camera_control(modelName);
    ros::spin();

    return 0;
}
