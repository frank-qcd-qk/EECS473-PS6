#include <ros/ros.h>
#include <std_srvs/Trigger.h> // this message type is defined in the current package
#include <osrf_gear/ConveyorBeltControl.h>
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "cxq41_ariac_boxMotion");
    ros::NodeHandle n;
    //! System Startup Call
    ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("");
    std_srvs::Trigger startup_srv;
    startup_srv.response.success = false;
    startup_client.call(startup_srv);
    while(!startup_srv.response.success){
        ROS_WARN("Not successful startup yet... Repeat Calling...");
        startup_client.call(startup_srv);
        ros::Duration(1).sleep();
    }
    ROS_INFO("Competition Startup Success")
    // ! End of System Startup Call


    return 0;
}
