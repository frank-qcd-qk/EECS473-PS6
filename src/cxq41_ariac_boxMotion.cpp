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
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    startup_srv.response.success = false;
    startup_client.call(startup_srv);
    while(!startup_srv.response.success){
        ROS_WARN("Not successful startup yet... Repeat Calling...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Competition Startup Success");
    // ! End of System Startup Call

    //! Start of conveyor initialization
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    conveyor_srv.request.power = 100.0;    //! Conveyor Start
    conveyor_srv.response.success = false;
    while(!conveyor_srv.response.success){
        ROS_WARN("Not successful starting conveyor... Repeat Calling...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Conveyor Startup Success");
    //! End of conveyor initialization

    //! Start of 


    return 0;
}
