#include <ros/ros.h>
#include <std_srvs/Trigger.h> // this message type is defined in the current package
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>
using namespace std;

//! Global Variabls:
bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam2_data; // Global information storage holder

//! Call back for Camera2
void camera2CallBack(const osrf_gear::LogicalCameraImage& message_Holder){
    if(g_take_new_snapshot){
        ROS_INFO_STREAM("Image from Cam 1: "<<message_Holder<<endl);
        g_cam2_data = message_Holder;
        g_take_new_snapshot = false;
    }
}

void start_competition(ros::NodeHandle & startCompetitionNode){
    ros::ServiceClient startup_client = startCompetitionNode.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    if(!startup_client.exists()){
        ROS_INFO("[Startup Phase]Waiting for the competition to be ready...");
        startup_client.waitForExistence();
        ROS_INFO("[Startup Phase]Competition is now ready...");
    }
    ROS_INFO("[Startup Phase]Requesting competition to start...");
    std_srvs::Trigger startup_srv;
    startup_srv.response.success = false;
    startup_client.call(startup_srv);
    while(!startup_srv.response.success){
        ROS_WARN("[Startup Phase]Not successful startup yet... Repeat Calling...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("[Startup Phase]Competition Startup Success>>>[Belt Control Phase]");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cxq41_ariac_boxMotion"); //Initialization of ros
    ros::NodeHandle n; //Creates RoS Node Handle
    //! Initialization Code:
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;
    ros::Subscriber camera2= n.subscribe("/ariac/logical_camera_2",1,camera2CallBack); 

    
    //! Competition Startup Call
    start_competition(n);

    while(ros::ok()){
        //* Start the conveyor before anything:
        conveyor_srv.request.power = 100.0;
        conveyor_srv.response.success = false;
        while(!conveyor_srv.response.success){
            ROS_WARN("Not successful starting conveyor... Repeat Calling...");
            conveyor_client.call(conveyor_srv);
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Conveyor Startup Success");

        while(g_cam2_data.models.size()>0){
            ROS_INFO("The current box z position is: %f",g_cam2_data.pose.position.z);
            ros::spinOnce(); //allow data from call back
            ros::Duration(0.5).sleep();
        }
        ros::spinOnce();
    }













    //! Start of conveyor initialization

    //! End of conveyor initialization

    //! Start of drone control
    drone_srv.request.shipment_type = "Dummy Shipment";
    drone_srv.response.success = false;
    while(!drone_srv.response.success){
        ROS_WARN("Not successful starting Drone... Repeat Calling...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();        
    }
    ROS_INFO("Drone Call Success");
    //! End of drone initialization

    return 0;
}
