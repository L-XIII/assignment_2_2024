/**
* \file service_node.cpp
* \brief Node providing a service to get the last navigation target
* \author Mohamed El shab
* \version 1.0
* \date 29/03/2025
*
* \details 
*
*
* Subscribes to:
* - /Get_Last_Target (geometry_msgs/Point): Receives target updates
*
* Services:
* - Get_Last_Target (assignment_2_2024/GetLastTarget): Returns last target coordinates
*
* Discription :
*
* This node maintains the last navigation target and provides a service to query it.
* It subscribes to target updates and serves the last known target coordinates.
*

**/


#include "ros/ros.h"
#include "assignment_2_2024/GetLastTarget.h"
#include "geometry_msgs/Point.h"  // Use Point for x and y coordinates

// Global variables to store the last target coordinates
geometry_msgs::Point last_target;  ///< Stores the last target coordinates


  /**
   * \brief brief Service callback to get the last target coordinates
   * \param req Service request
   * \param res Service response ,containing  x,y coordinates
   * \return true (service needed)
   */
bool getLastTarget(assignment_2_2024::GetLastTarget::Request &req,
                   assignment_2_2024::GetLastTarget::Response &res) {
    // Assign x and y coordinates from the last received target
    res.x = last_target.x;
    res.y = last_target.y;
    return true;
}


 /**
  * \brief 
  * \param msg Recieved target point message
  */
void goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    // Store the received target coordinates in the global variable
    last_target = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "service_node");
    ros::NodeHandle nh;

    // Subscribe to the topic that publishes target coordinates (x, y)
    ros::Subscriber goal_sub = nh.subscribe("/Get_Last_Target", 10, goalCallback);

    // Advertise the service to return the last target coordinates
    ros::ServiceServer service = nh.advertiseService("Get_Last_Target", getLastTarget);

    // Keep the node running to handle service requests
    ros::spin();

    return 0;
}

