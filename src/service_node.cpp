#include "ros/ros.h"
#include "assignment_2_2024/GetLastTarget.h"
#include "geometry_msgs/Point.h"  // Use Point for x and y coordinates

// Global variables to store the last target coordinates
geometry_msgs::Point last_target;

bool getLastTarget(assignment_2_2024::GetLastTarget::Request &req,
                   assignment_2_2024::GetLastTarget::Response &res) {
    // Assign x and y coordinates from the last received target
    res.x = last_target.x;
    res.y = last_target.y;
    return true;
}

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

