/**
* \file action_client_node.cpp
* \brief node that allows a user to set targets and interact with an action server.
* \author Mohamed El shab
* \version 1.0
* \date 29/03/2025
*
* \details 
*
*
* Subscribes to:
* - /odom (nav_msgs/Odometry): Robot odometry information
*
* Publishes to:
* - /PoseVel_state (assignment_2_2024/PoseVel): Current position and velocity.
* - /Get_Last_Target (geometry_msgs/Point): Last target position.
*
*
* Discription :
*
* This node provides an interface for setting navigation targets and interacting with
* a ROS action server. It also publishes the robot's current position and velocity,
* and maintains the last set target.
*

**/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2024/PoseVel.h>
#include <geometry_msgs/Point.h>
#include <assignment_2_2024/GetLastTarget.h>

typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> PlanningClient;

/**
 * \class ActionClientNode
 * \brief It handles the interaction with the action server and publishes the robot state. 
*/

class ActionClientNode {
public:
    
    /**
     * \brief a constructor that initializes the action client and ROS interfaces.
    */

    ActionClientNode() : ac("/reaching_goal", true) {
        // Subscribe to the odom topic to get the robot's position and velocity
        odom_sub = nh.subscribe("/odom", 10, &ActionClientNode::odomCallback, this);
        // Advertise the custom message topic
        pub = nh.advertise<assignment_2_2024::PoseVel>("/PoseVel_state", 10);
        // Advertise the last target topic
        last_target_pub = nh.advertise<geometry_msgs::Point>("/Get_Last_Target", 10);

        // Wait for the action server to start with a timeout of 5 seconds
        if (!ac.waitForServer(ros::Duration(10.0))) {
            ROS_ERROR("Action server not available!");
            ros::shutdown();  // Shutdown the ROS node gracefully
            exit(1);  // Exit the program
        } else {
            ROS_INFO("Action server started!");
        }
    }


         /**
          * \brief Send a navigation goal to the action server.
          * \param x (The target x coordinate)
          * \param y (The target y coordinate)   
          */  
        void sendGoal(double x, double y) {
        assignment_2_2024::PlanningGoal goal;  // Correct goal type for your action
        
        // Set the position
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        
        // Set orientation to face the goal (quaternion for 0 degrees yaw)
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;  // Identity quaternion (no rotation)

        ac.sendGoal(goal, 
                    boost::bind(&ActionClientNode::doneCb, this, _1, _2),
                    PlanningClient::SimpleActiveCallback(),
                    PlanningClient::SimpleFeedbackCallback());
        ROS_INFO("Goal sent: (%f, %f)", x, y);

    // Publish the last target to the /Get_Last_Target topic----------------------------------
        geometry_msgs::Point target_point;
        target_point.x = x;
        target_point.y = y;
        target_point.z = 0.0;  // Set the z value to 0
        last_target_pub.publish(target_point);
    }

      /**
       * \brief It cancel the current navigation goal
       */
    void cancelGoal() {
        ac.cancelGoal();
        ROS_INFO("Goal canceled");
    }
    
    
    
     /**
      * \brief publish the robot's position and velocity
      */
    void publishPositionVelocity() {
        if (current_odom) {
            assignment_2_2024::PoseVel msg;
            msg.x = current_odom->pose.pose.position.x;
            msg.y = current_odom->pose.pose.position.y;
            msg.vel_x = current_odom->twist.twist.linear.x;
            msg.vel_z = current_odom->twist.twist.angular.z;
            pub.publish(msg);
        }
    }
     
     
     
     
     /**
      * \brief handle feedback from the action server
      * \param feedback pointer to the feedback message
      */
    void handleFeedback(const assignment_2_2024::PlanningFeedback::ConstPtr& feedback) {
        ROS_INFO("Feedback received:");
        ROS_INFO("Actual Pose: x = %f, y = %f, z = %f", 
                 feedback->actual_pose.position.x, 
                 feedback->actual_pose.position.y, 
                 feedback->actual_pose.position.z);
        ROS_INFO("Status: %s", feedback->stat.c_str());
    }



     /**
      * \brief Callback for when the goal is done
      * \param state Final state of the goal
      * \param result Result message from the action server
      */
    void doneCb(const actionlib::SimpleClientGoalState& state, const assignment_2_2024::PlanningResult::ConstPtr& result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached successfully!");
        } else {
            ROS_INFO("Goal failed: %s", state.toString().c_str());
        }
    }

private:
    ros::NodeHandle nh;
    PlanningClient ac;  ///< Action client connecting to the navigation server
    ros::Subscriber odom_sub;  ///< Subscriber for odometry data
    ros::Publisher pub;   ///< Publisher for pose and velocity
    ros::Publisher last_target_pub;  ///< Publisher for last target
    boost::shared_ptr<nav_msgs::Odometry const> current_odom;  ///< Current odometry data

     /**
      * \brief Callback for odometry data
      * \param msg Recieved odometry message
      */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom = msg;  // Update the current odometry data
    }
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node");
    if (!ros::isInitialized()) {
        std::cout << "ROS initialization failed!" << std::endl;
        return 1;
    }
    std::cout << "ROS initialized successfully!" << std::endl;
    
    ActionClientNode node;
    
    ros::Rate rate(10);  // Loop rate to publish position and velocity at 10 Hz
    while (ros::ok()) {
        
        std::string cmd;
        std::cout << "- Press 't' to set a target" << std::endl;
        std::cout << "- Press 'c' to cancel" << std::endl; 
        std::cout.flush();
        std::cin >> cmd;

        if (cmd == "t") {
            double x, y;
            std::cout << "Enter target x: ";
            std::cin >> x;
            std::cout << "Enter target y: ";
            std::cin >> y;
            std::cout << "Target set to: " << x << ", " << y << std::endl;
            node.sendGoal(x, y);
        } else if (cmd == "c") {
            std::cout << "Goal canceled" << std::endl;
            node.cancelGoal();
        
        } else { 
            std::cout << "----------Enter a valid input----------" << std::endl;
        }

        // Publish position and velocity 
        node.publishPositionVelocity();
        ros::spinOnce();  
        rate.sleep();     // Sleep for the specified rate (10 Hz)
    }

    return 0;
}
