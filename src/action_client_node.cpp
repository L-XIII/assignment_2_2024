#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2024/PoseVel.h>
#include <geometry_msgs/Point.h>
#include <assignment_2_2024/GetLastTarget.h>

typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> PlanningClient;

class ActionClientNode {
public:
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

    // Method to send the goal to the action server-----------------------------------
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

    // function to cancel the goal----------------------------------------------------------
    void cancelGoal() {
        ac.cancelGoal();
        ROS_INFO("Goal canceled");
    }

    // function to publish the robot's position and velocity-----------------------------------------
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

    // Method to handle feedback (optional, if needed for additional feedback)---------------------------------------
    void handleFeedback(const assignment_2_2024::PlanningFeedback::ConstPtr& feedback) {
        ROS_INFO("Feedback received:");
        ROS_INFO("Actual Pose: x = %f, y = %f, z = %f", 
                 feedback->actual_pose.position.x, 
                 feedback->actual_pose.position.y, 
                 feedback->actual_pose.position.z);
        ROS_INFO("Status: %s", feedback->stat.c_str());
    }

    // Callback for when the goal is done------------------------------------
    void doneCb(const actionlib::SimpleClientGoalState& state, const assignment_2_2024::PlanningResult::ConstPtr& result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached successfully!");
        } else {
            ROS_INFO("Goal failed: %s", state.toString().c_str());
        }
    }

private:
    ros::NodeHandle nh;
    PlanningClient ac;  // Action client connecting to the correct server
    ros::Subscriber odom_sub;
    ros::Publisher pub;
    ros::Publisher last_target_pub;
    boost::shared_ptr<nav_msgs::Odometry const> current_odom;

    // Callback for odometry data
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
