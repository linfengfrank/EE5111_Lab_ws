#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>
using std::placeholders::_1;
class TurtleCoop : public rclcpp::Node {
public:
    TurtleCoop() : Node("turtle_coop") {
        // Publisher for the follower turtle's velocity
        follower_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
        // Subscriber for the leader turtle's pose
        leader_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&TurtleCoop::leaderPoseCallback, this, _1));

        // Subscriber for the follower turtle's pose (Please try to fill in)
        follower_sub_ = this->create_subscription<turtlesim::msg::Pose>(   
        "turtle2/pose", 10, std::bind(&TurtleCoop::followerPoseCallback, this, _1));
    }

private:
    // Callback for leader turtle's pose
    void leaderPoseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
        leader_pose_ = *msg;
        followLeader();
    }

    // Callback for follower turtle's pose
    void followerPoseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
        follower_pose_ = *msg;
    }

    // Function to wrap angle to (−π, π]
    static double wrapToPi(double a)
    {
        // Robust wrap to (−π, π]
        return std::atan2(std::sin(a), std::cos(a));
    }
    
    // Function to make the follower turtle follow the leader
    void followLeader() {
        double distance = std::sqrt(
            std::pow(leader_pose_.x - follower_pose_.x, 2) +
            std::pow(leader_pose_.y - follower_pose_.y, 2));

        // Compute follower orientation to track the leader (Please try to fill in)
        double dx = leader_pose_.x - follower_pose_.x;
        double dy = leader_pose_.y - follower_pose_.y;
        double angle_to_leader = atan2(dy, dx);

        double angle_difference = angle_to_leader - follower_pose_.theta;
        angle_difference = wrapToPi(angle_difference);

        // Create a twist message for controlling the follower turtle
        geometry_msgs::msg::Twist vel_msg;

        // Proportional controller for linear and angular velocities
        vel_msg.linear.x = 1.0 * distance;
        vel_msg.angular.z = 5.0 * angle_difference;

        // Publish the velocity to the follower turtle
        follower_pub_->publish(vel_msg);
    }

    // Member variables for storing poses
    turtlesim::msg::Pose leader_pose_;
    turtlesim::msg::Pose follower_pose_;

    // ROS 2 publisher and subscriber objects
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr follower_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr leader_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr follower_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleCoop>());
    rclcpp::shutdown();
    return 0;
}
