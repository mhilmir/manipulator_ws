#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "try_control_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("try_control_moveit");

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    // Set a target Pose
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.position.x = 0.168;
        msg.position.y = 0.0;
        msg.position.z = 0.2;
        // msg.orientation.x = 0.7071068;
        // msg.orientation.y = 0.0;
        // msg.orientation.z = 0.0;
        // msg.orientation.w = 0.7071068;
        // msg.position.x = -0.1;
        // msg.position.y = 0.0;
        // msg.position.z = 0.25;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Set tolerances for goal position and orientation
    move_group_interface.setGoalPositionTolerance(0.02);
    move_group_interface.setGoalOrientationTolerance(0.02);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // If planning succeeds, execute the planned motion
    if(success) {
        move_group_interface.execute(plan);
        std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds after execution
    } else {
        RCLCPP_ERROR(logger, "Planning failed for the arm!");  // Log an error if planning fails
    }

    // // Create the MoveIt MoveGroup Interface for the "gripper" planning group
    // auto gripper_interface = MoveGroupInterface(node, "gripper");

    // // Set the "close" position for the gripper and move it
    // gripper_interface.setNamedTarget("close");
    // if (gripper_interface.move()) {
    //     RCLCPP_INFO(logger, "Gripper closed successfully");  // Log success
    //     std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
    // } else {
    //     RCLCPP_ERROR(logger, "Failed to close the gripper");  // Log an error if it fails
    // }

    // // Move the arm back to the "home" position
    // move_group_interface.setNamedTarget("init");
    // if (move_group_interface.move()) {
    //     RCLCPP_INFO(logger, "Arm moved back to init position");  // Log success
    //     std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds

    //     // Open the gripper
    //     gripper_interface.setNamedTarget("open");
    //     if (gripper_interface.move()) {
    //     RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
    //     } else {
    //     RCLCPP_ERROR(logger, "Failed to open the gripper");  // Log an error if it fails
    //     }

    // } else {
    //     RCLCPP_ERROR(logger, "Failed to move the arm back to home position");  // Log an error if it fails
    // }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}