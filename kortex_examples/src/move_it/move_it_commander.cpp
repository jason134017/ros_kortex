#include <ros/ros.h>                             // 引用 ros.h 檔
/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
// class Listener
// {
//   public:
//     void callback(const std_msgs::String::ConstPtr& msg);
// };
// class ExampleMoveItTrajectories{
//   public:
//     ExampleMoveItTrajectories(){}
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_cpp_node");     // 初始化 hello_cpp_node
    ros::NodeHandle handler;                     // node 的 handler
    ROS_INFO("Hello World!");                    // 印出 Hello World
}