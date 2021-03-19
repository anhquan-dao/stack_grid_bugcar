#include <ros/ros.h>
#include <stack_grid_bugcar/stack_grid_node.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "stack_grid");
    
    stack_grid_bugcar::StackGridNode stack_node;
    stack_node.initMat();

    ROS_INFO_STREAM("END");

    stack_node.run();

    return 0;
}