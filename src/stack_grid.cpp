#include <ros/ros.h>
#include <stack_grid_bugcar/stack_grid_node.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "stack_grid");
    
    stack_grid_bugcar::StackGridBase stack_node;
    stack_node.initialize();
    stack_node.initDiagnostics();
    stack_node.initLayerHandler();
    stack_node.initMat();

    stack_node.run();
    // ros::Timer timer;
    // stack_node.setupTimer(timer);

    ros::spin();

    return 0;
}