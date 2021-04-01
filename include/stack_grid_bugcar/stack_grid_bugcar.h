#ifndef STACK_GRID_BUGCAR
#define STACK_GRID_BUGCAR

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <algorithm>
#include <memory>
#include <future>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <stack_grid_bugcar/stack_grid_node.h>
#include <stack_grid_bugcar/single_layer_handler.h>

namespace stack_grid_bugcar{
 
    class StackGrid : public StackGridBase, public costmap_2d::Layer, public costmap_2d::Costmap2D{
        public:
            StackGrid();
            ~StackGrid();
        
            virtual void onInitialize();
            
            virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                    double* max_y);
            virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

            bool isDiscretized(){
                return true;
            }

            virtual void matchSize();

        protected:
            void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

            uint8_t updateCharMap(const int8_t img_cell, uint8_t self_cell);   
            uint8_t translateOccupancyToCost(int8_t occupancyValue);

            char *cost_lookup_table;

            geometry_msgs::PoseStamped costmap_stamped_origin;     
    };
}
#endif
 