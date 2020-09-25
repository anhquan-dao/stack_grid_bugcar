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

#include <stack_grid_bugcar/initStackService.h>
#include <stack_grid_bugcar/killStackService.h>


#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#define DEBUG_
#ifdef DEBUG_
    #define DEBUG_INFO(content) ROS_INFO_STREAM(content)
    #define DEBUG_WARN(content) ROS_WARN_STREAM(content)
    #define DEBUG_FATAL(content) ROS_FATAL_STREAM(content)
#else
    #define DEBUG_INFO(content)
    #define DEBUG_WARN(content)
    #define DEBUG_FATAL(content)
#endif
 
namespace stack_grid{
 
    class StackGrid : public costmap_2d::Layer, public costmap_2d::Costmap2D{
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
        
        private:
            void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
            void initializeServiceClient();

            void getGrid(const nav_msgs::OccupancyGrid::ConstPtr input_layer, nav_msgs::OccupancyGrid *layer, std::string *topic_name_);
            bool isActive(const nav_msgs::OccupancyGrid &layer, const std::string &topic_name_);
            void publishCostmap(costmap_2d::Costmap2D cost_map_);    
            uint8_t updateCharMap(const int8_t master_cell, uint8_t self_cell);   
            uint8_t translateOccupancyToCost(int8_t occupancyValue);

            ros::NodeHandle *private_nh = NULL;

            ros::Publisher self_costmap_publisher;
            ros::Publisher costmap_origin_publisher;

            std::string static_global_topic;
            std::string static_local_topic;
            std::string behaviour_regulator_topic;
            std::string dynamic_obstacle_topic;
            std::string obstacle_tracking_topic;
            
            std::vector<const std::string*> subscribed_topics_;

            ros::ServiceClient client_python;
            ros::ServiceClient kill_python_srv;
            stack_grid_bugcar::initStackService initPythonTool;
            stack_grid_bugcar::killStackService killPythonTool;

            geometry_msgs::PoseStamped costmap_stamped_origin;
            

            std::string python_layer_topic;
            ros::Subscriber python_layer_sub;
            nav_msgs::OccupancyGrid python_layer;
            std::vector<signed char> python_layer_buffer;

            char *cost_lookup_table;
            float cell_count[2];
            double resolution;
            double mark_x_, mark_y_;
            double max_delay_time;
            bool enable_debug;
            std::string global_frame_;
            std::string robot_base_frame_;

            
    };
}
#endif
 