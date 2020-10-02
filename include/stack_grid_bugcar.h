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
#include <opencv2/opencv.hpp>

#include "simple_layer_obj.h"

namespace stack_grid_bugcar{
 
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

            void publishCostmap(costmap_2d::Costmap2D cost_map_);    
            uint8_t updateCharMap(const float img_cell, uint8_t self_cell);   
            uint8_t translateOccupancyToCost(int8_t occupancyValue);

            ros::NodeHandle *private_nh = NULL;

            ros::Publisher self_costmap_publisher;
            ros::Publisher costmap_origin_publisher;

            std::string static_topic;
            std::string dynamic_topic;
            std::string planning_ctrl_topic;
            std::string tracking_topic;
            
            std::vector<const std::string*> subscribed_topics_;

            geometry_msgs::PoseStamped costmap_stamped_origin;

            std::vector<boost::shared_ptr<SimpleLayerObj>> static_layers_handler;
            
            cv::Mat main_map_img;
            int cvMatDim[2];

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tf_listener{tfBuffer};

            char *cost_lookup_table;
            double max_delay_time;
            bool enable_debug;
            std::string global_frame_;            
    };
}
#endif
 