#ifndef SINGLE_LAYER_HANDLER
#define SINGLE_LAYER_HANDLER

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

namespace stack_grid_bugcar{

class LayerHandler{
    public:
        LayerHandler(){

        }    
        LayerHandler(std::string parent_node, std::string src_name, std::string global_frame, std::string msg_type, std::string topic);
        cv::Mat getLayerIMG();
        std::string get_frame_id();
        std::string get_name();
        std::string get_sub_topic();

        void update_stack_size(int size_x, int size_y);
        void update_stack_resolution(double resolution_);
        
        int transform_to_baselink(geometry_msgs::TransformStamped tf_3d_msg);

        void get_transformed_input(cv::Mat &output){
            data_img_8u.convertTo(output, output.type());
        }
        bool input_is_empty(){
            return raw_data_buffer.empty();
        }
        ros::Time getLatestTime();

    protected:
        void visualize_layer();

        ros::NodeHandle layer_handler;
        ros::CallbackQueue queue;
        ros::Publisher vis_publisher;
        ros::Subscriber layer_sub;
        std::string global_frame_;
        template<class msg_type> void callback(const typename msg_type::ConstPtr input_data);
        
        // cv::Mat data_img;
        cv::Mat data_img_8u;
        cv::Mat raw_data_buffer;

        double layer_resolution;
        double stack_resolution;
        cv::Size stack_dim = cv::Size(0,0);

        geometry_msgs::PoseStamped layer_origin;
        ros::Time last_callback_time;

        cv::Mat T_layer = cv::Mat(cv::Size(4,4),CV_32FC1);

        std::string obj_name;
        std::string sub_topic;
        bool visual = false;
        std::mutex data_mutex;
        
};
template<> void LayerHandler::callback<nav_msgs::OccupancyGrid>(const nav_msgs::OccupancyGrid::ConstPtr input_data);
template<> void LayerHandler::callback<sensor_msgs::PointCloud>(const sensor_msgs::PointCloud::ConstPtr input_data);
template<> void LayerHandler::callback<sensor_msgs::PointCloud2>(const sensor_msgs::PointCloud2::ConstPtr input_data);
template<> void LayerHandler::callback<sensor_msgs::LaserScan>(const sensor_msgs::LaserScan::ConstPtr input_data);

}
#endif
