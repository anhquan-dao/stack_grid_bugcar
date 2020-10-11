#ifndef SIMPLE_LAYER_OBJ
#define SIMPLE_LAYER_OBJ

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <algorithm>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

namespace stack_grid_bugcar{

static const int EMPTY_MSG_ERR = 2;
static int DEFAULT_OCCUPANCY_VALUE = 0;
static const int LATE_UPDATE_ERR = 3;

class SimpleLayerObj{
    public:
        SimpleLayerObj(){

        }    
        SimpleLayerObj(std::string parent_node, std::string src_name, std::string global_frame, std::string msg_type, std::string topic);
        cv::Mat getLayerIMG();
        std::string get_frame_id();
        std::string get_name();
        std::string get_sub_topic();
        void update_main_costmap_origin(geometry_msgs::PoseStamped costmap_origin_);
        void update_size(int size_x, int size_y);
        int transform_to_fit(geometry_msgs::TransformStamped tf_3d_msg);
        
        void link_mat(boost::shared_ptr<cv::Mat> extern_mat);

        void enableVisualization();
        void disableVisualization();
        ros::Time getLatestTime();

    protected:
        void visualize_layer();

        ros::NodeHandle layer_handler;
        ros::CallbackQueue queue;
        ros::Publisher vis_publisher;
        ros::Subscriber layer_sub;
        std::string global_frame_;
        template<class msg_type> void callback(const typename msg_type::ConstPtr input_data);
        
        cv::Mat data_img;
        cv::Mat data_img_float;
        boost::weak_ptr<cv::Mat> data_img_fit;
        cv::Mat prev_data_img_fit;
        std::vector<float> raw_data_buffer;

        double layer_resolution;
        cv::Size costmap_dim = cv::Size(0,0);

        geometry_msgs::PoseStamped layer_origin;
        geometry_msgs::PoseStamped costmap_origin;
        ros::Time last_callback_time;
        geometry_msgs::PoseStamped costmap_origin_old;

        cv::Mat T_layer = cv::Mat(cv::Size(4,4),CV_32FC1);

        std::string obj_name;
        std::string sub_topic;
        bool visual = false;
        boost::mutex data_mutex;
        
};
template<> void SimpleLayerObj::callback<nav_msgs::OccupancyGrid>(const nav_msgs::OccupancyGrid::ConstPtr input_data);
template<> void SimpleLayerObj::callback<sensor_msgs::PointCloud>(const sensor_msgs::PointCloud::ConstPtr input_data);
template<> void SimpleLayerObj::callback<sensor_msgs::PointCloud2>(const sensor_msgs::PointCloud2::ConstPtr input_data);
template<> void SimpleLayerObj::callback<sensor_msgs::LaserScan>(const sensor_msgs::LaserScan::ConstPtr input_data);

}
#endif
