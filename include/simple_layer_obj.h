#ifndef SIMPLE_LAYER_OBJ
#define SIMPLE_LAYER_OBJ

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

namespace stack_grid_bugcar{

class SimpleLayerObj{
    public:
        SimpleLayerObj(){

        }    
        SimpleLayerObj(std::string parent_node, std::string src_name, std::string global_frame, std::string msg_type, std::string topic);
        cv::Mat getLayerIMG(){
            return *data_img_fit;
        }
        std::string get_frame_id(){
            return layer_origin.header.frame_id;
        }
        void update_main_costmap_origin(geometry_msgs::PoseStamped costmap_origin_){
            costmap_origin.header = costmap_origin_.header;
            costmap_origin.pose = costmap_origin_.pose;
        }
        void update_size(int size_x, int size_y){
            costmap_dim.width = size_x;
            costmap_dim.height = size_y;
            *data_img_fit = cv::Mat::zeros(costmap_dim,CV_32FC1);
            if(!data_img_fit->isContinuous()){
                *data_img_fit = data_img_fit->clone();
            }
        }
        void transform_to_fit(geometry_msgs::TransformStamped tf_3d_msg){
            if(data_img_float.rows == 0 || data_img_float.cols == 0){
                ROS_INFO_STREAM("buffer size (0,0)");
                return;
            }
            data_img = data_img_float;

            T_layer =cv::Mat::eye(cv::Size(4,4),CV_32FC1);
            float angle = atan2(layer_origin.pose.orientation.z, layer_origin.pose.orientation.w);
            T_layer.at<float>(0,0) = cos(angle*2);
            T_layer.at<float>(1,1) = T_layer.at<float>(0,0);
            T_layer.at<float>(1,0) = sin(angle*2);
            T_layer.at<float>(0,1) = -T_layer.at<float>(1,0);
            T_layer.at<float>(0,3) = layer_origin.pose.position.x;
            T_layer.at<float>(1,3) = layer_origin.pose.position.y;

            cv::Mat T_3d_mat = cv::Mat::eye(cv::Size(4,4),CV_32FC1);
            angle = atan2( tf_3d_msg.transform.rotation.z, tf_3d_msg.transform.rotation.w);
            T_3d_mat.at<float>(0,0) = cos(angle*2);
            T_3d_mat.at<float>(1,1) = T_3d_mat.at<float>(0,0);
            T_3d_mat.at<float>(1,0) = sin(angle*2);
            T_3d_mat.at<float>(0,1) = -T_3d_mat.at<float>(1,0);
            T_3d_mat = T_3d_mat.t();
            T_3d_mat.at<float>(0,3) = -tf_3d_msg.transform.translation.x;
            T_3d_mat.at<float>(1,3) = -tf_3d_msg.transform.translation.y;
 
            T_layer = T_3d_mat * T_layer;
            
            cv::Mat T_costmap = cv::Mat::eye(cv::Size(4,4),CV_32FC1);
            T_costmap.at<float>(0,3) = -costmap_origin.pose.position.x;
            T_costmap.at<float>(1,3) = -costmap_origin.pose.position.y;
            
            cv::Mat tf_2d_mat = (T_costmap*T_layer)(cv::Range(0,2),cv::Range(0,4));
            tf_2d_mat.at<float>(0,2) = tf_2d_mat.at<float>(0,3) / layer_resolution;
            tf_2d_mat.at<float>(1,2) = tf_2d_mat.at<float>(1,3) / layer_resolution;
            tf_2d_mat = tf_2d_mat(cv::Range(0,2),cv::Range(0,3));
            
            if(data_img.type() != CV_32FC1)
                data_img.convertTo(data_img, CV_32FC1);
            if(data_img_fit->type() != CV_32FC1)
                data_img_fit->convertTo(*data_img_fit, CV_32FC1);
            
            cv::warpAffine(data_img,*data_img_fit,tf_2d_mat,costmap_dim);

            if(visual)
                visualize_layer();  
        }
        void link_mat(cv::Mat *extern_mat){
            data_img_fit = extern_mat;
        }
        void enableVisualization(){
            visual = true;
        }
        void disableVisualization(){
            visual = false;
        }
        ros::Time getLatestTime(){
            return layer_origin.header.stamp;
        }

    protected:
        void visualize_layer(){
            nav_msgs::OccupancyGrid vis;
            vis.header.frame_id = global_frame_;
            vis.header.stamp = ros::Time::now();
            
            vis.info.width = costmap_dim.width;
            vis.info.height = costmap_dim.height;
            vis.info.resolution = layer_resolution;

            vis.info.origin.position.x = costmap_origin.pose.position.x;
            vis.info.origin.position.y = costmap_origin.pose.position.y;

            vis.data.assign((float*)data_img_fit->datastart,(float*)data_img_fit->dataend);
            
            vis_publisher.publish(vis);
        }
        int8_t convertMatToOccupancy(const float mat_value, int8_t occupancy_value){
            return (int8_t)mat_value;
        }

        ros::Publisher vis_publisher;
        ros::Subscriber layer_sub;
        std::string global_frame_;
        template<class msg_type> void callback(const typename msg_type::ConstPtr input_data);
        
        cv::Mat data_img;
        cv::Mat data_img_float;
        cv::Mat *data_img_fit;
        std::vector<float> raw_data_buffer;

        double layer_resolution;
        cv::Size costmap_dim = cv::Size(0,0);

        geometry_msgs::PoseStamped layer_origin;
        geometry_msgs::PoseStamped costmap_origin;

        cv::Mat T_layer = cv::Mat(cv::Size(4,4),CV_32FC1);

        bool visual = false;
        
};


template<> void SimpleLayerObj::callback<nav_msgs::OccupancyGrid>(const nav_msgs::OccupancyGrid::ConstPtr input_data){
    layer_origin.header = input_data->header;
    layer_origin.pose = input_data->info.origin; 
    layer_resolution = input_data->info.resolution;

    data_img_float = cv::Mat(input_data->data).reshape(1,input_data->info.width);
    data_img_float.convertTo(data_img_float,CV_32FC1);
    if(!data_img_float.isContinuous()){
        data_img_float = data_img_float.clone();
    }
}

SimpleLayerObj::SimpleLayerObj(std::string parent_node, std::string src_name, std::string global_frame, std::string msg_type, std::string topic){
   
    if(msg_type == "OccupancyGrid"){
        ros::NodeHandle nh(parent_node + "/occupancy_grid_handler/" + src_name);
        vis_publisher = nh.advertise<nav_msgs::OccupancyGrid>(nh.getNamespace(),5);
        layer_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map/free_local_occupancy_grid",1,boost::bind(&SimpleLayerObj::callback<nav_msgs::OccupancyGrid>,this,_1)); 
    }
    ROS_INFO_STREAM("Created simple " + msg_type + " handler of type stack_grid_bugcar::SimpleLayerObj for " + src_name + ": " + topic);
    global_frame_ = global_frame;
}

}
#endif
