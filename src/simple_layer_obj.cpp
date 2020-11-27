#include "simple_layer_obj.h"

namespace stack_grid_bugcar{

    SimpleLayerObj::SimpleLayerObj(std::string parent_node, std::string src_name, std::string global_frame, std::string msg_type, std::string topic){
        //layer_handler.setCallbackQueue(&queue);
        obj_name = src_name;
        sub_topic = topic;
        global_frame_ = global_frame;
        if(msg_type == "OccupancyGrid"){
            vis_publisher = layer_handler.advertise<nav_msgs::OccupancyGrid>(parent_node + "/occupancy_grid_handler/" + src_name,5);
            layer_sub = layer_handler.subscribe<nav_msgs::OccupancyGrid>(sub_topic,1,boost::bind(&SimpleLayerObj::callback<nav_msgs::OccupancyGrid>,this,_1)); 
        }
        ROS_INFO_STREAM("Created simple " + msg_type + " handler of type stack_grid_bugcar::SimpleLayerObj for " + obj_name + ": " + sub_topic);
        
    }    

    cv::Mat SimpleLayerObj::getLayerIMG(){
        std::lock_guard<std::mutex> lg(data_mutex);
        return *data_img_fit.lock();
    }
    std::string SimpleLayerObj::get_frame_id(){
        return layer_origin.header.frame_id;
    }
    std::string SimpleLayerObj::get_name(){
        return obj_name;
    }
    std::string SimpleLayerObj::get_sub_topic(){
        return sub_topic;
    }
    void SimpleLayerObj::link_mat(std::shared_ptr<cv::Mat> extern_mat){
        data_img_fit = extern_mat;
    }
    void SimpleLayerObj::update_main_costmap_origin(geometry_msgs::PoseStamped costmap_origin_){
        costmap_origin.header = costmap_origin_.header;
        costmap_origin.pose = costmap_origin_.pose;
        if(abs(last_callback_time.toSec() - ros::Time::now().toSec()) < 2.0){
            costmap_origin_old.header = costmap_origin.header;
            costmap_origin_old.pose = costmap_origin.pose;
        }
    }
    void SimpleLayerObj::update_size(int size_x, int size_y){
        costmap_dim.width = size_x;
        costmap_dim.height = size_y;
        
        *data_img_fit.lock() = cv::Mat::zeros(costmap_dim,CV_32FC1);
        prev_data_img_fit = cv::Mat::zeros(costmap_dim, CV_32FC1);
        if(!data_img_fit.lock()->isContinuous()){
            *data_img_fit.lock() = data_img_fit.lock()->clone();
            cv::patchNaNs(*data_img_fit.lock(), (float)DEFAULT_OCCUPANCY_VALUE);
        }
        
    }
    int SimpleLayerObj::transform_to_fit(geometry_msgs::TransformStamped tf_3d_msg){
        std::lock_guard<std::mutex> lg(data_mutex);
        if(data_img_float.empty()){
            ROS_WARN_STREAM("Input for " + obj_name + " has not been published, topic: " + sub_topic);
            return EMPTY_MSG_ERR;
        }
        if(data_img_float.rows == 0 || data_img_float.cols == 0){
            ROS_WARN_STREAM("Input for " + obj_name + " is empty, topic: " + sub_topic);
            return EMPTY_MSG_ERR;
        }
        if(abs(last_callback_time.toSec() - ros::Time::now().toSec()) > 2){
            ROS_WARN_STREAM("Input for " + obj_name + " has not been updated for " <<
                     abs(last_callback_time.toSec() - ros::Time::now().toSec()) << ", topic: " + sub_topic);
            
            cv::Mat T_costmap = cv::Mat::eye(cv::Size(3,3),CV_32FC1);
            T_costmap.at<float>(0,2) = (costmap_origin_old.pose.position.x - costmap_origin.pose.position.x) / layer_resolution;
            T_costmap.at<float>(1,2) = (costmap_origin_old.pose.position.y - costmap_origin.pose.position.y) / layer_resolution;
            T_costmap = T_costmap(cv::Range(0,2),cv::Range(0,3));
            
            cv::warpAffine(prev_data_img_fit,*data_img_fit.lock(),T_costmap,costmap_dim, 
                    cv::INTER_LINEAR, cv::BORDER_CONSTANT, DEFAULT_OCCUPANCY_VALUE);
    
            cv::patchNaNs(*data_img_fit.lock(), (float)DEFAULT_OCCUPANCY_VALUE);
            return LATE_UPDATE_ERR;
        }

        data_img_float.copyTo(data_img);
        if(data_img.type() != CV_32FC1)
            data_img.convertTo(data_img, CV_32FC1);

        T_layer = cv::Mat::eye(cv::Size(4,4),CV_32FC1);
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
        T_3d_mat.at<float>(0,3) = tf_3d_msg.transform.translation.x;
        T_3d_mat.at<float>(1,3) = tf_3d_msg.transform.translation.y;

        T_layer = T_3d_mat.inv() * T_layer;

        cv::Mat T_costmap = cv::Mat::eye(cv::Size(4,4),CV_32FC1);
        T_costmap.at<float>(0,3) = costmap_origin_old.pose.position.x;
        T_costmap.at<float>(1,3) = costmap_origin_old.pose.position.y;
        
        cv::Mat tf_2d_mat = (T_costmap.inv()*T_layer);
        cv::Mat tf_2d_mat_2x3 = tf_2d_mat(cv::Range(0,2),cv::Range(0,3));
        tf_2d_mat_2x3.at<float>(0,2) = tf_2d_mat.at<float>(0,3) / layer_resolution;
        tf_2d_mat_2x3.at<float>(1,2) = tf_2d_mat.at<float>(1,3) / layer_resolution;

        cv::warpAffine(data_img,*data_img_fit.lock(),tf_2d_mat_2x3,costmap_dim, 
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, DEFAULT_OCCUPANCY_VALUE);
        
        cv::patchNaNs(*data_img_fit.lock(), (float)DEFAULT_OCCUPANCY_VALUE);
        data_img_fit.lock()->copyTo(prev_data_img_fit);
          
        return 1;
    }
    

    void SimpleLayerObj::enableVisualization(){
        visual = true;
    }
    void SimpleLayerObj::disableVisualization(){
        visual = false;
    }
    ros::Time SimpleLayerObj::getLatestTime(){
        return layer_origin.header.stamp;
    }

    void SimpleLayerObj::visualize_layer(){
        nav_msgs::OccupancyGrid vis;
        vis.header.frame_id = global_frame_;
        vis.header.stamp = ros::Time::now();
        
        vis.info.width = costmap_dim.width;
        vis.info.height = costmap_dim.height;
        vis.info.resolution = layer_resolution;

        vis.info.origin.position.x = costmap_origin.pose.position.x;
        vis.info.origin.position.y = costmap_origin.pose.position.y;

        vis.data.assign((float*)data_img_fit.lock()->datastart,(float*)data_img_fit.lock()->dataend);
        
        vis_publisher.publish(vis);
    }

    template<> void SimpleLayerObj::callback<nav_msgs::OccupancyGrid>(const nav_msgs::OccupancyGrid::ConstPtr input_data){
        std::lock_guard<std::mutex> lg(data_mutex);
        layer_origin.header = input_data->header;
        layer_origin.pose = input_data->info.origin; 
        layer_resolution = input_data->info.resolution;

        last_callback_time = layer_origin.header.stamp;
        
        data_img_float = cv::Mat(input_data->data).reshape(1,input_data->info.height);
        data_img_float.convertTo(data_img_float,CV_32FC1);
        
        if(!data_img_float.isContinuous()){
            data_img_float = data_img_float.clone();
        }

    }
    
}