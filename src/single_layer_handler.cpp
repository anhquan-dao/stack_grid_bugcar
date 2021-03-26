#include <stack_grid_bugcar/single_layer_handler.h>

namespace stack_grid_bugcar{

    LayerHandler::LayerHandler(std::string parent_node, std::string src_name, std::string global_frame, std::string msg_type, std::string topic){
        //layer_handler.setCallbackQueue(&queue);
        obj_name = src_name;
        sub_topic = topic;
        global_frame_ = global_frame;
        if(msg_type == "OccupancyGrid"){
            // vis_publisher = layer_handler.advertise<nav_msgs::OccupancyGrid>(parent_node + "/occupancy_grid_handler/" + src_name,5);
            layer_sub = layer_handler.subscribe<nav_msgs::OccupancyGrid>(sub_topic,1,boost::bind(&LayerHandler::callback<nav_msgs::OccupancyGrid>,this,_1)); 
        }
        ROS_INFO_STREAM("Created simple " + msg_type + " handler of type stack_grid_bugcar::LayerHandler for " + obj_name + ": " + sub_topic);
        
    }
    std::string LayerHandler::get_frame_id(){
        return layer_origin.header.frame_id;
    }
    std::string LayerHandler::get_name(){
        return obj_name;
    }
    std::string LayerHandler::get_sub_topic(){
        return sub_topic;
    }

    void LayerHandler::update_stack_size(int size_x, int size_y){
        stack_dim.width = size_x;
        stack_dim.height = size_y; 
    }

    void LayerHandler::update_stack_resolution(double resolution_){
        stack_resolution = resolution_;
    }

    int LayerHandler::transform_to_baselink(geometry_msgs::TransformStamped tf_3d_msg){
        std::lock_guard<std::mutex> lg(data_mutex);

        // cv::resize(raw_data_buffer, data_img, cv::Size(), layer_resolution/stack_resolution, layer_resolution/stack_resolution);
        // raw_data_buffer.copyTo(data_img);
        // data_img += 1;

        if(raw_data_buffer.empty()){
            ROS_WARN_STREAM("Input for " + obj_name + " has not been published, topic: " + sub_topic);
            return 3;
        }
        if(raw_data_buffer.rows == 0 || raw_data_buffer.cols == 0){
            ROS_WARN_STREAM("Input for " + obj_name + " is empty, topic: " + sub_topic);
            return 2;
        }
        if(abs(last_callback_time.toSec() - ros::Time::now().toSec()) > 2){
            ROS_WARN_STREAM("Input for " + obj_name + " has not been updated for " <<
                     abs(last_callback_time.toSec() - ros::Time::now().toSec()) << ", topic: " + sub_topic);
            return 1;
        }

        T_layer = cv::Mat::eye(cv::Size(3,3),CV_32FC1);
        float angle = atan2(layer_origin.pose.orientation.z, layer_origin.pose.orientation.w);
        T_layer.at<float>(0,0) = cos(angle*2);
        T_layer.at<float>(1,1) = T_layer.at<float>(0,0);
        T_layer.at<float>(1,0) = sin(angle*2);
        T_layer.at<float>(0,1) = -T_layer.at<float>(1,0);
        T_layer.at<float>(0,2) =  stack_dim.width * stack_resolution / 2 + layer_origin.pose.position.x;
        T_layer.at<float>(1,2) =  stack_dim.height * stack_resolution / 2 + layer_origin.pose.position.y;

        cv::Mat T_3d_mat = cv::Mat::eye(cv::Size(3,3),CV_32FC1);
        angle = atan2( tf_3d_msg.transform.rotation.z, tf_3d_msg.transform.rotation.w);
        T_3d_mat.at<float>(0,0) = cos(angle*2);
        T_3d_mat.at<float>(1,1) = T_3d_mat.at<float>(0,0);
        T_3d_mat.at<float>(1,0) = sin(angle*2);
        T_3d_mat.at<float>(0,1) = -T_3d_mat.at<float>(1,0);
        T_3d_mat.at<float>(0,2) = tf_3d_msg.transform.translation.x;
        T_3d_mat.at<float>(1,2) = tf_3d_msg.transform.translation.y;

        T_layer = T_3d_mat * T_layer;

        cv::Mat T_scaling = cv::Mat::eye(cv::Size(3,3), CV_32FC1);
        T_scaling.at<float>(0,0) *= layer_resolution/stack_resolution;
        T_scaling.at<float>(1,1) *= layer_resolution/stack_resolution;

        T_layer *= T_scaling;

        cv::Mat tf_2d_mat_2x3 = T_layer(cv::Range(0,2),cv::Range(0,3));
        tf_2d_mat_2x3.at<float>(0,2) /= stack_resolution;
        tf_2d_mat_2x3.at<float>(1,2) /= stack_resolution;
         
        raw_data_buffer.convertTo(data_img_8u, CV_8UC1);
        // ROS_INFO_STREAM("data_img_8u: " << data_img_8u.type() << " " << &data_img_8u);
        cv::warpAffine(data_img_8u, data_img_8u, tf_2d_mat_2x3, stack_dim, 
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);

        //ROS_INFO_STREAM("data_img_float: " << data_img_float.type() << " " << &data_img_float);

        return 0;
    }

    ros::Time LayerHandler::getLatestTime(){
        return layer_origin.header.stamp;
    }

    template<> void LayerHandler::callback<nav_msgs::OccupancyGrid>(const nav_msgs::OccupancyGrid::ConstPtr input_data){
        std::lock_guard<std::mutex> lg(data_mutex);

        layer_origin.header = input_data->header;
        layer_origin.pose = input_data->info.origin; 
        layer_resolution = input_data->info.resolution;

        last_callback_time = layer_origin.header.stamp;
        // ROS_INFO_STREAM("Yo");
        
        raw_data_buffer = cv::Mat(input_data->data, true).reshape(1,input_data->info.height);
        raw_data_buffer += 1;
        // ROS_INFO_STREAM("raw_data_buffer: " << raw_data_buffer.type() << " " << &raw_data_buffer);

        // ROS_INFO_STREAM(data_img.type());
        //ROS_INFO_STREAM(&data_img);
        //ROS_INFO_STREAM(&occupancy_grid_mat);
        
    }
    
}