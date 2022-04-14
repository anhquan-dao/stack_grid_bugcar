#include <stack_grid_bugcar/single_layer_handler.h>
#include <stack_grid_bugcar/stack_grid_node.h>

namespace stack_grid_bugcar
{

    LayerHandler::LayerHandler(std::string parent_node, std::string src_name, std::string global_frame, std::string msg_type, std::string topic)
    {
        // layer_handler.setCallbackQueue(&queue);
        obj_name = src_name;
        sub_topic = topic;
        global_frame_ = global_frame;
        if (msg_type == "OccupancyGrid")
        {
            // vis_publisher = layer_handler.advertise<nav_msgs::OccupancyGrid>(parent_node + "/occupancy_grid_handler/" + src_name,5);
            layer_sub = layer_handler.subscribe<nav_msgs::OccupancyGrid>(sub_topic, 1, boost::bind(&LayerHandler::callback<nav_msgs::OccupancyGrid>, this, _1));
        }
        ROS_INFO_STREAM("Created simple " + msg_type + " handler of type stack_grid_bugcar::LayerHandler for " + obj_name + ": " + sub_topic);
    }
    std::string LayerHandler::get_frame_id()
    {
        return layer_origin.header.frame_id;
    }
    std::string LayerHandler::get_name()
    {
        return obj_name;
    }
    std::string LayerHandler::get_sub_topic()
    {
        return sub_topic;
    }

    void LayerHandler::update_stack_size(int size_x, int size_y)
    {
        stack_dim.width = size_x;
        stack_dim.height = size_y;
    }

    void LayerHandler::update_stack_resolution(double resolution_)
    {
        stack_resolution = resolution_;
    }

    int LayerHandler::transform_to_baselink(geometry_msgs::TransformStamped tf_3d_msg)
    {
        std::lock_guard<std::mutex> lg(data_mutex);

        //
        // raw_data_buffer.copyTo(data_img);
        // data_img += 1;

        // if(raw_data_buffer.empty()){
        //     ROS_WARN_STREAM("Input for " + obj_name + " has not been published, topic: " + sub_topic);
        //     return 3;
        // }
        // if(raw_data_buffer.rows == 0 || raw_data_buffer.cols == 0){
        //     ROS_WARN_STREAM("Input for " + obj_name + " is empty, topic: " + sub_topic);
        //     return 2;
        // }
        if (abs(last_callback_time.toSec() - ros::Time::now().toSec()) > 2)
        {
            // ROS_WARN_STREAM("Input for " + obj_name + " has not been updated for " <<
            //          abs(last_callback_time.toSec() - ros::Time::now().toSec()) << ", topic: " + sub_topic);
            return StackGridBase::err_code::LAYER_LATE_UPDATE;
        }

        T_layer = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);

        cv::Point center = cv::Point(stack_dim.width / 2.0, stack_dim.height / 2.0);

        float angle = atan2(layer_origin.pose.orientation.z, layer_origin.pose.orientation.w) * 2;

        cv::Mat T_layer_rot;
        cv::getRotationMatrix2D(center, -angle * 180 / M_PI, 1.).convertTo(T_layer_rot, CV_32FC1);

        T_layer.at<float>(0, 0) = T_layer_rot.at<float>(0, 0);
        T_layer.at<float>(0, 1) = T_layer_rot.at<float>(0, 1);
        T_layer.at<float>(1, 0) = T_layer_rot.at<float>(1, 0);
        T_layer.at<float>(1, 1) = T_layer_rot.at<float>(1, 1);

        T_layer.at<float>(0, 2) = T_layer_rot.at<float>(0, 2);
        T_layer.at<float>(1, 2) = T_layer_rot.at<float>(1, 2);

        cv::Mat T_scaling = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
        double scale_factor = layer_resolution / stack_resolution;
        T_scaling.at<float>(0, 0) = scale_factor;
        T_scaling.at<float>(1, 1) = scale_factor;

        cv::Mat T_layer_trans = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
        T_layer_trans.at<float>(0, 2) = (stack_dim.width) / 2.0;
        T_layer_trans.at<float>(1, 2) = (stack_dim.height) / 2.0;

        cv::Mat T_layer_origin_trans = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
        T_layer_origin_trans.at<float>(0, 2) = (T_layer_rot.at<float>(0, 0) * layer_origin.pose.position.x / stack_resolution + T_layer_rot.at<float>(1, 0) * layer_origin.pose.position.y / stack_resolution);
        T_layer_origin_trans.at<float>(1, 2) = (T_layer_rot.at<float>(0, 1) * layer_origin.pose.position.x / stack_resolution + T_layer_rot.at<float>(1, 1) * layer_origin.pose.position.y / stack_resolution);

        T_layer = T_layer * T_layer_trans * T_layer_origin_trans;

        // std::cout << "Translation matrix of " << get_name() << std::endl;
        // std::cout << T_layer_origin_trans << std::endl;
        // std::cout << T_layer << std::endl;

        cv::Mat T_3d_rotation;

        // std::cout << "Angle: " << angle * 180 / M_PI << std::endl;
        angle = atan2(tf_3d_msg.transform.rotation.z, tf_3d_msg.transform.rotation.w) * 2;

        cv::getRotationMatrix2D(center, angle * 180 / M_PI, 1).convertTo(T_3d_rotation, CV_32FC1);

        cv::Mat T_3d_mat = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
        T_3d_mat.at<float>(0, 0) = T_3d_rotation.at<float>(0, 0);
        T_3d_mat.at<float>(0, 1) = T_3d_rotation.at<float>(0, 1);
        T_3d_mat.at<float>(0, 2) = T_3d_rotation.at<float>(0, 2);
        T_3d_mat.at<float>(1, 0) = T_3d_rotation.at<float>(1, 0);
        T_3d_mat.at<float>(1, 1) = T_3d_rotation.at<float>(1, 1);
        T_3d_mat.at<float>(1, 2) = T_3d_rotation.at<float>(1, 2);

        cv::Mat T_3d_trans = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
        T_3d_trans.at<float>(0, 2) = -tf_3d_msg.transform.translation.x / stack_resolution;
        T_3d_trans.at<float>(1, 2) = -tf_3d_msg.transform.translation.y / stack_resolution;

        T_3d_mat *= T_3d_trans;
        // std::cout << (T_3d_mat) << std::endl;
        // std::cout << "===================================" << std::endl;
        // std::cout << (T_3d_rotation) << std::endl;
        // std::cout << (T_layer) << std::endl;

        T_layer = T_3d_mat * T_layer;

        // cv::Mat T_scaling = cv::Mat::eye(cv::Size(3,3), CV_32FC1);
        // T_scaling.at<float>(0,0) *= layer_resolution/stack_resolution;
        // T_scaling.at<float>(1,1) *= layer_resolution/stack_resolution;

        T_layer *= T_scaling;

        cv::Mat tf_2d_mat_2x3 = T_layer(cv::Range(0, 2), cv::Range(0, 3));
        // tf_2d_mat_2x3.at<float>(0,2) /= stack_resolution;
        // tf_2d_mat_2x3.at<float>(1,2) /= stack_resolution;

        // T_layer.at<float>(0,2) /= stack_resolution;
        // T_layer.at<float>(1,2) /= stack_resolution;

        // std::cout << (T_layer) << std::endl;

        raw_data_buffer.convertTo(data_img_8u, CV_8UC1);

        // ROS_INFO_STREAM("data_img_8u: " << data_img_8u.type() << " " << &data_img_8u);
        // cv::resize(data_img_8u, data_img_8u, cv::Size(), layer_resolution/stack_resolution, layer_resolution/stack_resolution);
        cv::warpAffine(data_img_8u, data_img_8u, tf_2d_mat_2x3, stack_dim,
                       cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

        // ROS_INFO_STREAM("data_img_float: " << data_img_float.type() << " " << &data_img_float);

        return 0;
    }

    ros::Time LayerHandler::getLatestTime()
    {
        return layer_origin.header.stamp;
    }

    template <>
    void LayerHandler::callback<nav_msgs::OccupancyGrid>(const nav_msgs::OccupancyGrid::ConstPtr input_data)
    {
        std::lock_guard<std::mutex> lg(data_mutex);

        layer_origin.header = input_data->header;
        layer_origin.pose = input_data->info.origin;
        layer_resolution = input_data->info.resolution;

        last_callback_time = layer_origin.header.stamp;
        // ROS_INFO_STREAM("Yo");

        raw_data_buffer = cv::Mat(input_data->data, true).reshape(1, input_data->info.height);
        raw_data_buffer += 1;
        // ROS_INFO_STREAM("raw_data_buffer: " << raw_data_buffer.type() << " " << &raw_data_buffer);

        // ROS_INFO_STREAM(data_img.type());
        // ROS_INFO_STREAM(&data_img);
        // ROS_INFO_STREAM(&occupancy_grid_mat);
    }

}