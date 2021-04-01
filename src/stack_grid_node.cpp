// -*- lsst-c++ -*-
#include <ros/ros.h>
#include <stack_grid_bugcar/stack_grid_node.h>

namespace stack_grid_bugcar{
    void StackGridBase::initialize(){
        ros::NodeHandle nh("~/");

        ROS_INFO_STREAM("Initialize stack grid node");
        nh.param("global_frame", global_frame_, std::string("map"));
        nh.param("robot_base_frame", robot_frame, std::string("base_link"));

        while(1){
            try{
                tfBuffer.canTransform(robot_frame, global_frame_, ros::Time::now(), ros::Duration(1));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            break;
        }

        nh.param("resolution", resolution, 0.1);
        nh.param("height", height, 20);
        nh.param("width", width, 20);

        size_x = height/resolution;
        size_y = width/resolution;

        grid_.data.resize(size_y*size_x);

        nh.param("update_frequency", update_frequency, 10.0);
        nh.param("publish_frequency", publish_frequency, 10.0);

        nh.param("threshold_occupancy", threshold_occupancy, 50);
        nh.param("stack_policy", stack_policy, NO_MAP);

        ROS_INFO_STREAM("Global frame: " << global_frame_);
        ROS_INFO_STREAM("Robot frame: " << robot_frame);
        ROS_INFO_STREAM("Map of size " << size_x << "x" << size_y << " resolution: " << resolution);
        ROS_INFO_STREAM("Update frequency: " << update_frequency);
        /*
        *   Initialize similar to being costmap_2d plugin
        */

        publisher = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

        
    }

    void StackGridBase::initLayerHandler(){
        ros::NodeHandle nh("~/");

        std::string source_strings;
        nh.getParam("static_sources", source_strings); 
        nh.getParam("inflation_enable", inflation_enable);
        nh.getParam("inflation_radius", inflation_rad);
        nh.param("inscribed_radius", inscribed_rad);
        nh.param("track_unknown", track_unknown_, true);

        if(track_unknown_){
            default_value = -1;
            // DEFAULT_OCCUPANCY_VALUE = -1;
        }
        else{
            default_value = 0;
            // DEFAULT_OCCUPANCY_VALUE = 0;
        }

        ROS_INFO_STREAM("Subscribed to for static layer: " << source_strings.c_str());
        
        std::stringstream ss(source_strings);
        std::string source;
        int count = 0;
        while (ss >> source){
            std::lock_guard<std::mutex> lg(data_mutex);

            ros::NodeHandle source_node(nh, source);

            std::string msg_type, topic;
            bool enable_publish;
            source_node.getParam("msg_type", msg_type);
            source_node.getParam("topic", topic);
            source_node.param("enable_publish", enable_publish, false);
            static_layers_handler.push_back(std::shared_ptr<LayerHandler>(
                new LayerHandler(nh.getNamespace(), source, 
                                global_frame_, msg_type, topic)
            ));

            // async_map_process.push_back(std::shared_ptr<std::future<int>>(
            //     new std::future<int>));

            static_layers_handler.back()->update_stack_size(size_x, size_y);
            static_layers_handler.back()->update_stack_resolution(resolution);
        }
    }

    void StackGridBase::initMat(){
        stack = cv::Mat(cv::Size(size_x, size_y), CV_32FC1);
        stack = 0;

        temp_stack = cv::Mat(cv::Size(size_x, size_y), CV_32FC1);
        temp_stack = 0;

        publish_stack = cv::Mat(cv::Size(size_x, size_y), CV_8SC1, -1);
        threshold_stack = cv::Mat(stack);
         
        T_center_shift.at<float>(0,2) = -size_x / 2.0;
        T_center_shift.at<float>(1,2) = -size_y / 2.0;

        // std::cout << T_center_shift << std::endl;

        gaussian_kernel = cv::getGaussianKernel((inflation_rad/resolution)*2+1,0.0, CV_32FC1);
        gaussian_kernel = gaussian_kernel * gaussian_kernel.t();

        int dilate_radius_cells = inscribed_rad/resolution;
        cv::Mat dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                      cv::Size(dilate_radius_cells*2 + 1,dilate_radius_cells*2 + 1),
                      cv::Point(-1,-1));
    }
    
    void StackGridBase::run(){
        //async_publisher = std::async(std::launch::async, []{return true;});
        while(ros::ok()){
            ros::Rate r_(update_frequency);

            

            //async_publisher.wait_for(std::chrono::seconds(1));

            simpleStack(stack, stack, MAX_TEMP, stack_policy);
            
            stack.convertTo(threshold_stack, threshold_stack.type());

            thresholdStack(threshold_stack, threshold_occupancy);

            if(inflation_enable){
                inflateLayer(threshold_stack, gaussian_kernel, dilation_kernel);
            }

            threshold_stack.convertTo(publish_stack, CV_8SC1);
            publish_stack -= 1;
            publishStack(publish_stack);

            // imshowOccupancyGrid("Output stack", publish_stack);
            // async_publisher = std::async(std::launch::async, [this]{return publishStack(threshold_stack);});

            r_.sleep();
            //ROS_INFO_STREAM_THROTTLE(1, "Stack grid is running at " << measured_ns << " Hz");

            ros::spinOnce();
            
        }
    }
    
    void StackGridBase::getTransformToCurrent(){
        try{
            current_global_baselink_tf = tfBuffer.lookupTransform(robot_frame, global_frame_, ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
        }

        float angle = atan2(current_global_baselink_tf.transform.rotation.z,
                            current_global_baselink_tf.transform.rotation.w) * 2;

        T_global_baselink_current.at<float>(0,0) = cos(angle);
        T_global_baselink_current.at<float>(1,1) = cos(angle);
        T_global_baselink_current.at<float>(1,0) = sin(angle);
        T_global_baselink_current.at<float>(0,1) = -sin(angle);
        T_global_baselink_current.at<float>(0,2) = current_global_baselink_tf.transform.translation.x / resolution;
        T_global_baselink_current.at<float>(1,2) = current_global_baselink_tf.transform.translation.y / resolution;
        
        // std::cout << current_global_baselink_tf << std::endl;
        T_time_shift = T_global_baselink_current * T_global_baselink_old.inv();
        
        // std::cout << T_global_baselink_old << std::endl;
        // std::cout << T_time_shift << std::endl;

        T_time_shift = T_center_shift.inv() * (T_time_shift * T_center_shift);
        // T_time_shift = T_time_shift;

        // std::cout << T_time_shift << std::endl;
        T_global_baselink_current.copyTo(T_global_baselink_old);
       
    }

    void StackGridBase::shiftToCurrentFrame(cv::Mat &main_stack, cv::Mat T_shift){
        // std::cout << T_shift(cv::Range(0,2), cv::Range(0,3)) << std::endl;
        cv::warpAffine(main_stack, main_stack, T_shift(cv::Range(0,2), cv::Range(0,3)), stack.size(),
                       cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0.0);
    }


    int StackGridBase::processInputLayer(int index){
        ros::Time latest_time = static_layers_handler[index]->getLatestTime();
        geometry_msgs::TransformStamped geo_transform;
        try{
            geo_transform = tfBuffer.lookupTransform(static_layers_handler[index]->get_frame_id(), robot_frame,
                            (latest_time.toSec() > ros::Time(0).toSec() ? ros::Time(0) : latest_time));
        }
        catch (tf2::TransformException &ex){
            ROS_WARN_STREAM_THROTTLE(1, static_layers_handler[index]->get_name() << ": " << ex.what());
        }

        return static_layers_handler[index]->transform_to_baselink(geo_transform);
    }
    
    void StackGridBase::simpleStack(cv::Mat &prev_stack, cv::Mat &output_stack, int temp_policy, int main_policy){
        getTemporalStack(temp_policy);
        
        
        if(main_policy == NO_MAP){
            ROS_INFO_STREAM("YO");
            temp_stack.copyTo(output_stack);
        } else{
            getTransformToCurrent();
            shiftToCurrentFrame(stack, T_time_shift);

            switch(main_policy){
                case AVG_STACK:
                    prev_stack.copyTo(temp_stack, temp_stack == 0);
                    cv::addWeighted(temp_stack, 1.0, prev_stack, 0.0, 0.0, output_stack);
                    break;
            }
        }
        
        
    }

    void StackGridBase::getTemporalStack(int policy){
        if(temp_stack.size() != cv::Size(size_x, size_y)){
            temp_stack = cv::Mat(cv::Size(size_x, size_y), temp_stack.type(), 0);
        }
        temp_stack = 0;

        geometry_msgs::TransformStamped geo_transform;
        cv::Mat input_temp;
        temp_stack.convertTo(input_temp, temp_stack.type());

        if(policy == MAX_TEMP){
            for(int i =  0; i < static_layers_handler.size(); ++i){
                if(!static_layers_handler[i]->input_is_empty()){  
                    ros::Time latest_time = static_layers_handler[i]->getLatestTime();
                    
                    try{
                        geo_transform = tfBuffer.lookupTransform(static_layers_handler[i]->get_frame_id(), robot_frame,
                                        (latest_time.toSec() > ros::Time(0).toSec() ? ros::Time(0) : latest_time));
                    }
                    catch (tf2::TransformException &ex){
                        ROS_WARN("%s",ex.what());
                    }

                    // ROS_INFO_STREAM(geo_transform);
                    // return static_layers_handler[index]->transform_to_baselink(geo_transform);
                    static_layers_handler[i]->transform_to_baselink(geo_transform);

                    static_layers_handler[i]->get_transformed_input(input_temp);
                    // imshowOccupancyGrid("input", input_temp);
                    cv::max(temp_stack, input_temp, temp_stack); 
                }
            }
        }
        // imshowOccupancyGrid("fadsfas",input_temp);
    }
    
    void StackGridBase::thresholdStack(cv::Mat &input_stack, float threshold_value){
        cv::inRange(input_stack, 1, threshold_value, threshold_mask);

        input_stack.setTo(1, threshold_mask);
        input_stack.setTo(0, threshold_stack < 1);
        input_stack.setTo(101, threshold_stack > threshold_occupancy+1);
    }
    
    void StackGridBase::inflateLayer(cv::Mat &main_stack, cv::Mat &gaussian_kernel, cv::Mat &dilation_kernel){
        
        cv::Mat obstacle_mask;
        main_stack.convertTo(obstacle_mask, CV_32FC1);

        cv::dilate(obstacle_mask, obstacle_mask, dilation_kernel);
        //ROS_INFO_STREAM("Applying Gaussian blur");

        //imshowOccupancyGrid("obstacle_mask", obstacle_mask);
        cv::filter2D(obstacle_mask, obstacle_mask, -1.0, gaussian_kernel, cv::Point(-1,-1));
        //main_stack.setTo(-1, main_stack < 0);
        
        obstacle_mask.convertTo(obstacle_mask, main_stack.type());
        cv::max(main_stack, obstacle_mask, main_stack);
        //imshowOccupancyGrid("inflate", main_stack);
    }

    bool StackGridBase::publishStack(cv::Mat &main_stack){
       
        grid_.header.frame_id = robot_frame;
        grid_.header.stamp = ros::Time::now();

        grid_.info.origin.position.x = -height / 2.0;
        grid_.info.origin.position.y = -width / 2.0;
        grid_.info.origin.orientation.w = 1;
        grid_.info.map_load_time = ros::Time::now();

        grid_.info.width = size_x;
        grid_.info.height = size_y;
        grid_.info.resolution = resolution;
        
        if(!main_stack.isContinuous()){
            ROS_INFO_STREAM("Discontinuous");
            main_stack.clone();
        }
        grid_.data.assign(main_stack.datastart, main_stack.dataend);
        publisher.publish(grid_);
    }
}