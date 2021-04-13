// -*- lsst-c++ -*-
#include <ros/ros.h>
#include <stack_grid_bugcar/stack_grid_node.h>

namespace stack_grid_bugcar{

    void StackGridBase::initialize(){
        ros::NodeHandle nh("~/");

        ROS_INFO_STREAM("Initialize stack grid node");
        nh.param("global_frame", global_frame_, std::string("map"));
        nh.param("robot_base_frame", robot_frame, std::string("base_link"));
        // std::cout << "FUCK" << std::endl;
        while(1){
            try{
                tfBuffer.canTransform(robot_frame, global_frame_, ros::Time::now());
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                // std::cout << "FUCK" << std::endl;
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

    void StackGridBase::initDiagnostics(){
        ros::NodeHandle nh;
        diagnostics.add("Custom Stack Grid status", this, &StackGridBase::updateDiag);
        diagnostics.setHardwareID("none");

        sensor_fail = nh.advertise<std_msgs::Bool>("sensor_fail", 1);
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

            static_layers_handler.push_back(std::shared_ptr<LayerHandler>(
                new LayerHandler(nh.getNamespace(), source, 
                                global_frame_, msg_type, topic)
            ));

            // async_map_process.push_back(std::shared_ptr<std::future<int>>(
            //     new std::future<int>));

            static_layers_handler.back()->update_stack_size(size_x, size_y);
            static_layers_handler.back()->update_stack_resolution(resolution);

            layer_diagnostics.insert({source, 0});
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
        ros::Rate r_(update_frequency);
        while(ros::ok()){
            r_.reset();

            //async_publisher.wait_for(std::chrono::seconds(1));

            simpleStack(stack, stack, MAX_TEMP, stack_policy);
            
            stack.convertTo(threshold_stack, threshold_stack.type());

            threshold_stack.setTo(1, threshold_stack > 0.0);

            // thresholdStack(threshold_stack, threshold_occupancy);

            if(inflation_enable){
                inflateLayer(threshold_stack, gaussian_kernel, dilation_kernel);
            }

            threshold_stack.convertTo(publish_stack, CV_8SC1);
            publish_stack -= 1;
            publishStack(publish_stack);
            
            diagnostics.update();

            // imshowOccupancyGrid("Output stack", publish_stack);
            // async_publisher = std::async(std::launch::async, [this]{return publishStack(threshold_stack);});

            

            r_.sleep();
            actual_run_rate = 1.0/r_.cycleTime().toSec();
            ROS_INFO_STREAM_THROTTLE(1, "Stack grid is running at " << (actual_run_rate) << " Hz");

            ros::spinOnce();
            
        }
    }

    void StackGridBase::run_withTimer(const ros::TimerEvent& event){
        simpleStack(stack, stack, MAX_TEMP, stack_policy);
            
        stack.convertTo(threshold_stack, threshold_stack.type());

        thresholdStack(threshold_stack, threshold_occupancy);

        if(inflation_enable){
            inflateLayer(threshold_stack, gaussian_kernel, dilation_kernel);
        }

        threshold_stack.convertTo(publish_stack, CV_8SC1);
        publish_stack -= 1;
        publishStack(publish_stack);

        diagnostics.update();
    }

    void StackGridBase::setupTimer(ros::Timer &timer){
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(1.0/update_frequency), &StackGridBase::run_withTimer, this);
    }
    
    void StackGridBase::getTransformToCurrent(){
        try{
            current_global_baselink_tf = tfBuffer.lookupTransform(robot_frame, global_frame_, ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
        }

        double angle = atan2(current_global_baselink_tf.transform.rotation.z,
                             current_global_baselink_tf.transform.rotation.w) * 2;

        T_global_baselink_current.at<float>(0,0) = cos(angle);
        T_global_baselink_current.at<float>(1,1) = T_global_baselink_current.at<float>(0,0);
        T_global_baselink_current.at<float>(1,0) = sin(angle);
        T_global_baselink_current.at<float>(0,1) = -T_global_baselink_current.at<float>(1,0);
        T_global_baselink_current.at<float>(0,2) = current_global_baselink_tf.transform.translation.x / resolution;
        T_global_baselink_current.at<float>(1,2) = current_global_baselink_tf.transform.translation.y / resolution;
        
        // std::cout << current_global_baselink_tf << std::endl;
        T_time_shift = T_global_baselink_current * T_global_baselink_old.inv();
        
        // std::cout << "=================" << std::endl;
        // std::cout << T_global_baselink_old << std::endl;
        // std::cout << T_time_shift << std::endl;

        T_time_shift = T_center_shift.inv() * T_time_shift * T_center_shift;
        // T_time_shift = T_time_shift;

        // std::cout << T_time_shift << std::endl;
        T_global_baselink_current.copyTo(T_global_baselink_old);
        // std::cout << T_global_baselink_current << std::endl;
       
    }

    void StackGridBase::shiftToCurrentFrame(cv::Mat &main_stack, cv::Mat T_shift){
        // std::cout << T_shift(cv::Range(0,2), cv::Range(0,3)) << std::endl;
        cv::warpAffine(main_stack, main_stack, T_shift(cv::Range(0,2), cv::Range(0,3)), stack.size(),
                       cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0.0);
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

        std::lock_guard<std::mutex> lg(data_mutex);
        layer_diagnostics[static_layers_handler[index]->get_name()] = static_layers_handler[index]->transform_to_baselink(geo_transform);

        return layer_diagnostics[static_layers_handler[index]->get_name()];
    }
    
    void StackGridBase::simpleStack(cv::Mat &prev_stack, cv::Mat &output_stack, int temp_policy, int main_policy){
        getTemporalStack(temp_policy);
        
        
        if(main_policy == NO_MAP){
            temp_stack.copyTo(output_stack);
        } else{
            getTransformToCurrent();
            shiftToCurrentFrame(stack, T_time_shift);

            switch(main_policy){
                case AVG_STACK:
                    prev_stack.copyTo(temp_stack, temp_stack < 1);
                    cv::addWeighted(temp_stack, 1.0, prev_stack, 0.0, 0.0, output_stack);
                    // cv::max(output_stack, temp_stack, output_stack);
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
                int err_code = 0;
                if(!static_layers_handler[i]->input_is_empty()){
                    bool got_tf = true;  
                    ros::Time latest_time = static_layers_handler[i]->getLatestTime();
                    
                    try{
                        geo_transform = tfBuffer.lookupTransform(static_layers_handler[i]->get_frame_id(), robot_frame,
                                        (latest_time.toSec() > ros::Time(0).toSec() ? ros::Time(0) : latest_time));
                    }
                    catch (tf2::TransformException &ex){
                        ROS_WARN("%s",ex.what());
                        err_code = 3;
                        got_tf = false; 
                    }
                    if(got_tf){
                        // ROS_INFO_STREAM(geo_transform);
                        // return static_layers_handler[index]->transform_to_baselink(geo_transform);
                        err_code = static_layers_handler[i]->transform_to_baselink(geo_transform);
                        
                        // ROS_WARN_STREAM(err_code);
                        if(err_code !=0){
                            ROS_WARN_STREAM("Fail to get input " << static_layers_handler[i]->get_name());
                            continue;
                        }
                            
                        
                        static_layers_handler[i]->get_transformed_input(input_temp);
                        // imshowOccupancyGrid("input", input_temp);
                        cv::max(temp_stack, input_temp, temp_stack);
                    }
                } else
                    err_code = 2;

                std::lock_guard<std::mutex> lg(data_mutex);
                layer_diagnostics[static_layers_handler[i]->get_name()] = err_code;
            }
        }
        // imshowOccupancyGrid("fadsfas",input_temp);
    }
    
    void StackGridBase::thresholdStack(cv::Mat &input_stack, float threshold_value){
        cv::inRange(input_stack, 1, threshold_value, threshold_mask);

        input_stack.setTo(1, threshold_mask);
        input_stack.setTo(0, input_stack < 1);
        input_stack.setTo(101, input_stack > threshold_occupancy+1);
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

    void StackGridBase::updateDiag(diagnostic_updater::DiagnosticStatusWrapper& stat){
        std::lock_guard<std::mutex> lg(data_mutex);

        stat.add("Actual rate (Hz): ", actual_run_rate);
        stat.add("Desired rate (Hz): ", update_frequency);
        
        sensor_fail_check.data = false;
        bool layer_err = false;

        std::map<std::string, int>::iterator it;
       
        for(it = layer_diagnostics.begin(); it != layer_diagnostics.end(); it++){
            if(it->second == 0){
                stat.add(it->first, "OK");
            } else{
                layer_err = true;
                switch(it->second){
                    case StackGridBase::LAYER_LATE_UPDATE:
                        stat.add(it->first, "Layer has not been published or updated for some time");
                        break;
                    case StackGridBase::LAYER_EMPTY_BUFFER:
                        stat.add(it->first, "Layer is empty");
                        break;
                    case StackGridBase::LAYER_NO_TF:
                        stat.add(it->first, "No tf for layer's frame");
                        break;
                }
            }
        }
        

        if(layer_err == true){
            sensor_fail_check.data = true;
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "One or more layers are having errors");
        } else if(actual_run_rate < update_frequency){
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Can not achieve desired operatioon rate");
        } else{
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
        }
        sensor_fail.publish(sensor_fail_check);
    }
}