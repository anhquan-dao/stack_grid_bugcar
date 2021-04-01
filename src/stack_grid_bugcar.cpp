#include <stack_grid_bugcar/stack_grid_bugcar.h>
#include <pluginlib/class_list_macros.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(stack_grid_bugcar::StackGrid, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace stack_grid_bugcar{
 
StackGrid::StackGrid() {}

StackGrid::~StackGrid(){
    if(cost_lookup_table != NULL){
        delete[] cost_lookup_table;
    }
    cost_lookup_table = NULL;
}
 
void StackGrid::onInitialize(){
    ros::NodeHandle nh("~/" + name_);

    current_ = true;    

    nh.param("fixed_frame", global_frame_, std::string("map"));
    robot_frame = layered_costmap_->getGlobalFrameID();
    
    nh.param("threshold_occupancy", threshold_occupancy, 50);
    nh.param("stack_policy", stack_policy, NO_MAP);

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

    cost_lookup_table = new char [102];
    cost_lookup_table[0] = 0;
    cost_lookup_table[99] = 253;
    cost_lookup_table[100] = 254;
    cost_lookup_table[101] = 255;
    for(int i = 1; i < 99; i++){
        cost_lookup_table[i] = char(1 + (251 * (i - 1)) / 97);
    }
    
}   

void StackGrid::matchSize(){
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());

    stack_dim.width = size_x_;
    stack_dim.height = size_y_;
    resolution = resolution_;
    threshold_occupancy = 50;

    initMat();
    
    for(int i = 0; i < static_layers_handler.size(); ++i){
        static_layers_handler[i]->update_stack_size(stack_dim.width, stack_dim.height);
        static_layers_handler[i]->update_stack_resolution(resolution);
    }
}

void StackGrid::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
    enabled_ = config.enabled;
}

void StackGrid::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y){
    
    auto start = std::chrono::high_resolution_clock::now();
    double len = sqrt(pow(size_x_*resolution_,2)+pow(size_y_*resolution_,2));
    double dx = (len/2.0)*cos(M_PI/4);
    double dy = (len/2.0)*sin(M_PI/4);
    *max_x = robot_x + dx;
    *max_y = robot_y + dy;
    *min_x = robot_x - dx;
    *min_y = robot_y - dy;

    origin_x_ = layered_costmap_->getCostmap()->getOriginX();
    origin_y_ = layered_costmap_->getCostmap()->getOriginY();

    resetMaps();

    simpleStack(stack, stack, MAX_TEMP, stack_policy);
            
    stack.copyTo(threshold_stack);


    thresholdStack(threshold_stack, threshold_occupancy);

    if(inflation_enable){
        inflateLayer(threshold_stack, gaussian_kernel, dilation_kernel);
    }

    threshold_stack.convertTo(publish_stack, CV_8SC1);
    publish_stack -= 1;
    // publishStack(publish_stack);

    // imshowOccupancyGrid("Output stack", publish_stack);

    /*
    
    main_map_img = cv::Mat(size_x_,size_y_, CV_32FC1,(float)DEFAULT_OCCUPANCY_VALUE);
    
    for(int i =  0; i < static_layers_handler.size(); ++i){
        *async_map_process[i] = std::async(std::launch::async,&StackGrid::processMap, this, i );
    }
    while(!std::all_of(async_map_process.begin(), async_map_process.end(),
        [](auto f){return f->wait_for(std::chrono::seconds(0)) == std::future_status::ready;})){
        
        std::this_thread::yield();
    }
    
    for(int i =  0; i < static_layers_handler.size(); ++i){
        cv::max(main_map_img, *layer_mat[i], main_map_img);
        
    }

    main_map_img.copyTo(obstacle_mask);
    obstacle_mask.setTo(-1, obstacle_mask < 100);

    cv::dilate(obstacle_mask, dilation_mask, dilation_kernel);
    dilation_mask.setTo(99, dilation_mask == 100);
    
    cv::filter2D(dilation_mask, inflation_mask, -1.0, gaussian_kernel, cv::Point(-1,-1));
    inflation_mask.setTo(-1, inflation_mask < 1.0);

    cv::max(main_map_img, inflation_mask, main_map_img);
    cv::max(main_map_img, dilation_mask, main_map_img);

    */
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<int64_t, std::nano> dur_ns = (end - start);
    double measured_ns = dur_ns.count();
    measured_ns /= 1000000000;
    ROS_INFO_STREAM_THROTTLE(1, "Stack grid plugin running at " << 1.0/measured_ns);
    
}

void StackGrid::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                              int max_j){
    
    unsigned char *master_costmap_ = master_grid.getCharMap();
    
    if(!publish_stack.isContinuous()){
        publish_stack = publish_stack.clone();
    }
    //std::cout << main_map_img << std::endl;
    std::transform(publish_stack.datastart, publish_stack.dataend, master_costmap_, master_costmap_,
    boost::bind(&StackGrid::updateCharMap,this,_1,_2)); 
      
}

uint8_t StackGrid::updateCharMap(const int8_t img_cell, uint8_t master_cell){
    uint8_t cost = translateOccupancyToCost(img_cell);
    if(master_cell == 255){
        return cost;
    }
    return std::max(master_cell,cost);
}

uint8_t StackGrid::translateOccupancyToCost(int8_t occupancyValue){
    switch(occupancyValue){
        case -1:
            return cost_lookup_table[101];
        default:
            return cost_lookup_table[occupancyValue];
    }
}

} // end namespace
