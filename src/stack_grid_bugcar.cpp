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
    if(private_nh != NULL){
        delete private_nh;
    }
    cost_lookup_table = NULL;
    private_nh = NULL;
}
 
void StackGrid::onInitialize(){
    ros::NodeHandle nh("~/" + name_);
    private_nh = new ros::NodeHandle("/bugcar/" + name_);

    current_ = true;    
    
    global_frame_ = layered_costmap_->getGlobalFrameID();

    self_costmap_publisher = nh.advertise<nav_msgs::OccupancyGrid>(nh.getNamespace() + "/costmap", 1);
    costmap_stamped_origin.header.frame_id = global_frame_;

    std::string source_strings;
    nh.getParam("static_sources", source_strings); 
    nh.getParam("inflation_radius", inflation_rad_);
    nh.param("track_unknown", track_unknown_, true);

    if(track_unknown_){
        default_value_ = costmap_2d::NO_INFORMATION;
        DEFAULT_OCCUPANCY_VALUE = -1;
    }
    else{
        default_value_ = costmap_2d::FREE_SPACE;
        DEFAULT_OCCUPANCY_VALUE = 0;
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
        static_layers_handler.push_back(std::shared_ptr<SimpleLayerObj>(
            new SimpleLayerObj(nh.getNamespace(), source, 
                               global_frame_, msg_type, topic)
        ));
        async_map_process.push_back(std::shared_ptr<std::future<void>>(
            new std::future<void>));

        layer_mat.push_back(std::shared_ptr<cv::Mat>(new cv::Mat()));

        static_layers_handler.back()->link_mat(layer_mat.back());

        if(enable_publish)
            static_layers_handler.back()->enableVisualization();
        else
            static_layers_handler.back()->disableVisualization();
        ++count;
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
void StackGrid::processMap(int index){

    ros::Time latest_time = static_layers_handler[index]->getLatestTime();
    geometry_msgs::TransformStamped geo_transform;
    try{
        geo_transform = tfBuffer.lookupTransform(static_layers_handler[index]->get_frame_id(), global_frame_,
                (latest_time.toSec() > ros::Time(0).toSec() ? ros::Time(0) : latest_time));
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
    }
    static_layers_handler[index]->update_main_costmap_origin(costmap_stamped_origin);

    int err_code = static_layers_handler[index]->transform_to_fit(geo_transform);
} 

void StackGrid::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
    enabled_ = config.enabled;
}
 
void StackGrid::simpleStack(){
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
}
void StackGrid::inflateLayer(){
    main_map_img.copyTo(obstacle_mask);
    obstacle_mask.setTo(-1, obstacle_mask < 100);

    cv::dilate(obstacle_mask, dilation_mask, dilation_kernel);
    dilation_mask.setTo(99, dilation_mask == 100);
    
    cv::filter2D(dilation_mask, inflation_mask, -1.0, gaussian_kernel, cv::Point(-1,-1));
    inflation_mask.setTo(-1, inflation_mask < 1.0);

    cv::max(main_map_img, inflation_mask, main_map_img);
    cv::max(main_map_img, dilation_mask, main_map_img);
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

    //resetMaps();

    costmap_stamped_origin.pose.position.x = origin_x_;
    costmap_stamped_origin.pose.position.y = origin_y_;
    costmap_stamped_origin.header.stamp = ros::Time::now();

    simpleStack();
    //inflateLayer();

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
    int64_t measured_ns = dur_ns.count();
    ROS_INFO_STREAM(measured_ns);
    
}

void StackGrid::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                              int max_j){
    
    unsigned char *master_costmap_ = master_grid.getCharMap();
    
    if(!main_map_img.isContinuous()){
        main_map_img = main_map_img.clone();
    }
    //std::cout << main_map_img << std::endl;
    main_map_img.convertTo(main_map_img,CV_8SC1);
    std::transform(main_map_img.datastart, main_map_img.dataend, master_costmap_, master_costmap_,
    boost::bind(&StackGrid::updateCharMap,this,_1,_2)); 
      
}

void StackGrid::matchSize(){
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
    for(int i = 0; i < static_layers_handler.size(); ++i){
        static_layers_handler[i]->set_costmap_param(this);
    }
    main_map_img = cv::Mat::zeros(size_y_, size_x_, CV_32FC1);

    int inflation_rad_cells = inflation_rad_/resolution_;
    gaussian_kernel = cv::getGaussianKernel(inflation_rad_cells*2+1,0.0, CV_32FC1);
    gaussian_kernel = gaussian_kernel * gaussian_kernel.t();

    int dilate_radius_cells = layered_costmap_->getInscribedRadius()/resolution_;
    cv::Mat dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                      cv::Size(dilate_radius_cells*2 + 1,dilate_radius_cells*2 + 1),
                      cv::Point(-1,-1));
}

uint8_t StackGrid::updateCharMap(const int8_t img_cell, uint8_t master_cell){
    uint8_t cost = translateOccupancyToCost(img_cell);
    if(master_cell == 255){
        return cost;
    }
    return std::max(master_cell,cost);
}

void StackGrid::publishCostmap(costmap_2d::Costmap2D cost_map_){
    nav_msgs::OccupancyGrid grid_;
    grid_.header.frame_id = global_frame_;
    grid_.header.stamp = ros::Time::now();

    grid_.info.origin.position.x = cost_map_.getOriginX();
    grid_.info.origin.position.y = cost_map_.getOriginY();
    grid_.info.origin.orientation.w = 1;
    grid_.info.map_load_time = ros::Time::now();

    grid_.info.width = cost_map_.getSizeInCellsX();
    grid_.info.height = cost_map_.getSizeInCellsY();
    grid_.info.resolution = resolution_;
    grid_.data.resize(cost_map_.getSizeInCellsX()*cost_map_.getSizeInCellsY());

    for(int i = 0; i < grid_.info.height; ++i){
        for(int j = 0; j < grid_.info.width; ++j){
            int index = cost_map_.getIndex(j,i);
            grid_.data[index] = (int)cost_map_.getCost(j,i);
        }
    }
    self_costmap_publisher.publish(grid_);
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
