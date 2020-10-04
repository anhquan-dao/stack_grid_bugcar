#include "stack_grid_bugcar.h"
#include <pluginlib/class_list_macros.h>
 
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
        delete[] private_nh;
    }
    cost_lookup_table = NULL;
    private_nh = NULL;
}
 
void StackGrid::onInitialize(){
    ros::NodeHandle nh("~/" + name_);
    private_nh = new ros::NodeHandle("/bugcar/" + name_);

    current_ = true;    
    default_value_ = costmap_2d::FREE_SPACE;
    global_frame_ = layered_costmap_->getGlobalFrameID();

    self_costmap_publisher = nh.advertise<nav_msgs::OccupancyGrid>(nh.getNamespace() + "/costmap", 1);
    costmap_stamped_origin.header.frame_id = global_frame_;

    std::string source_strings;
    nh.getParam("static_sources",source_strings); 
    ROS_INFO_STREAM("Subscribed to for static layer: " << source_strings.c_str());
    
    std::stringstream ss(source_strings);
    std::string source;
    while (ss >> source){
        ros::NodeHandle source_node(nh, source);

        std::string msg_type, topic;
        bool enable_publish;
        source_node.getParam("msg_type", msg_type);
        source_node.getParam("topic", topic);
        source_node.param("enable_publish", enable_publish, false);
        static_layers_handler.push_back(boost::shared_ptr<SimpleLayerObj>(
            new SimpleLayerObj(nh.getNamespace(), "static_source", 
                               global_frame_, msg_type, topic)));
        
        process_map.push_back(boost::shared_ptr<boost::thread>(
            new boost::thread(boost::bind(&StackGrid::processMap, this, 
                              process_map.size() ))));
        layer_mat.push_back(boost::shared_ptr<cv::Mat>(new cv::Mat));
        process_check.push_back(false);

        static_layers_handler.back()->link_mat(layer_mat.back());
        if(enable_publish)
            static_layers_handler.back()->enableVisualization();
        else
            static_layers_handler.back()->disableVisualization();
    }

    cost_lookup_table = new char [101];
    cost_lookup_table[0] = 0;
    cost_lookup_table[99] = 253;
    cost_lookup_table[100] = 254;
    cost_lookup_table[101] = 255;
    for(int i = 1; i < 99; i++){
        cost_lookup_table[i] = char(1 + (251 * (i - 1)) / 97);
    }
    
}

void StackGrid::processMap(int index){
    while(ros::ok()){
        if(update && !process_check[index]){
            
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
            static_layers_handler[index]->transform_to_fit(geo_transform);
            process_check[index] = true;
            ROS_INFO_STREAM("called " << index);
        }
    }
}

void StackGrid::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
    enabled_ = config.enabled;
}
 
void StackGrid::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y){
    
    
    double len = sqrt(pow(size_x_*resolution_,2)+pow(size_y_*resolution_,2));
    double dx = (len/2.0)*cos(M_PI/4);
    double dy = (len/2.0)*sin(M_PI/4);
    *max_x = robot_x + dx;
    *max_y = robot_y + dy;
    *min_x = robot_x - dx;
    *min_y = robot_y - dy;

    unsigned int mx;
    unsigned int my;
    double wx;
    double wy;

    origin_x_ = layered_costmap_->getCostmap()->getOriginX();
    origin_y_ = layered_costmap_->getCostmap()->getOriginY();

    double dt = ros::Time::now().toNSec();

    resetMaps();

    costmap_stamped_origin.pose.position.x = origin_x_;
    costmap_stamped_origin.pose.position.y = origin_y_;
    costmap_stamped_origin.header.stamp = ros::Time::now();
    
    main_map_img = cv::Mat::zeros(size_x_,size_y_, CV_32FC1);
    /*
    cv::Mat overlay(size_x_,size_y_, CV_32FC1);
    for(int i = 0; i < static_layers_handler.size(); ++i){
        
        ros::Time latest_time = static_layers_handler[i]->getLatestTime();
        geometry_msgs::TransformStamped geo_transform;
        try{
            geo_transform = tfBuffer.lookupTransform(static_layers_handler[i]->get_frame_id(), global_frame_,
                 (latest_time.toSec() > ros::Time(0).toSec() ? ros::Time(0) : latest_time));
        }
        catch (tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
        }
        static_layers_handler[i]->update_main_costmap_origin(costmap_stamped_origin);
        static_layers_handler[i]->transform_to_fit(geo_transform);
        overlay = static_layers_handler[i]->getLayerIMG();
        cv::max(main_map_img, overlay, main_map_img);
    }
    */

    update = true;
    while(true){
        if(std::all_of(process_check.begin(), process_check.end(), [](bool i){return i == true;})){
            std::fill(process_check.begin(), process_check.end(), false);
            break;
        }
    }
    update = false;
    ROS_INFO_STREAM("Done updating");
    for(int i =  0; i < layer_mat.size(); ++i){
        cv::max(main_map_img, *layer_mat[i], main_map_img);
    }
}

void StackGrid::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                              int max_j){

    unsigned char *master_costmap_ = master_grid.getCharMap();
    
    if(!main_map_img.isContinuous()){
        main_map_img = main_map_img.clone();
        ROS_INFO_STREAM("Make continuous");
    }
    //std::cout << main_map_img << std::endl;
    main_map_img.convertTo(main_map_img,CV_8SC1);
    std::transform(main_map_img.datastart, main_map_img.dataend, master_costmap_, master_costmap_,
    boost::bind(&StackGrid::updateCharMap,this,_1,_2));
    ROS_INFO_STREAM("convert to std::vector");
    
}

void StackGrid::matchSize(){
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
    for(int i = 0; i < static_layers_handler.size(); ++i){
        static_layers_handler[i]->update_size(size_x_, size_y_);
    }
    main_map_img = cv::Mat::zeros(size_y_, size_x_, CV_32FC1);
}

uint8_t StackGrid::updateCharMap(const int8_t img_cell, uint8_t master_cell){
    uint8_t cost = translateOccupancyToCost(img_cell);
    //ROS_INFO_STREAM(cost << " : " << img_cell);
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
