#include "stack_grid_bugcar.h"
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS(stack_grid::StackGrid, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace stack_grid{
 
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

    self_costmap_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/bugcar/" + name_ + "/costmap_", 1);
    costmap_origin_publisher = private_nh->advertise<geometry_msgs::PoseStamped>("origin",5);

    nh.getParam("static_global_topic", static_global_topic);
    nh.getParam("static_local_topic", static_local_topic);
    nh.getParam("behaviour_regulator_topic", behaviour_regulator_topic);
    nh.getParam("dynamic_obstacle_topic", dynamic_obstacle_topic);
    nh.getParam("obstacle_tracking_topic", obstacle_tracking_topic);

    nh.param("max_delay_time", max_delay_time,1.0);

    costmap_stamped_origin.header.frame_id = global_frame_;

    if(!static_global_topic.empty())
        subscribed_topics_.push_back(&static_global_topic);
    else
        ROS_WARN_STREAM("No topic was provided for static_global_layer");
    if(!static_local_topic.empty())
        subscribed_topics_.push_back(&static_local_topic);
    else
        ROS_WARN_STREAM("No topic was provided for static_local_layer");
    if(!behaviour_regulator_topic.empty())
        subscribed_topics_.push_back(&behaviour_regulator_topic);
    else
        ROS_WARN_STREAM("No topic was provided for behaviour_regulator_layer");
    if(!dynamic_obstacle_topic.empty())
        subscribed_topics_.push_back(&dynamic_obstacle_topic); 
    else
        ROS_WARN_STREAM("No topic was provided for dynamic_obstacle_layer");
    if(!obstacle_tracking_topic.empty())
        subscribed_topics_.push_back(&obstacle_tracking_topic); 
    else
        ROS_WARN_STREAM("No topic was provided for obstaclle_tracking_layer");

    std::string st;
    for(std::vector<const std::string*>::iterator it = subscribed_topics_.begin(); it < subscribed_topics_.end(); ++it){
        st = st + " " + **it;
    }
    if(st.empty()){
        ROS_FATAL_STREAM("No input layers!!!");
        ros::shutdown();
    }
    ROS_INFO_STREAM("\tSubscribed to: " << st);

    cost_lookup_table = new char [101];
    cost_lookup_table[0] = 0;
    cost_lookup_table[99] = 253;
    cost_lookup_table[100] = 254;
    cost_lookup_table[101] = 255;
    for(int i = 1; i < 99; i++){
        cost_lookup_table[i] = char(1 + (251 * (i - 1)) / 97);
    }

}

void StackGrid::initializeServiceClient(){

    client_python = private_nh->serviceClient<stack_grid_bugcar::initStackService>("/initStackService");
    kill_python_srv = private_nh->serviceClient<stack_grid_bugcar::killStackService>("/endStackService");

    initPythonTool.request.costmap_name.data = private_nh->getNamespace();
    initPythonTool.request.layer_topic.clear();
    initPythonTool.request.layer_topic.resize(subscribed_topics_.size());

    for(int i = 0; i < subscribed_topics_.size(); ++i){
        std::string topic = *(subscribed_topics_[i]);
        initPythonTool.request.layer_topic[i].data = topic;
    }

    if(client_python.call(initPythonTool)){
        if(initPythonTool.response.init_success.data){
            python_layer_topic = initPythonTool.response.python_layer_topic.data;
            python_layer_sub = private_nh->subscribe<nav_msgs::OccupancyGrid>(python_layer_topic,1
                                                              ,boost::bind(&StackGrid::getGrid,this,_1,&python_layer,&python_layer_topic));
            ROS_INFO_STREAM("Successfully init python tool");
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
    costmap_origin_publisher.publish(costmap_stamped_origin);

    if(python_layer_buffer.size() != python_layer.data.size()){
        python_layer_buffer.resize(python_layer.data.size());
    }

    if(isActive(python_layer, python_layer_topic)){       
        python_layer_buffer = python_layer.data;
    }

}

void StackGrid::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                              int max_j){

    unsigned char *master_costmap_ = master_grid.getCharMap();

    std::transform(python_layer_buffer.begin(), python_layer_buffer.end(), master_costmap_, master_costmap_,
    boost::bind(&StackGrid::updateCharMap,this,_1,_2));

}

void StackGrid::matchSize(){
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
    
    private_nh->setParam("size_x", (int)size_x_);
    private_nh->setParam("size_y", (int)size_y_);
    private_nh->setParam("resolution", resolution_);
    private_nh->setParam("frame_id", global_frame_);
    private_nh->setParam("origin_x", (int)origin_x_);
    private_nh->setParam("origin_y", (int)origin_y_);

    initializeServiceClient();
}

void StackGrid::getGrid(const nav_msgs::OccupancyGrid::ConstPtr input_layer, nav_msgs::OccupancyGrid *layer,  std::string *topic_name_){

    // Copy occupancy grid information into a copy layer for later processing
    // Get upload time info
    layer->info.map_load_time = input_layer->info.map_load_time;
    layer->header.stamp = input_layer->header.stamp;

    // Get size info
    layer->info.width = input_layer->info.width;
    layer->info.height = input_layer->info.height;
    layer->info.resolution = input_layer->info.resolution;

    // Get origin and frame_id
    layer->info.origin = input_layer->info.origin;
    layer->header.frame_id = input_layer->header.frame_id;

    // Get occupancy data 
    if(layer->data.size() != input_layer->data.size())
        layer->data.reserve(input_layer->data.size());

    layer->data = input_layer->data;

}

bool StackGrid::isActive(const nav_msgs::OccupancyGrid &layer, const std::string &topic_name_){
    if(topic_name_.empty()){
        return false;
    }
    double dt = abs(ros::Time::now().toSec() - layer.info.map_load_time.toSec());
    if(dt > max_delay_time){
        ROS_FATAL_STREAM(topic_name_ << " took too long to load and will not be used... last update was " << dt << " seconds ago");
        return false;
    } 
    else 
        if(layer.data.capacity() == 0){
            ROS_WARN_STREAM(topic_name_ << " is empty... will not use this topic");
            return false;
        }
        else return true;
}

uint8_t StackGrid::updateCharMap(const int8_t self_cell, uint8_t master_cell){
    uint8_t cost = translateOccupancyToCost(self_cell);
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
