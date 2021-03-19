// -*- lsst-c++ -*-
#ifndef STACK_GRID_NODE
#define STACK_GRID_NODE

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <stack_grid_bugcar/stack_grid_bugcar.h>
#include <stack_grid_bugcar/single_layer_handler.h>

namespace stack_grid_bugcar{
    class StackGridNode{
        public:
            StackGridNode();
            ~StackGridNode(){}

           /**
            * Initalize matrices
            * @brief initialize all the necessary matrices after class construction
            */
            void initMat();
            void run();

        
        protected:
           /**
            * @brief Get transformed input data to baselink frame
            * 
            */
            int processMap(int index);

           /**
            * @brief Simple stacking method: avg(stack, input(s))
            * @param input_stack cv::Mat should be in CV_32F
            * @param output_stack cv::Mat should be in CV_8S
            */
            void simpleStack(cv::Mat &input_stack, cv::Mat &output_stack);
;
           /**
            * 
            * 
            */
            void thresholdStack(cv::Mat &stack, float threshold_value);
           /**
            * @brief Inflate map to create danger zone
            * @param main_stack Input Stack
            * @param gaussian_kernel Inflation kernel
            * @param dilation_kernel Dilation kernel
            */
            virtual void inflateLayer(cv::Mat &main_stack, cv::Mat &gaussian_kernel, cv::Mat &dilation_kernel);
            
           /**
            * @brief Shift stack from old pose to new pose
            * @param main_stack Input stack
            * @param T_shift Shifting matrix
            */ 
            void shiftToCurrentFrame(cv::Mat &main_stack, cv::Mat T_shift);

            bool publishStack(cv::Mat &main_stack);

            void imshowOccupancyGrid(std::string name, const cv::Mat &og_mat){
                cv::Mat img;
                og_mat.convertTo(img, CV_8UC1);
                img.setTo(255, og_mat == 0);
                cv::imshow(name, img);
                cv::waitKey(1);
            }

        private:
           /**
            * @brief Get transform matrix of old pose in latest frame
            */
            void getTransformToCurrent();
            
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tf_listener{tfBuffer};

            std::string global_frame_;
            std::string robot_frame;
            geometry_msgs::TransformStamped current_global_baselink_tf;
            geometry_msgs::TransformStamped old_global_baselink_tf;

            cv::Mat stack;
            cv::Mat threshold_stack;
            cv::Mat stack_int;
            cv::Mat temp_stack;
            cv::Mat unknown_mask;
            cv::Mat threshold_mask;

            cv::Mat obstacle_mask;
            cv::Mat inflation_mask;
            
            bool inflation_enable;
            double inflation_rad;
            double inscribed_rad;

            cv::Mat gaussian_kernel;
            cv::Mat dilation_kernel;

            cv::Mat T_global_baselink_current{cv::Mat::eye(cv::Size(3,3), CV_32FC1)};
            cv::Mat T_global_baselink_old{cv::Mat::eye(cv::Size(3,3), CV_32FC1)};
            cv::Mat T_time_shift{cv::Size(3,3), CV_32FC1};
            cv::Mat T_center_shift{cv::Mat::eye(cv::Size(3,3), CV_32FC1)};

            double update_frequency;
            int32_t cycle_in_millis;
            double publish_frequency;

            int32_t size_x; // Unit: pixel
            int32_t size_y;
            int32_t width; // Unit: meter
            int32_t height;
            double resolution; // Unit: meter/pixel

            bool track_unknown_;
            int default_value;
            int threshold_occupancy;

            ros::Publisher publisher;

            geometry_msgs::PoseStamped stack_origin;

            std::vector<std::shared_ptr<LayerHandler>> static_layers_handler;
            std::vector<std::shared_ptr<std::future<int>>> async_map_process;
            std::vector<std::shared_ptr<cv::Mat>> layer_mat;

            std::future<bool> async_publisher;
   
            std::mutex data_mutex;

            nav_msgs::OccupancyGrid grid_;

            char *cost_lookup_table;

    };
}
#endif