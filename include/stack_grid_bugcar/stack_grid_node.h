// -*- lsst-c++ -*-
#ifndef STACK_GRID_NODE
#define STACK_GRID_NODE

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <cmath>
#include <map>
#include <algorithm>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <stack_grid_bugcar/single_layer_handler.h>

namespace stack_grid_bugcar
{

    /** Temporal stack maximum policy */
    static const int MAX_TEMP = 0;
    static const int WEIGHTED_TEMP = 1;

    /** Main mapping/stacking policy */
    static const int NO_MAP = -1;
    static const int AVG_STACK = 0;

    class StackGridBase
    {
    public:
        StackGridBase();
        StackGridBase(ros::NodeHandle &nh_, ros::NodeHandle &private_nh_);
        ~StackGridBase() {}

        /**
         * @brief Intialization
         */
        virtual void initialize();

        /**
         * @brief Initialize layer handlers
         */
        void initLayerHandler();

        /**
         * @brief Initialize all the necessary matrices after class construction
         */
        void initMat();

        void initDiagnostics();

        void run();

        void run_withTimer(const ros::TimerEvent &event);

        void baseOperation(cv::Mat &main_stack, cv::Mat &output_stack, int temp_policy, int stack_policy, bool inflation_enable, bool timing, bool publish_enable);

        void setupTimer(ros::Timer &timer);

        /**
         * @brief Error code
         */
        enum err_code
        {
            LAYER_NO_TF = 3,
            LAYER_EMPTY_BUFFER = 1,
            LAYER_LATE_UPDATE = 2,
        };

    protected:
        /**
         * @brief Get transformed input data in baselink frame.
         * The operation will spawn a thread to process the input layer
         * @param index index of the layer handler
         */
        int processInputLayer(int index);

        /**
         * @brief Simple stacking method: avg(stack, input(s))
         * @param prev_stack previous stack
         * @param output_stack output stack
         * @param temp_policy policy for temporal stacking
         * @param main_policy policy for fusing temporal stack and main stack
         */
        void simpleStack(cv::Mat &prev_stack, cv::Mat &output_stack, int temp_policy, int main_policy);

        /**
         * @brief Temporarily stack all input layers for later use
         * @param policy stacking policy
         */
        void getTemporalStack(int policy);

        void thresholdStack(cv::Mat &stack, float threshold_value);
        void thresholdStack(cv::Mat &stack, float threshold_value, bool keep_danger);

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

        void publishStack(cv::Mat &main_stack);

        void imshowOccupancyGrid(std::string name, const cv::Mat &og_mat)
        {
            cv::Mat img;
            og_mat.convertTo(img, CV_8UC1);
            img.setTo(255, og_mat == 0);
            cv::imshow(name, img);
            cv::waitKey(1);
        }

        void updateDiag(diagnostic_updater::DiagnosticStatusWrapper &stat);

        void updateStatus(const ros::TimerEvent &)
        {
            diagnostics.update();
        }

    protected:
        /**
         * @brief Get transform matrix of old pose in latest frame
         */
        void getTransformToCurrent();

        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::NodeHandle> private_nh;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf_listener{tfBuffer};

        std::string global_frame_;
        std::string robot_frame;
        geometry_msgs::TransformStamped current_global_baselink_tf;

        /** Main stack to stack all inputs against*/
        cv::Mat stack;
        cv::Mat threshold_stack;

        /** @name Operating matrices
         * Used for intermediate calculation
         */
        ///@{
        /** Temporal stack to hold input layers for later operation */
        cv::Mat temp_stack;
        /** Total weight stack */
        cv::Mat weight_stack;
        cv::Mat temp_weight_stack;
        /** Mask for thresholding main stack */
        cv::Mat threshold_mask;
        /** Used for publishing */
        cv::Mat publish_stack;
        ///@}

        bool inflation_enable;
        double inflation_rad;
        double inscribed_rad;

        cv::Mat gaussian_kernel;
        cv::Mat dilation_kernel;

        /** Transform matrix between global and baselink frame at current time */
        cv::Mat T_global_baselink_current{cv::Mat::eye(cv::Size(3, 3), CV_32FC1)};
        /** Transform matrix between global and baselink frame at past time */
        cv::Mat T_global_baselink_old{cv::Mat::eye(cv::Size(3, 3), CV_32FC1)};
        /** Time shifting matrix between current and most recent transform */
        cv::Mat T_time_shift{cv::Size(3, 3), CV_32FC1};
        /** Center shift matrix */
        cv::Mat T_center_shift{cv::Mat::eye(cv::Size(3, 3), CV_32FC1)};

        double update_frequency;
        double publish_frequency;

        cv::Size stack_dim = cv::Size(0, 0);
        int &size_x = stack_dim.width; // Unit: pixel
        int &size_y = stack_dim.height;
        int width; // Unit: meter
        int height;
        double resolution; // Unit: meter/pixel
        int stack_policy;
        int temp_policy;
        bool keep_danger;

        bool track_unknown_;
        int default_value;
        int threshold_occupancy;

        ros::Publisher publisher;

        geometry_msgs::PoseStamped stack_origin;

        std::vector<std::shared_ptr<LayerHandler>> static_layers_handler;

        nav_msgs::OccupancyGrid grid_;

        std::mutex data_mutex;

        diagnostic_updater::Updater diagnostics;
        std::map<std::string, int> layer_diagnostics;
        ros::Timer diag_timer;

        double actual_run_rate;

        ros::Publisher sensor_fail;
        std_msgs::Bool sensor_fail_check;
    };
}
#endif