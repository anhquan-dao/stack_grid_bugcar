// -*- lsst-c++ -*-
#include <ros/ros.h>
#include <stack_grid_bugcar/stack_grid_node.h>

namespace stack_grid_bugcar
{

	StackGridBase::StackGridBase()
	{
		nh.reset(new ros::NodeHandle("~/"));
		private_nh.reset(new ros::NodeHandle("~/"));
	}
	StackGridBase::StackGridBase(ros::NodeHandle &nh_, ros::NodeHandle &private_nh_)
	{
		nh.reset(&nh_);
		private_nh.reset(&private_nh_);
	}

	void StackGridBase::initialize()
	{
		// ros::NodeHandle nh("~/");

		ROS_INFO_STREAM("Initialize stack grid node");
		private_nh->param("global_frame", global_frame_, std::string("map"));
		private_nh->param("robot_base_frame", robot_frame, std::string("base_link"));
		// std::cout << "FUCK" << std::endl;
		while (1)
		{
			try
			{
				tfBuffer.canTransform(robot_frame, global_frame_, ros::Time::now());
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("%s", ex.what());
				// std::cout << "FUCK" << std::endl;
				ros::Duration(1.0).sleep();
				continue;
			}
			break;
		}

		private_nh->param("resolution", resolution, 0.1);
		private_nh->param("height", height, 20);
		private_nh->param("width", width, 20);

		size_x = height / resolution;
		size_y = width / resolution;

		grid_.data.resize(size_y * size_x);

		private_nh->param("update_frequency", update_frequency, 10.0);
		private_nh->param("publish_frequency", publish_frequency, 10.0);

		private_nh->param("threshold_occupancy", threshold_occupancy, 50);
		private_nh->param("stack_policy", stack_policy, NO_MAP);
		private_nh->param("temp_policy", temp_policy, WEIGHTED_TEMP);
		private_nh->param("keep_danger", keep_danger, false);

		ROS_INFO_STREAM("Global frame: " << global_frame_);
		ROS_INFO_STREAM("Robot frame: " << robot_frame);
		ROS_INFO_STREAM("Map of size " << size_x << "x" << size_y << " resolution: " << resolution);
		ROS_INFO_STREAM("Update frequency: " << update_frequency);
		/*
		 *   Initialize similar to being costmap_2d plugin
		 */

		publisher = private_nh->advertise<nav_msgs::OccupancyGrid>("map", 1);
	}

	void StackGridBase::initDiagnostics()
	{
		// ros::NodeHandle nh;
		diagnostics.add("Custom Stack Grid status", this, &StackGridBase::updateDiag);
		diagnostics.setHardwareID("none");

		sensor_fail = private_nh->advertise<std_msgs::Bool>("sensor_fail", 1);
	}
	void StackGridBase::initLayerHandler()
	{
		// ros::NodeHandle nh("~/");

		std::string source_strings;
		private_nh->getParam("static_sources", source_strings);
		private_nh->getParam("inflation_enable", inflation_enable);
		private_nh->getParam("inflation_radius", inflation_rad);
		private_nh->param("inscribed_radius", inscribed_rad);
		private_nh->param("track_unknown", track_unknown_, true);
		private_nh->param("inflate_y_only", inflate_y_only, false);
		private_nh->param("convert_inflate_to_occupancy", convert_inflate_to_occupancy, false);

		if (track_unknown_)
		{
			default_value = -1;
			// DEFAULT_OCCUPANCY_VALUE = -1;
		}
		else
		{
			default_value = 0;
			// DEFAULT_OCCUPANCY_VALUE = 0;
		}

		ROS_INFO_STREAM("Subscribed to for static layer: " << source_strings.c_str());

		std::stringstream ss(source_strings);
		std::string source;
		int count = 0;
		while (ss >> source)
		{
			std::lock_guard<std::mutex> lg(data_mutex);

			ros::NodeHandle source_node(*nh, source);

			std::string msg_type, topic;
			double weight;

			bool enable_publish;
			source_node.getParam("msg_type", msg_type);
			source_node.getParam("topic", topic);
			weight = source_node.param("weight", 1.0);

			static_layers_handler.push_back(std::shared_ptr<LayerHandler>(
				new LayerHandler(private_nh->getNamespace(), source,
								 global_frame_, msg_type, topic, weight)));

			// async_map_process.push_back(std::shared_ptr<std::future<int>>(
			//     new std::future<int>));

			static_layers_handler.back()->update_stack_size(size_x, size_y);
			static_layers_handler.back()->update_stack_resolution(resolution);

			layer_diagnostics.insert({source, 0});
		}
	}

	void StackGridBase::initMat()
	{
		stack = cv::Mat(cv::Size(size_x, size_y), CV_32FC1);
		stack = 0;

		temp_stack = cv::Mat(cv::Size(size_x, size_y), CV_32FC1);
		temp_stack = 0;

		weight_stack = cv::Mat(cv::Size(size_x, size_y), CV_32FC1);
		weight_stack = 0;

		temp_weight_stack = cv::Mat(cv::Size(size_x, size_y), CV_32FC1);
		temp_weight_stack = 0;

		publish_stack = cv::Mat(cv::Size(size_x, size_y), CV_8SC1, -1);
		threshold_stack = cv::Mat(stack);

		T_center_shift.at<float>(0, 2) = -size_x / 2.0;
		T_center_shift.at<float>(1, 2) = -size_y / 2.0;

		// std::cout << T_center_shift << std::endl;

		int dilate_radius_cells = inscribed_rad/ resolution;
		dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
													cv::Size(dilate_radius_cells * 2 + 1, dilate_radius_cells * 2 + 1),
													cv::Point(-1, -1));

		gaussian_kernel = cv::getGaussianKernel((inflation_rad / resolution) * 2 + 1, 0.0, CV_32FC1);
		ROS_ERROR_STREAM(gaussian_kernel);

		if(!inflate_y_only)
		{	
			ROS_ERROR_STREAM(__func__ << " " << __LINE__);
			gaussian_kernel = gaussian_kernel * gaussian_kernel.t();	
		}
		else
		{
			cv::Mat mat = cv::Mat(gaussian_kernel.cols, gaussian_kernel.rows, CV_32FC1, 0.0);
			mat.at<float>(0, (mat.cols - 1)/2) = 1.0;			

			gaussian_kernel = gaussian_kernel * mat;
			gaussian_kernel = gaussian_kernel.t();
			for(int i=0; i < (gaussian_kernel.cols-1)/2; i++)
			{
				gaussian_kernel.at<float>((gaussian_kernel.cols-1)/2, i) = 0;
			}
			
		}

		if(convert_inflate_to_occupancy)
		{	
			gaussian_kernel.setTo(1.0, gaussian_kernel > 0.001);
			gaussian_kernel.convertTo(gaussian_kernel, CV_8U);
		}
	}

	void StackGridBase::run()
	{
		// async_publisher = std::async(std::launch::async, []{return true;});
		ros::Rate r_(update_frequency);
		while (ros::ok())
		{
			r_.reset();

			baseOperation(stack, publish_stack, temp_policy, stack_policy, inflation_enable, true, true);

			r_.sleep();

			ros::spinOnce();
		}
	}

	void StackGridBase::run_withTimer(const ros::TimerEvent &event)
	{
		baseOperation(stack, publish_stack, temp_policy, stack_policy, inflation_enable, true, true);
	}

	void StackGridBase::baseOperation(cv::Mat &main_stack, cv::Mat &output_stack, int temp_policy, int stack_policy, bool inflation_enable, bool timing, bool publish_enable)
	{
		auto start = std::chrono::high_resolution_clock::now();

		simpleStack(main_stack, main_stack, temp_policy, stack_policy);

		main_stack.convertTo(threshold_stack, threshold_stack.type());

		thresholdStack(threshold_stack, threshold_occupancy, keep_danger);

		if (inflation_enable)
			inflateLayer(threshold_stack, gaussian_kernel, dilation_kernel);

		threshold_stack.convertTo(output_stack, CV_8SC1);
		output_stack -= 1;

		if (publish_enable)
			publishStack(output_stack);

		diagnostics.update();

		//imshowOccupancyGrid("Output stack", output_stack);

		if (timing)
		{
			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<int64_t, std::nano> dur_ns = (end - start);
			double measured_ns = dur_ns.count();
			actual_run_rate = 1.0 / (measured_ns / 1000000000);
			if (actual_run_rate < update_frequency)
				ROS_WARN_STREAM("Stack grid plugin running at " << actual_run_rate);
		}
	}

	void StackGridBase::setupTimer(ros::Timer &timer)
	{
		// ros::NodeHandle nh;
		timer = private_nh->createTimer(ros::Duration(1.0 / update_frequency), &StackGridBase::run_withTimer, this);
	}

	void StackGridBase::getTransformToCurrent()
	{
		try
		{
			current_global_baselink_tf = tfBuffer.lookupTransform(robot_frame, global_frame_, ros::Time(0));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s", ex.what());
		}

		double angle = atan2(current_global_baselink_tf.transform.rotation.z,
							 current_global_baselink_tf.transform.rotation.w) *
					   2;

		T_global_baselink_current.at<float>(0, 0) = cos(angle);
		T_global_baselink_current.at<float>(1, 1) = T_global_baselink_current.at<float>(0, 0);
		T_global_baselink_current.at<float>(1, 0) = sin(angle);
		T_global_baselink_current.at<float>(0, 1) = -T_global_baselink_current.at<float>(1, 0);
		T_global_baselink_current.at<float>(0, 2) = current_global_baselink_tf.transform.translation.x / resolution;
		T_global_baselink_current.at<float>(1, 2) = current_global_baselink_tf.transform.translation.y / resolution;

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

	void StackGridBase::shiftToCurrentFrame(cv::Mat &main_stack, cv::Mat T_shift)
	{
		// std::cout << T_shift(cv::Range(0,2), cv::Range(0,3)) << std::endl;
		cv::warpAffine(main_stack, main_stack, T_shift(cv::Range(0, 2), cv::Range(0, 3)), stack.size(),
					   cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0.0);
	}

	int StackGridBase::processInputLayer(int index)
	{

		ros::Time latest_time = static_layers_handler[index]->getLatestTime();
		geometry_msgs::TransformStamped geo_transform;
		try
		{
			geo_transform = tfBuffer.lookupTransform(static_layers_handler[index]->get_frame_id(), robot_frame,
													 (latest_time.toSec() > ros::Time(0).toSec() ? ros::Time(0) : latest_time));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN_STREAM_THROTTLE(1, static_layers_handler[index]->get_name() << ": " << ex.what());
		}

		std::lock_guard<std::mutex> lg(data_mutex);
		layer_diagnostics[static_layers_handler[index]->get_name()] = static_layers_handler[index]->transform_to_baselink(geo_transform);

		return layer_diagnostics[static_layers_handler[index]->get_name()];
	}

	void StackGridBase::simpleStack(cv::Mat &prev_stack, cv::Mat &output_stack, int temp_policy, int main_policy)
	{
		getTemporalStack(temp_policy);

		if (main_policy == NO_MAP)
		{
			temp_stack.copyTo(output_stack);
		}
		else
		{
			getTransformToCurrent();
			shiftToCurrentFrame(stack, T_time_shift);

			switch (main_policy)
			{
			case AVG_STACK:
				prev_stack.copyTo(temp_stack, temp_stack < 1);
				cv::addWeighted(temp_stack, 1.0, prev_stack, 0.0, 0.0, output_stack);
				// cv::max(output_stack, temp_stack, output_stack);
				break;
			}
		}
	}

	void StackGridBase::getTemporalStack(int policy)
	{
		if (temp_stack.size() != cv::Size(size_x, size_y))
		{
			temp_stack = cv::Mat(cv::Size(size_x, size_y), temp_stack.type(), 0);
		}
		if (weight_stack.size() != cv::Size(size_x, size_y))
		{
			weight_stack = cv::Mat(cv::Size(size_x, size_y), weight_stack.type(), 0);
		}
		temp_stack = 0;
		weight_stack = 0;
		

		geometry_msgs::TransformStamped geo_transform;
		cv::Mat input_temp;
		temp_stack.convertTo(input_temp, temp_stack.type());

		if (policy <= WEIGHTED_TEMP)
		{
			for (int i = 0; i < static_layers_handler.size(); ++i)
			{
				int err_code = 0;
				if (!static_layers_handler[i]->input_is_empty())
				{
					bool got_tf = true;
					ros::Time latest_time = static_layers_handler[i]->getLatestTime();

					try
					{
						geo_transform = tfBuffer.lookupTransform(static_layers_handler[i]->get_frame_id(), robot_frame,
																 (latest_time.toSec() > ros::Time(0).toSec() ? ros::Time(0) : latest_time));
					}
					catch (tf2::TransformException &ex)
					{
						ROS_WARN("%s", ex.what());
						err_code = StackGridBase::err_code::LAYER_NO_TF;
						got_tf = false;
					}
					if (got_tf)
					{
						// ROS_INFO_STREAM(geo_transform);
						// return static_layers_handler[index]->transform_to_baselink(geo_transform);
						err_code = static_layers_handler[i]->transform_to_baselink(geo_transform);

						// ROS_WARN_STREAM(err_code);
						if (err_code != 0)
						{
							ROS_WARN_STREAM("Fail to get input " << static_layers_handler[i]->get_name());
							continue;
						}

						static_layers_handler[i]->get_transformed_input(input_temp);
						// imshowOccupancyGrid("input", input_temp);
						if(policy == MAX_TEMP)
						{
							cv::max(temp_stack, input_temp, temp_stack);
						}
						if(policy == WEIGHTED_TEMP)
						{	
							double layer_weight = static_layers_handler[i]->get_weight();
							temp_weight_stack = 0;
							temp_weight_stack.setTo(101*layer_weight, input_temp >= 1);
							weight_stack += temp_weight_stack;
							// ROS_INFO_STREAM("Weight of " << static_layers_handler[i]->get_name() << " : " << layer_weight);
							// imshowOccupancyGrid(static_layers_handler[i]->get_name(), temp_weight_stack);
							
							
							cv::addWeighted(temp_stack, 1.0, input_temp, static_layers_handler[i]->get_weight(), 0, temp_stack);
							// temp_stack.setTo(101, temp_stack > 101);
						}
					}
				}
				else
					err_code = StackGridBase::err_code::LAYER_EMPTY_BUFFER;

				std::lock_guard<std::mutex> lg(data_mutex);
				layer_diagnostics[static_layers_handler[i]->get_name()] = err_code;
			}
			if(policy == WEIGHTED_TEMP)
			{	
				weight_stack += 0.001;
				// //Initialize m
				// double minVal; 
				// double maxVal; 
				// cv::Point minLoc; 
				// cv::Point maxLoc;

				// minMaxLoc( weight_stack, &minVal, &maxVal, &minLoc, &maxLoc );
				// ROS_INFO_STREAM(minVal << " " << maxVal);
				// minMaxLoc( temp_stack, &minVal, &maxVal, &minLoc, &maxLoc );
				// ROS_INFO_STREAM("                  " <<minVal << " " << maxVal);

				temp_stack /= weight_stack;
				temp_stack *= 100;

				// minMaxLoc( temp_stack, &minVal, &maxVal, &minLoc, &maxLoc );
				// ROS_INFO_STREAM("                  " <<minVal << " " << maxVal);
			}
			
		}
		// imshowOccupancyGrid("fadsfas",temp_stack);
	}

	void StackGridBase::thresholdStack(cv::Mat &input_stack, float threshold_value)
	{
		cv::inRange(input_stack, 1, threshold_value, threshold_mask);

		input_stack.setTo(1, threshold_mask);
		input_stack.setTo(0, input_stack < 1);
		input_stack.setTo(101, input_stack > threshold_occupancy);
	}
	void StackGridBase::thresholdStack(cv::Mat &input_stack, float threshold_value, bool keep_danger)
	{	
		if(!keep_danger)
		{
			cv::inRange(input_stack, 0.9, threshold_value, threshold_mask);

			input_stack.setTo(1, threshold_mask);
			input_stack.setTo(0, input_stack < 0.9);
		}
		input_stack.setTo(101, input_stack >= threshold_occupancy);
	}

	void StackGridBase::inflateLayer(cv::Mat &main_stack, cv::Mat &gaussian_kernel, cv::Mat &dilation_kernel)
	{

		cv::Mat obstacle_mask;
		main_stack.convertTo(obstacle_mask, CV_32FC1);

		cv::dilate(obstacle_mask, obstacle_mask, dilation_kernel);
		// ROS_INFO_STREAM("Applying Gaussian blur");

		// imshowOccupancyGrid("obstacle_mask", obstacle_mask);
		if(!convert_inflate_to_occupancy)
		{
			cv::filter2D(obstacle_mask, obstacle_mask, -1.0, gaussian_kernel, cv::Point(-1, -1));
		}
		else
		{
			cv::dilate(obstacle_mask, obstacle_mask, gaussian_kernel);
		}
		// main_stack.setTo(-1, main_stack < 0);

		obstacle_mask.convertTo(obstacle_mask, main_stack.type());
		cv::max(main_stack, obstacle_mask, main_stack);
		// imshowOccupancyGrid("inflate", main_stack);
	}

	void StackGridBase::publishStack(cv::Mat &main_stack)
	{
		nav_msgs::OccupancyGridPtr grid_(new nav_msgs::OccupancyGrid);
		grid_->header.frame_id = robot_frame;
		grid_->header.stamp = ros::Time::now();

		grid_->info.origin.position.x = -height / 2.0;
		grid_->info.origin.position.y = -width / 2.0;
		grid_->info.origin.orientation.w = 1;
		grid_->info.map_load_time = ros::Time::now();

		grid_->info.width = size_x;
		grid_->info.height = size_y;
		grid_->info.resolution = resolution;

		if (!main_stack.isContinuous())
		{
			ROS_INFO_STREAM("Discontinuous");
			main_stack.clone();
		}
		grid_->data.assign(main_stack.datastart, main_stack.dataend);
		publisher.publish(grid_);
	}

	void StackGridBase::updateDiag(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		std::lock_guard<std::mutex> lg(data_mutex);

		stat.add("Actual rate (Hz): ", actual_run_rate);
		stat.add("Desired rate (Hz): ", update_frequency);

		sensor_fail_check.data = false;
		bool layer_err = false;

		std::map<std::string, int>::iterator it;

		for (it = layer_diagnostics.begin(); it != layer_diagnostics.end(); it++)
		{
			if (it->second == 0)
			{
				stat.add(it->first, "OK");
			}
			else
			{
				layer_err = true;
				switch (it->second)
				{
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

		if (layer_err == true)
		{
			sensor_fail_check.data = true;
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "One or more layers are having errors");
		}
		else if (actual_run_rate < update_frequency)
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Can not achieve desired operatioon rate");
		}
		else
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
		}
		sensor_fail.publish(sensor_fail_check);
	}

}