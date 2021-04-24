# stack_grid_bugcar
Custom standalone node and costmap2d plugin to handle OccupancyGrid inputs

# Installation
Pull the repository into the **src** of your catkin workspace and catkin_make as per ros package

# Standalone node usage
 - See the example .yaml file in **cfg** folder
 - Specify basic operation paramters
 
| Parameters               | Default       | Description   |	
| :------------------------|:-------------:| :-------------|
| global_frame 	            |	map           | Fixed reference frame for mapping operation
| robot_base_frame          | base_link     | Robot frame
| resolution 	              |	0.1           | Resolution of final output map
| width  		                | 30            | Map's width (meter)
| length 		                | 30            | Map's length (meter)
| update_frequency 	        | 10            | Update frequency (Hz)
| track unknown             | true          | Use unkown value for unexplored areas. If false, will set these ares to free
| threshold_occupancy       | 50            | Threshold on map's occupancy. Cells with greater occupancy value will be set to full occupied
| stack_policy              | -1            | Mapping method to use, -1 means no mapping. Currently not supported, set to -1
| inflation_enable          | false          | Enable built-in inflation. Enable this option will greatly reduced runtime rate
| inflation_radius			    | 1	            | Radius of inflation (meter)
| inscribed_radius			    | 0.5           | Radius of guarantee collision a.k.a. robot's radius (meter)
| static_sources			      |    	          | Name of observation sources
 
 - Specify parameters of input sources
 
 | Parameters               | Default       | Description   |
 | :------------------------|:-------------:| :-------------|
 | msg_type                 | OccupancyGrid | Type of message. Currently only supports OccupancyGrid message|
 | topic                    |               | Name of input topics|

# Costmap2d plugin usage
  - See the example .yaml file in **cfg** folder
  - Specify plugin name in common costmap params as **stack_grid_bugcar::StackGrid**
  - Replace **global_frame** parameter with **fixed_frame** parameter
  - Repeat other parameters similarly to the standalone node
  - No need to specify **robot_base_frame**, **width**, **length**, **resolution**, **update_frequency** as they are specified by the costmap
  


