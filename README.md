# ROS 2 pointcloud <-> laserscan converters

This is a ROS 2 package that provides components to convert `sensor_msgs/msg/PointCloud2` messages to `sensor_msgs/msg/LaserScan` messages and back.
It is essentially a port of the original ROS 1 package.

## pointcloud\_to\_laserscan::PointCloudToLaserScanNode

This ROS 2 component projects `sensor_msgs/msg/PointCloud2` messages into `sensor_msgs/msg/LaserScan` messages.

### Published Topics

* `scan` (`sensor_msgs/msg/LaserScan`) - The output laser scan.

### Subscribed Topics

* `cloud_in` (`sensor_msgs/msg/PointCloud2`) - The input point cloud. No input will be processed if there isn't at least one subscriber to the `scan` topic.

### Parameters

* `min_height` (double, default: 2.2e-308) - The minimum height to sample in the point cloud in meters.
* `max_height` (double, default: 1.8e+308) - The maximum height to sample in the point cloud in meters.
* `angle_min` (double, default: -π) - The minimum scan angle in radians.
* `angle_max` (double, default: π) - The maximum scan angle in radians.
* `angle_increment` (double, default: π/180) - Resolution of laser scan in radians per ray.
* `queue_size` (double, default: detected number of cores) - Input point cloud queue size.
* `scan_time` (double, default: 1.0/30.0) - The scan rate in seconds. Only used to populate the scan_time field of the output laser scan message.
* `range_min` (double, default: 0.0) - The minimum ranges to return in meters.
* `range_max` (double, default: 1.8e+308) - The maximum ranges to return in meters.
* `target_frame` (str, default: none) - If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.
* `transform_tolerance` (double, default: 0.01) - Time tolerance for transform lookups. Only used if a `target_frame` is provided.
* `use_inf` (boolean, default: true) - If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.

## pointcloud\_to\_laserscan::LaserScanToPointCloudNode

This ROS 2 component re-publishes `sensor_msgs/msg/LaserScan` messages as `sensor_msgs/msg/PointCloud2` messages.

### Published Topics

* `cloud` (`sensor_msgs/msg/PointCloud2`) - The output point cloud.

### Subscribed Topics

* `scan_in` (`sensor_msgs/msg/LaserScan`) - The input laser scan. No input will be processed if there isn't at least one subscriber to the `cloud` topic.

### Parameters

* `queue_size` (double, default: detected number of cores) - Input laser scan queue size.
* `target_frame` (str, default: none) - If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.
* `transform_tolerance` (double, default: 0.01) - Time tolerance for transform lookups. Only used if a `target_frame` is provided.

## Mid-360 Lidar 
I am using the fast-lio package. 



# Steps to Run 
## 1. Installation (Clone and building repo)   
```bash
# Clone the repository
git clone https://github.com/20-wash/pointcloud_to_laserscan_mid360.git

# Navigate into your workspace 
cd pointcloud_to_laserscan_mid360

# Build the workspace
colcon build 

# Source the workspace
source install/setup.bash
```

### a. Check the topic list
```bash
ros2 topic list
```
Analyze the topic list at beginning before running the node.

## 2. Play the bag file 
```bash
ros2 bag play rosbag2_2024_12_14-11_49_36_MID360/
```
### a. Check the topic list again
```bash
ros2 topic list
```
Analyze the topic list after playing the bag file. 
The topics recorded, are shown in the list. 

```bash
# To view the info of 3D_clould_point topic 
ros2 topic info /cloud_registered  

# To view the message of the topic
ros2 topic echo /cloud_registered 

# To check the verbose of the topic 
ros2 topic info /cloud_registered --verbose 
```
We can know that the QoS of thus published node is Reliable. But, on the original repo of (ros_perception/pointcloud_to_laserscan), the output /scan topic is of BEST_EFFORT. 

So, we need to make some change in original repo to account QoS problem. For that, in the node named ```pointcloud_to_laserscan_node.cpp``` inside ```src```, an extra line is added. 
```bash
// Biswash changes 
  // pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", rclcpp::SensorDataQoS().reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));
```
You can view this in thus cloned repo. 

## 3. Launch the Node 
```
ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py
````

## 4. Visualize Data in RVIz 
### a. Launch RVIz 
```
rviz2
```

### b. Edit the configuration in RVIz 
Set the frame to /camera_init

Add the topic : /scan



## 5. Testing (Verify the Node is Running)

### a. Check active topics:
```
ros2 topic list
```

### b. Confirm that /scan is being published:
```
#Check the info (msg types of topic /scan)
ros2 topic info /scan

#Check the message data
ros2 topic echo /scan
```

# Changes With Respect to original repo
## 1. Minimal Launch File 

## 2. Parameters defined inside the node named : ```pointcloud_to_laserscan_node.cpp```. 
```
// Global variables for parameters
std::string target_frame = "";
double transform_tolerance = 0.01;
double min_height = std::numeric_limits<double>::min();
double max_height = std::numeric_limits<double>::max();
double angle_min = -M_PI;
double angle_max = M_PI;
double angle_increment = M_PI / 180.0;
double scan_time = 1.0 / 30.0;
double range_min = 0.0;
double range_max = std::numeric_limits<double>::max();
bool use_inf = true;
double inf_epsilon = 1.0;
```

## 3. Suscriber and Publisher topic directly mentioned inside node named : ```pointcloud_to_laserscan_node.cpp```
The line commented ```//``` is of the original repo. 

The topics are explicitly mentioned as:  
1. ```cloud_registered``` : 

This topic is suscribed and then with respect to the parameters, the flattened map is made from the 3D_clould_point (/cloud_registered)


2. ```laser_scan```
This topic is published from the node, which published the flattened 2d laser message. 

```
// pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", rclcpp::SensorDataQoS().reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));


// sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
sub_.subscribe(this, "cloud_registered", qos.get_rmw_qos_profile());

```