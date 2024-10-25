# Kimera-Multi: Dataset Generation

## Hardware Requirements 

Hardware as used during dataset collection and testing:

1. Intel RealSense D455: https://www.intelrealsense.com/depth-camera-d455/

2. Clearpath UGV: https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/

3. Velodyne 3D lidar (* only being used for collecting ground truth data)

4.  Intel NUC computer with an Intel i7 4.70 GHz processor

## Notes as found

- Vision pipeline is implemented via Kimera-Semantics and Kimera-VIO.

- Kimera VIO is solely responsible for processing the raw data. The raw data consists of `IMU` + `Stereo` data. 
  IMU + Stereo Frame synch packets are sent to stereo vision front end module.

- VIO does publish the processed odometry as well as depth image data from rosbag. However these topics have not been subscirbed by any node in KImera-Multi.

```
rostopic info /thoth/kimera_vio_ros/odometry 
```

Type: nav_msgs/Odometry

Publishers: 
 * /thoth/kimera_vio_ros/kimera_vio_ros_node (http://om:44033/)

Subscribers: None
--- 

```
rostopic info /thoth/forward/depth/image_rect_raw
```
Type: sensor_msgs/Image

Publishers: 
 * /player_thoth (http://om:46597/)

Subscribers: None
---

- Example rosbag info:

rosbag info 10_14_thoth.bag 
path:         10_14_thoth.bag
version:      2.0
duration:     9:14s (554s)
start:        Oct 17 2022 13:32:46.59 (1666027966.59)
end:          Oct 17 2022 13:42:00.96 (1666028520.96)
size:         5.9 GB
messages:     387818
compression:  bz2 [20570/20570 chunks; 34.26%]
uncompressed: 17.0 GB @ 31.5 MB/s
compressed:    5.8 GB @ 10.8 MB/s (34.26%)
types:        nav_msgs/Odometry           [cd5e73d190d741a2f92e81eda573aca7]
              sensor_msgs/CameraInfo      [c9a58c1b0b154e0e6da7578cb991d214]
              sensor_msgs/CompressedImage [8f7a12909da2c9d3332d540a0977563f]
              sensor_msgs/Image           [060021388200f6f0f447d0fcd9c64743]
              sensor_msgs/Imu             [6a62c6daae103f4ff57a132d6f95cec2]
              sensor_msgs/PointCloud2     [1158d486dd51d683ce2f1be655c3c181]
topics:       /thoth/forward/color/camera_info                   16562 msgs    : sensor_msgs/CameraInfo     
              /thoth/forward/color/image_raw/compressed          16562 msgs    : sensor_msgs/CompressedImage
              /thoth/forward/depth/camera_info                   16583 msgs    : sensor_msgs/CameraInfo     
              /thoth/forward/depth/image_rect_raw                16582 msgs    : sensor_msgs/Image          
              /thoth/forward/imu                                221983 msgs    : sensor_msgs/Imu            
              /thoth/forward/infra1/camera_info                  16583 msgs    : sensor_msgs/CameraInfo     
              /thoth/forward/infra1/image_rect_raw/compressed    16583 msgs    : sensor_msgs/CompressedImage
              /thoth/forward/infra2/camera_info                  16583 msgs    : sensor_msgs/CameraInfo     
              /thoth/forward/infra2/image_rect_raw/compressed    16583 msgs    : sensor_msgs/CompressedImage
              /thoth/jackal_velocity_controller/odom             27718 msgs    : nav_msgs/Odometry          
              /thoth/lidar_points                                 5496 msgs    : sensor_msgs/PointCloud2
---

- The data is pulished onto the following topics as recorded in a individual rosbag for each robot.

robot_name/forward/
├── color/
│   ├── image_raw/compressed
│   └── camera_info
├── depth/
│   ├── image_rect_raw
│   └── camera_info
├── infra1/
│   ├── image_raw/compressed
│   └── camera_info
├── infra2/
│   ├── image_raw/compressed
│   └── camera_info
└── imu

robot_name/
└── jackal_velocity_controller/odom

robot_name/
└── lidar_points

*forward* has been defined as the camera_name.

- *image-transport* package is being used to decompress the compressed image obtained from rosbag into a raw image format.

    compressed_image -> image-transport -> image_raw (decompressed)  or vice-versa

- kimera_vio_ros node subscribes to the raw image data and processed it further to the Kimera-Multi pipeline.


## ROS Wrappers for Hardware

### Intel RealSense ROS Wrapper 
    Find instructions here: https://github.com/MIT-SPARK/Kimera-VIO-ROS/blob/master/docs/hardware_setup.md#setup to set up the camera for Kimera VIO.

### Jackal Controller for Wheel Odometry
    Find ROS tests here: https://www.clearpathrobotics.com/assets/guides/noetic/jackal/JackalTests.html#running-ros-tests

    Jackal robots come preinstalled with a set of test scripts as part of the `jackal_tests` ROS package, which can be run to verify robot functionality at the component and system levels.

    If your Jackal does not have the jackal_tests ROS package installed already, you can manually install it by opening terminal and running:

    ```
    sudo apt-get install ros-noetic-jackal-tests
    ```

## Data Source Setup and Initialization 

- Play ROS Bag with --clock paramter. This indicates that the data within the bag file should be played back according to the recorded timestamps, 
  simulating the real-time operation of the robots.

- Each ROS Bag is recorded in compressed format and therfore compressed images are published. Each compressed image is passed to decompress into a raw image format. 
  Then that raw image is passed forward to the VIO front end.

To tailor the data as per the Kimera-Multi requirements, we can follow the below steps: 

1. Record rosbag for Intel RealSense raw data (imu + stereo), lidar data (`optional` since it is used as gt) and jackal wheel odometry.

-  Refer to the [Intel RealSense ROS Wrapper](#intel-realsense-ros-wrapper) mentioned above. Follow steps 1-2 in installation to install the necessary packages.

-  For RealSense, we must record the following as needed by Kimera-Multi:

    - /camera/color/camera_info
    - /camera/color/image_raw
    - /camera/depth/camera_info
    - /camera/depth/image_rect_raw
    - /camera/infra1/camera_info
    - /camera/infra1/image_rect_raw
    - /camera/infra2/camera_info
    - /camera/infra2/image_rect_raw
    - /camera/imu

- Run the following commands:

In *TERMINAL 1*, run:

```
roscore
```
In *TERMINAL 2*, run:

```
roslaunch realsense2_camera rs_camera.launch camera:=forward unite_imu_method:=linear_interpolation enable_infra1:=true enable_infra2:=true
```

*(*enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose, confidence*.)
*(after setting parameter *unite_imu_method*, the imu topics are replaced with /camera/imu)


After passing the above command we should get the following topics:

- /forward/color/camera_info
- /forward/color/image_raw
- /forward/depth/camera_info
- /forward/depth/image_rect_raw
- /forward/infra1/camera_info
- /forward/infra1/image_rect_raw
- /forward/infra2/camera_info
- /forward/infra2/image_rect_raw
- /forward/imu

To confirm if the topics are being published as needed, in another *TERMINAl*, run:

```
rostopic list
```

2. Record */jackal_velocity_controller/odom* using the jackal_tests package as referenced in the [ROS Wrapper](#ros-wrappers-for-hardware) section under [jackal](#jackal-controller-for-wheel-odometry) above.

3. To record the rosbag with the above topics, run:

```
rosbag record -o $(arg bagfile_name) --bz2 (Use BZ2 to compress data, optional if we are concerned about storage)
/forward/color/camera_info
/forward/color/image_raw
/forward/depth/camera_info
/forward/depth/image_rect_raw
/forward/infra1/camera_info
/forward/infra1/image_rect_raw
/forward/infra2/camera_info
/forward/infra2/image_rect_raw
/forward/imu
/jackal_velocity_controller/odom

```

4. [OPTIONAL] Since we may need the compressed images (for optimal storage and transport of images using rosbag files), we can use the following: 

Example to record compressed left camera image,

```
rosrun image_transport republish raw in:=/forward/infra1/image_rect_raw compressed out:=/forward/infra1/image_rect_raw/compressed
```

5. 

## Rsbag Data usage

1. Now that we have the data with us in the rosbag file, to tailor the data for each robot we can use the mit_rosbag.launch file to remap the data topics. 
   The data pane of Kimera-Multi uses a custom rosbag launch file and ccan be launched using teh following command: 

```
roslaunch kimera_distributed mit_rosbag.launch bagfile:=$ROSBAG0 input_ns:=$ROBOT0 output_ns:=$ROBOT0 rate:=$RATE
```

Input namespace can be defined as: input_ns:=forward
Output namespace can be defined as: output_ns:=robot_name

We can edit the mit_rosbag.launch file to remap the topics to maintain consistenccy for data inputs to the vision pipeline.
The reamapping in the file can be defiend as, for example:

<remap from="/$(arg input_ns)/infra1/image_rect_raw/compressed" to="/$(arg output_ns)/$(arg input_ns)/infra1/image_rect_raw/compressed"/>

After defining input_ns and output_ns, the final topic should be remapped 

from:

/forward/infra1/image_rect_raw/compressed  (as recored by our rosbag)

to:

/robot_name/forward/infra1/image_rect_raw/compressed  (as required by Kimera vision front-end)

