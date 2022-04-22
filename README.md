## setup loader for kitti dataset
   - resource: https://github.com/tomas789/kitti2bag
   1. Setup:
      1. Source ros noetic if you haven’t already (do this before running any ros command):
        `source /opt/ros/noetic/setup.bash`
      2. Source development setup: 
         `cd ~/catkin_ws`
         `source devel/setup.bash`
   2. Create and enter working directory:
         `mkdir src/kitti_data`    
         `cd src/kitti_data`    
   3. Download kitti dataset, then convert to bag files:
         `git clone https://github.com/tomas789/kitti2bag.git`
         `cd kitti2bag`    
         `wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip`
         `wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip`
         `unzip 2011_09_26_drive_0002_sync.zip`
         `unzip 2011_09_26_calib.zip`
         `python3 -m kitti2bag -t 2011_09_26 -r 0002 raw_synced .`

## ov2slam & Rtabmap installation
      resources:
            - ov2slam: 'https://github.com/ov2slam/ov2slam'
            - rtabmap: 'https://github.com/introlab/rtabmap_ros'
      1. Inside `/catkin_ws/src`, clone
            `cd ~/catkin_ws/src`
            `git clone git@github.com:BenThaii/WHR_SLAM_pipeline.git`
      2. Install required dependencies
            `sudo apt install ros-noetic-pcl-ros`
            `cd ~/catkin_ws/src/WHR_SLAM_pipeline/ov2slam`
            `sudo apt install libgoogle-glog-dev`
            `chmod +x build_thirdparty.sh`
            `./build_thirdparty.sh`
      3. Install OpenCV
            follow link: 'https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/'
            note: 
                  1. checkout both "opencv" and "opencv_contrib" to branch "4.2.0"
                  2. instead of using:
                        cmake -D CMAKE_BUILD_TYPE=RELEASE \
                        -D CMAKE_INSTALL_PREFIX=/usr/local \
                        -D INSTALL_C_EXAMPLES=ON \
                        -D INSTALL_PYTHON_EXAMPLES=ON \
                        -D OPENCV_GENERATE_PKGCONFIG=ON \
                        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
                        -D BUILD_EXAMPLES=ON ..
                  use:
                        cmake -D CMAKE_BUILD_TYPE=RELEASE \
                        -D CMAKE_INSTALL_PREFIX=/usr \
                        -D INSTALL_C_EXAMPLES=ON \
                        -D INSTALL_PYTHON_EXAMPLES=ON \
                        -D OPENCV_GENERATE_PKGCONFIG=ON \
                        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
                        -D BUILD_EXAMPLES=ON ..
                  
      3. Install OpenGV
            `cd`
            `git clone https://github.com/laurentkneip/opengv`
            `cd opengv`
            `mkdir build`
            `cd build/`
            `cmake ..`
            `sudo make -j4 install`
      4. Installing rtabmap standalone dependencies
            resource: `https://github.com/introlab/rtabmap_ros`
            installing rtabmap dependencies:
                  `sudo apt install ros-noetic-rtabmap ros-noetic-rtabmap-ros`
                  `sudo apt remove ros-noetic-rtabmap ros-noetic-rtabmap-ros`
            installing rtabmap standalone libraries outside of 'catkin_ws'
                  `cd ~`
                  `git clone https://github.com/introlab/rtabmap.git rtabmap`
                  `cd rtabmap/build`
                  `cmake ..  [<---double dots included]`
                  `make -j6`
                  `sudo make install`
      5. Install both packages:
            `cd ~/catkin_ws`
            `catkin_make`

# Rtabmap for freight robot for Capstone 
      roslaunch rtabmap_ros freight_navigation_capstone_v2.launch

# Miscellaneous code for multiple purposes

### turtle bot commands:
      roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

      export TURTLEBOT3_MODEL=waffle
      roslaunch turtlebot3_gazebo turtlebot3_world.launch
      
      export TURTLEBOT3_MODEL=waffle
      roslaunch rtabmap_ros demo_turtlebot3_navigation.launch

### fetch navigation launch (original freight robot)
      roslaunch fetch_navigation freight_nav.launch

### ov2slam run commands:
      rosrun ov2slam ov2slam_node ~/catkin_ws/src/WHR_SLAM_pipeline/ov2slam/parameters_files/accurate/ov2slam_capstone.yaml


### Rtabmap SLAM with wheel odometry
      roslaunch rtabmap_ros rgbd_2dlidar_wheelodom.launch

### Rtabmap SLAM with icp_odometry
roslaunch rtabmap_ros rtabmap.launch \
   icp_odometry:=true \
   odom_guess_frame_id:=odom \
   subscribe_scan:=true \
   depth:=true \
   subscribe_rgbd:=true \
   rgbd_sync:=true \
   approx_rgbd_sync:=true \
   approx_sync:=true \
   frame_id:=base_link\
   odom_frame_id:=odom \
   scan_topic:=/base_scan_raw \
   odom_topic:=/icp_odom \
   wait_imu_to_init:=true \
   imu_topic:=/imu/data \
   rgb_topic:=/head_camera/rgb/image_raw \
   depth_topic:=/head_camera/depth_registered/image_raw \
   camera_info_topic:=/head_camera/rgb/camera_info \
   depth_camera_info_topic:=/head_camera/depth_registered/camera_info \
   qos:=2 \
   use_sim_time:=true \
   args:="--delete_db_on_start --RGBD/NeighborLinkRefining true --Reg/Strategy 1 --Reg/Force3DoF true --RGBD/ProximityBySpace true --Icp/CorrespondenceRatio 0.2 --Icp/VoxelSize 0.05" \
   odom_args:="--Icp/VoxelSize 0.05 --Icp/Epsilon 0.001 --Icp/PointToPlane true --Icp/PointToPlaneK 5 --Icp/PointToPlaneRadius 0.3 --Icp/MaxCorrespondenceDistance 0.1 --Icp/PM true --Icp/PMOutlierRatio 0.65 --Odom/Strategy 0 --Odom/GuessMotion true --Odom/ResetCountdown 0 --Odom/ScanKeyFrameThr 0.9"



### Rtabmap SLAM with stereo odometry
roslaunch rtabmap_ros rtabmap.launch \
   visual_odometry:=true \
   subscribe_rgbd:=true \
   stereo:=true \
   depth:=true \
   stereo_odom_rgbd_mapping:=true \
   left_image_topic:=/multisense_sl/camera/left/stereo_image_raw \
   right_image_topic:=/multisense_sl/camera/right/stereo_image_raw \
   left_camera_info_topic:=/multisense_sl/camera/left/stereo_info \
   right_camera_info_topic:=/multisense_sl/camera/right/stereo_info \
   odom_guess_frame_id:=\odom \
   rgbd_sync:=true \
   approx_rgbd_sync:=true \
   approx_sync:=true \
   frame_id:=base_link\
   odom_frame_id:=odom \
   scan_topic:=/base_scan_raw \
   odom_topic:=/stereo_odom \
   wait_imu_to_init:=true \
   imu_topic:=/imu/data \
   rgb_topic:=/head_camera/rgb/image_raw \
   depth_topic:=/head_camera/depth_registered/image_raw \
   camera_info_topic:=/head_camera/rgb/camera_info \
   depth_camera_info_topic:=/head_camera/depth_registered/camera_info \
   qos:=2 \
   use_sim_time:=true \
   args:="--delete_db_on_start --Rtabmap/TimeThr 700 --Rtabmap/DetectionRate 1 --Kp/WordsPerImage 200 --Kp/DetectorStrategy 0 --Kp/NNStrategy 1 --SURF/HessianThreshold 1000 --LccBow/MinInliers 10 --LccBow/EstimationType 1 --LccReextract/Activated true --LccReextract/MaxWords 500 --Kp/RoiRatios 0.03 0.03 0.04 0.04" \
   odom_args:="--Odom/Strategy 0 --Odom/EstimationType 1 --Odom/MinInliers 8 --OdomBow/NNDR 0.8 --Odom/MaxFeatures 1000 --GFTT/MinDistance 10 --GFTT/QualityLevel 0.00001  --Odom/RoiRatios 0.03 0.03 0.04 0.04"


Note: loop closure isn't very good right now for visual SLAM