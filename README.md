# Motion Distortion Compensation for Lidars

Author: Zhijie Yang <yangzhj@@shanghaitech.edu.cn>

A package that corrects the distortion introduced by the motion of Lidars. 

## Dependencies

* ###  velodyne_stamped

  ```git clone https://github.com/zhijie-yang/velodyne_stamped.git```

* ### BLAM (or any SLAM front end that gives transform and trajectory estimation)

  ```shell
  git clone https://github.com/erik-nelson/blam.git
  cd blam/
  ./update # Change to source internal/devel/setup.bash (or .zsh) after the update has been done once
  roslaunch blam_example test_online.launch
  ```



## Input and Output

Input: ```stamped_scan_msgs::Scan``` in topic ```velodyne_points_stamped```

Output: ```sensor_msgs::PointCloud2``` in topic ```undistorted_points```