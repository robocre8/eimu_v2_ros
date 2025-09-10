## Easy IMU ROS Package
This the **ROS2** Package for the using the **`Easy IMU Module`** (**`MPU9250 EIMU Module`**) with **ROS2** in a PC or microcomputer, after successful setup with the [eimu_setup_application](https://robocre8.gitbook.io/robocre8/eimu-tutorials/how-to-calibrate-and-setup-the-eimu).

> [!NOTE]  
> It should be used with your ros2 project running on Ubuntu - ros-humble, ros-jazzy, etc.

## How to Use the Package


- install the `libserial-dev` package on your linux machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```

- install `rosdep` so you can install necessary ros related dependencies for the package (if you have not).
  ```shell
  sudo apt-get update
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  ```

- cd into the **`src/`** folder of your **`ros workspace`** and clone the repo
  ```shell
  git clone https://github.com/robocre8/eimu_v2_ros.git
  ```

- from the **`src/`** folder, cd into the root directory of your **`ros workspace`** and run rosdep to install all necessary ros dependencies
  ```shell
  cd ../
  rosdep install --from-paths src --ignore-src -r -y
  ```

- build the packages with colcon (in the root directory of your **`ros workspace`**):
  ```shell
  colcon build --packages-select eimu_v2_ros --symlink-install
  ```
- before you start running the package, connect the **`Easy IMU Module`** to the Computer (PC or microcomputer) and check its serial port:
  > The best way to select the right serial port (if you are using multiple serial devices) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > You should see a value printed on the terminal (if the driver is connected and seen by the computer), your serial port would be -> **/dev/serial/by-path/[value]**. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  ```shell
  ls /dev/ttyU*
  ```
  > you should see **/dev/ttyUSB0** or **/dev/ttyUSB1** and so on

- go to the `config` folder inside the **`eimu_v2_ros`** package folder. You'll see two params file. Change the serial port value to that found in the previous step. You don't need to change the `frame_id` and `publish_frequncy` values. leave the `publish_tf_on_map_frame` value as it is in both param files.

- to vizualize in rviz (i.e quick test to see the IMU working), run:
  > *don't forget to source your `ros workspace`*
  ```shell
  ros2 launch eimu_v2_ros test.launch.py
  ``` 
  in another terminal run: 
  ```shell
  rviz2
  ```
  > Add TF and rotate the EIMU to see the transform from the imu frame to the map frame for test.

- to use in your project (e.g with a URDF file).
  > Ensure the name of the imu link frame in your URDF FILE is the same as that of the `frame_id` in the `eimu_v2_ros_start_params.yaml`
  
  First launch or run your robot's package file, then run:
  > *don't forget to source your `ros workspace`*
  ```shell
  ros2 launch eimu_v2_ros start.launch.py
  ```
  > the imu data should now be published with (or on) the robot's imu link frame.

>*NOTE: Feel free to use/edit the package as you see fit on your project.*

  
#### Sample robot test
 - please chekout the [**`easy_demo_bot`**](https://github.com/robocre8/easy_demo_bot) package to see proper sample of how combine the EIMU with the EPMC and ekf sensor-fusion in a mobile robot.
