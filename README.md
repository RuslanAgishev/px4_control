# PX4 drone control
ROS node to control a px4-firmware drone. Waypoints following, intelligent landing.

![jmavsim_flight](https://github.com/RuslanAgishev/px4_control/blob/master/sim_flight.png)

## Installation
1. Install Gazebo, jMAVsim simulators and Pixhawk tools, [reference](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html):
```bash
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/ubuntu.sh
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/requirements.txt

bash ubuntu.sh
```
Reboot when installation is finished:
```bash
sudo reboot
```

2. Install [ROS](http://wiki.ros.org/ROS/Installation) and [Mavros](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html):
```bash
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```
3. Clone and build PX4 [Firmware](https://dev.px4.io/v1.9.0/en/setup/building_px4.html):
```bash
mkdir ~/src; cd ~/src
git clone https://github.com/PX4/Firmware.git --recursive
cd ~/src/Firmware
make px4_sitl_default
```
4. Create ROS catkin workspace and build the package:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/RuslanAgishev/px4_control/
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Simulation
In order to fly in a simulator:

1. Launch a simulated environment (Jmavsim):
```bash
make px4_sitl_default jmavsim
```

2. Connect to the simulated drone:
```bash
roslaunch px4_control px4_sim.launch
```

3. Execute flight command:
```bash
rosrun px4_control drone_test.py
```

## Real drone with Vicon motion capture
[Referense](https://dev.px4.io/v1.9.0/en/ros/external_position_estimation.html): Using Vision or Motion Capture Systems for Position Estimation.

Follow the [instructions](https://dev.px4.io/v1.9.0/en/companion_computer/pixhawk_companion.html) to set-up a communication between a ground station and drone's onboard computer.

1. Install Vicon bridge ROS package:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/vicon_bridge.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

2. Make sure your UAV is tracked by Vicon motion capture system. Set-up a correct IP address of your Vicon computer [here](https://github.com/RuslanAgishev/px4_control/blob/master/launch/drone.launch#L9).

3. Connect to your drone and run external localization:
```bash
roslaunch px4_control drone.launch gcs_url:=udp://@GROUND_STATION_IP
```

4. Check that your localization is correct with the help of ```rqt_plot``` or in [QGroundcontrol](http://qgroundcontrol.com/), navigating to Widgets -> Analyse to see the data graphs.

5. If everything is correct, perform a test flight:
```bash
rosrun px4_control drone_test.py
```
