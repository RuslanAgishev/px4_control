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

## Citation
The package was used in the following papers. Feel free to cite them if you find the work relevant to your research.
```bibtex
@INPROCEEDINGS{8746668,
  author={Kalinov, Ivan and Safronov, Evgenii and Agishev, Ruslan and Kurenkov, Mikhail and Tsetserukou, Dzmitry},
  booktitle={2019 IEEE 89th Vehicular Technology Conference (VTC2019-Spring)}, 
  title={High-Precision UAV Localization System for Landing on a Mobile Collaborative Robot Based on an IR Marker Pattern Recognition}, 
  year={2019},
  volume={},
  number={},
  pages={1-6},
  keywords={Robot kinematics;Cameras;Unmanned aerial vehicles;Robot vision systems;Task analysis},
  doi={10.1109/VTCSpring.2019.8746668}}
```
```bibtex
@INPROCEEDINGS{8981574,
  author={Yashin, Grigoriy A. and Trinitatova, Daria and Agishev, Ruslan T. and Ibrahimov, Roman and Tsetserukou, Dzmitry},
  booktitle={2019 19th International Conference on Advanced Robotics (ICAR)}, 
  title={AeroVr: Virtual Reality-based Teleoperation with Tactile Feedback for Aerial Manipulation}, 
  year={2019},
  volume={},
  number={},
  pages={767-772},
  keywords={},
  doi={10.1109/ICAR46387.2019.8981574}}
```
```bibtex
@INPROCEEDINGS{9476826,
  author={Kalinov, Ivan and Petrovsky, Alexander and Agishev, Ruslan and Karpyshev, Pavel and Tsetserukou, Dzmitry},
  booktitle={2021 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Impedance-based Control for Soft UAV Landing on a Ground Robot in Heterogeneous Robotic System}, 
  year={2021},
  volume={},
  number={},
  pages={1653-1658},
  keywords={Legged locomotion;Force measurement;Automation;Force;Robot sensing systems;Impedance;Force sensors},
  doi={10.1109/ICUAS51884.2021.9476826}}
```
