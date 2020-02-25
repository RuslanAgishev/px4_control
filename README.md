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
2. Clone and build PX4 Firmware:
```bash
mkdir ~/src; cd ~/src
git clone https://github.com/PX4/Firmware.git --recursive
cd ~/src/Firmware
make px4_sitl_default jmavsim
```
3. Create ROS catkin workspace and build the package:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/RuslanAgishev/px4_control/
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

4. In order to fly in a simulator:

- launch a simulated environment (Jmavsim):
```bash
make px4_sitl_default jmavsim
```

- connect to the simulated drone:
```bash
roslaunch px4_control px4_sim.launch
```

- execute flight command:
```bash
rosrun px4_control drone_test.py
```
