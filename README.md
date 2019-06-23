# PX4 drone control
ROS node to control a px4-firmware drone. Waypoints following, intelligent landing.

![jmavsim_flight](https://github.com/RuslanAgishev/px4_control/blob/master/sim_flight.png)

In order to fly in simulator:

- launch a simulated environment (Jmavsim):
```bash
make posix_sitl_default jmavsim
```

- connect to the simulated drone:
```bash
roslaunch px4_control px4_sim.launch
```

- execute flight command:
```bash
rosrun px4_control drone_test.py
```
