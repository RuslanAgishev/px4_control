## Example usage

Prerequisites:
1. Install HTC Vive SteamVR.
2. ```pip install openvr``` OR [download](https://github.com/cmbruns/pyopenvr/releases) the installer.

Launch jMAVsim simulator:
```bash
make px4_sitl_default jmavsim
```
Establish connection with the drone via mavlink:
```bash
roslaunch px4_control px4_sim.launch
```
Take the VR controller you are going to use to control the drone.
And make sure it is tracked by HTC Vive base stations. You can use the script
[controller_test.py](https://github.com/RuslanAgishev/px4_control/blob/master/scripts/drone_arm/controller_test.py)
to find controller's position relative to the origin.

Start interactive drone control:
```bash
rosrun px4_control angular_interactive_control_vr.py
```
