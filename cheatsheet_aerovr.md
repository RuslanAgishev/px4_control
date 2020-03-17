## run simulator

    cd ~/PX4/Firmware
    make px4_sitl_default jmavsim

    cd ~/aerovr_ws/src/px4_control/scripts
    
    # connect to the simulated drone:
    roslaunch px4_control px4_sim.launch
    
    # EITHER execute test-flight command:
    rosrun px4_control drone_test.py
    
    # OR try with controller:
    rosrun px4_control position_interactive_control_vr.py

## rqt_plot

    rosrun rqt_plot rqt_plot
    # choose topic and press enter
    
## Crazyflie

    # ON OMEGA
    # roslaunch vicon_bridge vicon.launch
    # crazyflie drone is:
    # /vicon/cf4/cf4

    # ON OMEGA
    # connect to crazyflie
    roslaunch crazyflie_demo connect_crazyflie4.launch
    
    # ON OMEGA
    # launch rviz and controller
    roslaunch px4_control pos_control_viz.launch

## setup HTC Vive
    
    1) turn headset to the X-direction (facing window)

## Tests

    # vicon
    roslaunch vicon_bridge vicon.launch
    
    # controller
    python controller_test.py 10


## Technical params

        nick: 192.168.88.224
        odroid: 192.168.88.253
        vicon: 192.168.88.223
    
        есть еще fcu_url. 
        Там телеметрию указываем, что-то вроде /dev/ttyUSB0:921600 
        (баудрейт в конце)

## Connect to the big drone

ON DRONE
запустить этот лаунч для локализации по мокапу и получения данных с дрона
проверить, идут ли данные локализации на пиксхок
    
    roslaunch px4_control drone.launch gcs_url:=udp://@192.168.88.224
    
ON OMEGA (Nick's computer)
EITHER полетный скрипт (без пропеллеров, естественно сначала):
    
    roslaunch px4_control main.launch

OR drone_test ON OMEGA
    
    # uncomment five strings in main.launch
    # and comment out position_interactive_control_vr

## Configure pixhawk

    актуальная инструкция:
    https://dev.px4.io/v1.9.0/en/companion_computer/pixhawk_companion.html

    Sys_companion - баудрейт указывать 921600
    

## Add ip no interfaces

TODO: CHANGE THIS!

<!-- Check network config:

    sudo su
    vim /etc/network/interfaces
    (change ip to be different from lidar)
    address 192.168.0.150 for example
    :wq
    exit

Current config for skcr:

    # interfaces(5) file used by ifup(8) and ifdown(8)
    auto lo
    iface lo inet loopback
    auto wlp58s0
    auto eno1
    #allow-hotplug eth0
    iface eno1 inet static
            address 192.168.0.150
            netmask 255.255.255.0 -->

## Graphics in QGroundControl

    qgroundcontrol. Widgets -> Analyse
    
    
    там скрипт mocap2pixhwak делает преобразование координат с вайкона в маврос 
    и в нужный топик локализации отправляет данные 
    (/mavros/vision_position/pose, из визуальной позиции потом уже EKF в local_pose публикует)