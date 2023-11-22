# ros2_keya_servo
This is a ROS 2 node for controlling an Keya Servo motor KY170DD01005-08.


## Hardware setup
```bash
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 250000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```

## Manually starting
```bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml
```

```bash
ros2 run ros2_keya_servo ros2_keya_servo
```