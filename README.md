# ros2_keya_servo
This is a ROS 2 node for controlling an Keya Servo motor KY170DD01005-08.


## Hardware setup
```bash
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```

