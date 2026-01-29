## prerequisite
```
sudo apt-get install openssh-server
sudo apt-get install ros-<distro>-vrpn-mocap
```

## usage
- please create vicon object on the vicon software

- check server ip address, here we have ```server_ip = 192.168.0.101```. Ping it to confirm.
    ```
    ros2 launch vrpn_mocap client.launch.yaml server:=<server_ip> port:=3883
    ```
- After installation of vrpn, you can also just use the local file here
    ```
    ./vrpn.sh
    ```

- here we have around 100 Hz, check via
    ```
    ros2 topic hz /vrpn_mocap/<lala>/pose
    ```
- theoretically speaking, we can achieve ~250 Hz. Need someone with time to debug it.
