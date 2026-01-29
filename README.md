# modular_drone

```
pip3 install inputs

pip3 install pyserial
```

# NxtPX4v2 FCU Flashing
### 3.1. Firmware Upgrade
1. Install DFU library on your own computer
    ```
    sudo apt install dfu-util -y
    ```
2. Erase and load the binary file to the FCU.
    ```
    https://github.com/generalroboticslab/modular_drone & cd modular_drone
    ```
    Then, unplug your USB connection, press the DUF button, plug your FCU into your computer, and unpress the DFU button. Then, do:
    ```
    dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D ./px4_firmware/bootload/hkust_nxt-dual_bootloader.bin 
    # ignore the error
    ```

3. Load firmware to the FCU. Open QGC, and go to "Firmware" to upload ```.px4``` to FCU. Note that you should tick on "advance settings", and then select "Custom firmware file..." under the drop-down options.

   <div align=center>
   <img src="/ref/firmware.png"/>
   </div>

    Click "OK", it should pop out a file selection panel. Go the "build" file of this repo, and select [this](/px4_firmware/px4_firmware/hkust_nxt-dual_default.px4).

4. If you want to build from source on your own, refer to [this](./ref/fcu.md)

# How to Launch

## 1. Setup ROS2 Workspace

```bash
cd ros_ws
colcon build
source install/setup.bash
```
## 2. mavros & QGC
ROS2 can still use mavros px4.launch as in ROS1:
```
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```
As for QGC, just go to [here](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

## 3. Launch Offboard Circle Flight

The offboard circle flight will:
- Take off to 3.0m altitude
- Draw a circle with 1.0m radius
- Land automatically

```bash
ros2 launch uav_ctrl offboard_circle.launch.py
```

# 4. control rover from RC
1. use
```
ros2 topic echo /mavros/rc/in
```
2. write subscriber based on this and map it to rover command
3. rememeber to add SA kill switch check
