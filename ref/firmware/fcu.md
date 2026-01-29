# Firmware being built from source

1. Git clone PX4-Autopilot
    ```
    git clone https://github.com/HKUST-Aerial-Robotics/Nxt-FC
    ```

2. Then do,
    ```
    git submodule update --init --recursive
    make hkust_nxt-dual_bootloader
    ```
    after building the bootloader, you can find the binary file under 
    ```
    ~/Nxt-FC/PX4-Autopilot/build/hkust_nxt-dual_bootloader
    ```
3. Then, do
    ```
    make hkust_nxt-dual
    ```
    after building the firmware, you can find the .px4 file under 
    ```
    ~/Nxt-FC/PX4-Autopilot/build/hkust_nxt-dual_default
