sudo reboot
[200~sudo usermod -aG dialout "$(id -un)"~
~sudo usermod -aG dialout "$(id -un)"~
sudo usermod -aG dialout "$(id -un)"~
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
sudo usermod -aG dialout "$(id -un)"
chmod +x QGroundControl-<arch>.AppImage
cd downloads
sudo systemctl mask --now ModemManager.service
sudo apt remove --purge modemmanager
chmod +x QGroundControl.AppImage
[200~ls ~/Downloads | grep QGroundControl
~ls ~/Downloads | grep QGroundControl
ls ~/Downloads | grep QGroundControl
cd ~/Downloads
chmod +x QGroundControl*.AppImage
qgroundcontrol
./QGroundControl*.AppImage
uname -m
sudo apt update
sudo apt install git qtbase5-dev qt5-qmake qtbase5-dev-tools      qtdeclarative5-dev qml-module-qtquick-controls      qml-module-qtquick-dialogs qml-module-qtquick2      qml-module-qtgraphicaleffects qml-module-qtpositioning      qml-module-qtquick-window2 qml-module-qtquick-layouts      libqt5serialport5-dev libqt5svg5-dev      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
git clone https://github.com/mavlink/qgroundcontrol.git --recursive
cd qgroundcontrol
qmake qgroundcontrol.pro
make -j$(nproc)
git clone --recursive https://github.com/mavlink/qgroundcontrol.git
cd qgroundcontrol
git submodule update --init --recursive
cd qgroundcontrol
git submodule update --init --recursive
ls | grep qgroundcontrol.pro
sudo apt update
sudo apt install git cmake build-essential ninja-build     qtbase5-dev qtdeclarative5-dev qtpositioning5-dev     qml-module-qtquick-controls2 qml-module-qtquick-dialogs     qml-module-qtquick-layouts qml-module-qtquick-window2     qml-module-qtgraphicaleffects qml-module-qtpositioning     libqt5svg5-dev libqt5serialport5-dev     gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
cd \Downloads
cd qgroundcontrol
mkdir build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
mkdir build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
sudo apt update
sudo apt install git cmake build-essential ninja-build     qtbase5-dev qtdeclarative5-dev qtpositioning5-dev     qml-module-qtquick-controls2 qml-module-qtquick-dialogs     qml-module-qtquick-layouts qml-module-qtquick-window2     qml-module-qtgraphicaleffects qml-module-qtpositioning     libqt5svg5-dev libqt5serialport5-dev     gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
cd qrgroundcontrol
cd qgroundcontrol
cd Downloads
cd qgroundcontrol
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
cd ~/Downloads/qgroundcontrol
mkdir build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
sudo apt remove --purge cmake -y
sudo apt update
sudo apt install -y software-properties-common lsb-release apt-transport-https ca-certificates gnupg
# Add Kitware APT repo
sudo apt-key adv --fetch-keys https://apt.kitware.com/keys/kitware-archive-latest.asc
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main'
# Install latest cmake
sudo apt update
sudo apt install cmake -y
cmake --version
sudo apt remove --purge cmake -y
sudo apt update
sudo apt install -y software-properties-common lsb-release apt-transport-https ca-certificates gnupg wget build-essential
wget https://github.com/Kitware/CMake/releases/download/v3.29.6/cmake-3.29.6-linux-aarch64.tar.gz
tar -xvzf cmake-3.29.6-linux-aarch64.tar.gz
sudo mv cmake-3.29.6-linux-aarch64 /opt/cmake-3.29.6
sudo ln -sf /opt/cmake-3.29.6/bin/* /usr/local/bin/
cmake --version
# Go to home (or Downloads)
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.29.6/cmake-3.29.6-linux-aarch64.tar.gz
# Extract it
tar -xvzf cmake-3.29.6-linux-aarch64.tar.gz
# Move it to /opt
sudo mv cmake-3.29.6-linux-aarch64 /opt/cmake-3.29.6
# Link binaries into /usr/local/bin
sudo ln -sf /opt/cmake-3.29.6/bin/* /usr/local/bin/
cmake --version
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.29.6/cmake-3.29.6-linux-aarch64.tar.gz
tar -xvzf cmake-3.29.6-linux-aarch64.tar.gz
sudo mv cmake-3.29.6-linux-aarch64 /opt/cmake-3.29.6
sudo ln -sf /opt/cmake-3.29.6/bin/* /usr/local/bin/
# remove old broken folder
sudo rm -rf /opt/cmake-3.29.6
# move the new one in place
sudo mv cmake-3.29.6-linux-aarch64 /opt/cmake-3.29.6
# relink binaries
sudo ln -sf /opt/cmake-3.29.6/bin/* /usr/local/bin/
cmake --version
sudo mv cmake-3.29.6-linux-aarch64 /opt/cmake-3.29.6
alpha@opi:~$ cmake --version
# Check if CMake is installed in /opt
ls /opt/cmake-3.29.6/bin
sudo ln -sf /opt/cmake-3.29.6/bin/* /usr/local/bin/
hash -r   # clears old PATH cache
cmake --version
cd ~/Downloads/qgroundcontrol
mkdir -p build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
./qgroundcontrol
sudo apt update
sudo apt install -y build-essential ninja-build git     libgl1-mesa-dev libxcb-cursor-dev libxcb-icccm4-dev     libxcb-image0-dev libxcb-keysyms1-dev libxcb-render-util0-dev     libxcb-xinerama0-dev libxcb-xinput-dev libxcb-shape0-dev     libxcb-xfixes0-dev libxkbcommon-dev libxkbcommon-x11-dev
cd ~
git clone https://code.qt.io/qt/qt5.git qt6
sudo add-apt-repository ppa:beineri/opt-qt-6.6.2-jammy
sudo apt update
sudo apt install qt6-base-dev qt6-base-dev-tools qt6-tools-dev
cd ~/Downloads/qgroundcontrol
mkdir -p build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
./qgroundcontrol
# Install dependencies
sudo apt update
sudo apt install -y build-essential cmake ninja-build python3 git perl     libgl1-mesa-dev libxkbcommon-dev libxkbcommon-x11-dev     libxcb-cursor-dev libxcb-icccm4-dev libxcb-image0-dev     libxcb-keysyms1-dev libxcb-render-util0-dev libxcb-xinerama0-dev     libxcb-xinput-dev libxcb-shape0-dev libxcb-xfixes0-dev
# Get Qt source
cd ~
git clone https://code.qt.io/qt/qt5.git qt6
cd qt6
git checkout v6.8.3   # exact version QGC wants
perl init-repository --module-subset=qtbase,qtdeclarative,qtquickcontrols2,qtmultimedia
# Build
mkdir build
cd build
../configure -release -nomake examples -nomake tests
make -j$(nproc)
sudo make install
cd ~/qt6
perl init-repository --module-subset=qtbase,qtdeclarative,qtmultimedia
mkdir -p build && cd build
../configure -prefix /usr/local/Qt-6.8.3 -release -nomake examples -nomake tests
make -j$(nproc)
sudo make install
ls /usr/local/Qt-6.8.3/lib/cmake/Qt6
cd ~/qt6
mkdir -p build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release     -DCMAKE_INSTALL_PREFIX=/usr/local/Qt-6.8.3     -DFEATURE_optimize_size=ON     -DQT_BUILD_EXAMPLES=OFF     -DQT_BUILD_TESTS=OFF
ninja -j$(nproc)
sudo ninja install
ls /usr/local/Qt-6.8.3/lib/cmake/Qt6
echo 'export CMAKE_PREFIX_PATH=/usr/local/Qt-6.8.3/lib/cmake' >> ~/.bashrc
source ~/.bashrc
cd ~/Downloads/qgroundcontrol/build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
./qgroundcontrol
cd ~/qt6
perl init-repository --module-subset=qtcharts
-f
f
perl init-repository -f --module-subset=qtcharts
cd ~/qt6/build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release     -DCMAKE_INSTALL_PREFIX=/usr/local/Qt-6.8.3     -DQT_BUILD_EXAMPLES=OFF -DQT_BUILD_TESTS=OFF
cd repo
git clone https://github.com/JetsonHacksNano/installVSCode.git
cd installVSCode
./installVSCodeWithPython.sh
#1759526662
micromamba
#1759526687
cd ..
#1759526690
cd repo
#1759526699
ll
#1759526704
cd ..
#1759526714
"${SHELL}" <(curl -L micro.mamba.pm/install.sh)
#1759526746
micromamba
#1759526811
cd
#1759526813
cd repo
#1759526795
micromamba create env -n py312 python=3.12 numpy
#1759526814
ll
#1759526822
git clone git@github.com:generalroboticslab/DukeHumanoidV2.git
#1759526837
cd DukeHumanoidV2/control/
#1759526840
code .
#1759526915
micromamba create -n py312 python=3.12
#1759526992
# On macOS and Linux.
#1759526992
curl -LsSf https://astral.sh/uv/install.sh | sh
#1759527007
micromamba activate py312
#1759527036
pip install uv
#1759527112
uv pip install nanobind
#1759527179
uv pip install nanobind pkg-config
#1759527196
uv pip install numpy matplotlib nanobind msgpack-python orjson
#1759527206
cd repo
#1759527209
cd
#1759527211
cd repo
#1759527242
cd
#1759527242
ll
#1759527261
cd ~/repo
#1759527261
git clone https://github.com/microsoft/vcpkg.git
#1759527285
cd vcpkg && ./bootstrap-vcpkg.sh
#1759527287
# ./vcpkg install eigen3 cserialport tbb
#1759527287
sudo apt install zip ninja-build build-essential pkg-config
#1759527303
# substitute %PATH_TO_THIS_READ_ME_DIRECTORY% with the actual directory
#1759527303
cd %PATH_TO_THIS_READ_ME_DIRECTORY%
#1759527303
# install dependency using vcpkg.json
#1759527303
~/repo/vcpkg/vcpkg install
#1759527362
cd ..
#1759527363
ll
#1759527368
cd DukeHumanoidV2/control/
#1759527375
~/repo/vcpkg/vcpkg install
#1759527574
conda activate py312
#1759527580
micromamba activate py312
#1759527585
which python
#1759527731
## ADD USER TO NETDEV GROUP TO AVOID SUDO TO ACCESS NETWORK
#1759527731
sudo adduser $USER netdev
#1759527734
sudo adduser $USER dialout # for serial access
#1759527744
sudo apt update
#1759527749
sudo apt install can-utils libsocketcan-dev nano
#1759527760
modinfo gs_usb
#1759527765
lsmod | grep can
#1759527779
sudo dmesg
#1759527835
ipconfig
#1759527837
ifconfig
#1759527876
sudo nano /etc/udev/rules.d/99-candlelight.rules
#1759527890
sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sudo udevadm trigger
sudo apt install ./code_1.104.3-1759409451_amd64.deb 
sudo apt update
sudo apt install cmake ninja-build build-essential git -y
sudo apt-get install curl zip unzip tar
# install micromamba (conda)
"${SHELL}" <(curl -L micro.mamba.pm/install.sh)
sudo apt install net-tools
sudo apt install nvtop htop zsh -y
sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"
cd repo
mkdir repo
git clone git@github.com:generalroboticslab/DukeHumanoidV2.git
code
cd .ssh
ll
#1759527949
sudo ip link set can0 up type can bitrate 1000000
#1759527957
ip -details link show type can
#1759527967
code
#1759528027
source activate
#1759528027
conda activate /home/alpha/repo/micromamba/envs/py312
#1759528046
micromamba cativate py312
#1759528054
micromamba activate py312
#1759528066
uv pip install dearpygui
#1759528085
pip install dearpygui
#1759528093
uv pip install dearpygui
#1759528124
pip install dearpygui
#1759528158
cd repo
#1759528162
git clone https://github.com/hoffstadt/DearPyGui.git
#1759528216
sudo apt install cmake build-essential ninja-build libglfw3-dev libxi-dev libxinerama-dev libxcursor-dev libxrandr-dev
#1759528242
cd DearPyGui/
#1759528250
pip install -e .
#1759528290
micromamba activate py312
#1759528294
pip install -e .
#1759528306
source activate
#1759528307
conda activate /home/alpha/repo/micromamba/envs/py312
#1759528307
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/test_gui_motor.py
#1759528321
pip show dearpygui
#1759528333
source activate
#1759528333
conda activate /home/alpha/repo/micromamba/envs/py312
#1759528342
micromamba activate py312
#1759528353
python test_gui_motor.py 
#1759528457
tree -d 2
#1759528465
sudo apt install tree
#1759528477
tree
#1759528481
tree -d 2
#1759528487
tree -h
#1759528498
python test_gui_motor.py 
#1759528531
s control/build
#1759528534
ls control/build
#1759528543
ls build
#1759528590
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/motor_gui_control.py
#1759528603
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/modular_drone_motor_config.py
#1759528614
micromamba activate py312
#1759528625
pip install tqdm
#1759528628
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/modular_drone_motor_config.py
#1759528681
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/motor_gui_control.py
#1759528768
cd ..
#1759528771
ll
#1759528776
cd DearPyGui/
#1759528785
pip install .
#1759528844
sudo apt update
#1759528851
sudo apt install -y     build-essential cmake ninja-build git     python3-dev python3.12-dev python3-pip     libgl1-mesa-dev libx11-dev libxi-dev     libxrandr-dev libxxf86vm-dev libxcursor-dev
#1759528973
python modular_drone_motor_config.py 
#1759528856
git submodule update --init --recursive
#1759529028
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/modular_drone_motor_config.py
#1759529104
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/modular_drone_rover.py
#1759529321
source activate
#1759529321
conda activate /home/alpha/repo/micromamba/envs/py312
#1759529330
sudo ip link set can0 down
#1759529334
sudo ip link set can0 up type can bitrate 1000000
#1759529334
sudo ifconfig can0 txqueuelen 128 # increase
#1759529334
micromamba activate py312
#1759529334
python modular_drone_rover.py
#1759529923
source activate
#1759529923
conda activate /home/alpha/repo/micromamba/envs/py312
#1759624398
echo $OSH
#1759624410
cat .oh-my-bash/
#1759624419
cd .oh-my-bash/
#1759624445
cd
#1759624449
gedit .bash_history 
#1759624468
nvtop
#1759624557
git status
#1759624561
git remote -v
#1759715444
sudo pip install --upgrade OPi.GPIO
#1759715459
udo usermod -aG gpio <current_user>
#1759715462
sudo usermod -aG gpio <current_user>
#1759715468
sudo usermod -aG gpio alpha
#1759715484
python -v
#1759715487
python
#1759715502
sudo pip3 install --upgrade OPi.GPIO
#1759715508
python
#1759715509
python3
#1759715889
mkdir lala
#1759715891
cd kaka
#1759715892
cd lala
#1759715903
gedit ac900.py
#1759715977
chmod +x ac900.py 
#1759715980
python3 ac900.py 
#1759716095
sudo python3 ac900.py 
#1759716453
pip3 install pyserial
#1759716613
sudo python3 ac900.py 
#1759716627
python
#1759716629
python3
#1759716638
sudo python3 ac900.py 
#1759716679
python3 ac900.py 
#1759718750
ifconfig
#1759719267
git clone https://github.com/generalroboticslab/modular_drone.git
#1759719302
cd modular_drone/
#1759719339
cd scripts/
#1759719340
l
#1759719385
python3 rx.py 
#1759773836
git pull
#1759773851
code .
#1759773888
cd modular_drone/
#1759773890
git pull
#1759773955
sudo ip link set can0 down
#1759773979
sudo ip link set can0 up type can bitrate 1000000
#1759773984
sudo ifconfig can0 txqueuelen 128
#1759774002
python joystick_rover_control.py
#1759774046
git submodule update --init --recursive
#1759774108
git submodule add https://github.com/generalroboticslab/DukeHumanoidV2
#1759774122
git submodule update --init --recursive
#1759774153
cd DukeHumanoidV2/control/motor/
#1759774157
mkdir build
#1759774160
cd build/
#1759774167
cmake ..
#1759774242
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
#1759774244
cd ..
#1759774246
sudo rm -r build/
#1759774251
mkdir build
#1759774256
cd build/
#1759774259
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
#1759774309
pwd
#1759774312
cd DukeHumanoidV2/
#1759774314
l
#1759774316
cd control/
#1759774321
cd motor/
#1759774325
sudo rm -r build/
#1759774339
cd ..
#1759774408
pwd
#1759774412
mkdir build
#1759774416
cd build/
#1759774434
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
#1759774511
source activate
#1759774511
conda activate /home/alpha/repo/micromamba/envs/py312
#1759774525
conda activate py312
#1759774536
micromamba activate py312
#1759774625
python modular_drone_rover.py 
#1759774679
source activate
#1759774679
conda activate /home/alpha/repo/micromamba/envs/py312
#1759774679
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/repo/DukeHumanoidV2/control/modular_drone_rover.py
#1759775001
sudo ip link set can0 up
#1759775008
sudo ip link set can0 down
#1759775009
python modular_drone_rover.py 
#1759775015
sudo ip link set can0 up
#1759775026
sudo ip link set can0 down
#1759775032
sudo ip link set can0 up type can bitrate 1000000
#1759775037
sudo ifconfig can0 txqueuelen 128 # increase
#1759775041
python modular_drone_rover.py 
#1759775085
python joystick_rover_control.py 
#1759792084
cd modular_drone/
#1759792092
git status
#1759792095
cd scripts/
#1759792098
cd ..
#1759792133
sudo ip link set can0 down
#1759792337
sudo apt install ssh-*
#1759792605
git pull
#1759792638
git status
#1759792643
git remote -v
#1759792646
cd DukeHumanoidV2/
#1759792648
la
#1759792661
cd ..
#1759792726
sudo ip link set can0 down
#1759792732
sudo ip link set can0 up type can bitrate 1000000
#1759792741
micromamba activate py312
#1759792745
sudo ip link set can0 down
#1759792748
sudo ip link set can0 up type can bitrate 1000000
#1759792752
sudo ifconfig can0 txqueuelen 128
#1759792758
python joystick_rover_control.py
#1759794132
git add .
#1759794174
git pull
#1759794182
git config pull.rebase false
#1759794305
git pull
#1759794345
python joystick_rover_control.py
#1759797097
micromamba activate py312
#1759797102
sudo ip link set can0 down
#1759797107
sudo ip link set can0 up type can bitrate 1000000
#1759797110
sudo ifconfig can0 txqueuelen 128
#1759797119
sudo ip link set can0 down
#1759797183
sudo ip link set can0 up type can bitrate 1000000
#1759797194
sudo ifconfig can0 txqueuelen 128
#1759797198
python joystick_rover_control.py
#1759849224
micromamba activate py312
#1759849229
sudo ip link set can0 down
#1759849235
sudo ip link set can0 up type can bitrate 1000000
#1759849240
sudo ifconfig can0 txqueuelen 128
#1759849252
python joystick_rover_control.py
#1759855758
micromamba activate py312
#1759855762
sudo ip link set can0 down
#1759855769
sudo ip link set can0 up type can bitrate 1000000
#1759855774
sudo ifconfig can0 txqueuelen 128
#1759855784
python joystick_rover_control.py
#1759857541
micromamba activate py312
#1759857545
sudo ip link set can0 down
#1759857556
sudo ip link set can0 up type can bitrate 1000000
#1759857561
sudo ifconfig can0 txqueuelen 128
#1759857567
python joystick_rover_control.py
#1759862956
micromamba activate py312
#1759862962
sudo ip link set can0 down
#1759862968
sudo ip link set can0 up type can bitrate 1000000
#1759862973
sudo ifconfig can0 txqueuelen 128
#1759862979
python joystick_rover_control.py
#1759863005
sudo ip link set can0 down
#1759863011
sudo ip link set can0 up type can bitrate 1000000
#1759863014
sudo ifconfig can0 txqueuelen 128
#1759863020
python joystick_rover_control.py
#1759863039
sudo ip link set can0 down
#1759863043
sudo ip link set can0 up type can bitrate 1000000
#1759863047
sudo ifconfig can0 txqueuelen 128
#1759863050
python joystick_rover_control.py
#1760520666
source activate
#1760520667
conda activate /home/alpha/repo/micromamba/envs/py312
#1760520667
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/Desktop/code_1.104.3-1759409451_amd64.deb
#1760520748
source activate
#1760520748
conda activate /home/alpha/repo/micromamba/envs/py312
#1760520748
/home/alpha/repo/micromamba/envs/py312/bin/python /home/alpha/Desktop/servo.py
#1760521000
cd ~/repo/modular_*   # adjust to your repo path
#1760521078
cd ~/modular_drone
#1760521093
git status
#1760521143
/bin/python3 /home/alpha/modular_drone/servo_kinematics.py
#1760521214
sudo apt install python3-numpy
#1760521249
eval "$(micromamba shell hook -s bash)"
#1760521249
micromamba activate py312
#1760521249
python /home/alpha/modular_drone/servo_kinematics.py
#1760521262
/bin/python3 /home/alpha/modular_drone/servo_kinematics.py
#1760521348
git fetch origin
#1760521349
git pull origin main
#1760521357
/bin/python3 /home/alpha/modular_drone/servo_motor.py
#1760521461
dmesg | grep tty
#1760522586
/bin/python3 /home/alpha/modular_drone/servo_motor.py
#1760522692
dmesg | grep tty
#1760522773
sudo systemctl disable brltty
#1760522778
sudo systemctl mask brltty
#1760522788
/bin/python3 /home/alpha/modular_drone/servo_motor.py
#1760522889
dmesg | tail -n 20
#1760522911
/bin/python3 /home/alpha/modular_drone/servo_motor.py
#1760743600
nano .bash_history 
#1760743671
cd
#1760743691
sudo apt-get update
#1760743745
sudo apt install software-properties-common
#1760743751
sudo add-apt-repository universe
#1760743803
sudo apt update && sudo apt install curl -y
#1760743821
sudo apt-get update
#1760743899
cd /etc/apt/sources.list.d/
#1760743909
nano archive_uri-https_apt_kitware_com_ubuntu_-jammy.list 
#1760743924
mv archive_uri-https_apt_kitware_com_ubuntu_-jammy.list lala.temp
#1760743928
sudo mv archive_uri-https_apt_kitware_com_ubuntu_-jammy.list lala.temp
#1760743934
sudo apt-get update
#1760743944
cd
#1760743947
sudo apt update && sudo apt install curl -y
#1760743961
[200~export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
#1760744024
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
#1760744035
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
#1760744041
sudo dpkg -i /tmp/ros2-apt-source.deb
#1760744045
sudo apt update
#1760744060
sudo apt upgrade
#1760744219
sudo apt install ros-humble-desktop
#1760745289
sudo apt install ros-dev-tools
#1760745495
nano .bashrc
#1760745525
source /opt/ros/humble/setup.bash
#1760745548
ros2 run demo_nodes_cpp talker
#1760745557
tmux
#1760745566
sudo snap install tmux
#1760745574
ros2 run demo_nodes_cpp talker
#1760745583
ros2 run demo_nodes_py listener
#1760745592
source /opt/ros/humble/setup.bash
#1760745597
ros2 run demo_nodes_py listener
#1760745604
tmux
#1760745611
sudo snap install tmux
#1760745625
sudo snap install tmux --classic
#1760745699
ros2 run demo_nodes_py listener
#1760745692
ros2 run demo_nodes_cpp talker
#1760745631
tmux
#1760746499
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
#1760746551
cd /opt/
#1760746552
cd ros
#1760746552
l
#1760746554
cd humble/
#1760746554
l
#1760746561
cd share/
#1760746562
l
#1760746564
cd mavros
#1760746568
cd launch/
#1760746574
nano px4.launch 
#1760747408
dmesg | grep tty
#1760747477
sudo apt purge brltty
#1760747495
dmesg | grep tty
#1760747578
cd ros2_ws/
#1760747579
l
#1760747580
cd src
#1760747581
l
#1760747582
cd uav
#1760747582
l
#1760747584
cd launch/
#1760747586
l
#1760747596
ros2 launch px4.launch 
#1760747606
source ~/.bashrc
#1760747609
ros2 launch px4.launch 
#1760747617
cd
#1760747619
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
#1760747624
sudo bash ./install_geographiclib_datasets.sh   
#1760747645
cd modular_drone/
#1760747645
l
#1760747647
cd ros2_ws/
#1760747647
l
#1760747649
cd src
#1760747650
l
#1760747653
cd uav
#1760747656
cd launch/
#1760747657
l
#1760747664
ros2 launch px4.launch 
#1760747801
tmux ls
#1760747810
ros2 topic list
#1760747820
ros2 topic echo /mavros/imu/data
#1760750250
git clone https://github.com/IntelRealSense/librealsense.git
#1760750274
cd librealsense
#1760750279
git checkout v2.50.0
#1760750285
mkdir build && cd build
#1760750292
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
#1760750324
make -j$(nproc)
#1760750779
sudo make install
#1760750899
realsense-viewer 
#1760751123
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
#1760751127
realsense-viewer 
#1760751134
cd clear
#1760751135
cd
#1760751139
cd librealsense/
#1760751139
l
#1760751146
cd build/
#1760751157
sudo make uninstall
#1760751187
cd ..
#1760751190
sudo rm -r build/
#1760751194
mkdir build
#1760751214
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
#1760751220
cd build/
#1760751221
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
#1760751264
CD
#1760751266
cd
#1760751270
sudo rm -r librealsense/
#1760751280
git clone https://github.com/IntelRealSense/librealsense.git
#1760751383
sudo apt-get update
#1760751408
sudo apt-get upgrade
#1760751424
git clone https://github.com/IntelRealSense/librealsense.git
#1760751485
cd librealsense
#1760751490
git checkout v2.50.0
#1760751521
mkdir build && cd build
#1760751525
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
#1760751535
make -j$(nproc)
#1760752009
sudo make install
#1760752021
realsense-viewer
#1760752333
cd /etc/udev/
#1760752337
cd rules.d/
#1760752358
gedit 99-realsense-libusb.rules 
#1760752498
cd
#1760752503
git clone https://github.com/pattylo/realsense-ros2-legacy.git
#1760752519
mkdir -p ~/camera_ws/src
#1760752524
cd ~/camera_ws/src/
#1760752527
git clone https://github.com/pattylo/realsense-ros2-legacy
#1760752545
sudo rm -r realsense-ros2-legacy/
#1760752555
cd ~/camera_ws
#1760752561
[200~sudo apt-get install python3-rosdep -y
#1760752568
sudo apt-get install python3-rosdep -y
#1760752586
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
#1760752591
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
#1760752604
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
#1760752619
colcon build --symlink-install
#1760752758
source install/local_setup.bash
#1760752766
ros2 launch realsense2_camera t265.launch.py
#1760752771
source install/setup.bash 
#1760752779
l
#1760752786
cd src
#1760752787
l
#1760752789
cd realsense-ros2-legacy/
#1760752789
l
#1760752793
cd realsense2_
#1760752796
cd realsense2_camera
#1760752799
cd launch/
#1760752799
l
#1760752811
cd
#1760752813
cd camera_ws/
#1760752816
source install/setup.bash 
#1760752837
ros2 topic ;ilst
#1760752840
ros2 topic list
#1760752885
ros2 topic hz /camera/fisheye1/image_raw 
#1760752903
ros2 topic echo /camera/fisheye1/image_raw 
#1760752829
ros2 launch realsense2_camera rs_t265_launch.py 
#1760754587
ros2 topic list
#1760754602
ros2 topic type /camera/odom/sample 
#1760754759
ros2 topic echo /camera/odom/sample 
#1760843029
/bin/python3 /home/alpha/modular_drone/servo_motor.py
#1760843211
ls /dev/tty*
#1760843226

#1760843338
ls /dev/tty*
#1760843350
/bin/python3 /home/alpha/modular_drone/servo_motor.py
#1760847250
sudo apt install ssh-*
#1760847361
ifconfig
#1760847480
sudo apt install ssh-*
#1760847529
ifconfig
#1760848109
cd ros_ws/
#1760848133
colcon build --symlink-install
#1760917778
python3
#1760917854
python3 servo_motor.py 
#1760918057
python3 servo_array_control.py
#1760918561
python3 servo_motor.py
#1760919174
python3 servo_array_control.py
#1760919438
python3 servo_array_control.py 
#1760924408
sudo usermod -a -G video $USER
#1760924430
lsusb | grep -i realsense
#1760924481
lsusb
#1760924553
sudo apt-get update && sudo apt-get install -y librealsense2-utils
#1760924639
sudo mkdir -p /etc/apt/keyrings
#1760924639
curl -fsSL https://librealsense.intel.com/Debian/IntelRealSense_latest_pub.gpg | sudo tee /etc/apt/keyrings/realsense.gpg >/dev/null
#1760924640
echo "deb [signed-by=/etc/apt/keyrings/realsense.gpg] http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/realsense.list
#1760924649
sudo apt update
#1760924654
sudo apt install -y librealsense2-utils librealsense2-udev librealsense2-dev
#1760924714
sudo apt install -y git cmake build-essential libusb-1.0-0-dev pkg-config udev
#1760924728
cd ~
#1760924728
git clone https://github.com/IntelRealSense/librealsense.git
#1760924728
cd librealsense
#1760924728
mkdir build && cd build
#1760924728
cmake .. -DCMAKE_BUILD_TYPE=Release          -DBUILD_EXAMPLES=ON          -DBUILD_GRAPHICAL_EXAMPLES=OFF
#1760924730
make -j"$(nproc)"
#1760925169
sudo make install
#1760925194
cd ~/librealsense
#1760925194
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
#1760925194
sudo udevadm control --reload-rules && sudo udevadm trigger
#1760925211
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
#1760925227
rs-enumerate-devices
#1760925238
realsense-viewer
#1760925271
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925272
ros2 launch realsense2_camera rs_t265.launch.py
#1760925317
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925317
sudo apt update
#1760925322
sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
#1760925372
# workspace
#1760925372
mkdir -p ~/ros2_ws/src
#1760925372
cd ~/ros2_ws/src
#1760925372
# realsense-ros wrapper (ROS 2 branch)
#1760925372
git clone -b ros2 https://github.com/IntelRealSense/realsense-ros.git
#1760925373
# deps
#1760925373
cd ..
#1760925373
rosdep update
#1760925384
rosdep install --from-paths src --ignore-src -r -y
#1760925385
# build (release)
#1760925385
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#1760925395
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
#1760925395
source ~/.bashrc
#1760925403
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925403
ros2 launch realsense2_camera rs_t265.launch.py
#1760925451
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925451
ros2 launch realsense2_camera rs_launch.py device_type:=t265 enable_pose:=true publish_odom_tf:=true
#1760925516
pkill -f realsense-viewer || true
#1760925516
pkill -f realsense2_camera || true
#1760925526
pkill -f realsense-viewer || true
#1760925534
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925534
rs-enumerate-devices
#1760925570
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925570
ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   serial_no:=908412110035 initial_reset:=true
#1760925625
ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   usb_port_id:=6-1-2 initial_reset:=true
#1760925676
pkill -f realsense-viewer || true
#1760925676
pkill -f realsense2_camera || true
#1760925683
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760925713
rs-enumerate-devices
#1760925727
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925728
ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   serial_no:="908412110035" initial_reset:=true
#1760925784
lsusb | grep -i -e 8086 -e realsense -e 0b37
#1760925784
rs-enumerate-devices
#1760925839
which rs-enumerate-devices
#1760925839
ldd $(which rs-enumerate-devices) | grep realsense
#1760925856
echo | sudo tee /etc/ld.so.conf.d/realsense-local.conf >/dev/null <<< "/usr/local/lib"
#1760925858
sudo ldconfig
#1760925865
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760925873
test -f /etc/udev/rules.d/99-realsense-libusb.rules ||   sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
#1760925873
sudo udevadm control --reload-rules && sudo udevadm trigger
#1760925880
groups | tr ' ' '\n' | grep -x video || sudo usermod -a -G video $USER
#1760925880
# if it added you to 'video', open a NEW SSH session after the next step
#1760925939
# Prefer /usr/local SDK binaries/libs in this shell
#1760925939
export PATH=/usr/local/bin:$PATH
#1760925939
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760925939
# Replug the camera so we’re fresh
#1760925939
# (physically unplug → wait 3s → plug in)
#1760925939
# Compare BOTH tools (they may differ)
#1760925939
which -a rs-enumerate-devices
#1760925939
/usr/local/bin/rs-enumerate-devices         # expect this to exist
#1760925939
/opt/ros/humble/bin/rs-enumerate-devices    # ROS-bundled one
#1760925939
# Use the /usr/local one explicitly and check output:
#1760925998
pkill -f realsense-viewer || true
#1760925998
pkill -f realsense2_camera || true
#1760925998
source /opt/ros/$ROS_DISTRO/setup.bash
#1760925999
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760925999
ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   serial_no:="908412110035" initial_reset:=true
#1760926007
source /opt/ros/$ROS_DISTRO/setup.bash
#1760926007
ros2 topic list | grep camera
#1760926009
ros2 topic echo /camera/odom/sample --once
#1760926009
ros2 topic echo /camera/pose/sample --once
#1760926052
export PATH=/usr/local/bin:$PATH
#1760926052
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760926075
/usr/local/bin/rs-enumerate-devices
#1760926087
pkill -f realsense-viewer || true
#1760926087
pkill -f realsense2_camera || true
#1760926093
source /opt/ros/$ROS_DISTRO/setup.bash
#1760926093
env RS2_LOG_LEVEL=DEBUG LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH   ros2 launch realsense2_camera rs_launch.py     device_type:=t265 enable_pose:=true publish_odom_tf:=true     serial_no:="908412110035" initial_reset:=true
#1760926121
source /opt/ros/$ROS_DISTRO/setup.bash
#1760926121
ros2 topic list | grep camera
#1760926122
ros2 topic echo /camera/odom/sample --once
#1760926123
ros2 topic echo /camera/pose/sample --once
#1760926165
# same terminal; make sure env prefers /usr/local libs
#1760926165
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760926165
source /opt/ros/$ROS_DISTRO/setup.bash
#1760926166
# ensure nothing else is using the camera
#1760926166
pkill -f realsense-viewer || true
#1760926166
pkill -f realsense2_camera || true
#1760926166
# launch using usb_port_id (no serial parameter at all)
#1760926166
ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   usb_port_id:=8-1-2 initial_reset:=true
#1760926223
ldd /opt/ros/$ROS_DISTRO/lib/realsense2_camera/realsense2_camera_node | grep librealsense
#1760926234
pkill -f realsense-viewer || true
#1760926234
pkill -f realsense2_camera || true
#1760926234
# replug T265 (unplug → 3s → plug)
#1760926234
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760926234
source /opt/ros/$ROS_DISTRO/setup.bash
#1760926235
# Use the working library explicitly and avoid serial parsing by using USB port (yours changed to 8-1-2)
#1760926235
env LD_PRELOAD=/usr/local/lib/librealsense2.so RS2_LOG_LEVEL=DEBUG   ros2 launch realsense2_camera rs_launch.py     device_type:=t265 enable_pose:=true publish_odom_tf:=true     usb_port_id:=8-1-2 initial_reset:=true
#1760926245
source /opt/ros/$ROS_DISTRO/setup.bash
#1760926245
ros2 topic list | grep camera
#1760926246
ros2 topic echo /camera/odom/sample --once
#1760926451
cd ~/ros_ws/src
#1760926451
# if a realsense-ros folder already exists, remove or rename it first
#1760926451
git clone -b ros2 https://github.com/IntelRealSense/realsense-ros.git
#1760926508
cd ~/ros2_ws/src
#1760926508
git clone https://github.com/IntelRealSense/realsense-ros.git
#1760926515
cd realsense-ros
#1760926515
git fetch --tags
#1760926516
git checkout 4.56.4
#1760926527
cd ~/ros2_ws
#1760926527
rosdep update
#1760926539
rosdep install --from-paths src --ignore-src -r -y
#1760926564
# prefer your SDK
#1760926564
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760926564
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#1760926668
# new shell (or same one)
#1760926668
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760926668
source ~/ros2_ws/install/setup.bash
#1760926669
# replug the camera so it’s fresh (your port showed as 8-1-2 earlier)
#1760926669
ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   usb_port_id:=8-1-2 initial_reset:=true
#1760926725
cd ~/ros2_ws
#1760926725
rosdep update
#1760926737
rosdep install --from-paths src --ignore-src -r -y
#1760926738
# prefer your SDK
#1760926738
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760926738
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#1760926753
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760926753
/usr/local/bin/rs-enumerate-devices
#1760926770
ldd ~/ros2_ws/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node | grep librealsense
#1760926877
ldd ~/ros2_ws/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node
#1760926908
readelf -d ~/ros2_ws/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node | grep -E 'RPATH|RUNPATH'
#1760927007
ls ~/ros2_ws/install/realsense2_camera/lib
#1760927076
ldd ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so | grep -i realsense
#1760927141
# 1) Clean only this package so CMake re-detects librealsense
#1760927141
rm -rf ~/ros2_ws/build/realsense2_camera ~/ros2_ws/install/realsense2_camera ~/ros2_ws/log
#1760927141
# 2) Make /usr/local preferred at configure & link time
#1760927141
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH}
#1760927141
export CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH}
#1760927141
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
#1760927141
# 3) Rebuild the wrapper, embedding /usr/local into RPATH
#1760927141
cd ~/ros2_ws
#1760927141
colcon build --packages-select realsense2_camera --symlink-install   --cmake-args -DCMAKE_BUILD_TYPE=Release                -Drealsense2_DIR=/usr/local/lib/cmake/realsense2                -DCMAKE_INSTALL_RPATH=/usr/local/lib                -DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE                -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=TRUE
#1760927218
ldd ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so | grep -i realsense
#1760927313
sudo apt-get install -y patchelf
#1760927340
readelf -d ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so | grep -E 'RPATH|RUNPATH' || echo "(no runpath)"
#1760927351
patchelf --set-rpath /usr/local/lib:/opt/ros/humble/lib   ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so
#1760927364
LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH   ldd ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so | grep -i realsense
#1760927364
# EXPECT:
#1760927364
# librealsense2.so.2.56 => /usr/local/lib/librealsense2.so.2.56
#1760927420
sudo apt-get install -y patchelf
#1760927422
# See current runpath (likely /opt/ros/humble/lib)
#1760927422
readelf -d ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so | grep -E 'RPATH|RUNPATH' || echo "(no runpath)"
#1760927422
# Set runpath so /usr/local/lib is searched first
#1760927422
patchelf --set-rpath /usr/local/lib:/opt/ros/humble/lib   ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so
#1760927422
# (Also patch the node binary so it can find the same lib consistently)
#1760927422
patchelf --set-rpath /usr/local/lib:/opt/ros/humble/lib   ~/ros2_ws/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node
#1760927469
LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH   ldd ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so | grep -i librealsense
#1760927487
sudo ldconfig
#1760927487
# then re-run the ldd command above
#1760927508
ldd ~/ros2_ws/install/realsense2_camera/lib/librealsense2_camera.so | grep -i librealsense
#1760927562
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760927562
source ~/ros2_ws/install/setup.bash
#1760927562
pkill -f realsense-viewer || true
#1760927562
pkill -f realsense2_camera || true
#1760927570
RS2_LOG_LEVEL=DEBUG ros2 launch realsense2_camera rs_launch.py   enable_pose:=true publish_odom_tf:=true   wait_for_device_timeout:=30.0 initial_reset:=false
#1760927605
ls -l /usr/local/lib/librealsense2*
#1760927664
# get to the repo you built before (or clone fresh)
#1760927664
cd ~/librealsense 2>/dev/null || git clone https://github.com/IntelRealSense/librealsense.git ~/librealsense && cd ~/librealsense
#1760927664
git fetch --tags
#1760927665
git checkout v2.56.4
#1760927666
# build + install
#1760927666
rm -rf build && mkdir build && cd build
#1760927666
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF
#1760927671
make -j"$(nproc)"
#1760927930
sudo make install
#1760927932
sudo ldconfig
#1760927932
# sanity: you should now see .2.56.* files and symlinks
#1760927932
ls -l /usr/local/lib/librealsense2*
#1760928120
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928120
/usr/local/bin/rs-enumerate-devices
#1760928136
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928143
/usr/local/bin/rs-enumerate-devices
#1760928220
# Clean ONLY the wrapper so CMake re-detects librealsense
#1760928220
rm -rf ~/ros2_ws/build/realsense2_camera ~/ros2_ws/install/realsense2_camera ~/ros2_ws/log
#1760928220
# Prefer /usr/local during configure & link
#1760928220
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH}
#1760928220
export CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH}
#1760928220
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
#1760928220
cd ~/ros2_ws
#1760928220
colcon build --packages-select realsense2_camera --symlink-install   --cmake-args -DCMAKE_BUILD_TYPE=Release                -Drealsense2_DIR=/usr/local/lib/cmake/realsense2                -DCMAKE_INSTALL_RPATH=/usr/local/lib                -DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE                -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=TRUE
#1760928290
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928296
dmesg -w
#1760928334
lsusb -d 8087:0b37
#1760928417
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928417
/usr/local/bin/rs-enumerate-devices | grep -i "Physical Port"
#1760928424
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928429
/usr/local/bin/rs-enumerate-devices | grep -i "Physical Port"
#1760928483
# find the sysfs node for 8087:0b37 and print its name (e.g. 8-1 or 8-1-3)
#1760928483
for d in /sys/bus/usb/devices/*; do
  if [ -f "$d/idVendor" ] && [ -f "$d/idProduct" ] &&
     [ "$(cat "$d/idVendor")" = "8087" ] && [ "$(cat "$d/idProduct")" = "0b37" ]; then
    basename "$d"
  fi
done
#1760928498
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928498
source ~/ros2_ws/install/setup.bash
#1760928498
pkill -f realsense-viewer || true
#1760928498
pkill -f realsense2_camera || true
#1760928498
RS2_LOG_LEVEL=DEBUG ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   usb_port_id:=PORT initial_reset:=true
#1760928518
# find the sysfs node for 8087:0b37 and print its name (e.g. 8-1 or 8-1-3)
#1760928518
for d in /sys/bus/usb/devices/*; do
  if [ -f "$d/idVendor" ] && [ -f "$d/idProduct" ] &&
     [ "$(cat "$d/idVendor")" = "8087" ] && [ "$(cat "$d/idProduct")" = "0b37" ]; then
    basename "$d"
  fi
done
#1760928593
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928593
source ~/ros2_ws/install/setup.bash
#1760928593
pkill -f realsense-viewer || true
#1760928593
pkill -f realsense2_camera || true
#1760928609
RS2_LOG_LEVEL=DEBUG ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   usb_port_id:=8-1 initial_reset:=false
#1760928619
RS2_LOG_LEVEL=DEBUG ros2 launch realsense2_camera rs_launch.py   device_type:=t265 enable_pose:=true publish_odom_tf:=true   serial_no:="'908412110035'" initial_reset:=false
#1760928653
sudo -E
#1760928757
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928757
source ~/ros2_ws/install/setup.bash
#1760928757
LD_DEBUG=libs ros2 run realsense2_camera realsense2_camera_node 2>&1 | grep -m1 librealsense2
#1760928775
sudo -E env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH RS2_LOG_LEVEL=DEBUG   ros2 run realsense2_camera realsense2_camera_node --ros-args   -p device_type:=t265 -p enable_pose:=true -p publish_odom_tf:=true
#1760928860
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928860
source ~/ros2_ws/install/setup.bash
#1760928867
pkill -f realsense-viewer || true
#1760928867
pkill -f realsense2_camera || true
#1760928872
env LD_PRELOAD=/usr/local/lib/librealsense2.so.2.56.4 RS2_LOG_LEVEL=DEBUG   ros2 run realsense2_camera realsense2_camera_node --ros-args   -p device_type:=t265 -p enable_pose:=true -p publish_odom_tf:=true   -p usb_port_id:=8-1 -p initial_reset:=false
#1760928909
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760928909
source ~/ros2_ws/install/setup.bash
#1760928910
LD_DEBUG=libs,files RS2_LOG_LEVEL=DEBUG   ros2 run realsense2_camera realsense2_camera_node 2>&1 | tee /tmp/rs.log
#1760928923
# After it prints, search:
#1760928923
grep -E "librealsense2\.so" /tmp/rs.log | head -20
#1760929017
sudo -E /usr/local/bin/rs-enumerate-devices
#1760929074
sudo dmesg -C
#1760929079
dmesg -w
#1760929102
ls -l /dev/hidraw*
#1760929180
sudo dmesg -C
#1760929180
# NOW physically unplug the T265, wait 2s, plug it back in
#1760929180
dmesg -w
#1760929376
sudo dmesg -C
#1760929376
sudo dmesg -w
#1760929665
lsusb | egrep -i '03e7:2150|movidius|t265'
#1760929665
# should show: 03e7:2150 Movidius MA2X5X
#1760929715
lsusb | egrep -i '8087:0b37|03e7:2150|movidius|realsense|t265|intel' || echo "no match"
#1760929715
lsusb -t
#1760929727
sudo -E rs-fw-update -l
#1760929935
dmesg -w   # watch for reconnect messages (Ctrl+C to stop)
#1760930043
lsusb | egrep -i '8087:0b37|03e7:2150'
#1760930046
# plug it in fresh, then:
#1760930046
lsusb | egrep -i '03e7:2150|8087:0b37|movidius|realsense|t265' || echo "no match"
#1760930059
sudo find /usr/local/share /usr/share /lib/firmware -maxdepth 5 -type f -iname '*t265*bin' 2>/dev/null
#1760930121
# replace the path with what you found:
#1760930121
sudo -E rs-fw-update -f /path/to/t265_firmware.bin
#1760930121
# then unplug and replug the T265
#1760930232
sudo apt update
#1760930239
sudo apt install -y librealsense2-utils
#1760930241
realsense-viewer
#1760930391
sudo rm -f /etc/apt/sources.list.d/realsense*.list
#1760930391
sudo rm -f /etc/apt/sources.list.d/lala.temp
#1760930397
# key
#1760930397
curl -fsSL https://librealsense.intel.com/keys/Intel_RealSense.gpg  | sudo gpg --dearmor -o /usr/share/keyrings/librealsense-archive-keyring.gpg
#1760930397
# repo
#1760930397
echo "deb [signed-by=/usr/share/keyrings/librealsense-archive-keyring.gpg] \
https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
#1760930403
sudo apt update
#1760930408
sudo apt install -y librealsense2-utils
#1760930575
# ensure gpg tools are present
#1760930575
sudo apt-get update
#1760930583
sudo apt-get install -y gnupg dirmngr ca-certificates
#1760930585
# add the Intel RealSense repo (you already created the .list; keep it)
#1760930585
echo "deb [signed-by=/usr/share/keyrings/librealsense-archive-keyring.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
#1760930585
# fetch the missing key via a keyserver (the Intel URL 403'd)
#1760930585
sudo rm -f /usr/share/keyrings/librealsense-archive-keyring.gpg
#1760930585
gpg --keyserver hkps://keyserver.ubuntu.com --recv-keys C8B3A55A6F3EFCDE
#1760930596
gpg --export C8B3A55A6F3EFCDE | sudo tee /usr/share/keyrings/librealsense-archive-keyring.gpg >/dev/null
#1760930606
sudo apt-get update
#1760930618
sudo apt-get install -y librealsense2-utils librealsense2-udev-rules
#1760930639
sudo udevadm control --reload-rules
#1760930639
sudo udevadm trigger
#1760930655
sudo apt-get update
#1760930660
sudo apt-get install -y librealsense2-utils librealsense2-udev-rules
#1760930662
sudo udevadm control --reload-rules
#1760930662
sudo udevadm trigger
#1760930687
lsusb | egrep -i '03e7:2150|movidius|t265' || echo "no match"
#1760930687
# should show: 03e7:2150 Intel Myriad VPU [Movidius MA2X5X]
#1760930705
lsusb | egrep -i '03e7:2150|movidius|t265' || echo "no match"
#1760930705
# should show: 03e7:2150 Intel Myriad VPU [Movidius MA2X5X]
#1760930711
dpkg -L librealsense2-utils | grep -i 't265.*\.bin' || sudo find /usr /lib/firmware -type f -iname '*t265*fw*.bin' 2>/dev/null
#1760930738
sudo rs-fw-update -l
#1760930819
command -v rs-fw-update
#1760930819
# if it prints /usr/local/bin/rs-fw-update, rename it so it won't shadow the packaged one
#1760930819
sudo mkdir -p /usr/local/bin/.bak
#1760930819
[ -x /usr/local/bin/rs-fw-update ] && sudo mv /usr/local/bin/rs-fw-update /usr/local/bin/.bak/
#1760930819
[ -x /usr/local/bin/rs-enumerate-devices ] && sudo mv /usr/local/bin/rs-enumerate-devices /usr/local/bin/.bak/
#1760930819
hash -r
#1760930819
hash -r
#1760930841
sudo apt-get install -y gnupg ca-certificates curl
#1760930843
curl -fsSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC8B3A55A6F3EFCDE' | gpg --dearmor | sudo tee /usr/share/keyrings/librealsense-archive-keyring.gpg >/dev/null
#1760930843
echo "deb [signed-by=/usr/share/keyrings/librealsense-archive-keyring.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
#1760930843
sudo apt-get update
#1760930852
sudo apt-get install -y librealsense2-utils librealsense2-udev-rules
#1760930854
sudo udevadm control --reload-rules
#1760930854
sudo udevadm trigger
#1760930866
lsusb | egrep -i '03e7:2150|movidius|t265' || echo "no match"
#1760930876
dpkg -L librealsense2-utils | grep -iE 't265|tracking|\.bin$' || sudo find /usr /lib/firmware -type f -iregex '.*\(t265\|tracking\).*\.\(bin\|fw\)$' 2>/dev/null
#1760930889
sudo /usr/bin/rs-fw-update -l
#1760930896
# list devices as seen by the APT-provided tool (should show a "Recovery" device)
#1760930896
sudo /usr/bin/rs-fw-update -l
#1760930896
# update (replace <FW.bin> with the exact path you found)
#1760930896
sudo /usr/bin/rs-fw-update -f "<FW.bin>"
#1760931055
lsusb | egrep -i '03e7:2150|8087:0b37|movidius|t265' || echo "no match"
#1760931089
# If this yields nothing, your updater was built without TM2/T265
#1760931089
strings /usr/bin/rs-fw-update 2>/dev/null | grep -iE 'tm2|t265' || echo "no TM2 strings in this binary"
#1760931105
# See what versions are available to you
#1760931105
apt-cache madison librealsense2-utils | tac
#1760931105
# Pick a version from ~2.4x–2.50 range (these typically include TM2).
#1760931105
# Example only — replace the =VERSION parts with one that madison listed for arm64 on your mirror:
#1760931105
sudo apt-get install -y   librealsense2-utils=<VERSION>   librealsense2=<VERSION>   librealsense2-udev-rules=<VERSION>
#1760931105
# Prevent apt from “helpfully” upgrading you back to the TM2-less build
#1760931105
sudo apt-mark hold librealsense2 librealsense2-utils librealsense2-udev-rules
#1760931113
strings /usr/bin/rs-fw-update | grep -iE 'tm2|t265'
#1760931131
find ~/ /usr /lib/firmware -type f -iregex '.*\(tm2\|t265\|tracking\).*\.\(bin\|fw\)$' 2>/dev/null
#1760931189
sudo apt-mark unhold librealsense2 librealsense2-utils librealsense2-udev-rules
#1760931191
sudo apt-get install -y   librealsense2-utils=2.55.1-0~realsense.3292   librealsense2=2.55.1-0~realsense.3292   librealsense2-udev-rules=2.55.1-0~realsense.3292
#1760931193
# verify TM2 strings exist in the new binary
#1760931193
strings /usr/bin/rs-fw-update | grep -iE 'tm2|t265' || echo "still no TM2"
#1760931234
sudo apt-get install -y   librealsense2-utils=2.55.1-0~realsense.3337   librealsense2=2.55.1-0~realsense.3337   librealsense2-udev-rules=2.55.1-0~realsense.3337
#1760931235
strings /usr/bin/rs-fw-update | grep -iE 'tm2|t265'
#1760931262
apt-cache madison librealsense2 librealsense2-utils librealsense2-gl librealsense2-udev-rules | tac
#1760931278
# try the 3337 set:
#1760931278
sudo apt-get install -y   librealsense2-utils=2.55.1-0~realsense.3337   librealsense2=2.55.1-0~realsense.3337   librealsense2-gl=2.55.1-0~realsense.3337   librealsense2-udev-rules=2.55.1-0~realsense.3337 || # fall back to the 3292 set if 3337 isn't available for one of them:
sudo apt-get install -y   librealsense2-utils=2.55.1-0~realsense.3292   librealsense2=2.55.1-0~realsense.3292   librealsense2-gl=2.55.1-0~realsense.3292   librealsense2-udev-rules=2.55.1-0~realsense.3292
#1760931291
sudo apt-mark hold librealsense2 librealsense2-utils librealsense2-gl librealsense2-udev-rules
#1760931304
strings /usr/bin/rs-fw-update | grep -iE 'tm2|t265' || echo "still no TM2"
#1760931304
which -a rs-fw-update   # make sure /usr/bin is the one you use
#1760931314
sudo /usr/bin/rs-fw-update -l
#1760931314
# if it lists a Recovery device, flash the TM2 firmware (replace path)
#1760931314
sudo /usr/bin/rs-fw-update -f /path/to/TM2_FW.bin
#1760931314
# then unplug/replug and confirm it switches to healthy VID:PID (e.g. 8087:0b37)
#1760931314
lsusb | egrep -i '8087:0b37|t265|realsense|intel'
#1760931436
# 0) Prep: make sure ROS’s wrapper doesn’t mask your new binary later
#1760931436
sudo mkdir -p /usr/local/bin/.bak
#1760931436
[ -x /opt/ros/humble/bin/rs-fw-update ] && sudo mv /opt/ros/humble/bin/rs-fw-update /usr/local/bin/.bak/rs-fw-update.ros
#1760931436
hash -r
#1760931436
# 1) Build deps
#1760931436
sudo apt-get update
#1760931445
sudo apt-get install -y git cmake build-essential pkg-config     libusb-1.0-0-dev libudev-dev
#1760931447
# 2) Fetch librealsense and checkout a TM2-capable tag
#1760931447
git clone https://github.com/IntelRealSense/librealsense.git
#1760931489
cd librealsense
#1760931489
# v2.50.0 (or v2.48.0) is a sweet spot for TM2
#1760931489
git checkout v2.50.0
#1760931490
# 3) Build tools (no viewer/examples), force rsusb backend, and ENABLE TM2
#1760931490
mkdir -p build && cd build
#1760931490
cmake ..   -DBUILD_WITH_TM2=ON   -DBUILD_TOOLS=ON   -DBUILD_EXAMPLES=OFF   -DBUILD_GRAPHICAL_EXAMPLES=OFF   -DFORCE_RSUSB_BACKEND=ON
#1760931498
make -j"$(nproc)"
#1760931730
sudo make install   # installs to /usr/local/bin
#1760931731
# 4) Install (or refresh) udev rules so root-less access works later
#1760931731
cd ..
#1760931731
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
#1760931731
sudo udevadm control --reload-rules && sudo udevadm trigger
#1760931732
# 5) Sanity check the new updater really has TM2 strings
#1760931783
strings /usr/local/bin/rs-fw-update | grep -Ei 'tm2|t265' || echo "# 5) Sanity check the new updater really has TM2 strings still not TM2-enabled"
#1760931789
# With the camera showing 03e7:2150 (recovery) in lsusb:
#1760931789
sudo /usr/local/bin/rs-fw-update -l     # should list a Recovery device now
#1760931792
# Flash:
#1760931792
sudo /usr/local/bin/rs-fw-update -f /path/to/TM2_FW.bin
#1760931792
# Unplug → replug the camera
#1760931792
lsusb | egrep -i '8087:0b37|t265|realsense|intel'
#1760931792
# Optional: list devices (should now show T265 instead of Movidius recovery)
#1760931792
sudo /usr/local/bin/rs-enumerate-devices
#1760931873
# make /usr/local/bin come before ROS’s /opt/ros/... in PATH
#1760931873
echo 'export PATH=/usr/local/bin:$PATH' >> ~/.bashrc
#1760931873
hash -r
#1760931873
which -a rs-fw-update rs-enumerate-devices
#1760931886
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/librealsense-local.conf
#1760931886
sudo ldconfig
#1760931887
ldd /usr/local/bin/rs-enumerate-devices | grep realsense
#1760931924
sudo apt-mark hold librealsense2 librealsense2-utils librealsense2-gl librealsense2-udev-rules
#1760931945
/usr/local/bin/realsense-viewer
#1760932023
export PATH=/usr/local/bin:$PATH
#1760932023
hash -r
#1760932023
which -a rs-enumerate-devices
#1760932041
/usr/local/bin/rs-enumerate-devices 
#1760932051
/usr/local/bin/rs-pose  
#1760932170
sudo tee /etc/udev/rules.d/99-t265-usb.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8087", ATTR{idProduct}=="0b37", TEST=="power/control", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", ATTR{idProduct}=="2150", TEST=="power/control", ATTR{power/control}="on"
EOF

#1760932170
sudo udevadm control --reload-rules && sudo udevadm trigger
#1760932175
export PATH=/usr/local/bin:$PATH
#1760932175
hash -r
#1760932175
which -a rs-enumerate-devices rs-pose
#1760932195
lsusb | egrep -i '8087:0b37|t265'        # should show Intel RealSense T265
#1760932195
/usr/local/bin/rs-enumerate-devices      # should list the T265 + sensors
#1760932195
/usr/local/bin/rs-pose                   # watch pose roll in
#1760932200
dmesg -w                                 # keep an eye out for USB resets
#1760932264
lsusb | egrep -i '8087:0b37|t265'        # should show Intel RealSense T265
#1760932264
/usr/local/bin/rs-enumerate-devices      # should list the T265 + sensors
#1760932264
/usr/local/bin/rs-pose                   # watch pose roll in
#1760932270
dmesg -w                                 # keep an eye out for USB resets
#1760932290
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
#1760932308
source /opt/ros/humble/setup.bash
#1760932314
ros2 launch realsense2_camera rs_launch.py
#1760932372
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'
#1760932398
# 1) Workspace
#1760932398
source /opt/ros/humble/setup.bash
#1760932398
mkdir -p ~/rs2_ros_ws/src && cd ~/rs2_ros_ws/src
#1760932398
# 2) Get the ROS2 wrapper (ros2 branch)
#1760932398
git clone --depth 1 -b ros2 https://github.com/IntelRealSense/realsense-ros.git
#1760932398
# 3) Resolve deps
#1760932398
cd ..
#1760932398
rosdep update
#1760932410
rosdep install --from-paths src -i -r -y
#1760932411
# 4) Build against your /usr/local librealsense
#1760932411
colcon build --symlink-install
#1760932412
source install/setup.bash
#1760932420
# ~/t265.yaml
#1760932420
cat > ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: "linear_interpolation"   # common choice for IMU fusion
YAML

#1760932420
# Start node with params
#1760932420
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
#1760932459
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node | grep -i realsense
#1760932535
# Close realsense-viewer if it’s open
#1760932535
pkill -f realsense-viewer || true
#1760932535
# Force ROS to use your /usr/local LRS (2.50.x with TM2)
#1760932535
env LD_PRELOAD=/usr/local/lib/librealsense2.so.2.50     LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH     ros2 launch realsense2_camera rs_launch.py
#1760932566
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'
#1760932567
ros2 topic echo /camera/pose/sample
#1760932643
# 0) (optional) remove the apt binary to avoid confusion
#1760932643
sudo apt remove -y ros-humble-realsense2-camera
#1760932650
# 1) Make sure CMake can find your /usr/local LRS (you already did ldconfig, but verify):
#1760932650
test -f /usr/local/lib/cmake/realsense2/realsense2Config.cmake || echo "Missing realsense2 CMake config"
#1760932650
# 2) Workspace
#1760932650
source /opt/ros/humble/setup.bash
#1760932651
mkdir -p ~/t265_ws/src && cd ~/t265_ws/src
#1760932651
git clone --depth 1 -b ros2 https://github.com/IntelRealSense/realsense-ros.git
#1760932651
cd ..
#1760932651
# 3) Install deps
#1760932651
rosdep update
#1760932662
rosdep install --from-paths src -i -r -y
#1760932663
# 4) Build, forcing CMake to use /usr/local librealsense
#1760932663
colcon build --merge-install   --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 -DCMAKE_BUILD_TYPE=Release
#1760932664
# 5) Use the workspace
#1760932664
source ~/t265_ws/install/setup.bash
#1760932673
# Your params file is fine
#1760932673
ros2 run realsense2_camera realsense2_camera_node   --ros-args --params-file ~/t265.yaml
#1760932852
sudo apt remove -y ros-humble-realsense2-camera ros-humble-librealsense2
#1760932888
source /opt/ros/humble/setup.bash
#1760932888
mkdir -p ~/t265_ws/src && cd ~/t265_ws/src
#1760932888
git clone https://github.com/IntelRealSense/realsense-ros.git
#1760932896
cd realsense-ros
#1760932896
git fetch --tags
#1760932897
git checkout -b v4.50.0 tags/4.50.0     # if this tag isn’t present, try 4.51.1
#1760932897
cd ~/t265_ws
#1760932897
rosdep update
#1760932908
rosdep install --from-paths src -i -r -y
#1760932921
colcon build --merge-install   --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 -DCMAKE_BUILD_TYPE=Release
#1760933010
source ~/t265_ws/install/setup.bash
#1760933031
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node | grep -i librealsense
#1760933031
# Expect: ... => /usr/local/lib/librealsense2.so.2.50 (0x...)
#1760933107
cd ~/t265_ws/src
#1760933107
rm -rf realsense-ros
#1760933107
git clone https://github.com/IntelRealSense/realsense-ros.git
#1760933116
cd realsense-ros
#1760933116
git fetch --tags
#1760933116
git checkout 4.50.0     # good match for LRS 2.50.x; if missing, try: git checkout 4.51.1
#1760933234
# in the repo you just cloned
#1760933234
cd ~/t265_ws/src/realsense-ros
#1760933234
git fetch --tags
#1760933234
# pick the newest 4.5x tag that exists (tries 4.50.*, 4.51.*, then 4.49.*)
#1760933234
TAG=$(git tag -l '4.50.*' '4.51.*' '4.49.*' --sort=-v:refname | head -n1)
#1760933234
echo "Checking out tag: $TAG"
#1760933234
git checkout "$TAG"
#1760933234
# build against your /usr/local librealsense 2.50.x
#1760933234
cd ~/t265_ws
#1760933234
rosdep install --from-paths src -i -r -y
#1760933235
colcon build --merge-install   --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 -DCMAKE_BUILD_TYPE=Release
#1760933296
# use the workspace + ensure /usr/local is first at runtime
#1760933296
source ~/t265_ws/install/setup.bash
#1760933298
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760933298
# sanity: the node should link to /usr/local/lib/librealsense2.so.2.50...
#1760933298
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node | grep -i 'librealsense2\.so'
#1760933298
# launch (close the viewer first so it doesn’t grab the device)
#1760933298
pkill -f realsense-viewer || true
#1760933298
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
#1760933373
# in another terminal:
#1760933373
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'
#1760933374
ros2 topic echo /camera/pose/sample
#1760933464
[INFO] [camera]: Built with LibRealSense v2.56.4
#1760933465
[INFO] [camera]: Running with LibRealSense v2.56.4
#1760933473
# 0) Stop anything using the camera
#1760933473
pkill -f realsense-viewer || true
#1760933473
pkill -f realsense2_camera_node || true
#1760933478
# Keep udev rules, but remove 2.56 runtime/dev libs so CMake can't find them
#1760933478
sudo apt purge -y ros-humble-librealsense2 librealsense2 librealsense2-gl librealsense2-utils || true
#1760933481
# Verify only /usr/local shows up
#1760933481
ldconfig -p | grep -i librealsense || true
#1760933571
# You had these on hold; unhold first:
#1760933571
sudo apt-mark unhold librealsense2 librealsense2-gl librealsense2-utils librealsense2-udev-rules || true
#1760933571
# Purge the libraries (keep udev rules if you like; it’s just rules, no libs)
#1760933571
sudo apt purge -y --allow-change-held-packages librealsense2 librealsense2-gl librealsense2-utils ros-humble-librealsense2
#1760933589
ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null
#1760933589
# Remove any 2.56 files that snuck into /usr/local
#1760933589
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56
#1760933589
# Make sure the unversioned .so symlinks point at 2.50.x
#1760933589
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
#1760933589
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so
#1760933589
# Refresh the linker cache and verify ONLY 2.50 remains
#1760933589
sudo ldconfig
#1760933589
ldconfig -p | grep -i librealsense
#1760933589
# Expect: only /usr/local entries for 2.50
#1760933618
source ~/t265_ws/install/setup.bash
#1760933619
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760933619
# Sanity: the node must link to /usr/local … 2.50
#1760933619
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760933619
# Launch
#1760933619
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
#1760933630
ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null
#1760933630
# Remove any 2.56 files that snuck into /usr/local
#1760933630
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56
#1760933630
# Make sure the unversioned .so symlinks point at 2.50.x
#1760933630
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
#1760933630
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so
#1760933630
# Refresh the linker cache and verify ONLY 2.50 remains
#1760933630
sudo ldconfig
#1760933630
ldconfig -p | grep -i librealsense
#1760933630
# Expect: only /usr/local entries for 2.50
#1760933638
# Stop anything that might have the device open
#1760933638
pkill -f realsense-viewer || true
#1760933638
pkill -f realsense2_camera_node || true
#1760933638
# Clean & rebuild your workspace (you already checked out tag 4.51.1)
#1760933638
cd ~/t265_ws
#1760933638
rm -rf build/ install/ log/
#1760933639
colcon build --merge-install   --cmake-args     -DCMAKE_PREFIX_PATH=/usr/local     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2     -DCMAKE_BUILD_TYPE=Release
#1760933668
source ~/t265_ws/install/setup.bash
#1760933669
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760933669
# Sanity: the node must link to /usr/local … 2.50
#1760933669
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760933669
# Launch
#1760933669
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
#1760933234
git checkout "$TAG"
#1760933234
# build against your /usr/local librealsense 2.50.x
#1760933234
rosdep install --from-paths src -i -r -y
#1760933235
colcon build --merge-install   --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 -DCMAKE_BUILD_TYPE=Release
#1760933296
# use the workspace + ensure /usr/local is first at runtime
#1760933298
# sanity: the node should link to /usr/local/lib/librealsense2.so.2.50...
#1760933298
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node | grep -i 'librealsense2\.so'
#1760933298
# launch (close the viewer first so it doesn’t grab the device)
#1760933373
# in another terminal:
#1760933373
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'
#1760933374
ros2 topic echo /camera/pose/sample
#1760933464
[INFO] [camera]: Built with LibRealSense v2.56.4
#1760933465
[INFO] [camera]: Running with LibRealSense v2.56.4
#1760933473
# 0) Stop anything using the camera
#1760933478
# Keep udev rules, but remove 2.56 runtime/dev libs so CMake can't find them
#1760933478
sudo apt purge -y ros-humble-librealsense2 librealsense2 librealsense2-gl librealsense2-utils || true
#1760933481
# Verify only /usr/local shows up
#1760933481
ldconfig -p | grep -i librealsense || true
#1760933571
# You had these on hold; unhold first:
#1760933619
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760933630
ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null
#1760933638
# Stop anything that might have the device open
#1760933639
colcon build --merge-install   --cmake-args     -DCMAKE_PREFIX_PATH=/usr/local     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2     -DCMAKE_BUILD_TYPE=Release
#1760933668
source ~/t265_ws/install/setup.bash
#1760933669
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760933742
12:11:21 alpha@opi t265_ws → # You had these on hold; unhold first:
#1760933742
sudo apt-mark unhold librealsense2 librealsense2-gl librealsense2-utils librealsense2-udev-rules || true
#1760933742
# Purge the libraries (keep udev rules if you like; it’s just rules, no libs)
#1760933742
sudo apt purge -y --allow-change-held-packages librealsense2 librealsense2-gl librealsense2-utils ros-humble-librealsense2
#1760933745
Canceled hold on librealsense2.
#1760933745
Canceled hold on librealsense2-gl.
#1760933745
Canceled hold on librealsense2-utils.
#1760933745
Canceled hold on librealsense2-udev-rules.
#1760933746
Reading package lists... Done
#1760933746
Building dependency tree... Done
#1760933746
Reading state information... Done
#1760933746
The following packages were automatically installed and are no longer required:
#1760933747
Use 'sudo apt autoremove' to remove them.
#1760933748
The following packages will be REMOVED:
#1760933748
0 upgraded, 0 newly installed, 4 to remove and 0 not upgraded.
#1760933748
After this operation, 71.5 MB disk space will be freed.
#1760933749
(Reading database ... 260278 files and directories currently installed.)
#1760933749
Removing librealsense2-utils:arm64 (2.56.5-0~realsense.6682) ...
#1760933749
Removing librealsense2-gl:arm64 (2.56.5-0~realsense.6682) ...
#1760933749
Removing librealsense2:arm64 (2.56.5-0~realsense.6682) ...
#1760933749
Removing ros-humble-librealsense2 (2.56.4-1jammy.20250722.184445) ...
#1760933749
Processing triggers for libc-bin (1:2.35-0ubuntu3.4+2widevine) ...
#1760933749
12:13:04 alpha@opi t265_ws → ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null
#1760933749
lrwxrwxrwx 1 root root        24 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
#1760933750
lrwxrwxrwx 1 root root        21 Oct 19 23:42 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
#1760933751
lrwxrwxrwx 1 root root        23 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
#1760933751
12:13:09 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
#1760933752
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760933752
[INFO] [1760933620.114486768] [camera]: RealSense ROS v4.51.1
#1760933752
[INFO] [1760933620.114680432] [camera]: Built with LibRealSense v2.56.4
#1760933753
[INFO] [1760933620.114701431] [camera]: Running with LibRealSense v2.56.4
#1760933753
[WARN] [1760933620.129893562] [camera]: No RealSense devices were found!
#1760933753
12:13:47 alpha@opi t265_ws → ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null
#1760933753
# Remove any 2.56 files that snuck into /usr/local
#1760933753
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56
#1760933753
# Make sure the unversioned .so symlinks point at 2.50.x
#1760933753
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
#1760933753
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so
#1760933753
# Refresh the linker cache and verify ONLY 2.50 remains
#1760933753
sudo ldconfig
#1760933754
ldconfig -p | grep -i librealsense
#1760933754
# Expect: only /usr/local entries for 2.50
#1760933754
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
#1760933754
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
#1760933754
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
#1760933754
lrwxrwxrwx 1 root root        21 Oct 20 00:13 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
#1760933754
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
#1760933754
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
#1760933754
lrwxrwxrwx 1 root root        23 Oct 20 00:13 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
#1760933754
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
#1760933754
12:13:50 alpha@opi t265_ws → # Stop anything that might have the device open
#1760933755
pkill -f realsense-viewer || true
#1760933755
pkill -f realsense2_camera_node || true
#1760933755
# Clean & rebuild your workspace (you already checked out tag 4.51.1)
#1760933755
cd ~/t265_ws
#1760933755
rm -rf build/ install/ log/
#1760933755
colcon build --merge-install   --cmake-args     -DCMAKE_PREFIX_PATH=/usr/local     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2     -DCMAKE_BUILD_TYPE=Release
#1760933770
[0.613s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [2.16s]
--- stderr: realsense2_camera                          
CMake Warning at CMakeLists.txt:104 (find_package):
  Could not find a configuration file for package "realsense2" that is
  compatible with requested version "2.51.1".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/realsense2/realsense2Config.cmake, version: 2.50.0



CMake Error at CMakeLists.txt:106 (message):
  



   Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases

  



---
Failed   <<< realsense2_camera [2.94s, exited with code 1]

Summary: 2 packages finished [15.5s]
  1 package failed: realsense2_camera
  3 packages had stderr output: realsense2_camera realsense2_camera_msgs realsense2_description
12:14:15 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Sanity: the node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# Launch
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
12:14:29 alpha@opi t265_ws → 12:11:21 alpha@opi t265_ws → # You had these on hold; unhold first:
sudo apt-mark unhold librealsense2 librealsense2-gl librealsense2-utils librealsense2-udev-rules || true

# Purge the libraries (keep udev rules if you like; it’s just rules, no libs)
sudo apt purge -y --allow-change-held-packages librealsense2 librealsense2-gl librealsense2-utils ros-humble-librealsense2
Canceled hold on librealsense2.
Canceled hold on librealsense2-gl.
Canceled hold on librealsense2-utils.
Canceled hold on librealsense2-udev-rules.
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following packages were automatically installed and are no longer required:
  libatk-bridge2.0-dev libatk1.0-dev libatspi2.0-dev libcairo2-dev libdatrie-dev libepoxy-dev libfribidi-dev libgdk-pixbuf-2.0-dev libgraphite2-dev libgtk-3-dev libharfbuzz-dev
  libharfbuzz-gobject0 libmbim-glib4 libmbim-proxy libpango1.0-dev libpixman-1-dev libqmi-glib5 libqmi-proxy libthai-dev libxcomposite-dev libxdamage-dev libxtst-dev pango1.0-tools
  wayland-protocols
Use 'sudo apt autoremove' to remove them.
The following packages will be REMOVED:
  librealsense2* librealsense2-gl* librealsense2-utils* ros-humble-librealsense2*
0 upgraded, 0 newly installed, 4 to remove and 0 not upgraded.
After this operation, 71.5 MB disk space will be freed.
(Reading database ... 260278 files and directories currently installed.)
Removing librealsense2-utils:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2-gl:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2:arm64 (2.56.5-0~realsense.6682) ...
Removing ros-humble-librealsense2 (2.56.4-1jammy.20250722.184445) ...
Processing triggers for libc-bin (1:2.35-0ubuntu3.4+2widevine) ...
12:13:04 alpha@opi t265_ws → ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null

# Remove any 2.56 files that snuck into /usr/local
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56

# Make sure the unversioned .so symlinks point at 2.50.x
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so

# Refresh the linker cache and verify ONLY 2.50 remains
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect: only /usr/local entries for 2.50
lrwxrwxrwx 1 root root        24 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        24 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        21 Oct 19 23:42 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
lrwxrwxrwx 1 root root        23 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:13:09 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Sanity: the node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# Launch
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
[INFO] [1760933620.114486768] [camera]: RealSense ROS v4.51.1
[INFO] [1760933620.114680432] [camera]: Built with LibRealSense v2.56.4
[INFO] [1760933620.114701431] [camera]: Running with LibRealSense v2.56.4
[WARN] [1760933620.129893562] [camera]: No RealSense devices were found!
!!:s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)
12:13:47 alpha@opi t265_ws → ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null

# Remove any 2.56 files that snuck into /usr/local
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56

# Make sure the unversioned .so symlinks point at 2.50.x
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so

# Refresh the linker cache and verify ONLY 2.50 remains
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect: only /usr/local entries for 2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        21 Oct 20 00:13 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
lrwxrwxrwx 1 root root        23 Oct 20 00:13 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:13:50 alpha@opi t265_ws → # Stop anything that might have the device open
pkill -f realsense-viewer || true
pkill -f realsense2_camera_node || true

# Clean & rebuild your workspace (you already checked out tag 4.51.1)
cd ~/t265_ws
rm -rf build/ install/ log/
colcon build --merge-install \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    -DCMAKE_BUILD_TYPE=Release
[0.613s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
#1760933771
Starting >>> realsense2_camera_msgs
#1760933771
--- stderr: realsense2_camera_msgs                                
#1760933771
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
#1760933772
CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
#1760933773
CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
#1760933773
Call Stack (most recent call first):
#1760933773
This warning is for project developers.  Use -Wno-dev to suppress it.
#1760933774
Finished <<< realsense2_camera_msgs [12.1s]
#1760933775
Starting >>> realsense2_camera
#1760933775
Starting >>> realsense2_description
#1760933775
--- stderr: realsense2_description                                                                         
#1760933775
CMake Warning:
#1760933776
Finished <<< realsense2_description [2.16s]
#1760933776
--- stderr: realsense2_camera                          
#1760933776
CMake Warning at CMakeLists.txt:104 (find_package):
#1760933777
CMake Error at CMakeLists.txt:106 (message):
#1760933777
---
#1760933777
Failed   <<< realsense2_camera [2.94s, exited with code 1]
#1760933778
Summary: 2 packages finished [15.5s]
#1760933778
12:14:15 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
#1760933778
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760933778
# Sanity: the node must link to /usr/local … 2.50
#1760933778
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760933778
# Launch
#1760933778
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
#1760933779
Package not found
#1760933779
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
#1760933779
Package 'realsense2_camera' not found
#1760933779
12:14:29 alpha@opi t265_ws → 
#1760933783
sudo ldconfig
#1760933783
ldconfig -p | grep -i librealsense
#1760933783
# Expect ONLY 2.50 entries (plus the -gl 2.50 you already have)
#1760933818
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
#1760933824
sudo ldconfig
#1760933832
ldconfig -p | grep -i librealsense
#1760933874
cd ~/t265_ws/src/realsense-ros
#1760933874
git fetch --tags
#1760933874
# Find the newest 4.50.* or 4.49.* tag that asks for <= 2.50.0
#1760933874
CANDIDATE=$(
  for t in $(git tag -l '4.50.*' '4.49.*' --sort=-v:refname); do
    min=$(git show "$t:CMakeLists.txt" | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$min" ] || continue
    if dpkg --compare-versions "$min" le 2.50.0; then echo "$t"; break; fi
  done
)
#1760933874
echo "Checking out $CANDIDATE"
#1760933874
git checkout "$CANDIDATE"
#1760933931
cd ~/t265_ws/src/realsense-ros
#1760933931
git fetch --tags
#1760933932
# List nearby tags
#1760933932
git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname | head -n 15
#1760933932
# Auto-pick the newest tag that requires <= 2.50.0
#1760933932
CAND=$(
  for t in $(git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname); do
    need=$(git show "$t:realsense2_camera/CMakeLists.txt" 2>/dev/null \
            | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$need" ] || continue
    if dpkg --compare-versions "$need" le 2.50.0; then echo "$t"; break; fi
  done
)
#1760933932
echo "Checking out $CAND"; git checkout "$CAND"
#1760934004
cd ~/t265_ws/src/realsense-ros
#1760934004
git fetch --tags
#1760934004
git tag -l '4.5*' --sort=-v:refname | head -n 40
#1760934004
# Try these, in order; the first that exists will do:
#1760934004
for t in 4.50.1 4.50.0 4.49.1 4.49.0 4.48.1 4.48.0; do
  git checkout $t && break
done
#1760934016
cd ~/t265_ws/src/realsense-ros
#1760934016
grep -n "find_package(realsense2" realsense2_camera/CMakeLists.txt
#1760934016
# Replace whatever version it asks for (e.g. 2.51.1) with 2.50.0:
#1760934016
sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/'   realsense2_camera/CMakeLists.txt
#1760934103
sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/'   realsense2_camera/CMakeLists.txt
#1760934103
git --no-pager diff -- realsense2_camera/CMakeLists.txt | sed -n '1,120p'
#1760934179
cd ~/t265_ws
#1760934179
rm -rf build/ install/ log/
#1760934179
colcon build --merge-install   --cmake-args     -DCMAKE_PREFIX_PATH=/usr/local     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2     -DCMAKE_BUILD_TYPE=Release
#1760934259
# env
#1760934259
source ~/t265_ws/install/setup.bash
#1760934259
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760934259
# sanity: node must link to /usr/local … 2.50
#1760934259
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760934260
# nothing else should be holding the camera
#1760934260
pkill -f realsense-viewer || true
#1760934260
# launch with a T265 filter + your params
#1760934260
ros2 run realsense2_camera realsense2_camera_node   --ros-args -p device_type:=t265 --params-file ~/t265.yaml
#1760934807
Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]
#1760934820
# ~/t265.yaml
#1760934820
realsense2_camera:
#1760934886
cat > ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16
YAML

#1760934886
# sanity check
#1760934886
cat ~/t265.yaml
#1760934886
# launch
#1760934886
ros2 run realsense2_camera realsense2_camera_node   --ros-args --params-file ~/t265.yaml
#1760934964
cat >| ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16   # must be <= 32
YAML

#1760934974
# close anything that might hold the camera
#1760934974
pkill -f realsense-viewer || true
#1760934974
# launch (explicitly filter to T265 + set the queue)
#1760934974
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p tracking_module.frames_queue_size:=16   --params-file ~/t265.yaml
#1760934993
# see the topics
#1760934993
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'
#1760934994
# confirm pose is streaming
#1760934994
ros2 topic echo -n 1 /camera/pose/sample
#1760934995
ros2 topic hz /camera/pose/sample
#1760935073
pkill -f realsense-viewer || true
#1760935073
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p tracking_module.frames_queue_size:=16   --params-file ~/t265.yaml
#1760935161
# (realsense-viewer closed)
#1760935161
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p tracking_module.frames_queue_size:=16   --params-file ~/t265.yaml
#1760935241
pkill -f realsense-viewer || true
#1760935249
ros2 run realsense2_camera realsense2_camera_node
#1760935355
pkill -f realsense-viewer || true
#1760935089
# list the camera topics
#1760935089
ros2 topic list | grep '^/camera/'
#1760935090
# see the type and QoS of pose
#1760935090
ros2 topic info -v /camera/pose/sample
#1760935091
# print exactly one pose message
#1760935091
ros2 topic echo --once /camera/pose/sample
#1760935091
# check rates (run each for a few seconds)
#1760935091
ros2 topic hz /camera/pose/sample
#1760935408
ros2 topic hz /camera/gyro/sample
#1760935415
ros2 topic hz /camera/accel/sample
#1760935419
# same overlay and DDS domain as Terminal A
#1760935419
source ~/t265_ws/install/setup.bash
#1760935419
echo "RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID"
#1760935419
# see what’s being published
#1760935419
ros2 node list
#1760935420
ros2 topic list | sort | sed -n '1,200p'
#1760935421
# pose can be under either name depending on driver version
#1760935421
ros2 topic info -v /camera/pose/sample || true
#1760935422
ros2 topic info -v /camera/odom/sample || true
#1760935422
# print one message when present
#1760935422
ros2 topic echo --once /camera/pose/sample || ros2 topic echo --once /camera/odom/sample
#1760935424
# IMU (explicit QoS to match sensor_data publishers)
#1760935424
ros2 topic echo /camera/gyro/sample   --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935424
ros2 topic echo /camera/accel/sample   --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935425
# check rates
#1760935425
ros2 topic hz /camera/gyro/sample
#1760935442
ros2 topic hz /camera/accel/sample
#1760935445
ros2 param dump /camera | egrep 'enable_(pose|gyro|accel)|frames_queue_size|publish_tf'
#1760935471
source ~/t265_ws/install/setup.bash
#1760935471
echo "RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID"
#1760935562
source /opt/ros/humble/setup.bash
#1760935562
source ~/t265_ws/install/setup.bash
#1760935563
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760935563
export ROS_DOMAIN_ID=0
#1760935574
ros2 node list
#1760935574
ros2 node info /camera
#1760935652
sudo apt update
#1760935661
sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds
#1760935675
# then in every terminal:
#1760935675
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760935675
source /opt/ros/humble/setup.bash
#1760935675
source ~/t265_ws/install/setup.bash
#1760935683
ros2 node list
#1760935684
ros2 node info /camera
#1760935684
ros2 topic list | grep '^/camera/'
#1760935685
# Pose can be /camera/pose/sample or /camera/odom/sample depending on build
#1760935685
ros2 topic info -v /camera/pose/sample || true
#1760935686
ros2 topic info -v /camera/odom/sample || true
#1760935686
# Print one pose (try both names)
#1760935686
ros2 topic echo --once /camera/pose/sample || ros2 topic echo --once /camera/odom/sample
#1760935688
# IMU topics are usually best_effort; match QoS to see data:
#1760935688
ros2 topic echo /camera/gyro/sample    --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935688
ros2 topic echo /camera/accel/sample   --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935361
ros2 run realsense2_camera realsense2_camera_node
#1760935887
# list what exists (no /camera prefix)
#1760935887
ros2 topic list | egrep '^/(accel|gyro|fisheye|pose|odom|tf)'
#1760935888
# echo IMU with matching QoS
#1760935888
ros2 topic echo /gyro/sample    --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935889
ros2 topic echo /accel/sample   --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935899
# list what exists (no /camera prefix)
#1760935899
ros2 topic list | egrep '^/(accel|gyro|fisheye|pose|odom|tf)'
#1760935900
# echo IMU with matching QoS
#1760935900
ros2 topic echo /gyro/sample    --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935900
ros2 topic echo /accel/sample   --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935961
# list what exists (no /camera prefix)
#1760935961
ros2 topic list | egrep '^/(accel|gyro|fisheye|pose|odom|tf)'
#1760935962
# echo IMU with matching QoS
#1760935962
ros2 topic echo /gyro/sample    --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760936032
ros2 topic echo /accel/sample   --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935922
ros2 run realsense2_camera realsense2_camera_node
#1760937134
sudo apt install -y python3-tf-transformations python3-numpy
#1760937149
ros2 topic list | egrep '/camera/(odom|pose)/sample'
#1760937150
ros2 topic echo -n 1 /camera/odom/sample   --qos-reliability reliable --qos-durability volatile || ros2 topic echo -n 1 /camera/pose/sample   --qos-reliability reliable --qos-durability volatile
#1760937196
ros2 topic list | egrep '/camera/(odom|pose)/sample'
#1760937238
ros2 topic list | egrep '/(odom|pose)/sample$'
#1760933753
12:13:47 alpha@opi t265_ws → ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null
#1760933753
# Remove any 2.56 files that snuck into /usr/local
#1760933753
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56
#1760933755
colcon build --merge-install   --cmake-args     -DCMAKE_PREFIX_PATH=/usr/local     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2     -DCMAKE_BUILD_TYPE=Release
#1760933770
[0.613s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [2.16s]
--- stderr: realsense2_camera                          
CMake Warning at CMakeLists.txt:104 (find_package):
  Could not find a configuration file for package "realsense2" that is
  compatible with requested version "2.51.1".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/realsense2/realsense2Config.cmake, version: 2.50.0



CMake Error at CMakeLists.txt:106 (message):
  



   Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases

  



---
Failed   <<< realsense2_camera [2.94s, exited with code 1]

Summary: 2 packages finished [15.5s]
  1 package failed: realsense2_camera
  3 packages had stderr output: realsense2_camera realsense2_camera_msgs realsense2_description
12:14:15 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Sanity: the node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# Launch
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
12:14:29 alpha@opi t265_ws → 12:11:21 alpha@opi t265_ws → # You had these on hold; unhold first:
sudo apt-mark unhold librealsense2 librealsense2-gl librealsense2-utils librealsense2-udev-rules || true

# Purge the libraries (keep udev rules if you like; it’s just rules, no libs)
sudo apt purge -y --allow-change-held-packages librealsense2 librealsense2-gl librealsense2-utils ros-humble-librealsense2
Canceled hold on librealsense2.
Canceled hold on librealsense2-gl.
Canceled hold on librealsense2-utils.
Canceled hold on librealsense2-udev-rules.
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following packages were automatically installed and are no longer required:
  libatk-bridge2.0-dev libatk1.0-dev libatspi2.0-dev libcairo2-dev libdatrie-dev libepoxy-dev libfribidi-dev libgdk-pixbuf-2.0-dev libgraphite2-dev libgtk-3-dev libharfbuzz-dev
  libharfbuzz-gobject0 libmbim-glib4 libmbim-proxy libpango1.0-dev libpixman-1-dev libqmi-glib5 libqmi-proxy libthai-dev libxcomposite-dev libxdamage-dev libxtst-dev pango1.0-tools
  wayland-protocols
Use 'sudo apt autoremove' to remove them.
The following packages will be REMOVED:
  librealsense2* librealsense2-gl* librealsense2-utils* ros-humble-librealsense2*
0 upgraded, 0 newly installed, 4 to remove and 0 not upgraded.
After this operation, 71.5 MB disk space will be freed.
(Reading database ... 260278 files and directories currently installed.)
Removing librealsense2-utils:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2-gl:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2:arm64 (2.56.5-0~realsense.6682) ...
Removing ros-humble-librealsense2 (2.56.4-1jammy.20250722.184445) ...
Processing triggers for libc-bin (1:2.35-0ubuntu3.4+2widevine) ...
12:13:04 alpha@opi t265_ws → ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null

# Remove any 2.56 files that snuck into /usr/local
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56

# Make sure the unversioned .so symlinks point at 2.50.x
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so

# Refresh the linker cache and verify ONLY 2.50 remains
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect: only /usr/local entries for 2.50
lrwxrwxrwx 1 root root        24 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        24 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        21 Oct 19 23:42 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
lrwxrwxrwx 1 root root        23 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:13:09 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Sanity: the node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# Launch
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
[INFO] [1760933620.114486768] [camera]: RealSense ROS v4.51.1
[INFO] [1760933620.114680432] [camera]: Built with LibRealSense v2.56.4
[INFO] [1760933620.114701431] [camera]: Running with LibRealSense v2.56.4
[WARN] [1760933620.129893562] [camera]: No RealSense devices were found!
!!:s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)
12:13:47 alpha@opi t265_ws → ls -l /usr/local/lib/librealsense2* /usr/local/lib/librealsense2-gl* 2>/dev/null

# Remove any 2.56 files that snuck into /usr/local
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2-gl.so.2.56

# Make sure the unversioned .so symlinks point at 2.50.x
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so

# Refresh the linker cache and verify ONLY 2.50 remains
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect: only /usr/local entries for 2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        21 Oct 20 00:13 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
lrwxrwxrwx 1 root root        23 Oct 20 00:13 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:13:50 alpha@opi t265_ws → # Stop anything that might have the device open
pkill -f realsense-viewer || true
pkill -f realsense2_camera_node || true

# Clean & rebuild your workspace (you already checked out tag 4.51.1)
cd ~/t265_ws
rm -rf build/ install/ log/
colcon build --merge-install \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    -DCMAKE_BUILD_TYPE=Release
[0.613s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
#1760933774
Finished <<< realsense2_camera_msgs [12.1s]
#1760933776
Finished <<< realsense2_description [2.16s]
#1760933777
Failed   <<< realsense2_camera [2.94s, exited with code 1]
#1760933778
Summary: 2 packages finished [15.5s]
#1760933778
12:14:15 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
#1760933778
# Sanity: the node must link to /usr/local … 2.50
#1760933778
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760933778
# Launch
#1760933778
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
#1760933779
12:14:29 alpha@opi t265_ws → 
#1760933783
# Expect ONLY 2.50 entries (plus the -gl 2.50 you already have)
#1760933818
sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
#1760933874
# Find the newest 4.50.* or 4.49.* tag that asks for <= 2.50.0
#1760933874
CANDIDATE=$(
  for t in $(git tag -l '4.50.*' '4.49.*' --sort=-v:refname); do
    min=$(git show "$t:CMakeLists.txt" | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$min" ] || continue
    if dpkg --compare-versions "$min" le 2.50.0; then echo "$t"; break; fi
  done
)
#1760933874
echo "Checking out $CANDIDATE"
#1760933874
git checkout "$CANDIDATE"
#1760933932
# List nearby tags
#1760933932
git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname | head -n 15
#1760933932
# Auto-pick the newest tag that requires <= 2.50.0
#1760933932
CAND=$(
  for t in $(git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname); do
    need=$(git show "$t:realsense2_camera/CMakeLists.txt" 2>/dev/null \
            | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$need" ] || continue
    if dpkg --compare-versions "$need" le 2.50.0; then echo "$t"; break; fi
  done
)
#1760933932
echo "Checking out $CAND"; git checkout "$CAND"
#1760934004
git fetch --tags
#1760934004
git tag -l '4.5*' --sort=-v:refname | head -n 40
#1760934004
# Try these, in order; the first that exists will do:
#1760934004
for t in 4.50.1 4.50.0 4.49.1 4.49.0 4.48.1 4.48.0; do
  git checkout $t && break
done
#1760934016
cd ~/t265_ws/src/realsense-ros
#1760934016
grep -n "find_package(realsense2" realsense2_camera/CMakeLists.txt
#1760934016
# Replace whatever version it asks for (e.g. 2.51.1) with 2.50.0:
#1760934016
sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/'   realsense2_camera/CMakeLists.txt
#1760934103
sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/'   realsense2_camera/CMakeLists.txt
#1760934103
git --no-pager diff -- realsense2_camera/CMakeLists.txt | sed -n '1,120p'
#1760934179
colcon build --merge-install   --cmake-args     -DCMAKE_PREFIX_PATH=/usr/local     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2     -DCMAKE_BUILD_TYPE=Release
#1760934259
# env
#1760934259
source ~/t265_ws/install/setup.bash
#1760934259
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
#1760934259
# sanity: node must link to /usr/local … 2.50
#1760934259
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node   | grep -i 'librealsense2\.so'
#1760934260
# nothing else should be holding the camera
#1760934260
# launch with a T265 filter + your params
#1760934260
ros2 run realsense2_camera realsense2_camera_node   --ros-args -p device_type:=t265 --params-file ~/t265.yaml
#1760934807
Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]
#1760934820
# ~/t265.yaml
#1760934820
realsense2_camera:
#1760934886
cat > ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16
YAML

#1760934886
# sanity check
#1760934886
cat ~/t265.yaml
#1760934886
# launch
#1760934886
ros2 run realsense2_camera realsense2_camera_node   --ros-args --params-file ~/t265.yaml
#1760934964
cat >| ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16   # must be <= 32
YAML

#1760934974
# close anything that might hold the camera
#1760934974
# launch (explicitly filter to T265 + set the queue)
#1760934974
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p tracking_module.frames_queue_size:=16   --params-file ~/t265.yaml
#1760934993
# see the topics
#1760934993
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'
#1760934994
# confirm pose is streaming
#1760934994
ros2 topic echo -n 1 /camera/pose/sample
#1760934995
ros2 topic hz /camera/pose/sample
#1760935073
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p tracking_module.frames_queue_size:=16   --params-file ~/t265.yaml
#1760935161
# (realsense-viewer closed)
#1760935161
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p tracking_module.frames_queue_size:=16   --params-file ~/t265.yaml
#1760935887
# list what exists (no /camera prefix)
#1760935887
ros2 topic list | egrep '^/(accel|gyro|fisheye|pose|odom|tf)'
#1760935888
# echo IMU with matching QoS
#1760935888
ros2 topic echo /gyro/sample    --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935889
ros2 topic echo /accel/sample   --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760935922
ros2 run realsense2_camera realsense2_camera_node
#1760937134
sudo apt install -y python3-tf-transformations python3-numpy
#1760937150
ros2 topic echo -n 1 /camera/odom/sample   --qos-reliability reliable --qos-durability volatile || ros2 topic echo -n 1 /camera/pose/sample   --qos-reliability reliable --qos-durability volatile
#1760937196
ros2 topic list | egrep '/camera/(odom|pose)/sample'
#1760937238
ros2 topic list | egrep '/(odom|pose)/sample$'
#1760937423
# Make sure the unversioned .so symlinks point at 2.50.x
#1760937423
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
#1760937423
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so
#1760937423
# Refresh the linker cache and verify ONLY 2.50 remains
#1760937423
sudo ldconfig
#1760937423
ldconfig -p | grep -i librealsense
#1760937423
# Expect: only /usr/local entries for 2.50
#1760937423
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
#1760937423
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
#1760937423
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
#1760937424
lrwxrwxrwx 1 root root        21 Oct 20 00:13 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
#1760937424
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
#1760937424
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
#1760937424
lrwxrwxrwx 1 root root        23 Oct 20 00:13 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
#1760937424
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
#1760937424
12:13:50 alpha@opi t265_ws → # Stop anything that might have the device open
#1760937424
pkill -f realsense-viewer || true
#1760937425
pkill -f realsense2_camera_node || true
#1760937425
# Clean & rebuild your workspace (you already checked out tag 4.51.1)
#1760937425
cd ~/t265_ws
#1760937425
rm -rf build/ install/ log/
#1760937425
colcon build --merge-install   --cmake-args     -DCMAKE_PREFIX_PATH=/usr/local     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2     -DCMAKE_BUILD_TYPE=Release
#1760937493
[0.613s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [2.16s]
--- stderr: realsense2_camera                          
CMake Warning at CMakeLists.txt:104 (find_package):
  Could not find a configuration file for package "realsense2" that is
  compatible with requested version "2.51.1".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/realsense2/realsense2Config.cmake, version: 2.50.0



CMake Error at CMakeLists.txt:106 (message):
  



   Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases

  



---
Failed   <<< realsense2_camera [2.94s, exited with code 1]

Summary: 2 packages finished [15.5s]
  1 package failed: realsense2_camera
  3 packages had stderr output: realsense2_camera realsense2_camera_msgs realsense2_description
12:14:15 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Sanity: the node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# Launch
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
12:14:29 alpha@opi t265_ws → 12:11:21 alpha@opi t265_ws → # You had these on hold; unhold first:
sudo apt-mark unhold librealsense2 librealsense2-gl librealsense2-utils librealsense2-udev-rules || true

# Purge the libraries (keep udev rules if you like; it’s just rules, no libs)
sudo apt purge -y --allow-change-held-packages librealsense2 librealsense2-gl librealsense2-utils ros-humble-librealsense2
Canceled hold on librealsense2.
Canceled hold on librealsense2-gl.
Canceled hold on librealsense2-utils.
Canceled hold on librealsense2-udev-rules.
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following packages were automatically installed and are no longer required:
  libatk-bridge2.0-dev libatk1.0-dev libatspi2.0-dev libcairo2-dev libdatrie-dev libepoxy-dev libfribidi-dev libgdk-pixbuf-2.0-dev libgraphite2-dev libgtk-3-dev libharfbuzz-dev
  libharfbuzz-gobject0 libmbim-glib4 libmbim-proxy libpango1.0-dev libpixman-1-dev libqmi-glib5 libqmi-proxy libthai-dev libxcomposite-dev libxdamage-dev libxtst-dev pango1.0-tools
  wayland-protocols
Use 'sudo apt autoremove' to remove them.
The following packages will be REMOVED:
  librealsense2* librealsense2-gl* librealsense2-utils* ros-humble-librealsense2*
0 upgraded, 0 newly installed, 4 to remove and 0 not upgraded.
After this operation, 71.5 MB disk space will be freed.
(Reading database ... 260278 files and directories currently installed.)
Removing librealsense2-utils:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2-gl:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2:arm64 (2.56.5-0~realsense.6682) ...
Removing ros-humble-librealsense2 (2.56.4-1jammy.20250722.184445) ...
Processing triggers for libc-bin (1:2.35-0ubuntu3.4+2widevine) ...
12:14:29 alpha@opi t265_ws → ot found2_camera_node: No such file or directoryt265.yaml\cription/librealsense/releasese):(find_package):TH doesn't contain any 'local_setup.*' files.
#1760937493
12:11:21: command not found
#1760937493
librealsense2 was already not on hold.
#1760937493
librealsense2-gl was already not on hold.
#1760937493
librealsense2-utils was already not on hold.
#1760937494
librealsense2-udev-rules was already not on hold.
#1760937494
Reading package lists... Done
#1760937494
Building dependency tree... Done
#1760937494
Reading state information... Done
#1760937495
Package 'librealsense2-utils' is not installed, so not removed
#1760937495
Package 'librealsense2' is not installed, so not removed
#1760937495
Package 'librealsense2-gl' is not installed, so not removed
#1760937495
Package 'ros-humble-librealsense2' is not installed, so not removed
#1760937495
The following packages were automatically installed and are no longer required:
#1760937496
Use 'sudo apt autoremove' to remove them.
#1760937497
0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
#1760937497
Canceled: command not found
#1760937498
Building: command not found
#1760937499
libatk-bridge2.0-dev: command not found
#1760937499
libharfbuzz-gobject0: command not found
#1760937499
wayland-protocols: command not found
#1760937499
Command 'Use' not found, did you mean:
#1760937500
librealsense2*: command not found
#1760937500
0: command not found
#1760937500
After: command not found
#1760937500
Reading: command not found
#1760937501
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `('
#1760937501
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `('
#1760937501
bash: syntax error near unexpected token `('
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
lrwxrwxrwx: command not found
bash: librealsense2-gl.so.2.50: cannot overwrite existing file
lrwxrwxrwx: command not found
bash: librealsense2-gl.so.2.50.0: cannot overwrite existing file
-rw-r--r--: command not found
-rw-r--r--: command not found
lrwxrwxrwx: command not found
lrwxrwxrwx: command not found
-rw-r--r--: command not found
lrwxrwxrwx: command not found
-rw-r--r--: command not found
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
#1760937501
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
#1760937501
12:13:09: command not found
#1760937501
Package not found
#1760937502
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
#1760937502
Package 'realsense2_camera' not found
#1760937502
[INFO]: command not found
#1760937503
[WARN]: command not found
#1760937503
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2): substitution failed
#1760937504
bash: librealsense2.so.2.50: cannot overwrite existing file
#1760937504
bash: librealsense2.so.2.50.0: cannot overwrite existing file
#1760937505
bash: librealsense2.so.2.56.4: cannot overwrite existing file
#1760937505
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
#1760937505
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
12:13:50: command not found
[0.508s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
#1760937506
Starting >>> realsense2_camera_msgs
#1760937506
--- stderr: realsense2_camera_msgs                                
#1760937506
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
#1760937507
CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
#1760937508
CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
#1760937508
Call Stack (most recent call first):
#1760937508
This warning is for project developers.  Use -Wno-dev to suppress it.
#1760937509
Finished <<< realsense2_camera_msgs [11.8s]
#1760937510
Starting >>> realsense2_camera
#1760937510
Starting >>> realsense2_description
#1760937510
--- stderr: realsense2_description                                                                         
#1760937510
CMake Warning:
#1760937511
Finished <<< realsense2_description [1.89s]
#1760937511
--- stderr: realsense2_camera                          
#1760937511
CMake Warning at CMakeLists.txt:104 (find_package):
#1760937512
CMake Error at CMakeLists.txt:106 (message):
#1760937512
---
#1760937512
Failed   <<< realsense2_camera [2.89s, exited with code 1]
#1760937512
Summary: 2 packages finished [15.2s]
#1760937513
[0.613s]: command not found
#1760937513
bash: syntax error near unexpected token `>'
---: command not found
bash: syntax error near unexpected token `('
#1760937534
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `include'
bash: syntax error near unexpected token `ament_execute_extensions'
#1760937534
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
bash: syntax error near unexpected token `('
#1760937538
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `find_package'
bash: syntax error near unexpected token `include'
#1760937538
bash: syntax error near unexpected token `ament_execute_extensions'
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
bash: syntax error near unexpected token `('
#1760937541
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `find_package'
bash: syntax error near unexpected token `include'
#1760937541
bash: syntax error near unexpected token `ament_execute_extensions'
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
CMake: command not found
Manually-specified: command not found
realsense2_DIR: command not found
---: command not found
Finished: command not found
bash: syntax error near unexpected token `>'
#1760937542
bash: syntax error near unexpected token `>'
---: command not found
CMake: command not found
Manually-specified: command not found
realsense2_DIR: command not found
---: command not found
Finished: command not found
---: command not found
bash: syntax error near unexpected token `('
#1760937542
bash: syntax error near unexpected token `('
Intel: command not found
---: command not found
Failed: command not found
Summary:: command not found
bash: cd: too many arguments
bash: cd: -3: invalid option
cd: usage: cd [-L|[-P [-e]] [-@]] [dir]
12:14:15: command not found
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
Command 'Package' not found, did you mean:
  command 'package' from deb mailagent (1:3.1-106-1)
Try: sudo apt install <deb name>
Command 'ldd:' not found, did you mean:
  command 'ldd' from deb libc-bin (2.35-0ubuntu3.11)
Try: sudo apt install <deb name>
Command 'Package' not found, did you mean:
  command 'package' from deb mailagent (1:3.1-106-1)
Try: sudo apt install <deb name>
12:14:29: command not found
12:16:20 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect ONLY 2.50 entries (plus the -gl 2.50 you already have)
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/lib^C
12:16:55 alpha@opi t265_ws → ^C
12:16:55 alpha@opi t265_ws → sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
12:16:58 alpha@opi t265_ws → sudo ldconfig
12:17:05 alpha@opi t265_ws → ldconfig -p | grep -i librealsense
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:17:12 alpha@opi t265_ws → cd ~/t265_ws/src/realsense-ros
git fetch --tags

# Find the newest 4.50.* or 4.49.* tag that asks for <= 2.50.0
CANDIDATE=$(
  for t in $(git tag -l '4.50.*' '4.49.*' --sort=-v:refname); do
    min=$(git show "$t:CMakeLists.txt" | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$min" ] || continue
    if dpkg --compare-versions "$min" le 2.50.0; then echo "$t"; break; fi
  done
)
echo "Checking out $CANDIDATE"
git checkout "$CANDIDATE"
Checking out 
fatal: empty string is not a valid pathspec. please use . instead if you meant to match all paths
12:17:54 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
git fetch --tags

# List nearby tags
git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname | head -n 15

# Auto-pick the newest tag that requires <= 2.50.0
CAND=$(
  for t in $(git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname); do
    need=$(git show "$t:realsense2_camera/CMakeLists.txt" 2>/dev/null \
            | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$need" ] || continue
    if dpkg --compare-versions "$need" le 2.50.0; then echo "$t"; break; fi
  done
)
echo "Checking out $CAND"; git checkout "$CAND"
Checking out 
fatal: empty string is not a valid pathspec. please use . instead if you meant to match all paths
12:18:52 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
git fetch --tags
git tag -l '4.5*' --sort=-v:refname | head -n 40
# Try these, in order; the first that exists will do:
for t in 4.50.1 4.50.0 4.49.1 4.49.0 4.48.1 4.48.0; do
  git checkout $t && break
done
4.57.3
4.57.2
4.56.4
4.56.3
4.56.1
4.55.1-RC
4.55.1
4.54.1
4.51.1
error: pathspec '4.50.1' did not match any file(s) known to git
error: pathspec '4.50.0' did not match any file(s) known to git
error: pathspec '4.49.1' did not match any file(s) known to git
error: pathspec '4.49.0' did not match any file(s) known to git
error: pathspec '4.48.1' did not match any file(s) known to git
error: pathspec '4.48.0' did not match any file(s) known to git
12:20:04 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
grep -n "find_package(realsense2" realsense2_camera/CMakeLists.txt
# Replace whatever version it asks for (e.g. 2.51.1) with 2.50.0:
sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/' \
  realsense2_camera/CMakeLists.txt
96:find_package(realsense2_camera_msgs REQUIRED)
104:find_package(realsense2 2.51.1)
12:20:16 alpha@opi realsense-ros ±| ✗|→ sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/' \
  realsense2_camera/CMakeLists.txt
git --no-pager diff -- realsense2_camera/CMakeLists.txt | sed -n '1,120p'
diff --git a/realsense2_camera/CMakeLists.txt b/realsense2_camera/CMakeLists.txt
index 84c9e5bd..4dcd142f 100644
--- a/realsense2_camera/CMakeLists.txt
+++ b/realsense2_camera/CMakeLists.txt
@@ -101,7 +101,7 @@ find_package(tf2_ros REQUIRED)
 find_package(tf2 REQUIRED)
 find_package(diagnostic_updater REQUIRED)
 
-find_package(realsense2 2.51.1)
+find_package(realsense2 2.50.0)
 if(NOT realsense2_FOUND)
     message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
 endif()
12:21:43 alpha@opi realsense-ros ±| ✗|→ cd ~/t265_ws
rm -rf build/ install/ log/

colcon build --merge-install \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    -DCMAKE_BUILD_TYPE=Release
[0.510s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [1.93s]
[Processing: realsense2_camera]                             
Finished <<< realsense2_camera [52.9s]                           

Summary: 3 packages finished [1min 5s]
  2 packages had stderr output: realsense2_camera_msgs realsense2_description
12:24:05 alpha@opi t265_ws → # env
source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# sanity: node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# nothing else should be holding the camera
pkill -f realsense-viewer || true

# launch with a T265 filter + your params
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args -p device_type:=t265 --params-file ~/t265.yaml
[INFO] [1760934261.147517502] [camera]: RealSense ROS v4.51.1
[INFO] [1760934261.147736831] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934261.147761330] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934261.193490552] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934261.193657966] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934261.193679548] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934261.194019917] [camera]: Device with port number 6-1 was found.
[INFO] [1760934261.194059874] [camera]: Device USB type: 3.1
[INFO] [1760934261.210363136] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934261.210463759] [camera]: getParameters...
[INFO] [1760934261.210688629] [camera]: JSON file is not provided
[INFO] [1760934261.210731212] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934261.210758628] [camera]: Device Serial No: 908412110035
[INFO] [1760934261.210785752] [camera]: Device physical port: 6-1-2
[INFO] [1760934261.210817543] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934261.210847876] [camera]: Device Product ID: 0x0B37
[INFO] [1760934261.210879376] [camera]: Sync Mode: Off
[WARN] [1760934261.214857922] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760934261.271479221] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934261.297844200] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934261.301729415] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934261.301851038] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934261.301889537] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934261.301959536] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934261.325573861] [camera]: RealSense Node Is Up!
[WARN] [1760934261.326608091] [camera]: 
[INFO] [1760934718.437839509] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934718.948396740] [camera]: Close Sensor. 
[INFO] [1760934718.948687818] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:31:59 alpha@opi t265_ws → Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]
Could: command not found
12:33:27 alpha@opi t265_ws → # ~/t265.yaml
realsense2_camera:
  ros__parameters:
    device_type: "t265"
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: "linear_interpolation"
    tracking_module:
      frames_queue_size: 16
realsense2_camera:: command not found
ros__parameters:: command not found
device_type:: command not found
enable_fisheye1:: command not found
enable_fisheye2:: command not found
enable_gyro:: command not found
enable_accel:: command not found
enable_pose:: command not found
publish_tf:: command not found
unite_imu_method:: command not found
tracking_module:: command not found
frames_queue_size:: command not found
12:33:43 alpha@opi t265_ws → cat > ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16
YAML

# sanity check
cat ~/t265.yaml

# launch
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args --params-file ~/t265.yaml
bash: /home/alpha/t265.yaml: cannot overwrite existing file
realsense2_camera:
  ros__parameters:
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: "linear_interpolation"   # common choice for IMU fusion
[INFO] [1760934887.004142223] [camera]: RealSense ROS v4.51.1
[INFO] [1760934887.004340845] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934887.004363011] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934887.047347866] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934887.047512655] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934887.047536279] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934887.047876066] [camera]: Device with port number 6-1 was found.
[INFO] [1760934887.047905523] [camera]: Device USB type: 3.1
[INFO] [1760934887.058720350] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934887.058813099] [camera]: getParameters...
[INFO] [1760934887.059103011] [camera]: JSON file is not provided
[INFO] [1760934887.059147635] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934887.059165135] [camera]: Device Serial No: 908412110035
[INFO] [1760934887.059183510] [camera]: Device physical port: 6-1-2
[INFO] [1760934887.059199843] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934887.059220551] [camera]: Device Product ID: 0x0B37
[INFO] [1760934887.059235134] [camera]: Sync Mode: Off
[WARN] [1760934887.059768000] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760934887.115480984] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934887.124204595] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934887.128312654] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934887.128405111] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934887.128425236] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934887.128446527] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934887.151678864] [camera]: RealSense Node Is Up!
[WARN] [1760934887.153468793] [camera]: 
[INFO] [1760934961.298617834] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934961.781705293] [camera]: Close Sensor. 
[INFO] [1760934961.782442032] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:36:02 alpha@opi t265_ws → cat >| ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16   # must be <= 32
YAML
12:36:04 alpha@opi t265_ws → # close anything that might hold the camera
pkill -f realsense-viewer || true

# launch (explicitly filter to T265 + set the queue)
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760934974.888563501] [camera]: RealSense ROS v4.51.1
[INFO] [1760934974.888760956] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934974.888782831] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934974.959134348] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934974.959294762] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934974.959317803] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934974.959658173] [camera]: Device with port number 6-1 was found.
[INFO] [1760934974.959700755] [camera]: Device USB type: 3.1
[INFO] [1760934974.970050092] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934974.970136424] [camera]: getParameters...
[INFO] [1760934974.970370921] [camera]: JSON file is not provided
[INFO] [1760934974.970402420] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934974.970423712] [camera]: Device Serial No: 908412110035
[INFO] [1760934974.970443545] [camera]: Device physical port: 6-1-2
[INFO] [1760934974.970462503] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934974.970481169] [camera]: Device Product ID: 0x0B37
[INFO] [1760934974.970499252] [camera]: Sync Mode: Off
[INFO] [1760934975.022336352] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934975.030382434] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934975.034443537] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934975.034540660] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934975.034561077] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934975.034589951] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934975.057933417] [camera]: RealSense Node Is Up!
[WARN] [1760934975.065670628] [camera]: 
[INFO] [1760934991.261126346] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934991.756072531] [camera]: Close Sensor. 
[INFO] [1760934991.756324235] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:36:32 alpha@opi t265_ws → # see the topics
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'

# confirm pose is streaming
ros2 topic echo -n 1 /camera/pose/sample
ros2 topic hz /camera/pose/sample
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
WARNING: topic [/camera/pose/sample] does not appear to be published yet
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760935074.383173192] [camera]: RealSense ROS v4.51.1
[INFO] [1760935074.383377356] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935074.383396897] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935074.456934548] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935074.457104587] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935074.457124712] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935074.457467415] [camera]: Device with port number 6-1 was found.
[INFO] [1760935074.457503289] [camera]: Device USB type: 3.1
[INFO] [1760935074.467354470] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935074.467451301] [camera]: getParameters...
[INFO] [1760935074.467680840] [camera]: JSON file is not provided
[INFO] [1760935074.467707089] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935074.467728380] [camera]: Device Serial No: 908412110035
[INFO] [1760935074.467748797] [camera]: Device physical port: 6-1-2
[INFO] [1760935074.467767755] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935074.467787880] [camera]: Device Product ID: 0x0B37
[INFO] [1760935074.467805963] [camera]: Sync Mode: Off
[INFO] [1760935074.517358194] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935074.524492833] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935074.528233108] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935074.528318274] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935074.528341023] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935074.528359690] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935074.552003281] [camera]: RealSense Node Is Up!
[WARN] [1760935074.562638741] [camera]: 
[INFO] [1760935157.492823617] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935157.982522974] [camera]: Close Sensor. 
[INFO] [1760935157.982804428] [camera]: Close Sensor - Done. 
12:39:18 alpha@opi t265_ws → # (realsense-viewer closed)
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760935162.434496648] [camera]: RealSense ROS v4.51.1
[INFO] [1760935162.434708103] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935162.434728811] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935162.494593934] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935162.494753474] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935162.494777390] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935162.495115426] [camera]: Device with port number 6-1 was found.
[INFO] [1760935162.495152467] [camera]: Device USB type: 3.1
[INFO] [1760935162.505502975] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935162.505599515] [camera]: getParameters...
[INFO] [1760935162.505860553] [camera]: JSON file is not provided
[INFO] [1760935162.505896428] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935162.505921510] [camera]: Device Serial No: 908412110035
[INFO] [1760935162.505945718] [camera]: Device physical port: 6-1-2
[INFO] [1760935162.505969343] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935162.505992093] [camera]: Device Product ID: 0x0B37
[INFO] [1760935162.506011051] [camera]: Sync Mode: Off
[INFO] [1760935162.558326206] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935162.567514149] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935162.571551920] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935162.571652544] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935162.571673835] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935162.571693376] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935162.595201057] [camera]: RealSense Node Is Up!
[WARN] [1760935162.601663417] [camera]: 
[INFO] [1760935237.774146010] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935238.281780618] [camera]: Close Sensor. 
[INFO] [1760935238.282085405] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:40:38 alpha@opi t265_ws → pkill -f realsense-viewer || true
12:40:41 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935250.391354660] [camera]: RealSense ROS v4.51.1
[INFO] [1760935250.391556782] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935250.391577490] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935250.463490314] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935250.463645769] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935250.463672602] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935250.464337009] [camera]: Device with port number 6-1 was found.
[INFO] [1760935250.464430049] [camera]: Device USB type: 3.1
[INFO] [1760935250.475097595] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935250.475199677] [camera]: getParameters...
[INFO] [1760935250.475445840] [camera]: JSON file is not provided
[INFO] [1760935250.475482006] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935250.475497464] [camera]: Device Serial No: 908412110035
[INFO] [1760935250.475514964] [camera]: Device physical port: 6-1-2
[INFO] [1760935250.475532172] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935250.475549380] [camera]: Device Product ID: 0x0B37
[INFO] [1760935250.475563380] [camera]: Sync Mode: Off
[WARN] [1760935250.476078456] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935250.527147346] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935250.534601066] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935250.538329384] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935250.538421841] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935250.538443133] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935250.538465591] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935250.561876443] [camera]: RealSense Node Is Up!
[WARN] [1760935250.570527728] [camera]: 
pkill -f realsense-viewer || true
[INFO] [1760935349.644436211] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935350.124920056] [camera]: Close Sensor. 
[INFO] [1760935350.125223093] [camera]: Close Sensor - Done. 
12:42:30 alpha@opi t265_ws → pkill -f realsense-viewer || true
12:42:35 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935361.965760076] [camera]: RealSense ROS v4.51.1
[INFO] [1760935361.965948198] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935361.965969781] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935362.033975799] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935362.034143213] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935362.034164213] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935362.034508374] [camera]: Device with port number 6-1 was found.
[INFO] [1760935362.034537249] [camera]: Device USB type: 3.1
[INFO] [1760935362.044824469] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935362.044921009] [camera]: getParameters...
[INFO] [1760935362.045151423] [camera]: JSON file is not provided
[INFO] [1760935362.045200714] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935362.045223172] [camera]: Device Serial No: 908412110035
[INFO] [1760935362.045242713] [camera]: Device physical port: 6-1-2
[INFO] [1760935362.045261379] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935362.045280046] [camera]: Device Product ID: 0x0B37
[INFO] [1760935362.045298420] [camera]: Sync Mode: Off
[WARN] [1760935362.045842662] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935362.102351937] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935362.110771060] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935362.114980913] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935362.115073662] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935362.115107495] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935362.115131703] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935362.138579892] [camera]: RealSense Node Is Up!
[WARN] [1760935362.140583612] [camera]: 
  --ros-args
-p device_type:=t265
-p tracking_module.frames_queue_size:=16
--params-file ~/t265.yaml
[INFO] [1760935790.282803340] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935790.763740896] [camera]: Close Sensor. 
[INFO] [1760935790.764017100] [camera]: Close Sensor - Done. 
12:49:51 alpha@opi t265_ws → # list what exists (no /camera prefix)
ros2 topic list | egrep '^/(accel|gyro|fisheye|pose|odom|tf)'

# echo IMU with matching QoS
ros2 topic echo /gyro/sample  \
  --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
ros2 topic echo /accel/sample \
  --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
WARNING: topic [/gyro/sample] does not appear to be published yet
Could not determine the type for the passed topic
WARNING: topic [/accel/sample] does not appear to be published yet
Could not determine the type for the passed topic
12:51:29 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935922.550604222] [camera]: RealSense ROS v4.51.1
[INFO] [1760935922.550799344] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935922.550822677] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935922.619154219] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935922.619300633] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935922.619323966] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935922.619664920] [camera]: Device with port number 6-1 was found.
[INFO] [1760935922.619698169] [camera]: Device USB type: 3.1
[INFO] [1760935922.630093017] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935922.630187516] [camera]: getParameters...
[INFO] [1760935922.630482095] [camera]: JSON file is not provided
[INFO] [1760935922.630522344] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935922.630542761] [camera]: Device Serial No: 908412110035
[INFO] [1760935922.630562302] [camera]: Device physical port: 6-1-2
[INFO] [1760935922.630580677] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935922.630599343] [camera]: Device Product ID: 0x0B37
[INFO] [1760935922.630616843] [camera]: Sync Mode: Off
[WARN] [1760935922.631133669] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935922.682548208] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935922.690808378] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935922.694459408] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935922.694549532] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935922.694572573] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935922.694593281] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935922.717899440] [camera]: RealSense Node Is Up!
[WARN] [1760935922.726386233] [camera]: 
[INFO] [1760937130.590209568] [camera]: Stop Sensor: Tracking Module
[INFO] [1760937131.099256052] [camera]: Close Sensor. 
[INFO] [1760937131.099578047] [camera]: Close Sensor - Done. 
01:12:11 alpha@opi t265_ws → sudo apt install -y python3-tf-transformations python3-numpy
[sudo] password for alpha: 
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
E: Unable to locate package python3-tf-transformations
01:12:18 alpha@opi t265_ws → ros2 topic list | egrep '/camera/(odom|pose)/sample'
ros2 topic echo -n 1 /camera/odom/sample \
  --qos-reliability reliable --qos-durability volatile || \
ros2 topic echo -n 1 /camera/pose/sample \
  --qos-reliability reliable --qos-durability volatile
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
01:12:31 alpha@opi t265_ws → ros2 topic list | egrep '/camera/(odom|pose)/sample'
01:13:17 alpha@opi t265_ws → ros2 topic list | egrep '/(odom|pose)/sample$'
01:13:58 alpha@opi t265_ws → 
# Make sure the unversioned .so symlinks point at 2.50.x
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so

# Refresh the linker cache and verify ONLY 2.50 remains
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect: only /usr/local entries for 2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        21 Oct 20 00:13 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
lrwxrwxrwx 1 root root        23 Oct 20 00:13 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:13:50 alpha@opi t265_ws → # Stop anything that might have the device open
pkill -f realsense-viewer || true
pkill -f realsense2_camera_node || true

# Clean & rebuild your workspace (you already checked out tag 4.51.1)
cd ~/t265_ws
rm -rf build/ install/ log/
colcon build --merge-install \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    -DCMAKE_BUILD_TYPE=Release
[0.613s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [2.16s]
--- stderr: realsense2_camera                          
CMake Warning at CMakeLists.txt:104 (find_package):
  Could not find a configuration file for package "realsense2" that is
  compatible with requested version "2.51.1".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/realsense2/realsense2Config.cmake, version: 2.50.0



CMake Error at CMakeLists.txt:106 (message):
  



   Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases

  



---
Failed   <<< realsense2_camera [2.94s, exited with code 1]

Summary: 2 packages finished [15.5s]
  1 package failed: realsense2_camera
  3 packages had stderr output: realsense2_camera realsense2_camera_msgs realsense2_description
12:14:15 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Sanity: the node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# Launch
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
12:14:29 alpha@opi t265_ws → 12:11:21 alpha@opi t265_ws → # You had these on hold; unhold first:
sudo apt-mark unhold librealsense2 librealsense2-gl librealsense2-utils librealsense2-udev-rules || true

# Purge the libraries (keep udev rules if you like; it’s just rules, no libs)
sudo apt purge -y --allow-change-held-packages librealsense2 librealsense2-gl librealsense2-utils ros-humble-librealsense2
Canceled hold on librealsense2.
Canceled hold on librealsense2-gl.
Canceled hold on librealsense2-utils.
Canceled hold on librealsense2-udev-rules.
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following packages were automatically installed and are no longer required:
  libatk-bridge2.0-dev libatk1.0-dev libatspi2.0-dev libcairo2-dev libdatrie-dev libepoxy-dev libfribidi-dev libgdk-pixbuf-2.0-dev libgraphite2-dev libgtk-3-dev libharfbuzz-dev
  libharfbuzz-gobject0 libmbim-glib4 libmbim-proxy libpango1.0-dev libpixman-1-dev libqmi-glib5 libqmi-proxy libthai-dev libxcomposite-dev libxdamage-dev libxtst-dev pango1.0-tools
  wayland-protocols
Use 'sudo apt autoremove' to remove them.
The following packages will be REMOVED:
  librealsense2* librealsense2-gl* librealsense2-utils* ros-humble-librealsense2*
0 upgraded, 0 newly installed, 4 to remove and 0 not upgraded.
After this operation, 71.5 MB disk space will be freed.
(Reading database ... 260278 files and directories currently installed.)
Removing librealsense2-utils:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2-gl:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2:arm64 (2.56.5-0~realsense.6682) ...
Removing ros-humble-librealsense2 (2.56.4-1jammy.20250722.184445) ...
Processing triggers for libc-bin (1:2.35-0ubuntu3.4+2widevine) ...
12:14:29 alpha@opi t265_ws → ot found2_camera_node: No such file or directoryt265.yaml\cription/librealsense/releasese):(find_package):TH doesn't contain any 'local_setup.*' files.
12:11:21: command not found
librealsense2 was already not on hold.
librealsense2-gl was already not on hold.
librealsense2-utils was already not on hold.
librealsense2-udev-rules was already not on hold.
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Package 'librealsense2-utils' is not installed, so not removed
Package 'librealsense2' is not installed, so not removed
Package 'librealsense2-gl' is not installed, so not removed
Package 'ros-humble-librealsense2' is not installed, so not removed
The following packages were automatically installed and are no longer required:
  libatk-bridge2.0-dev libatk1.0-dev libatspi2.0-dev libcairo2-dev libdatrie-dev libepoxy-dev libfribidi-dev libgdk-pixbuf-2.0-dev libgraphite2-dev libgtk-3-dev libharfbuzz-dev
  libharfbuzz-gobject0 libmbim-glib4 libmbim-proxy libpango1.0-dev libpixman-1-dev libqmi-glib5 libqmi-proxy libthai-dev libxcomposite-dev libxdamage-dev libxtst-dev pango1.0-tools
  wayland-protocols
Use 'sudo apt autoremove' to remove them.
0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
Canceled: command not found
Canceled: command not found
Canceled: command not found
Canceled: command not found
Reading: command not found
Building: command not found
Reading: command not found
Command 'The' not found, did you mean:
  command 'he' from deb node-he (1.2.0-3)
  command 'the' from deb the (3.3~rc1-3build1)
Try: sudo apt install <deb name>
libatk-bridge2.0-dev: command not found
libharfbuzz-gobject0: command not found
wayland-protocols: command not found
Command 'Use' not found, did you mean:
  command 'ase' from deb ase (3.22.1-1ubuntu1)
  command 'nse' from deb ns2 (2.35+dfsg-3.1)
Try: sudo apt install <deb name>
Command 'The' not found, did you mean:
  command 'the' from deb the (3.3~rc1-3build1)
  command 'he' from deb node-he (1.2.0-3)
Try: sudo apt install <deb name>
librealsense2*: command not found
0: command not found
After: command not found
Reading: command not found
bash: syntax error near unexpected token `('
#1760937542
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `('
#1760937542
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `('
#1760937545
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
#1760937545
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
12:13:09: command not found
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
[INFO]: command not found
[INFO]: command not found
[INFO]: command not found
[WARN]: command not found
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2): substitution failed
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
bash: librealsense2-gl.so.2.50: cannot overwrite existing file
bash: librealsense2-gl.so.2.50: cannot overwrite existing file
bash: librealsense2-gl.so.2.50.0: cannot overwrite existing file
bash: librealsense2-gl.so.2.50.0: cannot overwrite existing file
-rw-r--r--: command not found
-rw-r--r--: command not found
bash: librealsense2.so.2.50: cannot overwrite existing file
bash: librealsense2.so.2.50.0: cannot overwrite existing file
-rw-r--r--: command not found
bash: librealsense2.so.2.56.4: cannot overwrite existing file
-rw-r--r--: command not found
bash: syntax error near unexpected token `libc6,AArch64'
#1760937545
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
#1760937546
bash: syntax error near unexpected token `libc6,AArch64'
12:13:50: command not found
[0.508s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [11.8s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [1.89s]
--- stderr: realsense2_camera                          
CMake Warning at CMakeLists.txt:104 (find_package):
  Could not find a configuration file for package "realsense2" that is
  compatible with requested version "2.51.1".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/realsense2/realsense2Config.cmake, version: 2.50.0



CMake Error at CMakeLists.txt:106 (message):
  



   Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases

  



---
Failed   <<< realsense2_camera [2.89s, exited with code 1]
                                 
Summary: 2 packages finished [15.2s]
  1 package failed: realsense2_camera
  3 packages had stderr output: realsense2_camera realsense2_camera_msgs realsense2_description
[0.613s]: command not found
bash: syntax error near unexpected token `>'
---: command not found
bash: syntax error near unexpected token `('
#1760937549
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `include'
bash: syntax error near unexpected token `ament_execute_extensions'
#1760937549
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
bash: syntax error near unexpected token `('
#1760937552
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `find_package'
bash: syntax error near unexpected token `include'
#1760937553
bash: syntax error near unexpected token `ament_execute_extensions'
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
bash: syntax error near unexpected token `('
#1760937556
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `find_package'
bash: syntax error near unexpected token `include'
#1760937556
bash: syntax error near unexpected token `ament_execute_extensions'
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
CMake: command not found
Manually-specified: command not found
realsense2_DIR: command not found
---: command not found
Finished: command not found
bash: syntax error near unexpected token `>'
#1760937556
bash: syntax error near unexpected token `>'
---: command not found
CMake: command not found
Manually-specified: command not found
realsense2_DIR: command not found
---: command not found
Finished: command not found
---: command not found
bash: syntax error near unexpected token `('
#1760937557
bash: syntax error near unexpected token `('
Intel: command not found
---: command not found
Failed: command not found
Summary:: command not found
bash: cd: too many arguments
bash: cd: -3: invalid option
cd: usage: cd [-L|[-P [-e]] [-@]] [dir]
12:14:15: command not found
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
Command 'Package' not found, did you mean:
  command 'package' from deb mailagent (1:3.1-106-1)
Try: sudo apt install <deb name>
Command 'ldd:' not found, did you mean:
  command 'ldd' from deb libc-bin (2.35-0ubuntu3.11)
Try: sudo apt install <deb name>
Command 'Package' not found, did you mean:
  command 'package' from deb mailagent (1:3.1-106-1)
Try: sudo apt install <deb name>
12:14:29: command not found
12:16:20 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect ONLY 2.50 entries (plus the -gl 2.50 you already have)
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/lib^C
12:16:55 alpha@opi t265_ws → ^C
12:16:55 alpha@opi t265_ws → sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
12:16:58 alpha@opi t265_ws → sudo ldconfig
12:17:05 alpha@opi t265_ws → ldconfig -p | grep -i librealsense
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:17:12 alpha@opi t265_ws → cd ~/t265_ws/src/realsense-ros
git fetch --tags

# Find the newest 4.50.* or 4.49.* tag that asks for <= 2.50.0
CANDIDATE=$(
  for t in $(git tag -l '4.50.*' '4.49.*' --sort=-v:refname); do
    min=$(git show "$t:CMakeLists.txt" | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$min" ] || continue
    if dpkg --compare-versions "$min" le 2.50.0; then echo "$t"; break; fi
  done
)
echo "Checking out $CANDIDATE"
git checkout "$CANDIDATE"
Checking out 
fatal: empty string is not a valid pathspec. please use . instead if you meant to match all paths
12:17:54 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
git fetch --tags

# List nearby tags
git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname | head -n 15

# Auto-pick the newest tag that requires <= 2.50.0
CAND=$(
  for t in $(git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname); do
    need=$(git show "$t:realsense2_camera/CMakeLists.txt" 2>/dev/null \
            | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$need" ] || continue
    if dpkg --compare-versions "$need" le 2.50.0; then echo "$t"; break; fi
  done
)
echo "Checking out $CAND"; git checkout "$CAND"
Checking out 
fatal: empty string is not a valid pathspec. please use . instead if you meant to match all paths
12:18:52 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
git fetch --tags
git tag -l '4.5*' --sort=-v:refname | head -n 40
# Try these, in order; the first that exists will do:
for t in 4.50.1 4.50.0 4.49.1 4.49.0 4.48.1 4.48.0; do
  git checkout $t && break
done
4.57.3
4.57.2
4.56.4
4.56.3
4.56.1
4.55.1-RC
4.55.1
4.54.1
4.51.1
error: pathspec '4.50.1' did not match any file(s) known to git
error: pathspec '4.50.0' did not match any file(s) known to git
error: pathspec '4.49.1' did not match any file(s) known to git
error: pathspec '4.49.0' did not match any file(s) known to git
error: pathspec '4.48.1' did not match any file(s) known to git
error: pathspec '4.48.0' did not match any file(s) known to git
12:20:04 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
grep -n "find_package(realsense2" realsense2_camera/CMakeLists.txt
# Replace whatever version it asks for (e.g. 2.51.1) with 2.50.0:
sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/' \
  realsense2_camera/CMakeLists.txt
96:find_package(realsense2_camera_msgs REQUIRED)
104:find_package(realsense2 2.51.1)
12:20:16 alpha@opi realsense-ros ±| ✗|→ sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/' \
  realsense2_camera/CMakeLists.txt
git --no-pager diff -- realsense2_camera/CMakeLists.txt | sed -n '1,120p'
diff --git a/realsense2_camera/CMakeLists.txt b/realsense2_camera/CMakeLists.txt
index 84c9e5bd..4dcd142f 100644
--- a/realsense2_camera/CMakeLists.txt
+++ b/realsense2_camera/CMakeLists.txt
@@ -101,7 +101,7 @@ find_package(tf2_ros REQUIRED)
 find_package(tf2 REQUIRED)
 find_package(diagnostic_updater REQUIRED)
 
-find_package(realsense2 2.51.1)
+find_package(realsense2 2.50.0)
 if(NOT realsense2_FOUND)
     message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
 endif()
12:21:43 alpha@opi realsense-ros ±| ✗|→ cd ~/t265_ws
rm -rf build/ install/ log/

colcon build --merge-install \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    -DCMAKE_BUILD_TYPE=Release
[0.510s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [1.93s]
[Processing: realsense2_camera]                             
Finished <<< realsense2_camera [52.9s]                           

Summary: 3 packages finished [1min 5s]
  2 packages had stderr output: realsense2_camera_msgs realsense2_description
12:24:05 alpha@opi t265_ws → # env
source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# sanity: node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# nothing else should be holding the camera
pkill -f realsense-viewer || true

# launch with a T265 filter + your params
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args -p device_type:=t265 --params-file ~/t265.yaml
[INFO] [1760934261.147517502] [camera]: RealSense ROS v4.51.1
[INFO] [1760934261.147736831] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934261.147761330] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934261.193490552] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934261.193657966] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934261.193679548] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934261.194019917] [camera]: Device with port number 6-1 was found.
[INFO] [1760934261.194059874] [camera]: Device USB type: 3.1
[INFO] [1760934261.210363136] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934261.210463759] [camera]: getParameters...
[INFO] [1760934261.210688629] [camera]: JSON file is not provided
[INFO] [1760934261.210731212] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934261.210758628] [camera]: Device Serial No: 908412110035
[INFO] [1760934261.210785752] [camera]: Device physical port: 6-1-2
[INFO] [1760934261.210817543] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934261.210847876] [camera]: Device Product ID: 0x0B37
[INFO] [1760934261.210879376] [camera]: Sync Mode: Off
[WARN] [1760934261.214857922] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760934261.271479221] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934261.297844200] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934261.301729415] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934261.301851038] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934261.301889537] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934261.301959536] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934261.325573861] [camera]: RealSense Node Is Up!
[WARN] [1760934261.326608091] [camera]: 
[INFO] [1760934718.437839509] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934718.948396740] [camera]: Close Sensor. 
[INFO] [1760934718.948687818] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:31:59 alpha@opi t265_ws → Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]
Could: command not found
12:33:27 alpha@opi t265_ws → # ~/t265.yaml
realsense2_camera:
  ros__parameters:
    device_type: "t265"
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: "linear_interpolation"
    tracking_module:
      frames_queue_size: 16
realsense2_camera:: command not found
ros__parameters:: command not found
device_type:: command not found
enable_fisheye1:: command not found
enable_fisheye2:: command not found
enable_gyro:: command not found
enable_accel:: command not found
enable_pose:: command not found
publish_tf:: command not found
unite_imu_method:: command not found
tracking_module:: command not found
frames_queue_size:: command not found
12:33:43 alpha@opi t265_ws → cat > ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16
YAML

# sanity check
cat ~/t265.yaml

# launch
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args --params-file ~/t265.yaml
bash: /home/alpha/t265.yaml: cannot overwrite existing file
realsense2_camera:
  ros__parameters:
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: "linear_interpolation"   # common choice for IMU fusion
[INFO] [1760934887.004142223] [camera]: RealSense ROS v4.51.1
[INFO] [1760934887.004340845] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934887.004363011] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934887.047347866] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934887.047512655] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934887.047536279] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934887.047876066] [camera]: Device with port number 6-1 was found.
[INFO] [1760934887.047905523] [camera]: Device USB type: 3.1
[INFO] [1760934887.058720350] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934887.058813099] [camera]: getParameters...
[INFO] [1760934887.059103011] [camera]: JSON file is not provided
[INFO] [1760934887.059147635] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934887.059165135] [camera]: Device Serial No: 908412110035
[INFO] [1760934887.059183510] [camera]: Device physical port: 6-1-2
[INFO] [1760934887.059199843] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934887.059220551] [camera]: Device Product ID: 0x0B37
[INFO] [1760934887.059235134] [camera]: Sync Mode: Off
[WARN] [1760934887.059768000] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760934887.115480984] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934887.124204595] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934887.128312654] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934887.128405111] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934887.128425236] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934887.128446527] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934887.151678864] [camera]: RealSense Node Is Up!
[WARN] [1760934887.153468793] [camera]: 
[INFO] [1760934961.298617834] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934961.781705293] [camera]: Close Sensor. 
[INFO] [1760934961.782442032] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:36:02 alpha@opi t265_ws → cat >| ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16   # must be <= 32
YAML
12:36:04 alpha@opi t265_ws → # close anything that might hold the camera
pkill -f realsense-viewer || true

# launch (explicitly filter to T265 + set the queue)
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760934974.888563501] [camera]: RealSense ROS v4.51.1
[INFO] [1760934974.888760956] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934974.888782831] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934974.959134348] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934974.959294762] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934974.959317803] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934974.959658173] [camera]: Device with port number 6-1 was found.
[INFO] [1760934974.959700755] [camera]: Device USB type: 3.1
[INFO] [1760934974.970050092] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934974.970136424] [camera]: getParameters...
[INFO] [1760934974.970370921] [camera]: JSON file is not provided
[INFO] [1760934974.970402420] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934974.970423712] [camera]: Device Serial No: 908412110035
[INFO] [1760934974.970443545] [camera]: Device physical port: 6-1-2
[INFO] [1760934974.970462503] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934974.970481169] [camera]: Device Product ID: 0x0B37
[INFO] [1760934974.970499252] [camera]: Sync Mode: Off
[INFO] [1760934975.022336352] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934975.030382434] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934975.034443537] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934975.034540660] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934975.034561077] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934975.034589951] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934975.057933417] [camera]: RealSense Node Is Up!
[WARN] [1760934975.065670628] [camera]: 
[INFO] [1760934991.261126346] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934991.756072531] [camera]: Close Sensor. 
[INFO] [1760934991.756324235] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:36:32 alpha@opi t265_ws → # see the topics
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'

# confirm pose is streaming
ros2 topic echo -n 1 /camera/pose/sample
ros2 topic hz /camera/pose/sample
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
WARNING: topic [/camera/pose/sample] does not appear to be published yet
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760935074.383173192] [camera]: RealSense ROS v4.51.1
[INFO] [1760935074.383377356] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935074.383396897] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935074.456934548] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935074.457104587] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935074.457124712] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935074.457467415] [camera]: Device with port number 6-1 was found.
[INFO] [1760935074.457503289] [camera]: Device USB type: 3.1
[INFO] [1760935074.467354470] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935074.467451301] [camera]: getParameters...
[INFO] [1760935074.467680840] [camera]: JSON file is not provided
[INFO] [1760935074.467707089] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935074.467728380] [camera]: Device Serial No: 908412110035
[INFO] [1760935074.467748797] [camera]: Device physical port: 6-1-2
[INFO] [1760935074.467767755] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935074.467787880] [camera]: Device Product ID: 0x0B37
[INFO] [1760935074.467805963] [camera]: Sync Mode: Off
[INFO] [1760935074.517358194] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935074.524492833] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935074.528233108] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935074.528318274] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935074.528341023] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935074.528359690] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935074.552003281] [camera]: RealSense Node Is Up!
[WARN] [1760935074.562638741] [camera]: 
[INFO] [1760935157.492823617] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935157.982522974] [camera]: Close Sensor. 
[INFO] [1760935157.982804428] [camera]: Close Sensor - Done. 
12:39:18 alpha@opi t265_ws → # (realsense-viewer closed)
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760935162.434496648] [camera]: RealSense ROS v4.51.1
[INFO] [1760935162.434708103] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935162.434728811] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935162.494593934] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935162.494753474] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935162.494777390] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935162.495115426] [camera]: Device with port number 6-1 was found.
[INFO] [1760935162.495152467] [camera]: Device USB type: 3.1
[INFO] [1760935162.505502975] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935162.505599515] [camera]: getParameters...
[INFO] [1760935162.505860553] [camera]: JSON file is not provided
[INFO] [1760935162.505896428] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935162.505921510] [camera]: Device Serial No: 908412110035
[INFO] [1760935162.505945718] [camera]: Device physical port: 6-1-2
[INFO] [1760935162.505969343] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935162.505992093] [camera]: Device Product ID: 0x0B37
[INFO] [1760935162.506011051] [camera]: Sync Mode: Off
[INFO] [1760935162.558326206] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935162.567514149] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935162.571551920] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935162.571652544] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935162.571673835] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935162.571693376] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935162.595201057] [camera]: RealSense Node Is Up!
[WARN] [1760935162.601663417] [camera]: 
[INFO] [1760935237.774146010] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935238.281780618] [camera]: Close Sensor. 
[INFO] [1760935238.282085405] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:40:38 alpha@opi t265_ws → pkill -f realsense-viewer || true
12:40:41 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935250.391354660] [camera]: RealSense ROS v4.51.1
[INFO] [1760935250.391556782] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935250.391577490] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935250.463490314] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935250.463645769] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935250.463672602] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935250.464337009] [camera]: Device with port number 6-1 was found.
[INFO] [1760935250.464430049] [camera]: Device USB type: 3.1
[INFO] [1760935250.475097595] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935250.475199677] [camera]: getParameters...
[INFO] [1760935250.475445840] [camera]: JSON file is not provided
[INFO] [1760935250.475482006] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935250.475497464] [camera]: Device Serial No: 908412110035
[INFO] [1760935250.475514964] [camera]: Device physical port: 6-1-2
[INFO] [1760935250.475532172] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935250.475549380] [camera]: Device Product ID: 0x0B37
[INFO] [1760935250.475563380] [camera]: Sync Mode: Off
[WARN] [1760935250.476078456] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935250.527147346] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935250.534601066] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935250.538329384] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935250.538421841] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935250.538443133] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935250.538465591] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935250.561876443] [camera]: RealSense Node Is Up!
[WARN] [1760935250.570527728] [camera]: 
pkill -f realsense-viewer || true
[INFO] [1760935349.644436211] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935350.124920056] [camera]: Close Sensor. 
[INFO] [1760935350.125223093] [camera]: Close Sensor - Done. 
12:42:30 alpha@opi t265_ws → pkill -f realsense-viewer || true
12:42:35 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935361.965760076] [camera]: RealSense ROS v4.51.1
[INFO] [1760935361.965948198] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935361.965969781] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935362.033975799] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935362.034143213] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935362.034164213] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935362.034508374] [camera]: Device with port number 6-1 was found.
[INFO] [1760935362.034537249] [camera]: Device USB type: 3.1
[INFO] [1760935362.044824469] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935362.044921009] [camera]: getParameters...
[INFO] [1760935362.045151423] [camera]: JSON file is not provided
[INFO] [1760935362.045200714] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935362.045223172] [camera]: Device Serial No: 908412110035
[INFO] [1760935362.045242713] [camera]: Device physical port: 6-1-2
[INFO] [1760935362.045261379] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935362.045280046] [camera]: Device Product ID: 0x0B37
[INFO] [1760935362.045298420] [camera]: Sync Mode: Off
[WARN] [1760935362.045842662] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935362.102351937] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935362.110771060] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935362.114980913] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935362.115073662] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935362.115107495] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935362.115131703] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935362.138579892] [camera]: RealSense Node Is Up!
[WARN] [1760935362.140583612] [camera]: 
  --ros-args
-p device_type:=t265
-p tracking_module.frames_queue_size:=16
--params-file ~/t265.yaml
[INFO] [1760935790.282803340] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935790.763740896] [camera]: Close Sensor. 
[INFO] [1760935790.764017100] [camera]: Close Sensor - Done. 
12:49:51 alpha@opi t265_ws → # list what exists (no /camera prefix)
ros2 topic list | egrep '^/(accel|gyro|fisheye|pose|odom|tf)'

# echo IMU with matching QoS
ros2 topic echo /gyro/sample  \
  --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
ros2 topic echo /accel/sample \
  --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
WARNING: topic [/gyro/sample] does not appear to be published yet
Could not determine the type for the passed topic
WARNING: topic [/accel/sample] does not appear to be published yet
Could not determine the type for the passed topic
12:51:29 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935922.550604222] [camera]: RealSense ROS v4.51.1
[INFO] [1760935922.550799344] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935922.550822677] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935922.619154219] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935922.619300633] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935922.619323966] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935922.619664920] [camera]: Device with port number 6-1 was found.
[INFO] [1760935922.619698169] [camera]: Device USB type: 3.1
[INFO] [1760935922.630093017] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935922.630187516] [camera]: getParameters...
[INFO] [1760935922.630482095] [camera]: JSON file is not provided
[INFO] [1760935922.630522344] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935922.630542761] [camera]: Device Serial No: 908412110035
[INFO] [1760935922.630562302] [camera]: Device physical port: 6-1-2
[INFO] [1760935922.630580677] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935922.630599343] [camera]: Device Product ID: 0x0B37
[INFO] [1760935922.630616843] [camera]: Sync Mode: Off
[WARN] [1760935922.631133669] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935922.682548208] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935922.690808378] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935922.694459408] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935922.694549532] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935922.694572573] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935922.694593281] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935922.717899440] [camera]: RealSense Node Is Up!
[WARN] [1760935922.726386233] [camera]: 
[INFO] [1760937130.590209568] [camera]: Stop Sensor: Tracking Module
[INFO] [1760937131.099256052] [camera]: Close Sensor. 
[INFO] [1760937131.099578047] [camera]: Close Sensor - Done. 
01:12:11 alpha@opi t265_ws → sudo apt install -y python3-tf-transformations python3-numpy
[sudo] password for alpha: 
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
E: Unable to locate package python3-tf-transformations
01:12:18 alpha@opi t265_ws → ros2 topic list | egrep '/camera/(odom|pose)/sample'
ros2 topic echo -n 1 /camera/odom/sample \
  --qos-reliability reliable --qos-durability volatile || \
ros2 topic echo -n 1 /camera/pose/sample \
  --qos-reliability reliable --qos-durability volatile
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
01:12:31 alpha@opi t265_ws → ros2 topic list | egrep '/camera/(odom|pose)/sample'
01:13:17 alpha@opi t265_ws → ros2 topic list | egrep '/(odom|pose)/sample$'
01:13:58 alpha@opi t265_ws → 
# Make sure the unversioned .so symlinks point at 2.50.x
sudo ln -sf librealsense2.so.2.50 /usr/local/lib/librealsense2.so
sudo ln -sf librealsense2-gl.so.2.50 /usr/local/lib/librealsense2-gl.so

# Refresh the linker cache and verify ONLY 2.50 remains
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect: only /usr/local entries for 2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        24 Oct 20 00:13 /usr/local/lib/librealsense2-gl.so -> librealsense2-gl.so.2.50
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        26 Oct 17 21:46 /usr/local/lib/librealsense2-gl.so.2.50 -> librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
-rw-r--r-- 1 root root  28070760 Oct 19 21:52 /usr/local/lib/librealsense2-gl.so.2.50.0
lrwxrwxrwx 1 root root        21 Oct 20 00:13 /usr/local/lib/librealsense2.so -> librealsense2.so.2.50
lrwxrwxrwx 1 root root        23 Oct 17 21:46 /usr/local/lib/librealsense2.so.2.50 -> librealsense2.so.2.50.0
-rw-r--r-- 1 root root 169577072 Oct 19 23:41 /usr/local/lib/librealsense2.so.2.50.0
lrwxrwxrwx 1 root root        23 Oct 20 00:13 /usr/local/lib/librealsense2.so.2.56 -> librealsense2.so.2.56.4
-rw-r--r-- 1 root root  12950560 Oct 19 22:38 /usr/local/lib/librealsense2.so.2.56.4
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:13:50 alpha@opi t265_ws → # Stop anything that might have the device open
pkill -f realsense-viewer || true
pkill -f realsense2_camera_node || true

# Clean & rebuild your workspace (you already checked out tag 4.51.1)
cd ~/t265_ws
rm -rf build/ install/ log/
colcon build --merge-install \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    -DCMAKE_BUILD_TYPE=Release
[0.613s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [2.16s]
--- stderr: realsense2_camera                          
CMake Warning at CMakeLists.txt:104 (find_package):
  Could not find a configuration file for package "realsense2" that is
  compatible with requested version "2.51.1".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/realsense2/realsense2Config.cmake, version: 2.50.0



CMake Error at CMakeLists.txt:106 (message):
  



   Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases

  



---
Failed   <<< realsense2_camera [2.94s, exited with code 1]

Summary: 2 packages finished [15.5s]
  1 package failed: realsense2_camera
  3 packages had stderr output: realsense2_camera realsense2_camera_msgs realsense2_description
12:14:15 alpha@opi t265_ws → source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Sanity: the node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# Launch
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/t265.yaml
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
12:14:29 alpha@opi t265_ws → 12:11:21 alpha@opi t265_ws → # You had these on hold; unhold first:
sudo apt-mark unhold librealsense2 librealsense2-gl librealsense2-utils librealsense2-udev-rules || true

# Purge the libraries (keep udev rules if you like; it’s just rules, no libs)
sudo apt purge -y --allow-change-held-packages librealsense2 librealsense2-gl librealsense2-utils ros-humble-librealsense2
Canceled hold on librealsense2.
Canceled hold on librealsense2-gl.
Canceled hold on librealsense2-utils.
Canceled hold on librealsense2-udev-rules.
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following packages were automatically installed and are no longer required:
  libatk-bridge2.0-dev libatk1.0-dev libatspi2.0-dev libcairo2-dev libdatrie-dev libepoxy-dev libfribidi-dev libgdk-pixbuf-2.0-dev libgraphite2-dev libgtk-3-dev libharfbuzz-dev
  libharfbuzz-gobject0 libmbim-glib4 libmbim-proxy libpango1.0-dev libpixman-1-dev libqmi-glib5 libqmi-proxy libthai-dev libxcomposite-dev libxdamage-dev libxtst-dev pango1.0-tools
  wayland-protocols
Use 'sudo apt autoremove' to remove them.
The following packages will be REMOVED:
  librealsense2* librealsense2-gl* librealsense2-utils* ros-humble-librealsense2*
0 upgraded, 0 newly installed, 4 to remove and 0 not upgraded.
After this operation, 71.5 MB disk space will be freed.
(Reading database ... 260278 files and directories currently installed.)
Removing librealsense2-utils:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2-gl:arm64 (2.56.5-0~realsense.6682) ...
Removing librealsense2:arm64 (2.56.5-0~realsense.6682) ...
Removing ros-humble-librealsense2 (2.56.4-1jammy.20250722.184445) ...
Processing triggers for libc-bin (1:2.35-0ubuntu3.4+2widevine) ...
12:14:29 alpha@opi t265_ws → ot found2_camera_node: No such file or directoryt265.yaml\cription/librealsense/releasese):(find_package):TH doesn't contain any 'local_setup.*' files.
12:11:21: command not found
librealsense2 was already not on hold.
librealsense2-gl was already not on hold.
librealsense2-utils was already not on hold.
librealsense2-udev-rules was already not on hold.
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Package 'librealsense2-utils' is not installed, so not removed
Package 'librealsense2' is not installed, so not removed
Package 'librealsense2-gl' is not installed, so not removed
Package 'ros-humble-librealsense2' is not installed, so not removed
The following packages were automatically installed and are no longer required:
  libatk-bridge2.0-dev libatk1.0-dev libatspi2.0-dev libcairo2-dev libdatrie-dev libepoxy-dev libfribidi-dev libgdk-pixbuf-2.0-dev libgraphite2-dev libgtk-3-dev libharfbuzz-dev
  libharfbuzz-gobject0 libmbim-glib4 libmbim-proxy libpango1.0-dev libpixman-1-dev libqmi-glib5 libqmi-proxy libthai-dev libxcomposite-dev libxdamage-dev libxtst-dev pango1.0-tools
  wayland-protocols
Use 'sudo apt autoremove' to remove them.
0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
Canceled: command not found
Canceled: command not found
Canceled: command not found
Canceled: command not found
Reading: command not found
Building: command not found
Reading: command not found
Command 'The' not found, did you mean:
  command 'he' from deb node-he (1.2.0-3)
  command 'the' from deb the (3.3~rc1-3build1)
Try: sudo apt install <deb name>
libatk-bridge2.0-dev: command not found
libharfbuzz-gobject0: command not found
wayland-protocols: command not found
Command 'Use' not found, did you mean:
  command 'ase' from deb ase (3.22.1-1ubuntu1)
  command 'nse' from deb ns2 (2.35+dfsg-3.1)
Try: sudo apt install <deb name>
Command 'The' not found, did you mean:
  command 'the' from deb the (3.3~rc1-3build1)
  command 'he' from deb node-he (1.2.0-3)
Try: sudo apt install <deb name>
librealsense2*: command not found
0: command not found
After: command not found
Reading: command not found
bash: syntax error near unexpected token `('
#1760937557
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `('
#1760937557
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `('
#1760937557
bash: librealsense2-gl.so.2.50: cannot overwrite existing file
#1760937558
bash: librealsense2-gl.so.2.50.0: cannot overwrite existing file
#1760937559
lrwxrwxrwx: command not found
#1760937559
-rw-r--r--: command not found
#1760937560
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
#1760937560
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
12:13:09: command not found
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
[INFO]: command not found
[INFO]: command not found
[INFO]: command not found
[WARN]: command not found
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2): substitution failed
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
bash: librealsense2-gl.so.2.50: cannot overwrite existing file
bash: librealsense2-gl.so.2.50: cannot overwrite existing file
bash: librealsense2-gl.so.2.50.0: cannot overwrite existing file
bash: librealsense2-gl.so.2.50.0: cannot overwrite existing file
-rw-r--r--: command not found
-rw-r--r--: command not found
bash: librealsense2.so.2.50: cannot overwrite existing file
bash: librealsense2.so.2.50.0: cannot overwrite existing file
-rw-r--r--: command not found
bash: librealsense2.so.2.56.4: cannot overwrite existing file
-rw-r--r--: command not found
bash: syntax error near unexpected token `libc6,AArch64'
#1760937560
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
bash: syntax error near unexpected token `libc6,AArch64'
#1760937560
bash: syntax error near unexpected token `libc6,AArch64'
12:13:50: command not found
[0.508s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [11.8s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [1.89s]
--- stderr: realsense2_camera                          
CMake Warning at CMakeLists.txt:104 (find_package):
  Could not find a configuration file for package "realsense2" that is
  compatible with requested version "2.51.1".

  The following configuration files were considered but not accepted:

    /usr/local/lib/cmake/realsense2/realsense2Config.cmake, version: 2.50.0



CMake Error at CMakeLists.txt:106 (message):
  



   Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases

  



---
Failed   <<< realsense2_camera [2.89s, exited with code 1]
                                 
Summary: 2 packages finished [15.2s]
  1 package failed: realsense2_camera
  3 packages had stderr output: realsense2_camera realsense2_camera_msgs realsense2_description
[0.613s]: command not found
bash: syntax error near unexpected token `>'
---: command not found
bash: syntax error near unexpected token `('
#1760937563
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `include'
bash: syntax error near unexpected token `ament_execute_extensions'
#1760937564
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
bash: syntax error near unexpected token `('
#1760937567
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `find_package'
bash: syntax error near unexpected token `include'
#1760937567
bash: syntax error near unexpected token `ament_execute_extensions'
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
bash: syntax error near unexpected token `('
#1760937567
Policy: command not found
#1760937567
Command 'are' not found, but there are 17 similar ones.
#1760937568
Command 'the' not found, but can be installed with:
#1760937568
sudo apt install the
#1760937570
bash: syntax error near unexpected token `('
bash: syntax error near unexpected token `find_package'
bash: syntax error near unexpected token `include'
#1760937570
bash: syntax error near unexpected token `ament_execute_extensions'
bash: syntax error near unexpected token `rosidl_generate_interfaces'
This: command not found
CMake: command not found
Manually-specified: command not found
realsense2_DIR: command not found
---: command not found
Finished: command not found
bash: syntax error near unexpected token `>'
#1760937571
bash: syntax error near unexpected token `>'
---: command not found
CMake: command not found
Manually-specified: command not found
realsense2_DIR: command not found
---: command not found
Finished: command not found
---: command not found
bash: syntax error near unexpected token `('
#1760937571
Could: command not found
#1760937571
compatible: command not found
#1760937571
Command 'The' not found, did you mean:
#1760937571
Try: sudo apt install <deb name>
#1760937571
bash: /usr/local/lib/cmake/realsense2/realsense2Config.cmake,: No such file or directory
#1760937572
bash: syntax error near unexpected token `('
Intel: command not found
---: command not found
Failed: command not found
Summary:: command not found
bash: cd: too many arguments
bash: cd: -3: invalid option
cd: usage: cd [-L|[-P [-e]] [-@]] [dir]
12:14:15: command not found
Package not found
ldd: /lib/realsense2_camera/realsense2_camera_node: No such file or directory
Package 'realsense2_camera' not found
Command 'Package' not found, did you mean:
  command 'package' from deb mailagent (1:3.1-106-1)
Try: sudo apt install <deb name>
Command 'ldd:' not found, did you mean:
  command 'ldd' from deb libc-bin (2.35-0ubuntu3.11)
Try: sudo apt install <deb name>
Command 'Package' not found, did you mean:
  command 'package' from deb mailagent (1:3.1-106-1)
Try: sudo apt install <deb name>
12:14:29: command not found
12:16:20 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
sudo ldconfig
ldconfig -p | grep -i librealsense
# Expect ONLY 2.50 entries (plus the -gl 2.50 you already have)
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
        librealsense2.so.2.56 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.56
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
bash: :s^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*: substitution failed
12:16:23 alpha@opi t265_ws → ^C[INFO] [1760933622.336352480] [rclcpp]: signal_handler(signum=2)sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/lib^C
12:16:55 alpha@opi t265_ws → ^C
12:16:55 alpha@opi t265_ws → sudo rm -f /usr/local/lib/librealsense2.so.2.56 /usr/local/lib/librealsense2.so.2.56.*
12:16:58 alpha@opi t265_ws → sudo ldconfig
12:17:05 alpha@opi t265_ws → ldconfig -p | grep -i librealsense
        librealsense2.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2.so.2.50
        librealsense2.so (libc6,AArch64) => /usr/local/lib/librealsense2.so
        librealsense2-gl.so.2.50 (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so.2.50
        librealsense2-gl.so (libc6,AArch64) => /usr/local/lib/librealsense2-gl.so
12:17:12 alpha@opi t265_ws → cd ~/t265_ws/src/realsense-ros
git fetch --tags

# Find the newest 4.50.* or 4.49.* tag that asks for <= 2.50.0
CANDIDATE=$(
  for t in $(git tag -l '4.50.*' '4.49.*' --sort=-v:refname); do
    min=$(git show "$t:CMakeLists.txt" | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$min" ] || continue
    if dpkg --compare-versions "$min" le 2.50.0; then echo "$t"; break; fi
  done
)
echo "Checking out $CANDIDATE"
git checkout "$CANDIDATE"
Checking out 
fatal: empty string is not a valid pathspec. please use . instead if you meant to match all paths
12:17:54 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
git fetch --tags

# List nearby tags
git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname | head -n 15

# Auto-pick the newest tag that requires <= 2.50.0
CAND=$(
  for t in $(git tag -l '4.50.*' '4.49.*' '4.48.*' --sort=-v:refname); do
    need=$(git show "$t:realsense2_camera/CMakeLists.txt" 2>/dev/null \
            | sed -n 's/.*find_package(realsense2 \([0-9.]\+\).*/\1/p' | head -n1)
    [ -n "$need" ] || continue
    if dpkg --compare-versions "$need" le 2.50.0; then echo "$t"; break; fi
  done
)
echo "Checking out $CAND"; git checkout "$CAND"
Checking out 
fatal: empty string is not a valid pathspec. please use . instead if you meant to match all paths
12:18:52 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
git fetch --tags
git tag -l '4.5*' --sort=-v:refname | head -n 40
# Try these, in order; the first that exists will do:
for t in 4.50.1 4.50.0 4.49.1 4.49.0 4.48.1 4.48.0; do
  git checkout $t && break
done
4.57.3
4.57.2
4.56.4
4.56.3
4.56.1
4.55.1-RC
4.55.1
4.54.1
4.51.1
error: pathspec '4.50.1' did not match any file(s) known to git
error: pathspec '4.50.0' did not match any file(s) known to git
error: pathspec '4.49.1' did not match any file(s) known to git
error: pathspec '4.49.0' did not match any file(s) known to git
error: pathspec '4.48.1' did not match any file(s) known to git
error: pathspec '4.48.0' did not match any file(s) known to git
12:20:04 alpha@opi realsense-ros ±|✔|→ cd ~/t265_ws/src/realsense-ros
grep -n "find_package(realsense2" realsense2_camera/CMakeLists.txt
# Replace whatever version it asks for (e.g. 2.51.1) with 2.50.0:
sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/' \
  realsense2_camera/CMakeLists.txt
96:find_package(realsense2_camera_msgs REQUIRED)
104:find_package(realsense2 2.51.1)
12:20:16 alpha@opi realsense-ros ±| ✗|→ sed -i -E 's/find_package\(realsense2[[:space:]]+[0-9.]+\)/find_package(realsense2 2.50.0)/' \
  realsense2_camera/CMakeLists.txt
git --no-pager diff -- realsense2_camera/CMakeLists.txt | sed -n '1,120p'
diff --git a/realsense2_camera/CMakeLists.txt b/realsense2_camera/CMakeLists.txt
index 84c9e5bd..4dcd142f 100644
--- a/realsense2_camera/CMakeLists.txt
+++ b/realsense2_camera/CMakeLists.txt
@@ -101,7 +101,7 @@ find_package(tf2_ros REQUIRED)
 find_package(tf2 REQUIRED)
 find_package(diagnostic_updater REQUIRED)
 
-find_package(realsense2 2.51.1)
+find_package(realsense2 2.50.0)
 if(NOT realsense2_FOUND)
     message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
 endif()
12:21:43 alpha@opi realsense-ros ±| ✗|→ cd ~/t265_ws
rm -rf build/ install/ log/

colcon build --merge-install \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    -DCMAKE_BUILD_TYPE=Release
[0.510s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/alpha/t265_ws/install' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.
Starting >>> realsense2_camera_msgs
--- stderr: realsense2_camera_msgs                                
CMake Warning (dev) at /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:20 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:52 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning (dev) at /opt/ros/humble/share/python_cmake_module/cmake/Modules/FindPythonExtra.cmake:140 (find_package):
  Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules
  are removed.  Run "cmake --help-policy CMP0148" for policy details.  Use
  the cmake_policy command to set the policy and suppress this warning.

Call Stack (most recent call first):
  /opt/ros/humble/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:23 (find_package)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:26 (rosidl_generate_interfaces)
This warning is for project developers.  Use -Wno-dev to suppress it.

CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_camera_msgs [12.1s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
--- stderr: realsense2_description                                                                         
CMake Warning:
  Manually-specified variables were not used by the project:

    realsense2_DIR


---
Finished <<< realsense2_description [1.93s]
[Processing: realsense2_camera]                             
Finished <<< realsense2_camera [52.9s]                           

Summary: 3 packages finished [1min 5s]
  2 packages had stderr output: realsense2_camera_msgs realsense2_description
12:24:05 alpha@opi t265_ws → # env
source ~/t265_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# sanity: node must link to /usr/local … 2.50
ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node \
  | grep -i 'librealsense2\.so'

# nothing else should be holding the camera
pkill -f realsense-viewer || true

# launch with a T265 filter + your params
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args -p device_type:=t265 --params-file ~/t265.yaml
[INFO] [1760934261.147517502] [camera]: RealSense ROS v4.51.1
[INFO] [1760934261.147736831] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934261.147761330] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934261.193490552] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934261.193657966] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934261.193679548] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934261.194019917] [camera]: Device with port number 6-1 was found.
[INFO] [1760934261.194059874] [camera]: Device USB type: 3.1
[INFO] [1760934261.210363136] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934261.210463759] [camera]: getParameters...
[INFO] [1760934261.210688629] [camera]: JSON file is not provided
[INFO] [1760934261.210731212] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934261.210758628] [camera]: Device Serial No: 908412110035
[INFO] [1760934261.210785752] [camera]: Device physical port: 6-1-2
[INFO] [1760934261.210817543] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934261.210847876] [camera]: Device Product ID: 0x0B37
[INFO] [1760934261.210879376] [camera]: Sync Mode: Off
[WARN] [1760934261.214857922] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760934261.271479221] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934261.297844200] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934261.301729415] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934261.301851038] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934261.301889537] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934261.301959536] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934261.325573861] [camera]: RealSense Node Is Up!
[WARN] [1760934261.326608091] [camera]: 
[INFO] [1760934718.437839509] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934718.948396740] [camera]: Close Sensor. 
[INFO] [1760934718.948687818] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:31:59 alpha@opi t265_ws → Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]
Could: command not found
12:33:27 alpha@opi t265_ws → # ~/t265.yaml
realsense2_camera:
  ros__parameters:
    device_type: "t265"
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: "linear_interpolation"
    tracking_module:
      frames_queue_size: 16
realsense2_camera:: command not found
ros__parameters:: command not found
device_type:: command not found
enable_fisheye1:: command not found
enable_fisheye2:: command not found
enable_gyro:: command not found
enable_accel:: command not found
enable_pose:: command not found
publish_tf:: command not found
unite_imu_method:: command not found
tracking_module:: command not found
frames_queue_size:: command not found
12:33:43 alpha@opi t265_ws → cat > ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16
YAML

# sanity check
cat ~/t265.yaml

# launch
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args --params-file ~/t265.yaml
bash: /home/alpha/t265.yaml: cannot overwrite existing file
realsense2_camera:
  ros__parameters:
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: "linear_interpolation"   # common choice for IMU fusion
[INFO] [1760934887.004142223] [camera]: RealSense ROS v4.51.1
[INFO] [1760934887.004340845] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934887.004363011] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934887.047347866] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934887.047512655] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934887.047536279] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934887.047876066] [camera]: Device with port number 6-1 was found.
[INFO] [1760934887.047905523] [camera]: Device USB type: 3.1
[INFO] [1760934887.058720350] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934887.058813099] [camera]: getParameters...
[INFO] [1760934887.059103011] [camera]: JSON file is not provided
[INFO] [1760934887.059147635] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934887.059165135] [camera]: Device Serial No: 908412110035
[INFO] [1760934887.059183510] [camera]: Device physical port: 6-1-2
[INFO] [1760934887.059199843] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934887.059220551] [camera]: Device Product ID: 0x0B37
[INFO] [1760934887.059235134] [camera]: Sync Mode: Off
[WARN] [1760934887.059768000] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760934887.115480984] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934887.124204595] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934887.128312654] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934887.128405111] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934887.128425236] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934887.128446527] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934887.151678864] [camera]: RealSense Node Is Up!
[WARN] [1760934887.153468793] [camera]: 
[INFO] [1760934961.298617834] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934961.781705293] [camera]: Close Sensor. 
[INFO] [1760934961.782442032] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:36:02 alpha@opi t265_ws → cat >| ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_fisheye1: true
    enable_fisheye2: true
    enable_gyro: true
    enable_accel: true
    enable_pose: true
    publish_tf: true
    unite_imu_method: linear_interpolation
    tracking_module:
      frames_queue_size: 16   # must be <= 32
YAML
12:36:04 alpha@opi t265_ws → # close anything that might hold the camera
pkill -f realsense-viewer || true

# launch (explicitly filter to T265 + set the queue)
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760934974.888563501] [camera]: RealSense ROS v4.51.1
[INFO] [1760934974.888760956] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760934974.888782831] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760934974.959134348] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760934974.959294762] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760934974.959317803] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760934974.959658173] [camera]: Device with port number 6-1 was found.
[INFO] [1760934974.959700755] [camera]: Device USB type: 3.1
[INFO] [1760934974.970050092] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760934974.970136424] [camera]: getParameters...
[INFO] [1760934974.970370921] [camera]: JSON file is not provided
[INFO] [1760934974.970402420] [camera]: Device Name: Intel RealSense T265
[INFO] [1760934974.970423712] [camera]: Device Serial No: 908412110035
[INFO] [1760934974.970443545] [camera]: Device physical port: 6-1-2
[INFO] [1760934974.970462503] [camera]: Device FW version: 0.2.0.951
[INFO] [1760934974.970481169] [camera]: Device Product ID: 0x0B37
[INFO] [1760934974.970499252] [camera]: Sync Mode: Off
[INFO] [1760934975.022336352] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760934975.030382434] [camera]: Starting Sensor: Tracking Module
[INFO] [1760934975.034443537] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934975.034540660] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760934975.034561077] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760934975.034589951] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760934975.057933417] [camera]: RealSense Node Is Up!
[WARN] [1760934975.065670628] [camera]: 
[INFO] [1760934991.261126346] [camera]: Stop Sensor: Tracking Module
[INFO] [1760934991.756072531] [camera]: Close Sensor. 
[INFO] [1760934991.756324235] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:36:32 alpha@opi t265_ws → # see the topics
ros2 topic list | egrep '/camera/(pose|accel|gyro|fisheye)'

# confirm pose is streaming
ros2 topic echo -n 1 /camera/pose/sample
ros2 topic hz /camera/pose/sample
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
WARNING: topic [/camera/pose/sample] does not appear to be published yet
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760935074.383173192] [camera]: RealSense ROS v4.51.1
[INFO] [1760935074.383377356] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935074.383396897] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935074.456934548] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935074.457104587] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935074.457124712] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935074.457467415] [camera]: Device with port number 6-1 was found.
[INFO] [1760935074.457503289] [camera]: Device USB type: 3.1
[INFO] [1760935074.467354470] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935074.467451301] [camera]: getParameters...
[INFO] [1760935074.467680840] [camera]: JSON file is not provided
[INFO] [1760935074.467707089] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935074.467728380] [camera]: Device Serial No: 908412110035
[INFO] [1760935074.467748797] [camera]: Device physical port: 6-1-2
[INFO] [1760935074.467767755] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935074.467787880] [camera]: Device Product ID: 0x0B37
[INFO] [1760935074.467805963] [camera]: Sync Mode: Off
[INFO] [1760935074.517358194] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935074.524492833] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935074.528233108] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935074.528318274] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935074.528341023] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935074.528359690] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935074.552003281] [camera]: RealSense Node Is Up!
[WARN] [1760935074.562638741] [camera]: 
[INFO] [1760935157.492823617] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935157.982522974] [camera]: Close Sensor. 
[INFO] [1760935157.982804428] [camera]: Close Sensor - Done. 
12:39:18 alpha@opi t265_ws → # (realsense-viewer closed)
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p device_type:=t265 \
  -p tracking_module.frames_queue_size:=16 \
  --params-file ~/t265.yaml
[INFO] [1760935162.434496648] [camera]: RealSense ROS v4.51.1
[INFO] [1760935162.434708103] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935162.434728811] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935162.494593934] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935162.494753474] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935162.494777390] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935162.495115426] [camera]: Device with port number 6-1 was found.
[INFO] [1760935162.495152467] [camera]: Device USB type: 3.1
[INFO] [1760935162.505502975] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935162.505599515] [camera]: getParameters...
[INFO] [1760935162.505860553] [camera]: JSON file is not provided
[INFO] [1760935162.505896428] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935162.505921510] [camera]: Device Serial No: 908412110035
[INFO] [1760935162.505945718] [camera]: Device physical port: 6-1-2
[INFO] [1760935162.505969343] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935162.505992093] [camera]: Device Product ID: 0x0B37
[INFO] [1760935162.506011051] [camera]: Sync Mode: Off
[INFO] [1760935162.558326206] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935162.567514149] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935162.571551920] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935162.571652544] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935162.571673835] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935162.571693376] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935162.595201057] [camera]: RealSense Node Is Up!
[WARN] [1760935162.601663417] [camera]: 
[INFO] [1760935237.774146010] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935238.281780618] [camera]: Close Sensor. 
[INFO] [1760935238.282085405] [camera]: Close Sensor - Done. 
[ros2run]: Segmentation fault
12:40:38 alpha@opi t265_ws → pkill -f realsense-viewer || true
12:40:41 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935250.391354660] [camera]: RealSense ROS v4.51.1
[INFO] [1760935250.391556782] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935250.391577490] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935250.463490314] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935250.463645769] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935250.463672602] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935250.464337009] [camera]: Device with port number 6-1 was found.
[INFO] [1760935250.464430049] [camera]: Device USB type: 3.1
[INFO] [1760935250.475097595] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935250.475199677] [camera]: getParameters...
[INFO] [1760935250.475445840] [camera]: JSON file is not provided
[INFO] [1760935250.475482006] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935250.475497464] [camera]: Device Serial No: 908412110035
[INFO] [1760935250.475514964] [camera]: Device physical port: 6-1-2
[INFO] [1760935250.475532172] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935250.475549380] [camera]: Device Product ID: 0x0B37
[INFO] [1760935250.475563380] [camera]: Sync Mode: Off
[WARN] [1760935250.476078456] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935250.527147346] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935250.534601066] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935250.538329384] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935250.538421841] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935250.538443133] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935250.538465591] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935250.561876443] [camera]: RealSense Node Is Up!
[WARN] [1760935250.570527728] [camera]: 
pkill -f realsense-viewer || true
[INFO] [1760935349.644436211] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935350.124920056] [camera]: Close Sensor. 
[INFO] [1760935350.125223093] [camera]: Close Sensor - Done. 
12:42:30 alpha@opi t265_ws → pkill -f realsense-viewer || true
12:42:35 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935361.965760076] [camera]: RealSense ROS v4.51.1
[INFO] [1760935361.965948198] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935361.965969781] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935362.033975799] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935362.034143213] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935362.034164213] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935362.034508374] [camera]: Device with port number 6-1 was found.
[INFO] [1760935362.034537249] [camera]: Device USB type: 3.1
[INFO] [1760935362.044824469] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935362.044921009] [camera]: getParameters...
[INFO] [1760935362.045151423] [camera]: JSON file is not provided
[INFO] [1760935362.045200714] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935362.045223172] [camera]: Device Serial No: 908412110035
[INFO] [1760935362.045242713] [camera]: Device physical port: 6-1-2
[INFO] [1760935362.045261379] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935362.045280046] [camera]: Device Product ID: 0x0B37
[INFO] [1760935362.045298420] [camera]: Sync Mode: Off
[WARN] [1760935362.045842662] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935362.102351937] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935362.110771060] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935362.114980913] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935362.115073662] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935362.115107495] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935362.115131703] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935362.138579892] [camera]: RealSense Node Is Up!
[WARN] [1760935362.140583612] [camera]: 
  --ros-args
-p device_type:=t265
-p tracking_module.frames_queue_size:=16
--params-file ~/t265.yaml
[INFO] [1760935790.282803340] [camera]: Stop Sensor: Tracking Module
[INFO] [1760935790.763740896] [camera]: Close Sensor. 
[INFO] [1760935790.764017100] [camera]: Close Sensor - Done. 
12:49:51 alpha@opi t265_ws → # list what exists (no /camera prefix)
ros2 topic list | egrep '^/(accel|gyro|fisheye|pose|odom|tf)'

# echo IMU with matching QoS
ros2 topic echo /gyro/sample  \
  --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
ros2 topic echo /accel/sample \
  --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
WARNING: topic [/gyro/sample] does not appear to be published yet
Could not determine the type for the passed topic
WARNING: topic [/accel/sample] does not appear to be published yet
Could not determine the type for the passed topic
12:51:29 alpha@opi t265_ws → ros2 run realsense2_camera realsense2_camera_node
[INFO] [1760935922.550604222] [camera]: RealSense ROS v4.51.1
[INFO] [1760935922.550799344] [camera]: Built with LibRealSense v2.50.0
[INFO] [1760935922.550822677] [camera]: Running with LibRealSense v2.50.0
[INFO] [1760935922.619154219] [camera]: Device with serial number 908412110035 was found.

[INFO] [1760935922.619300633] [camera]: Device with physical ID 6-1-2 was found.
[INFO] [1760935922.619323966] [camera]: Device with name Intel RealSense T265 was found.
[INFO] [1760935922.619664920] [camera]: Device with port number 6-1 was found.
[INFO] [1760935922.619698169] [camera]: Device USB type: 3.1
[INFO] [1760935922.630093017] [camera]: No calib_odom_file. No input odometry accepted.
[INFO] [1760935922.630187516] [camera]: getParameters...
[INFO] [1760935922.630482095] [camera]: JSON file is not provided
[INFO] [1760935922.630522344] [camera]: Device Name: Intel RealSense T265
[INFO] [1760935922.630542761] [camera]: Device Serial No: 908412110035
[INFO] [1760935922.630562302] [camera]: Device physical port: 6-1-2
[INFO] [1760935922.630580677] [camera]: Device FW version: 0.2.0.951
[INFO] [1760935922.630599343] [camera]: Device Product ID: 0x0B37
[INFO] [1760935922.630616843] [camera]: Sync Mode: Off
[WARN] [1760935922.631133669] [camera]: Could not set param: tracking_module.frames_queue_size with 256 Range: [0, 32]: parameter 'tracking_module.frames_queue_size' could not be set: Parameter {tracking_module.frames_queue_size} doesn't comply with integer range.
[INFO] [1760935922.682548208] [camera]: Stopping Sensor: Tracking Module
[INFO] [1760935922.690808378] [camera]: Starting Sensor: Tracking Module
[INFO] [1760935922.694459408] [camera]: Open profile: stream_type: Fisheye(1), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935922.694549532] [camera]: Open profile: stream_type: Fisheye(2), Format: Y8, Width: 848, Height: 800, FPS: 30
[INFO] [1760935922.694572573] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[INFO] [1760935922.694593281] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 62
[INFO] [1760935922.717899440] [camera]: RealSense Node Is Up!
[WARN] [1760935922.726386233] [camera]: 
[INFO] [1760937130.590209568] [camera]: Stop Sensor: Tracking Module
[INFO] [1760937131.099256052] [camera]: Close Sensor. 
[INFO] [1760937131.099578047] [camera]: Close Sensor - Done. 
01:12:11 alpha@opi t265_ws → sudo apt install -y python3-tf-transformations python3-numpy
[sudo] password for alpha: 
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
E: Unable to locate package python3-tf-transformations
01:12:18 alpha@opi t265_ws → ros2 topic list | egrep '/camera/(odom|pose)/sample'
ros2 topic echo -n 1 /camera/odom/sample \
  --qos-reliability reliable --qos-durability volatile || \
ros2 topic echo -n 1 /camera/pose/sample \
  --qos-reliability reliable --qos-durability volatile
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: -n
01:12:31 alpha@opi t265_ws → ros2 topic list | egrep '/camera/(odom|pose)/sample'
01:13:17 alpha@opi t265_ws → ros2 topic list | egrep '/(odom|pose)/sample$'
01:13:58 alpha@opi t265_ws → 
#1760937622
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760937623
export ROS_DOMAIN_ID=0
#1760937623
source /opt/ros/humble/setup.bash
#1760937623
source ~/t265_ws/install/setup.bash
#1760937623
# see what's there
#1760937623
ros2 node list
#1760937624
ros2 topic list | grep -E '/(odom|pose|gyro|accel|fisheye)/'
#1760937625
# print one pose message (reliable QoS)
#1760937625
ros2 topic echo --once /camera/odom/sample --qos-reliability reliable --qos-durability volatile   || ros2 topic echo --once /camera/pose/sample --qos-reliability reliable --qos-durability volatile   || ros2 topic echo --once /odom/sample --qos-reliability reliable --qos-durability volatile   || ros2 topic echo --once /pose/sample --qos-reliability reliable --qos-durability volatile
#1760937627
# IMU (best-effort QoS)
#1760937627
ros2 topic echo /gyro/sample    --qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 10
#1760937744
ros2 topic list | grep -E '^/(camera/)?(odom|pose)/sample$'
#1760937839
ros2 topic list | grep -E '(^|/)pose($|/sample$)|(^|/)odom($|/sample$)'
#1760937936
# env (match what you use everywhere)
#1760937936
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760937936
export ROS_DOMAIN_ID=0
#1760937936
source /opt/ros/humble/setup.bash
#1760937936
source ~/t265_ws/install/setup.bash
#1760937937
# find the pose topic
#1760937937
ros2 topic list | grep -E '(^|/)pose($|/sample$)|(^|/)odom($|/sample$)'
#1760937937
ros2 topic list -t | grep -E 'nav_msgs/msg/Odometry|geometry_msgs/msg/Pose'
#1760937938
# quick peek at the camera node’s pubs
#1760937938
ros2 node info /camera | sed -n '/Publishers:/,/Service Servers:/p'
#1760937939
# print one message (no -n flag in ros2; use --once)
#1760937939
ros2 topic echo --once /pose/sample  ||  ros2 topic echo --once /odom/sample
#1760937590
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760937590
export ROS_DOMAIN_ID=0
#1760937590
source /opt/ros/humble/setup.bash
#1760937591
source ~/t265_ws/install/setup.bash
#1760937591
# if your shell has noclobber, allow overwriting the YAML once:
#1760937591
set +o noclobber 2>/dev/null || true
#1760937591
cat > ~/t265.yaml <<'YAML'
realsense2_camera:
  ros__parameters:
    device_type: t265
    enable_pose: true
    enable_gyro: true
    enable_accel: true
    publish_tf: true
    tracking_module:
      frames_queue_size: 16
YAML

#1760937591
# run the node (keep this terminal open)
#1760937591
ros2 run realsense2_camera realsense2_camera_node   --ros-args -p device_type:=t265             -p tracking_module.frames_queue_size:=16             --params-file ~/t265.yaml
#1760937977
# env (match what you use everywhere)
#1760937977
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760937977
export ROS_DOMAIN_ID=0
#1760937977
source /opt/ros/humble/setup.bash
#1760937978
source ~/t265_ws/install/setup.bash
#1760937978
# find the pose topic
#1760937978
ros2 topic list | grep -E '(^|/)pose($|/sample$)|(^|/)odom($|/sample$)'
#1760937979
ros2 topic list -t | grep -E 'nav_msgs/msg/Odometry|geometry_msgs/msg/Pose'
#1760937979
# quick peek at the camera node’s pubs
#1760937979
ros2 node info /camera | sed -n '/Publishers:/,/Service Servers:/p'
#1760937980
# print one message (no -n flag in ros2; use --once)
#1760937980
ros2 topic echo --once /pose/sample  ||  ros2 topic echo --once /odom/sample
#1760938104
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760938104
export ROS_DOMAIN_ID=0
#1760938104
source /opt/ros/humble/setup.bash
#1760938105
source ~/t265_ws/install/setup.bash
#1760938114
ros2 topic list -t | egrep 'nav_msgs/msg/Odometry|geometry_msgs/msg/PoseStamped'
#1760938128
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p enable_pose:=true   -p publish_tf:=true   -p enable_gyro:=true   -p enable_accel:=true   -p enable_fisheye1:=true   -p enable_fisheye2:=true   -p camera_name:=camera   -p tracking_module.frames_queue_size:=16
#1760938158
ros2 topic list | grep -E '^/(camera/)?(odom|pose)/sample$'
#1760938435
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760938435
export ROS_DOMAIN_ID=0
#1760938435
source /opt/ros/humble/setup.bash
#1760938435
source ~/t265_ws/install/setup.bash
#1760938436
# See exactly what topics exist (and their types)
#1760938436
ros2 topic list -t | grep -E '/camera/(odom|pose)/sample'
#1760938437
# Print one message (first that exists will work)
#1760938437
ros2 topic echo --once /camera/odom/sample  ||  ros2 topic echo --once /camera/pose/sample
#1760938439
# Optional: check rate
#1760938439
ros2 topic hz /camera/odom/sample  ||  ros2 topic hz /camera/pose/sample
#1760938613
python3 your_controller.py --ros-args -r /camera/odom/sample:=/camera/pose/sample
#1760939386
python3 servo_motor.py
#1760941066
ls /dev/ttyUSB*
#1760941066
dmesg | grep -i tty | tail
#1760941078
python3 - <<'PY'
import time
from common.lx16a import LX16A
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
for s in (150, 0):  # 150 forward, then stop
    L.motor_mode(s); R.motor_mode(s); time.sleep(1)
# reverse test (comment out if not needed)
for s in (-150, 0):
    L.motor_mode(s); R.motor_mode(s); time.sleep(1)
print("motor test done")
PY

#1760941109
python3 - <<'PY'
import time
from common.lx16a import LX16A
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
for s in (150, 0):  # 150 forward, then stop
    L.motor_mode(s); R.motor_mode(s); time.sleep(1)
# reverse test (comment out if not needed)
for s in (-150, 0):
    L.motor_mode(s); R.motor_mode(s); time.sleep(1)
print("motor test done")
PY

#1760941223
python3 - <<'PY'
import time
from common.lx16a import LX16A
LEFT_CMD_SIGN, RIGHT_CMD_SIGN = +1, -1
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
fwd = 180
L.motor_mode(LEFT_CMD_SIGN*fwd)
R.motor_mode(RIGHT_CMD_SIGN*fwd)
time.sleep(1.2)
L.motor_mode(0); R.motor_mode(0)
print("done")
PY

#1760941288
python3 - <<'PY'
import time
from common.lx16a import LX16A
LEFT_CMD_SIGN, RIGHT_CMD_SIGN = +1, -1
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
fwd = 180
L.motor_mode(LEFT_CMD_SIGN*fwd)
R.motor_mode(RIGHT_CMD_SIGN*fwd)
time.sleep(1.2)
L.motor_mode(0); R.motor_mode(0)
print("done")
PY

#1760941292
python3 - <<'PY'
import time
from common.lx16a import LX16A
LEFT_CMD_SIGN, RIGHT_CMD_SIGN = +1, -1
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
fwd = 180
L.motor_mode(LEFT_CMD_SIGN*fwd)
R.motor_mode(RIGHT_CMD_SIGN*fwd)
time.sleep(1.2)
L.motor_mode(0); R.motor_mode(0)
print("done")
PY

#1760941323
python3 - <<'PY'
import time
from common.lx16a import LX16A
LEFT_CMD_SIGN, RIGHT_CMD_SIGN = -1, +1
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
fwd = 180
L.motor_mode(LEFT_CMD_SIGN*fwd)
R.motor_mode(RIGHT_CMD_SIGN*fwd)
time.sleep(1.2)
L.motor_mode(0); R.motor_mode(0)
print("done")
PY

#1760941368
python3 - <<'PY'
import time
from common.lx16a import LX16A
LEFT_CMD_SIGN, RIGHT_CMD_SIGN = +1, -1
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
fwd = 180
L.motor_mode(LEFT_CMD_SIGN*fwd)
R.motor_mode(RIGHT_CMD_SIGN*fwd)
time.sleep(1.2)
L.motor_mode(0); R.motor_mode(0)
print("done")
PY

#1760941404
python3 - <<'PY'
import time
from common.lx16a import LX16A
LEFT_CMD_SIGN, RIGHT_CMD_SIGN = +1, -1
LX16A.initialize("/dev/ttyUSB0", 0.1)
L, R = LX16A(2), LX16A(3)
fwd = 180
L.motor_mode(LEFT_CMD_SIGN*fwd)
R.motor_mode(RIGHT_CMD_SIGN*fwd)
time.sleep(1.


#1760941409
c
#1760941446
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760941446
export ROS_DOMAIN_ID=0
#1760941446
source /opt/ros/humble/setup.bash
#1760941446
source ~/t265_ws/install/setup.bash
#1760941447
cd ~/modular_drone
#1760941447
python3 close_loop_control.py --ros-args -r /camera/odom/sample:=/camera/pose/sample
#1760941546
python3 -m pip install --user tf-transformations
#1760941897
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760941897
export ROS_DOMAIN_ID=0
#1760941897
source /opt/ros/humble/setup.bash
#1760941897
source ~/t265_ws/install/setup.bash
#1760941898
cd ~/modular_drone
#1760941898
python3 close_loop_control.py --ros-args -r /camera/odom/sample:=/camera/pose/sample
#1760942100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760942100
export ROS_DOMAIN_ID=0
#1760942100
source /opt/ros/humble/setup.bash
#1760942101
source ~/t265_ws/install/setup.bash
#1760942101
cd ~/modular_drone
#1760942101
python3 close_loop_control.py --ros-args -r /camera/odom/sample:=/camera/pose/sample
#1760942238
python3 servo_array_control.py
#1760942359
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760942366
source /opt/ros/humble/setup.bash
#1760942374
source ~/t265_ws/install/setup.bash
#1760942380
cd ~/modular_drone
#1760942385
python3 close_loop_control.py
#1760943097
echo $RMW_IMPLEMENTATION   # must print: rmw_cyclonedds_cpp
#1760943097
echo $ROS_DOMAIN_ID        # must match the camera terminal (0)
#1760943108
echo $RMW_IMPLEMENTATION 
#1760943115
echo $ROS_DOMAIN_ID
#1760943126
ros2 topic echo --once /camera/pose/sample
#1760941051
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760941051
export ROS_DOMAIN_ID=0
#1760941051
source /opt/ros/humble/setup.bash
#1760941052
source ~/t265_ws/install/setup.bash
#1760941052
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p device_type:=t265   -p tracking_module.frames_queue_size:=16   --params-file ~/t265.yaml
#1760943254
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#1760943254
export ROS_DOMAIN_ID=0
#1760943254
source /opt/ros/humble/setup.bash
#1760943254
source ~/t265_ws/install/setup.bash
#1760943255
ros2 topic list -t | grep -E '/camera/(pose|odom)/sample'
#1760943255
ros2 topic echo --once /camera/pose/sample
#1760943271
python3 close_loop_control.py   --ros-args   -r /camera/pose/sample:=/pose/sample   -r /camera/odom/sample:=/odom/sample
#1760943293
cd ~/modular_drone
#1760943293
python3 close_loop_control.py
#1760946983
0.20 0.20 0
#1760946987
python3 close_loop_control.py
#1760947416
cd ~/modular_drone
#1760947416
git init -b main        # if this errors, do: git init && git branch -M main
#1760947416
git add -A
#1760947416
git commit -m "Initial commit"
#1760947416
git remote add origin <YOUR-REMOTE-URL>  # e.g. git@github.com:user/modular_drone.git
#1760947416
git push -u origin main
#1760947500
import time
#1760947500
import numpy as np
#1760947500
from math import pi, atan2, hypot, sin, cos
#1760947501
from common.lx16a import LX16A
#1760947501
import threading
#1760947501
import rclpy
#1760947501
from rclpy.node import Node
#1760947502
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
#1760947502
from nav_msgs.msg import Odometry
#1760947502
class Motor:
#1760947505
track_width = 0.136
#1760947505
wheel_diameter = 0.077
#1760947506
wheel_radius = wheel_diameter * 0.5
#1760947506
half_track_width = track_width * 0.5
#1760947506
max_angular_speed = 4.8
#1760947506
RAD_PER_CMD = 4.8 / 1000.0
#1760947506
CMD_PER_RAD = 1.0 / RAD_PER_CMD
#1760947507
CMD_MIN, CMD_MAX = -1000, 1000
#1760947507
# keep your latest signs
#1760947507
LEFT_CMD_SIGN  = 1
#1760947507
RIGHT_CMD_SIGN = -1
#1760947507
# simple limits / gains
#1760947507
V_MAX = 0.4
#1760947508
W_MAX = 0.7
#1760947508
TH_DEADBAND = 3.0 * pi / 180.0
#1760947508
def normalize_angle(a):
#1760947508
def plan_from_keyboard():
#1760947508
def wheel_radps_to_cmd(omega):
#1760947508
def yaw_from_quat(qx, qy, qz, qw):
#1760947508
class OdomPose:
#1760947510
class OdomListener(Node):
#1760947511
RELATIVE_INPUT = True
#1760947511
def compute_wheel_radps_from_vw(v, w):
#1760947511
def send_wheels(motor: Motor, om_l, om_r, swap_lr=False, invert=False):
#1760947511
def p_go_to(motor: Motor,
#1760947514
def main():
#1760947553
pythong3 servo_array_control.py 
#1760947567
python3 servo_array_control.py
#1760978392
cd /opt/ros/humble/
#1760978394
cd share/
#1760978398
cd vrpn
#1760978401
l
#1760978411
sudo apt install ros-<rosdistro>-vrpn-mocap
#1760978421
sudo apt install ros-humble-vrpn-mocap
#1760978445
cd vrpn
#1760978449
cd ..
#1760979047
ros2 launch vrpn_mocap client.launch.yaml server:=<server_ip> port:=<port>
#1760979059
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.101
#1760979072
cd
#1760979075
gedit .bashrc
#1760979083
source /opt/ros/humble/setup.bash
#1760979089
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.101
#1760979196
ros2 topic list
#1760979202
ros2 topic type /vrpn_mocap/mod_alpha/pose
#1760980922
ifconfig
#1760981005
cd
#1760981008
nano .bash_history 
