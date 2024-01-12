# NVIDIA JETSON AGX ORIN在Ubuntu20.04上部署D435i相机



## 一、安装依赖库



在安装时，建议先拔出D435i相机（但本人试过似乎没有影响），然后执行以下指令
```shell
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev pkg-config
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev
```



注册服务器公钥

```shell
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```



服务器添加到存储库列表

```shel
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u
```

其中，`focal`表示Ubuntu20.04的版本号



安装SDK

```shell
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev

# 选装
sudo apt-get install librealsense2-dbg
```



将D435i接上ORIN（一定要用USB3.\*的线，USB2.\*传输速率不足会很卡），验证安装结果

```shell
realsense-viewer
```



将Stereo Module和RGB Camera置于**on**就可以看到结果了

- Stereo Module——深度图
- RGB Module——RGB图
- Motion Module——机载IMU

<img src="./image/realsense viewer.png" style="zoom:100%;" />



## 二、安装librealsense-ros驱动



### 2.1 直接安装（方法一）



由于源码安装可能存在很多问题，因此可以采用直接安装的方法

使用JETSON写好的RealSenseSDK包

```shell
# 注：最新的包可能与固件版本不匹配，导致报错，但报错不影响运行
git clone https://github.com/jetsonhacks/installRealSenseSDK.git
# 可以指定其他较低的版本
# git clone -b v2.50.0 https://github.com/IntelRealSense/librealsense.git

cd installRealSenseSDK/
sudo chmod u+x ./installLibrealsense.sh
./installLibrealsense.sh
```



安装ROS包

```shell
sudo apt-get install ros-noetic-realsense2-camera
# 用于三维显示的库
sudo apt-get install ros-noetic-realsense2-description
```

其中，noetic是Ubuntu20.04的ROS版本号，即 `$ROS_DISTRO`



### 2.2 源码安装（方法二）



ORIN是ARM架构的，源码编译可能会出现很多问题**（不推荐）**

安装ros-noetic-rgbd-launch

```shell
sudo apt-get install librealsense2-dkms
sudo apt-get install ros-noetic-rgbd-launch
```



在ROS工作空间编译源码

```shell
# 进入工作空间
cd ~/catkin_ws/src/

# 拉取源码，如果网络有问题导致下载失败就在Windows上下载
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..

# 安装ddynamic-reconfigure
sudo apt-get install ros-noetic-ddynamic-reconfigure

# 编译源码
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```



添加bashrc环境变量

```shell
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## 三、RVIZ上验证



启动ROS相机节点

```text
roslaunch realsense2_camera rs_camera.launch
```

注：Realsense Viewer和ROS会有冲突，需要先关闭Realsense Viewer



查看ROS话题，会出现很多`/camera`的相关话题

```shell
rostopic list
```

<img src="./image/rostopic list.png" style="zoom:100%;" />



我们需要关注的两个话题是`/camera/color/image_raw`和`/camera/depth/image_rect_raw`



启动rviz，新增上述两个话题

```shell
rviz
```

注：如果无法显示或无相关topic，可能是`rs_camera.launch`文件没有配置好，可以在本文所在目录下找到配置好的文件

如果使用的是直接安装的方法，则launch文件的路径在`/opt/ros/noetic/share/realsense2_camera/launch/rs_camera.launch`

<img src="./image/rviz topic.png" style="zoom:125%;" />

<img src="./image/rviz view.png" style="zoom:150%;" />



## 四、其他问题



### 4.1 D435i发送报错信息



若在直接安装方法中直接下载最新的RealSenseSDK包，在启动ROS相机节点后，可能会出现以下无关紧要的错误信息，可能是固件和SDK版本不匹配的原因，可以尝试下载其他版本的包，但是仍然可以发布RGB和Depth话题，所以可以不理会

<img src="./image/error information.png" style="zoom:100%;" />



### 4.2 修改launch文件



有时候找不到启动文件或功能包，记得在工作空间下执行一下`.bash`脚本，否则可能会出错

```shell
cd ~/catkin_ws
source devel/setup.bash
```
