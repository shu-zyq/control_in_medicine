# 《精准医疗中的控制技术》课程论文所用程序
本学期上了《精准医疗中的控制技术》这门通识课，在这里分享出我在期末的课程论文中所使用的代码，仅用于学习和交流。
## 关于论文及创新点
在现代的精准医疗中，机械臂进行精准手术的应用越来越广泛，而其中路径规划部分的安全性是研究和应用的重点，因此本论文主要针对安全性，侧重于复杂环境，在RRT\*算法的基础上做出了改进，并提出了安全性的量化指标。<br>
因此本程序是基于ROS环境的高维机械臂路径规划算法。实验数据表明，本文所提出的基于RRT\*改进的路径规划算法在复杂环境中，在安全性和效率上均有较大的提升。
## 运行环境
Ubuntu18.04 + ROS melodic 或者 Ubuntu20.04 + ROS neotic<br>
建议使用Ubuntu20.04，如果是Ubuntu18.04则需要用conda创建python3以上版本的虚拟环境，因为在程序中使用到了深度学习的程序需要在python3中运行，而18.04终端的python版本是2.7
## 所需安装的库
moveit、ompl、cuda、pytorch等<br>
其中moveit和ompl安装源代码或者二进制形式的都行，二进制的安装方式更简单
## 程序运行前
1.将我放在github目录中的几个包都放入工作空间的src目录中<br>
2.程序中一些地方的路径需要根据自己的放置路径来改，注意在程序中的sys.path.append()函数中的路径也要同步修改<br>
3.用catkin_make编译工作空间，如果是用catkin build，可能会报错<br>
4.编译无报错，并确保将要运行的python文件的权限是可执行的，开始运行程序<br>
## 程序运行
1.加载机器人模型
```
roslaunch elfin_robot_bringup elfin3_bringup.launch
```
2.运行gazebo仿真
```
roslaunch elfin_gazebo elfin3_empty_world.launch
```
3.运行rviz仿真
```
roslaunch elfin3_moveit_config moveit_planning_execution.launch
```
4.启动elfin basic api
```
roslaunch elfin_basic_api elfin_basic_api.launch 
```
5.显示三维点云
```
rosrun obstacle publish_pointcloud.py
```
6.发布障碍物
```
rosrun obstacle stl_marker_publisher_1.py
```
7.启动RRT
```
rosrun rrt_zyq demo7.py
```

## 系统、库的安装
（如果都已经安装好了则不需要看这一部分）<br>
以下均为我在曾经安装时所作笔记，如遇其他报错欢迎交流
### UBUNTU20.04安装
1.用一个大于8G的U盘作为启动盘，下载约4.1G的系统映像，写入U盘<br>
2.设置好磁盘分区<br>
3.重启电脑，F2进入BIOS，在boot maneger里关闭secure boot，并将第一启动选择从Windows改为USB<br>
4.进入ubuntu安装界面，可选择语言为中文<br>
5.先不连WIFI<br>
6.注意别安装在了Windows所在的磁盘上<br>
7.找到自己的分区，分区设置（仅供参考）：<br>
引导分区	500MB	主分区	空间起始位置	EFI系统分区<br>
内存交换分区	10GB	主分区	空间起始位置	交换空间<br>
/根挂载点分区	剩下的全给	主分区	空间起始位置	Ext4日志文件系统<br>
安装启动引导器的设备	/dev/...(分区对应的编号)	efi	500MB
### ROS neotic安装
```
1.配置公钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
2.添加ROS软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# 或者使用中科大镜像
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
3.更新软件源
sudo apt update
4.安装ROS-neotic
sudo apt install ros-noetic-desktop-full
5.激活环境变量
source /opt/ros/noetic/setup.bash
6.将环境变量放入 ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
7.初始化rosdep
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

8.如果出现死锁问题
正在等待缓存锁：无法获得锁 /var/lib/dpkg/lock-frontend。锁正由进程 9511（unatte
网上解决办法：强制解锁
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock
我的解决办法：重启，有效
```
### OMPL和MoveIt的安装
如果是二进制安装，方法十分简单，网上可查

### NVIDIA、CUDA、CUDNN的安装
```
1.添加源
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
2.检查可安装的驱动
ubuntu-drivers devices
3.找到最适合的驱动安装，安装recommended标记的，通常也是数字版本最大的那个
sudo apt install nvidia-driver-XXX
# 也可以自动安装系统推荐那个
sudo ubuntu-drivers autoinstall
# 如果没有遇到报错，说明安装成功，此时调用nvidia-smi指令可能还是看不到显卡信息，sudo reboot重启系统后能看到
4.找到nvidia驱动版本对应的CUDA版本：
https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html
nvidia驱动版本：nvidia-smi中的Driver Version可见，我是560.35.03
然后在 https://developer.nvidia.com/cuda-toolkit-archive 中选择符合自己版本的cuda进行安装
Install Type 选择 runfile 最方便，两行命令行：
wget https://developer.download.nvidia.com/compute/cuda/12.6.3/local_installers/cuda_12.6.3_560.35.05_linux.run
sudo sh cuda_12.6.3_560.35.05_linux.run
5.进行安装
Continue继续
accept 回车
回车取消选择驱动，Install安装
出现summary时安装完成
6.在.bashrc里配置环境变量：sudo gedit ~/.bashrc
添加以下3行：
export PATH=$PATH:/usr/local/cuda/bin  
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64  
export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/cuda/lib64
刷新环境变量：source ~/.bashrc
7.查看CUDA安装情况：nvcc -V
8.去官网下载cudnn：https://developer.nvidia.com/rdp/cudnn-download
选择Local Installer for Linux x86_64 (Tar)下载
9.下载后解压进入该目录拷贝相关文件
第一步更建议直接去网上下，快一些
wget https://developer.download.nvidia.com/compute/cudnn/9.5.1/local_installers/cudnn-local-repo-ubuntu2004-9.5.1_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2004-9.5.1_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2004-9.5.1/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cudnn-cuda-12
10.查看安装版本
cat /usr/local/cuda/include/cudnn_version.h | grep CUDNN_MAJOR -A 2
11.监控GPU状态
watch -n 1 nvidia-smi
(以下为我的版本配置，是截至2024.11最新版本)
NVDIA版本：560.35.03
CUDA版本：12.6
cuDNN版本：9.5.1
```
