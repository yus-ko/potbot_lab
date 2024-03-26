# potbot_example

# Installation

はじめに https://github.com/yus-ko/potbot をビルドしてください

次に以下のコマンドで必要なパッケージをインストールしてください

```bash
cd ~/catkin_ws/src
git clone https://github.com/vstoneofficial/megarover_samples
git clone https://github.com/vstoneofficial/vs_rover_options_description
git clone https://github.com/vstoneofficial/megarover_description
git clone https://github.com/yus-ko/multiple_robots_slam
```
```bash
cd ~/catkin_ws
catkin build megarover_samples vs_rover_options_description megarover_description multi_turtlebot_gazebo
```

# 起動方法

```bash
roslaunch potbot_example multi_robot.launch
```
gazebo環境

```bash  
potbot_megarover21.launch
```
potbot起動
