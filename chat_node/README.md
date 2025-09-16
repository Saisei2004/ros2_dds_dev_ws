# chat_cli — Terminal Chat (rclcpp + Fast DDS)

## Requirements
- ROS 2 (`${ROS_DISTRO}` = `humble` / `iron` / `jazzy` など)
- dnf 系 Linux（Fedora / RHEL / CentOS Stream）

## Install (dnf)
```bash
sudo dnf install -y   ros-${ROS_DISTRO}-rclcpp   ros-${ROS_DISTRO}-std-msgs   ros-${ROS_DISTRO}-rmw-fastrtps-cpp   gcc gcc-c++ cmake make   python3-colcon-common-extensions python3-rosdep   ncurses-devel
```

## Project Layout
```
ros2_ws/
  src/
    chat_cli/
      package.xml
      CMakeLists.txt
      src/chat_node.cpp
```

## Build
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select chat_cli
source install/setup.bash
```

## Run (Fast DDS)
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=21   # 任意の“ルーム”ID
ros2 run chat_cli chat --user alice --topic chat_room --reliable --transient-local --depth 50
```

## Minimal Usage
- 文字を入力して **Enter** で送信
- `/help` でコマンド一覧
