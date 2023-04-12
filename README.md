# gtts_ros

Integration of the [gtts](https://pypi.org/project/gTTS/) within a ROS 2 node.

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/gtts_ros.git
$ pip3 install -r requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch gtts_bringup gtts.launch.py
```
