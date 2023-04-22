# TurtleSnake Game

## About The Project
This project consist in the implementation of the classic **Snake** game in the new **ROS** environment. The game is based on the **turtlesim** package, with the purpose to get more confident with basic ROS tools, such as Subscribers, Publisher, Services and Parameters. It's written in *python*, the **C++** version is in the other branch.

The game use **turtlesim_node** and **turtle_teleop_key** node to controll the snake. As soon as the game start, a new turtle spawn, becoming the target to the snake. Once the snake reach the target, it will be moved to the tail of the snake, and a new turtle will be spawn. The game repeat undefinetly... but... stay away from the wall, or the snake will die.

<div align="center">
<img src="https://github.com/mataruzz/turtleSnake_game/blob/dev/turtleSnake_game.gif"> 
  
Gif speed: x3
</div>

## Installation
Since this game is based on other packages, turtlesim must be installed first
```sh
sudo apt-get install ros-$(rosversion -d)-turtlesim
```
Then, clone the turtleSnake_game repository:
```sh
cd ~/
git clone https://github.com/mataruzz/turtleSnake_game.git
```

## Run
Build the project with ***CATKIN BUILD***
```sh
cd ~/turtleSnake_game
catkin build
```
Source the environment
```sh
source devel/setup.bash
```
Launch the game
```sh
roslaunch game start_game.launch
```
