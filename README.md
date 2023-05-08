# TurtleSnake Game

## About The Project
Relive the nostalgia of the classic **Snake** game with a unique twist in the Turtlesim Snake Game repository. Developed using **Turtlesim**, a popular robot simulator in ROS, this project demonstrates the flexibility and fun of ROS in a game setting. Developed with the aim to increase confidence in fundamental **ROS** tools such as *Subscribers*, *Publishers*, *Services*, and *Parameters*, this project provides an engaging and fun way to learn. With the game implemented in both *Python* and *C++*, users can explore the differences between the two and hone their skills in each language. Both the scripts are located in src/game/src folder.

The game use **turtlesim_node** and **turtle_teleop_key** node to controll the snake. As soon as the game starts, a new turtle spawn, becoming the target to the snake. Once the snake reach the target, it will be moved to the tail of the snake, and a new turtle will be spawn. The game repeat undefinetly... but... stay away from the wall, or the snake will die.

<div align="center">
<img src="https://github.com/mataruzz/turtleSnake_game/blob/main/turtleSnake_game.gif"> 
  
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
