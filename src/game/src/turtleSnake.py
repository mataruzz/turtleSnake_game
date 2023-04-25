#!/usr/bin/env python3

import rospy
import random
import math
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, SetPen, Kill
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

turtles = []        # List containing all the turtle present on board
snake_lenght = 1

class Turtle:
    def __init__(self, name):
        self.name = name
        self.pose = Pose()

        rospy.Subscriber(f"/{name}/pose", Pose, self.update_pose)
        self.pub = rospy.Publisher(f"/{name}/cmd_vel", Twist, queue_size=10)

    def update_pose(self, data):
        self.pose = data

    def set_zero_pen(self):
        rospy.wait_for_service(f"/{self.name}/set_pen")
        zero_pen = rospy.ServiceProxy(f"/{self.name}/set_pen", SetPen)
        zero_pen(0, 0, 0, 0, 1)

    def spawn(self):
        rospy.wait_for_service("/spawn")
        spawn_new_turtle = rospy.ServiceProxy("/spawn", Spawn)
        rand_x = random.uniform(0.0, 11)
        rand_y = random.uniform(0.0, 11)
        rand_theta = random.uniform(0.0, 2*math.pi)
        spawn_new_turtle(rand_x, rand_y, rand_theta, self.name)

        # We set the pose manually since the subscriber can require more time than the execution code
        self.pose.x = rand_x
        self.pose.y = rand_y
        self.pose.theta = rand_theta
        self.pose.linear_velocity = 0.0
        self.pose.angular_velocity = 0.0

        self.set_zero_pen()
        print(f"Spawn new turtle: {self.name}")

    def move_to_end(self, front_turtle_pose):
        delta_x = 0.6
        delta_y = 0.6

        rospy.wait_for_service(f"/{self.name}/teleport_absolute")
        move_turtle = rospy.ServiceProxy(f"/{self.name}/teleport_absolute", TeleportAbsolute)

        move_turtle(front_turtle_pose.x - delta_x*math.cos(front_turtle_pose.theta),
                    front_turtle_pose.y - delta_y*math.sin(front_turtle_pose.theta),
                    front_turtle_pose.theta)
    
    def stop_turtle(self):
        zero_vel_twist = Twist()
        self.pub.publish(zero_vel_twist)


def is_inside_area(turtle1_pose, turtle2_pose):
    delta_x = 0.5
    delta_y = 0.5
    check_x = turtle1_pose.x <= turtle2_pose.x + delta_x and turtle1_pose.x >= turtle2_pose.x - delta_x
    check_y = turtle1_pose.y <= turtle2_pose.y + delta_y and turtle1_pose.y >= turtle2_pose.y - delta_y
    
    if check_x and check_y:
        return True
    else:
        return False

def reset_game():
    rospy.wait_for_service('/reset')
    reset_game = rospy.ServiceProxy("/reset", Empty)
    reset_game()
    change_background()

def hit_the_wall():
    x_size = [0, 11.05]
    y_size = [0, 11.05]
    if turtles[0].pose.x == x_size[0] or turtles[0].pose.x >= x_size[1] or turtles[0].pose.y == y_size[0] or turtles[0].pose.y >= y_size[1]:
        return True
    else:
        return False

def change_background(r=69, g=86, b=255):
    rospy.wait_for_service('/clear')

    rospy.set_param('/turtlesim/background_r' , r)
    rospy.set_param('/turtlesim/background_g' , g)
    rospy.set_param('/turtlesim/background_b' , b)

    change_bkg_color = rospy.ServiceProxy("/clear", Empty)
    change_bkg_color()

def kill_snake():
    rospy.wait_for_service('/kill')
    kill = rospy.ServiceProxy("/kill", Kill)

    for i in range(len(turtles)-1, -1, -1):
        kill(turtles[i].name)
        print(f"Killed turtle: {turtles[i].name}")
        rospy.sleep(0.05)
    snake_lenght == 0

def stop_snake():
    for i in range(len(turtles)-1):
        turtles[i].stop_turtle()


# Where the actual game is located
def callback(data):
    global turtles
    global snake_lenght

    #Updating head (=turtle1) pose
    turtles[0].pose = data
    
    # Reassignment only for a clearer variable name
    head_pose = turtles[0].pose
    new_vel = Twist()

    if is_inside_area(head_pose, turtles[-1].pose):
        # Teleport the target turtle to the end of the snake when we are "close enough"
        turtles[-1].move_to_end(turtles[-2].pose)

        y = Turtle(f"turtle{snake_lenght+2}")
        y.spawn()
        turtles.append(y)

        snake_lenght += 1
        
    if hit_the_wall():
        change_background(255, 0, 0)
        stop_snake()
        kill_snake()

        rospy.loginfo("You hit the WALL! You LOST :(")
        rospy.signal_shutdown("You Lost")

    if snake_lenght > 1:  
        for i in range(1, len(turtles)-1):
            delta_pos = math.sqrt(pow(turtles[i-1].pose.x - turtles[i].pose.x ,2) + pow(turtles[i-1].pose.y - turtles[i].pose.y,2))
            ang_goal = math.atan2(turtles[i-1].pose.y - turtles[i].pose.y, turtles[i-1].pose.x - turtles[i].pose.x)
            delta_ang = math.atan2(math.sin(ang_goal - turtles[i].pose.theta), math.cos(ang_goal - turtles[i].pose.theta))

            if delta_pos < 0.7:
                new_vel.linear.x = 0
                new_vel.angular.z = 0
            else:
                kl = 3                              # linear vel proportional coefficient
                ka = 15                             # angular vel proportional coefficient
                new_vel.linear.x = kl * delta_pos
                new_vel.angular.z =  ka * delta_ang

            turtles[i].pub.publish(new_vel)


if __name__ == "__main__":
    # Initializing the node
    rospy.init_node("turtleSnake_game", anonymous = False)

    # Resetting the board as default settings
    reset_game()

    # Creating the object for the first turtle (already spawned when runned the node: "turtlesim_node")
    turtle1 = Turtle("turtle1")
    turtle1.set_zero_pen()
    turtles.append(turtle1)
    
    # Spawning a new turtle
    turtle2 = Turtle("turtle2")
    turtle2.spawn()
    turtles.append(turtle2)

    rospy.Subscriber("/turtle1/pose", Pose, callback)

    rospy.spin()
