#include "ros/ros.h"
#include <string>
#include <cstdlib>
#include <iostream>
#include <bits/stdc++.h>
#include <ctime>
#include <random>
#include <math.h>
#include <typeinfo>

#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/Kill.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

class Turtle{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;

    public:
        turtlesim::Pose pose;
        std::string name;
        ros::Publisher pub;
        bool activeSub;

        Turtle(const std::string turtleName) {
            name = turtleName;
            sub = nh.subscribe("/"+name+"/pose", 100, &Turtle::updatePose , this);
            pub = nh.advertise<geometry_msgs::Twist>("/"+name+"/cmd_vel", 10);
            activeSub = false;
        }

        void updatePose(const turtlesim::Pose::ConstPtr& data)
        {
            this->pose = *data;
            activeSub = true;
        }

        void setZeroPen()
        {
            turtlesim::SetPen srv;
            ros::ServiceClient client = nh.serviceClient<turtlesim::SetPen>("/"+name+"/set_pen");
            client.waitForExistence();

            srv.request.off = 1;

            if (!client.call(srv))
            {
                ROS_ERROR("Failed to call service: /%s/set_pen", name.c_str());
            }
        }

        void spawn()
        {
            turtlesim::Spawn srv;
            ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
            client.waitForExistence();
            // Variables needed to create floats random numbers for the angle
            std::random_device rd;
            std::default_random_engine gen(rd());
            std::uniform_real_distribution<double> distribution(0.0, 2*M_PI);
            
            srv.request.x = rand()%12; //Where 12 is the sixe of the board
            srv.request.y = rand()%12;
            srv.request.theta = distribution(gen);
            srv.request.name = name;

            if (!client.call(srv))
            {
                ROS_ERROR("Failed to call service: '/spawn'");
            }
            pose.x = srv.request.x;
            pose.y = srv.request.y;
            pose.theta = srv.request.theta;
            pose.linear_velocity = 0.0;
            pose.angular_velocity = 0.0;
            
            setZeroPen();

            std::cout << "Spawn new turtle: " << name << std::endl;
        }

        void moveToEnd(turtlesim::Pose &frontTurtlePose)
        {
            float deltaX = 0.6;
            float deltaY = 0.6;

            turtlesim::TeleportAbsolute srv;
            ros::ServiceClient client = nh.serviceClient<turtlesim::TeleportAbsolute>("/"+name+"/teleport_absolute");
            client.waitForExistence();


            srv.request.x = frontTurtlePose.x - deltaX*std::cos(frontTurtlePose.theta);
            srv.request.y = frontTurtlePose.y - deltaY*std::sin(frontTurtlePose.theta);
            srv.request.theta = frontTurtlePose.theta;

            if (!client.call(srv))
            {
                ROS_ERROR("Failed to call service: /%s/teleport_absolute", name.c_str());
            }
        }

        void stopTurtle()
        {
            geometry_msgs::Twist zeroVelTwist;
            pub.publish(zeroVelTwist);
        }
};


void changeBackground(float r=69, float g=86, float b=255)
{
    ros::NodeHandle cb;
    std_srvs::Empty srv;
    
    cb.setParam("/turtlesim/background_r", r);
    cb.setParam("/turtlesim/background_g", g);
    cb.setParam("/turtlesim/background_b", b);

    ros::ServiceClient client = cb.serviceClient<std_srvs::Empty>("/clear");
    client.waitForExistence();

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service: '/clear'");
    }
}

void resetGame(ros::NodeHandle &rg)
{
    ros::ServiceClient client = rg.serviceClient<std_srvs::Empty>("/reset");
    client.waitForExistence();
    std_srvs::Empty srv;
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service: '/reset'");
        
    }

    changeBackground();
}

bool isInsideArea(turtlesim::Pose*& turtle1Pose, turtlesim::Pose*& turtle2Pose)
{
    float deltaX{0.5}, deltaY{0.5};
    bool xCheck{turtle1Pose->x <= turtle2Pose->x + deltaX && turtle1Pose->x >= turtle2Pose->x - deltaX};
    bool yCheck{turtle1Pose->y <= turtle2Pose->y + deltaY && turtle1Pose->y >= turtle2Pose->y - deltaY};

    if (xCheck && yCheck)
        return true;
    else
        return false;

}

void stopSnake(std::vector<std::unique_ptr<Turtle>> &turtles)
{
    for (int i = 0; i < turtles.size()-1; i++)
    {
        turtles.at(i)->stopTurtle();
    }
}

bool hitTheWall(turtlesim::Pose*& headPose)
{
    std::vector<float> xSize{0.0, 11.05}, ySize{0.0, 11.05};

    if (headPose->x == xSize.at(0) || headPose->x >= xSize.at(1) || headPose->y == ySize.at(0) || headPose->y >= ySize.at(1))
        return true;
    else
        return false;
}

void killSnake(std::vector<std::unique_ptr<Turtle>> &turtles)
{
    ros::NodeHandle nh;

    turtlesim::Kill srv;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("/kill");
    client.waitForExistence();
    
    for (int i=turtles.size()-1; i>=0; i--)
    {   
        srv.request.name = turtles.at(i)->name;
        if (!client.call(srv))
            ROS_ERROR("Failed to call service: /kill on %s", srv.request.name.c_str());

        ros::Duration(0.05).sleep();
    }
}

void startGame(ros::NodeHandle &nh)
{
    int snakeLength = 0;
    int maxLengthSnake = 200;
    double deltaPos;
    double angGoal;
    double deltaAng;
    geometry_msgs::Twist newVel;
    double freq{70}; // I decided to use a frequency of 70 because the subscriber subscribe with an average of 62Hz
    ros::Rate rate(freq);


    /* Vector containing all the spawned turtles. I tried to create a simple vector made of Turtle object, but I faced a problem
       where once instantiate the second object, the subscriber of the previous was dying, loosing the updation of the actual pose. */  
    std::vector<std::unique_ptr<Turtle>> turtles;

    turtles.emplace_back(std::make_unique<Turtle>("turtle1"));
    turtles.back()->setZeroPen();
    snakeLength = 1;

    turtles.emplace_back(std::make_unique<Turtle>("turtle2"));
    turtles.back()->spawn();
    // Here we do not increase snakeLength because the above turtle has been spawned, but it still not part of the snake itself
    
    // For better comprehension, saving the head pose as new name
    turtlesim::Pose* headPose{&(turtles[0]->pose)};

    while (ros::ok())
    {
        // Waiting the subscriber to be active
        while (!turtles.at(0)->activeSub)
        {
            ros::spinOnce();
        }

        turtlesim::Pose* targetTurtle{&(turtles.back()->pose)};

        if (isInsideArea(headPose, targetTurtle))
        {
            turtles[snakeLength]->moveToEnd(turtles[snakeLength-1]->pose);
            turtles.emplace_back(std::make_unique<Turtle>("turtle"+std::to_string(snakeLength+2)));
            turtles.back()->spawn();
            snakeLength += 1;
        } 

        if (hitTheWall(headPose))
        {
            changeBackground(255, 0, 0);
            stopSnake(turtles);
            killSnake(turtles);

            ROS_INFO("You hit the WALL! You LOST :(");
            ros::shutdown();
        }

        if (snakeLength > 1)
        {
            for(int i=1; i<snakeLength; i++)
            {
                deltaPos = sqrt(pow(turtles.at(i-1)->pose.x - turtles.at(i)->pose.x,2) + pow(turtles.at(i-1)->pose.y - turtles.at(i)->pose.y,2) );
                angGoal = atan2(turtles.at(i-1)->pose.y - turtles.at(i)->pose.y, turtles.at(i-1)->pose.x - turtles.at(i)->pose.x);
                deltaAng = atan2(sin(angGoal - turtles.at(i)->pose.theta), cos(angGoal - turtles.at(i)->pose.theta));

                if (deltaPos < 0.7)
                {
                    newVel.linear.x = 0;
                    newVel.angular.z = 0;
                }
                else
                {
                    int kl = 3;
                    int ka = 15;
                    newVel.linear.x = kl*deltaPos;
                    newVel.angular.z = ka*deltaAng;
                }
                turtles.at(i)->pub.publish(newVel);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }   
}


int main(int argc, char **argv)
{
    // Making rundom number always change
    srand ( time(NULL) );

    // Initializing the node
    ros::init(argc, argv, "turtleSnake_game");
    ros::NodeHandle nh;

    // Resetting the board as default settings
    resetGame(nh);
    startGame(nh);
}