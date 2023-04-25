#include "ros/ros.h"
#include <string>
#include <cstdlib>
#include <iostream>
#include <bits/stdc++.h>
#include <ctime>
#include <random>
#include <math.h>

#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/Kill.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

class Turtle{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
    public:
        std::string name;
        turtlesim::Pose pose;
        ros::Publisher pub;

        Turtle(std::string turtleName){
            name = turtleName;
            sub = n.subscribe("/"+name+"/pose", 100, &Turtle::updatePose, this);
            pub = n.advertise<geometry_msgs::Twist>("/"+name+"/cmd_vel", 10);
        }


        void updatePose(const turtlesim::Pose::ConstPtr& data)
        {
            this->pose = *data;
        }

        void setZeroPen()
        {
            turtlesim::SetPen srv;
            ros::ServiceClient client = n.serviceClient<turtlesim::SetPen>("/"+name+"/set_pen");
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
            ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("/spawn");
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

        void moveToEnd(const turtlesim::Pose frontTurtlePose)
        {
            float deltaX = 0.6;
            float deltaY = 0.6;

            turtlesim::TeleportAbsolute srv;
            ros::ServiceClient client = n.serviceClient<turtlesim::TeleportAbsolute>("/"+name+"/teleport_absolute");
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

// Global variables
std::vector<Turtle> turtles{};

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

void resetGame()
{
    ros::NodeHandle rg;
    ros::ServiceClient client = rg.serviceClient<std_srvs::Empty>("/reset");
    client.waitForExistence();
    std_srvs::Empty srv;
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service: '/reset'");
        
    }

    changeBackground();
}

bool isInsideArea(const turtlesim::Pose::ConstPtr& turtle1Pose,const turtlesim::Pose& turtle2Pose)
{
    float deltaX{0.5}, deltaY{0.5};
    bool xCheck{turtle1Pose->x <= turtle2Pose.x + deltaX && turtle1Pose->x >= turtle2Pose.x - deltaX};
    bool yCheck{turtle1Pose->y <= turtle2Pose.y + deltaY && turtle1Pose->y >= turtle2Pose.y - deltaY};

    if (xCheck && yCheck)
        return true;
    else
        return false;

}

void stopSnake()
{
    for (int i = 0; i < turtles.size()-1; i++)
    {
        turtles.at(i).stopTurtle();
    }
}

bool hitTheWall()
{
    std::vector<float> xSize{0.0, 11.05}, ySize{0.0, 11.05};
    

    if (turtles.at(0).pose.x == xSize.at(0) || turtles.at(0).pose.x >= xSize.at(1) || turtles.at(0).pose.y == ySize.at(0) || turtles.at(0).pose.y >= ySize.at(1))
        return true;
    else
        return false;
}

void killSnake()
{
    ros::NodeHandle nh;

    turtlesim::Kill srv;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("/kill");
    client.waitForExistence();
    
    for (int i=turtles.size()-1; i>=0; i--)
    {   
        srv.request.name = turtles.at(i).name;
        if (!client.call(srv))
            ROS_ERROR("Failed to call service: /kill on %s", srv.request.name.c_str());

        ros::Duration(0.05).sleep();

    }

}

void callback(const turtlesim::Pose::ConstPtr& msg)
{   
    
    turtles.at(0).pose = *msg;
    turtlesim::Pose::ConstPtr head_pose{msg};
    geometry_msgs::Twist newVel;


    // if (isInsideArea(head_pose, turtles.back().pose))
    // {
    //     turtles.back().moveToEnd(turtles.at(turtles.size()-2).pose);
        
    //     // Turtle newTurtle("turtle" + std::to_string(turtles.size()+1));
    //     // newTurtle.spawn();
    //     // turtles.push_back(newTurtle); 
    // }

    // if (hitTheWall())
    // {
    //     changeBackground(255, 0, 0);
    //     stopSnake();
    //     killSnake();

    //     ROS_INFO("You hit the WALL!! You LOST :(");
    //     ros::shutdown();
    // }

    if (turtles.size() > 1)
    {
        for (int i=1; i <= turtles.size()-1; i++) // only i < than
        {
            float  deltaPos = sqrt(pow(turtles.at(i-1).pose.x - turtles.at(i).pose.x, 2) + pow(turtles.at(i-1).pose.y - turtles.at(i).pose.y, 2));
            float angGoal = atan2(turtles.at(i-1).pose.y - turtles.at(i).pose.y, turtles.at(i-1).pose.x - turtles.at(i).pose.x);
            float deltaAng = atan2(sin(angGoal - turtles.at(i).pose.theta), cos(angGoal - turtles.at(i).pose.theta));

            std::cout << deltaPos << "," << angGoal << "," << deltaAng << std::endl;

            if (deltaPos < 0.7)
            {
                newVel.linear.x = 0;
                newVel.angular.z = 0;
            }
            else
            {
                newVel.linear.x = 3 * deltaPos;
                newVel.angular.z = 8 * deltaAng;
            }
            std::cout << turtles.at(i).pose << std::endl;
            std::cout << turtles.at(i-1).pose << std::endl;
            turtles.at(i).pub.publish(newVel);
        }
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
    resetGame();
    Turtle turtle1("turtle1");
    turtle1.setZeroPen();
    turtles.push_back(turtle1);

    Turtle turtle2("turtle2");
    turtle2.spawn();
    turtles.push_back(turtle2);
    
    // for (int i=3; i <= 3; i++)
    // {
    //     Turtle turtle2("turtle"+std::to_string(i));
    //     turtle2.spawn();
    //     turtles.push_back(turtle2);
    // }

    ros::Subscriber game = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10, callback);
    
    ros::spin();

}
