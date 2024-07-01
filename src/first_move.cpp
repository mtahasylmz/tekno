#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <mutex>

#include <fstream>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <algorithm>
#include <tf/tf.h>


#define WHEEL_SEPERATION 12345
#define WHEEL_DIAMETER 54321
#define MAX_LINEAR_VELOCITY 1
#define MIN_LINEAR_VELOCITY -1
#define MAX_ANGULAR_VELOCITY 1
#define MIN_ANGULAR_VELOCITY -1

double left_wheel_velocity, right_wheel_velocity;
geometry_msgs::Twist command_vel;
std::mutex vel_mutex;


char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
    return (buf);
}

void keyboardLoop() {
    char ch;
    while (ros::ok()) {
        ch = getch();  // Get the pressed key
        std::lock_guard<std::mutex> lock(vel_mutex);  // Lock the mutex to modify command_vel safely
        if (ch == 27) {  // ESC key (27) to terminate
            break;
        } else if (ch == 'w') {  // Forward
            command_vel.linear.x = 1;
            command_vel.angular.z = 0;
        } else if (ch == 's') {  // Backward
            command_vel.linear.x = -1;
            command_vel.angular.z = 0;
        } else if (ch == 'a') {  // Turn left
            command_vel.linear.x = 0;
            command_vel.angular.z = 1;
        } else if (ch == 'd') {  // Turn right
            command_vel.linear.x = 0;
            command_vel.angular.z = -1;
        } else if (ch == ' ') {  // Stop
            command_vel.linear.x = 0;
            command_vel.angular.z = 0;
        }
    }
}



void LinkStatesCb(const gazebo_msgs::LinkStates::ConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot::left_wheel_link") {
            left_wheel_velocity = msg->twist[i].angular.y;
        } else if (msg->name[i] == "robot::right_wheel_link") {
            right_wheel_velocity = msg->twist[i].angular.y;
        }
    }
}

geometry_msgs::Point robot_position;
geometry_msgs::Quaternion quat;
double roll, pitch, yaw;
void ModelStatesCb(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot") {
            robot_position = msg->pose[i].position;
            quat = msg->pose[i].orientation;
        
            tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        
        }
    }
}

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::ostringstream ss;
    for (const auto& range : msg->ranges) {
        ss << range << " ";
    }
    ROS_INFO("I heard: [%s]", ss.str().c_str());
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "random_kinematics");
    ros::NodeHandle nh;

    ros::Publisher command_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber link_states_subscriber = nh.subscribe("/gazebo/link_states", 1, LinkStatesCb);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, ModelStatesCb);
    ros::Subscriber lidar_subscriber = nh.subscribe("/my_robot/laser/scan", 10, chatterCallback);
    
    ros::Rate control_rate(100);

    ros::Duration(1).sleep();

    {
        std::ifstream task_file(ros::package::getPath("tekno") + "/config/robot_task.txt");
        std::string line;
        double number;
        ROS_INFO("Opening task file,");
        while (getline(task_file, line)) {
            std::istringstream sin(line);
            while (sin >> number) {
                std::cout << number << " ";
            }
            std::cout << "\n";
        }
        task_file.close();
    }

    std::srand(static_cast<unsigned int>(time(nullptr))); // Random number seed

    std::thread keyboard_thread(keyboardLoop);  // Start the keyboard input thread

    geometry_msgs::Twist command_vel;
    
    int counter = 0;

    while (ros::ok()) {
        // if (counter >= 200) {
        //     command_vel.angular.z = 1;
        // }
        // if (counter > 278.5 && counter < 288) {
        //     command_vel.angular.z = -1;

        // }
        // if (counter > 288) {
        //     command_vel.angular.z = 0;
        // }
        // if (counter > 600) {
        //     command_vel.linear.x = 1;
        // }

        // command_velocity_publisher.publish(command_vel);
        

        
       
        // double radian = (counter * M_PI) / 180.0;
        // double hiz = sin(radian);

        // command_vel.linear.x = 1;
        // command_vel.angular.z = 0;

        
        // double aci =  atan((  5- robot_position.y) / ( 7 - robot_position.x))  ;
        

        // if ( aci < yaw + 0.1 ){
        //     command_vel.angular.z = 0.5 ;
        //     command_vel.linear.x = 0;
        //     ROS_INFO("aci: %f, %f", aci, yaw);   
        
        //     ROS_INFO("Current robot position: %f, %f, %f", robot_position.x, robot_position.y, robot_position.z);
        //     ROS_INFO("Current left-right wheel angular velocities: %f, %f", left_wheel_velocity, right_wheel_velocity);
        //     ROS_INFO("New velocity targets: %f, %f", command_vel.linear.x, command_vel.angular.z);
        //     ROS_INFO("NEW angular things: %f, %f, %f", roll, pitch, yaw);
            
        // }   
                
        // command_velocity_publisher.publish(command_vel);                        
        // counter ++ ;
        
        {
            std::lock_guard<std::mutex> lock(vel_mutex);  // Lock the mutex to access command_vel safely
            command_velocity_publisher.publish(command_vel);
        }

        ros::spinOnce();
        control_rate.sleep();

    }

    keyboard_thread.join();  // Wait for the keyboard input thread to finish
    
    return 0;
}

