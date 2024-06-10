#include <fstream>

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>


#define WHEEL_SEPERATION 12345
#define WHEEL_DIAMETER 54321
#define MAX_LINEAR_VELOCITY 1
#define MIN_LINEAR_VELOCITY -1
#define MAX_ANGULAR_VELOCITY 1
#define MIN_ANGULAR_VELOCITY -1


double left_wheel_velocity, right_wheel_velocity;

void LinkStatesCb(const gazebo_msgs::LinkStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot::left_wheel_link") {
            left_wheel_velocity = msg->twist[i].angular.y;
        } else if (msg->name[i] == "robot::right_wheel_link") {
            right_wheel_velocity = msg->twist[i].angular.y;
        }
    }
}

geometry_msgs::Point robot_position;

void ModelStatesCb(const gazebo_msgs::ModelStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot") {
            robot_position = msg->pose[i].position;
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "random_kinematics");
    ros::NodeHandle nh;

    ros::Publisher command_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber link_states_subscriber = nh.subscribe("/gazebo/link_states", 1, LinkStatesCb);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, ModelStatesCb);

    ros::Rate control_rate(100);

    ros::Duration(1).sleep();


    {
        std::ifstream task_file(ros::package::getPath("tekno") + "/config/robot_task.txt");
        std::string line;
        double number;
        ROS_INFO("Opening task file,");
        while (getline(task_file, line)) {
            //get target position, obstacles positions
            std::istringstream sin(line);
            while (sin >> number){
                std::cout<<number<<" ";
            }
            std::cout<<"\n";
        }

        task_file.close();
    }



    std::srand(static_cast<unsigned int>(time(nullptr))); //random number seed

    geometry_msgs::Twist command_vel;
    int counter = 0;
    float xpos = 0;
    float ypos = 0;
    //command_vel.linear.x = 1;
    int bak = 0;
    //command_vel.angular.z = 1;
    
    command_velocity_publisher.publish(command_vel);
    while (ros::ok()) {
        if(counter >= 200){command_vel.angular.z = 1;}
        if (counter > 278.5 && counter < 286) {
            //counter = 0;             
            //ROS_INFO("Counter: %d\n", bak);counter = 0;
            command_vel.angular.z = -1;
            //command_vel.linear.x = 1;
            ROS_INFO("Current robot position: %f, %f, %f", robot_position.x, robot_position.y, robot_position.z);
            ROS_INFO("Current left-right wheel angular velocities: %f, %f", left_wheel_velocity, right_wheel_velocity);
            ROS_INFO("New velocity targets:  %f, %f \n", command_vel.linear.x, command_vel.angular.z);
        }
        if(counter > 285.5){command_vel.angular.z = 0;}
        if(counter > 600){
            command_vel.linear.x = 1;
        }

       command_velocity_publisher.publish(command_vel);

        counter++;
        ros::spinOnce();
        control_rate.sleep();
    }


    return 0;
}
