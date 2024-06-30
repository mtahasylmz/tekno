#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <mutex>

#define WHEEL_SEPARATION 12345
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
            // Get target position, obstacles positions
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

    while (ros::ok()) {
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
