#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <csignal>

void printMenu()
{
    std::cout << "\n==================== Command Menu ====================" << std::endl;
    std::cout << "Please select one of the following commands:" << std::endl;
    std::cout << "1. 'straight'  : Generate a Geodesic Path" << std::endl;
    std::cout << "2. 'spline'    : Generate a Hermite Spline Path" << std::endl;
    std::cout << "3. 'reset'     : Reset the current state" << std::endl;
    std::cout << "4. 'move'      : Move to interpolated waypoints" << std::endl;
    std::cout << "5. 'transfer'  : Transfer waypoints file" << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "Enter your choice: ";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_listener_node");
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("nrs_command", 10);

    // AsyncSpinner 시작
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        std_msgs::String msg;

        std::string input;
        std::cin >> input;

        if (input == "straight")
        {
            msg.data = "generate_Geodesic_Path";
            command_pub.publish(msg);
            std::cout << "Generating Geodesic Path..." << std::endl;
        }
        else if (input == "spline")
        {
            msg.data = "generate_Hermite_Spline_path";
            command_pub.publish(msg);
            std::cout << "Generating Hermite Spline Path..." << std::endl;
        }
        else if (input == "reset")
        {
            msg.data = "reset";
            command_pub.publish(msg);
            std::cout << "Resetting the current state..." << std::endl;
        }
        else if (input == "move")
        {
            msg.data = "interpolated_waypoints";
            command_pub.publish(msg);
            std::cout << "Moving to interpolated waypoints..." << std::endl;
        }
        else if (input == "transfer")
        {
            msg.data = "file_transfer";
            command_pub.publish(msg);
            std::cout << "Transferring the waypoints file..." << std::endl;
        }
        else if (input == "m")
        {
            printMenu();
        }
        else
        {
            std::cout << "Invalid command. Please enter a valid option from the menu." << std::endl;
        }

        ros::Duration(0.1).sleep(); // Sleep to prevent CPU overload
    }

    return 0;
}
