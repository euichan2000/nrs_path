#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("moveit_command", 10);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok())
    {
        std_msgs::String msg;
        // std::cout << "Enter 'a' to start the robot: ";
        std::string input;
        std::cin >> input;

        if (input == "straight")
        {
            msg.data = "generate_Geodesic_Path";
            command_pub.publish(msg);
        }
        if (input == "spline")
        {
            msg.data = "generate_Hermite_Spline_path";
            command_pub.publish(msg);
        }

        if (input == "visual" or input == "move")
        {
            msg.data = "start";
            command_pub.publish(msg);
        }
        if (input == "reset")
        {
            msg.data = "reset";
            command_pub.publish(msg);
        }
        if (input == "move")
        {
            msg.data = "interpolated_waypoints";
            command_pub.publish(msg);
        }

        ros::Duration(1.0).sleep(); // Sleep to prevent CPU overload
    }

    return 0;
}
