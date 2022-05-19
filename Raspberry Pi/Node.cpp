/*
************************************************************
** Author: Penjani Mkandawire
** Date: 10/12/2021
** Email: mkandawire15@gmail.com
************************************************************
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using std::cout;
using std::endl;

double x_direction = 0;
double y_direction = 0;
double M_speed = 0;

void handle_movement( const geometry_msgs::Twist& movement) 
{
    cout << "Received move command: x = " << movement.linear.x << " y = " << movement.linear.y <<>

    // Extract the X-Axis direction movement from the message
    x_direction = movement.linear.x;

    // Extract the Y-Axis direction movement from the message
    y_direction = movement.linear.y;

    // Extract the speed
    M_speed = movement.linear.z;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Controller_PPi");

    ros::NodeHandle n;

    // Publisher to send code to the Arduino serial monitor
    ros::Publisher direction_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscriber to receive movement commands from laptop
    ros::Subscriber sub = n.subscribe("movement", 1, handle_movement);

    ros::Rate loop_rate(1000);

    int count = 0;
    
    while(ros::ok())
    {
        geometry_msgs::Twist msg;

        // Going left
        if (x_direction == -1)
        {
            msg.linear.x = -1;
            direction_pub.publish(msg);
        }

        // Going right
        else if (x_direction == 1)
        {
            msg.linear.x = 1;
            direction_pub.publish(msg);
        }

        // Going back
        else if (y_direction == -1)
        {
            msg.linear.y = -1;
            direction_pub.publish(msg);
        }

        // Going forward
        else if (y_direction == 1)
        {
            msg.linear.y = 1;
            direction_pub.publish(msg);
        }

        // Adjust Speed
        else if(M_speed >= 55 && M_speed <= 255)
        {
            msg.linear.z = M_speed;
            direction_pub.publish(msg);
	}
	
        x_direction = 0;
        y_direction = 0;
        M_speed = 0;

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
