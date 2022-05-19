/*
************************************************************
** Author: Penjani Mkandawire
** Date: 10/12/2021
** Email: mkandawire15@gmail.com
************************************************************
*/

#include <ncurses.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using std::cin;
using std::cout;
using std::endl;

// Console control
void initcurses()
{
    // Initialize library
    initscr();
    
    // Enable control characters
    cbreak();
    
    // Disable getch echoing
    noecho();
    
    // Flush terminal buffer
    intrflush(stdscr, TRUE);
    
    // Enable arrow keys
    keypad(stdscr, TRUE);
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "Controller_Laptop");
    
	ros::NodeHandle n;
    
    // Publisher to send code to the Arduino serial monitor
    ros::Publisher direction_pub = n.advertise<geometry_msgs::Twist>("movement", 1);
    
    initcurses();
    
    int ch;
    
    int CurrentSpeed = 255;
    
    cout << "Enter direction: " << endl;
    
    while( (ch = getch()) && ros::ok())
    {            
        char d = (char)ch;
        
        geometry_msgs::Twist msg;
        
        if(d == 'W' || d == 'w')
        {
            cout << "Move forward.\r" << endl;
            msg.linear.y = 1;
        }
        else if(d == 'S' || d == 's')
        {
            cout << "Move backward.\r" << endl;
            msg.linear.y = -1;
        }
        else if(d == 'A' || d == 'a')
        {
            cout << "Move left.\r" << endl;
            msg.linear.x = -1;   
        }
        else if(d == 'D' || d == 'd')
        {
            cout << "Move right.\r" << endl;
            msg.linear.x = 1;
        }
        else if(d == '=' || d == '+')
        {
            CurrentSpeed++;
            
            if(CurrentSpeed >= 55 && CurrentSpeed <= 255)
            {
                msg.linear.z = CurrentSpeed;
                cout << "Increase Speed to: " << CurrentSpeed << " .\r" << endl;
            }
            else
            {
                cout << "Invalid Speed: " << CurrentSpeed << " . Valid range is 55 - 255 .\r" << endl;
            }
        }
        else if(d == '-' || d == '_')
        {
            CurrentSpeed--;
            
            if(CurrentSpeed >= 55 && CurrentSpeed <= 255)
            {
                msg.linear.z = CurrentSpeed;
                cout << "Decrease Speed to: " << CurrentSpeed << " .\r" << endl;
            }
            else
            {
                cout << "Invalid Speed: " << CurrentSpeed << " . Valid range is 55 - 255 .\r" << endl;
            }
        }
        
        direction_pub.publish(msg);

        ros::spinOnce();
    }

    endwin();
    
    cout << "Finished" << endl;
    
    // Don't exit the program.
    ros::spin();
}