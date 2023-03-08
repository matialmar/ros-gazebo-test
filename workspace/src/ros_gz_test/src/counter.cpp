#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "counter");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Int32>("test/counter", 10);

    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok())
    {
        std_msgs::Int32 msg;
        msg.data = count;

        pub.publish(msg);

        ROS_INFO("Contador: %d", msg.data);

        ros::spinOnce();
        loop_rate.sleep();
        
        ++count;
    }
    return 0;
}
