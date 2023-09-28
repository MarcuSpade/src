#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/ChannelFloat32.h"

class FrontOdometry{
    public:
    sensor_msgs::ChannelFloat32 front_odom_read;
    ros::NodeHandle n;
    FrontOdometry(){
        sub_odom = n.subscribe("/odom_raw", 100, &FrontOdometry::odom_callback, this);
        pub_odom = n.advertise<sensor_msgs::ChannelFloat32>("/front_solver_odom",100);

    };
    ~FrontOdometry(){};
    void odom_callback(const nav_msgs::Odometry& msg)
    {
        input_x = msg.twist.twist.linear.x;
        input_y = msg.twist.twist.angular.z;
    }
    void publish_odom()
    {   
        front_odom_read.values.push_back(input_x);
        front_odom_read.values.push_back(input_y);
        pub_odom.publish(front_odom_read);
    }

    void clearMsg()
    {   
        front_odom_read.values.clear();
    }

    private:
    float input_x {0.0};
    float input_y {0.0};
    ros::Subscriber sub_odom;
    ros::Publisher pub_odom;
    nav_msgs::Odometry odom_raw;
};

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "front_odometry_node");
  std::shared_ptr<FrontOdometry> myFrontOdometry = std::make_shared<FrontOdometry>();
  ros::Rate loop_rate(10);
  ROS_INFO("solver_traction: front_odometry_node running...");

  while (myFrontOdometry -> n.ok())
  { 
    myFrontOdometry -> publish_odom();
    myFrontOdometry -> clearMsg();
    ros::spinOnce();  
    loop_rate.sleep();
  }
  return 0;
}