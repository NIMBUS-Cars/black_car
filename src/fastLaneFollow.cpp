#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>


    double speed;
    ros::Publisher acker_pub;
    ros::Publisher bool_pub;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    bool car_stopped = false;

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        if(car_stopped == false) {
          speed = 1;
        } else {
          speed = 0.8;
        }
        double TTC = 2;
        bool in_ttc_range = false;
        double increment = scan_msg->angle_increment;
        double angle = scan_msg->angle_min;
        ackermann_msgs::AckermannDriveStamped brake_output;
        
        for(int i = 0; i < scan_msg->ranges.size(); i++) {
          double distance = scan_msg->ranges[i];
          double speed_angle = speed * cos(angle);
          if(speed_angle == -0) {
            speed_angle = 0;
          } else if (speed_angle < 0) {
            speed_angle = speed_angle * -1;
          }


          if(angle >= -.5517 && angle <= 0.5517) {
            TTC = distance / speed_angle;

            if(TTC < 1.5) {
              in_ttc_range = true;
            } 
          }
          angle += increment;
        }

        if(in_ttc_range) {

          brake_output.drive.speed = 0;
          car_stopped = true;
        } else {
          brake_output.drive.speed = 1;
          car_stopped = false;
        }
        acker_pub.publish(brake_output);

    }


    int main(int argc, char **argv)
    {
      ros::init(argc, argv, "safety_node");
      ros::NodeHandle n;
        n = ros::NodeHandle();
        std::string drive_topic;
        n.getParam("/nav_drive_topic", drive_topic);

        speed = 0.0;

        acker_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1000);
        scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scan_callback);

        ros::Rate loop_rate(10);

        int count = 0;
        while (ros::ok())
        {

          ros::spinOnce();

          loop_rate.sleep();
          ++count;
        }


        return 0;
        
    }