#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <ctime>
#include <math.h>
    

    double RADIUS = 0.7;
    double car_length = 0.28;
    double current_speed = 2;
    // publishers and subscribers
    ros::Subscriber scan_sub;
    ros::Publisher drive_pub;


    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
      double min = 1000000;
      double min_angle;
      double inc_angle = scan_msg->angle_min;
      double increment = scan_msg->angle_increment;
      std::vector<float> ranges = scan_msg->ranges;

      //find min distance from car
      //MIGHT HAVE TO CHANGE RANGE FOR THIS TO MAKE MORE VIABLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      for(int i = 0; i < scan_msg->ranges.size(); i++) {
        if(inc_angle > -1.57 && inc_angle < 1.57 && scan_msg->ranges[i] < min) {
          min = scan_msg->ranges[i];
          min_angle = inc_angle;
        }
        inc_angle += increment;
      }

      double inc_angle2 = scan_msg->angle_min;
      double arc_angle = acos((RADIUS * RADIUS - 2 * min * min) / (-2 * min * min));

      //set all ranges within radius to 0
      for(int i = 0; i < scan_msg->ranges.size(); i++) {
        double angle_dist = ranges[i];

        if(inc_angle2 > (min_angle - arc_angle) && inc_angle2 < (min_angle + arc_angle)) {
          ranges[i] = 0;
        }
        inc_angle2 += increment;

      }
      
      int biggest_gap = 0;
      int curr_gap = 0;
      int last_index = 0;

      //find larger half of array on side of longest set of 0s
      for(int i = 0; i < scan_msg->ranges.size(); i++) {
        if(ranges[i] != 0) {
          curr_gap++;
          if(curr_gap > biggest_gap) {
            biggest_gap = curr_gap;
            last_index = i;
          }
        } else {
          curr_gap = 0;
        }

      }

      int first_index = last_index - (biggest_gap - 1);
      float big_gap[biggest_gap];
      double max = 0;
      double max_angle = -5;
      double inc_angle3 = scan_msg->angle_min;
      double first_angle;
      double last_angle;
      
      //copy max array into its own array that we're gonna iterate through to find max distance from car in subarray
      //MIGHT HAVE TO LIMIT DEGREES IT CAN GRAB MAX FROM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      for(int i = 0; i < scan_msg->ranges.size(); i++) {
        if(i >= first_index && i <= last_index && scan_msg->ranges[i] >= max && inc_angle3 > -2 && inc_angle3 < 2) {
          max = scan_msg->ranges[i];
          max_angle = inc_angle3;
        }

        if(i == first_index) {
          first_angle = inc_angle3;
        } else if (i == last_index) {
          last_angle = inc_angle3;
        }

        inc_angle3 += increment;
      }

      if(max_angle == -5) {
        max_angle = 0;
      }

      double min_dist_between_angles = 100;
      double inc_angle4 = scan_msg->angle_min;

      for(int i = 0; i < scan_msg->ranges.size(); i++) {
        if((i >= first_index && i <= last_index) && ((inc_angle4 >= 0 && inc_angle4 <= max_angle) || (inc_angle4 <= 0 && inc_angle4 >= max_angle)) && scan_msg->ranges[i] < min_dist_between_angles) {
          min_dist_between_angles = scan_msg->ranges[i];
        }
        inc_angle4 += increment;
      }

      if(min_dist_between_angles == 100) {
        min_dist_between_angles = scan_msg->ranges[540];
      }

      max = min_dist_between_angles;
      double radiusOfCurvature = (max * max) / (2 * abs(max * sin(max_angle)));
      double new_steering_angle = atan(1 / (sqrt((pow(radiusOfCurvature, 2) - pow(car_length / 2, 2)) / pow(car_length, 2))));

      if(max_angle < 0) {
        new_steering_angle = -new_steering_angle;
      }

      if(new_steering_angle < first_angle || new_steering_angle > last_angle) {
        new_steering_angle = max_angle;
      }

      // calculate ttc to closest wall in front of it ish
      double inc_angle5 = scan_msg->angle_min;
      double ttc_angle = 5;
      double ttc_dist;

      for(int i = 0; i < scan_msg->ranges.size(); i++) {
        if(inc_angle5 > -0.2 && inc_angle5 < 0.2 && inc_angle5 <= ttc_angle) {
          ttc_angle = inc_angle5;
          ttc_dist = scan_msg->ranges[i];
        }
        inc_angle5 += increment;
      }

      double speed_angle = current_speed * cos(ttc_angle);

      double ttc = ttc_dist / speed_angle;

      //new weird shit

      double relativeTTCDist = (ttc-0.5) * current_speed;

      double allowableSpeed = sqrt(2 * 1.7 * relativeTTCDist);

      if(abs(new_steering_angle) >(7.0/180)*M_PI){
            if(allowableSpeed > 0.8){
                allowableSpeed = 0.8;
            }
        }

        if(ttc < 1.1 && allowableSpeed > 2) {
          allowableSpeed = 2;
        }

      if(allowableSpeed > 2) {
        allowableSpeed = 2;
      }

      double speed = allowableSpeed;
      current_speed = speed;

      ackermann_msgs::AckermannDriveStamped steering_angle;
      steering_angle.drive.steering_angle = new_steering_angle;
      steering_angle.drive.speed = speed;
      ROS_INFO_STREAM(speed);

      drive_pub.publish(steering_angle);
    
    }


    int main(int argc, char **argv)
    {
      ros::init(argc, argv, "pure_pursuit");
      ros::NodeHandle n;
      n = ros::NodeHandle();
      std::string drive_topic;
      n.getParam("/nav_drive_topic", drive_topic);

        scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scan_callback);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1000);

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