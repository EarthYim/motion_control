#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <math.h>
#include <array>
#define PI 3.14159265

class Control {
	public:
		geometry_msgs::Pose2D current_pose;
		// ros::Publisher pub_pose2d;

		void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
			
			current_pose.x = msg->pose.pose.position.x;
			current_pose.y = msg->pose.pose.position.y;

			tf::Quaternion q(
					msg->pose.pose.orientation.x,
					msg->pose.pose.orientation.y,
					msg->pose.pose.orientation.z,
					msg->pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			current_pose.theta = yaw;
			// pub_pose2d.publish(current_pose);
			
			ROS_INFO("angle: %f", current_pose.theta);
		}

		std::array<double, 2> get_path (current_pose, wp) {
			std::array<double, 2> path;
			path[0] = NULL; // heading
			path[1] = NULL; // displacement
			return path
		}

		bool arrived (current_pose, wp) {
			double diff; 
			// calculate diff
			
			if (diff == 0) {
				 return true;
			 }
			 else{
				 return false;
			 }
		}

		void move_to(path) {
		}
};
	
std::array<double, 2> cardiod(double theta, double radius) {
	std::array<double, 2> wp;
	double r = radius + radius*cos(theta);
	wp[0] = r*cos(theta);
	wp[1] = r*sin(theta);
	return wp;
}
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "motion_ctrl");

	ros::NodeHandle n;

	//ros::Subscriber odom_readout = n.subscribe("/odom", 10, odomCallback);
	
	control.get_pose();

	while  control.arrived() {


	}
	ros::Publisher vel_command_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
	ros::Rate loop_rate(10);
	


	loop_rate.sleep();	
	}
	return 0;
}
