#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#define PI 3.14159265

class Control {
	public:
		
		//class attributes
		geometry_msgs::Pose2D current_pose;
		geometry_msgs::Twist current_vel;
		geometry_msgs::Pose2D path;
		geometry_msgs::Pose2D wp;
		double displacement;
		geometry_msgs::Twist command;
		
		// class variables
		double p_error_x = 0.0;
		double p_error_z = 0.0;
		int iteration = 100; //steps
		double radius = 1.0;
		double threshold = 0.1;
		double Kp_x = 0.1;
		double Kd_x = 0.05; 
		double Kp_z = 0.8; 
		double Kd_z = 0.05;
		
		// ros::Publisher pub_pose2d;

		//class methods
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
			
			// ROS_INFO("angle: %f", current_pose.theta);

			current_vel.linear.x = msg->twist.twist.linear.x;
			current_vel.angular.z = msg->twist.twist.angular.z;
		}

		void cardiod(double theta, double radius) {
			double r = radius + radius*cos(theta);
			wp.x = r*cos(theta);
			wp.y = r*sin(theta);
			wp.theta = 0.0;
			ROS_INFO("Cardoid wp @theta:%f, x:%f, y:%f", theta, wp.x, wp.y);
		}

		void get_wp(int count, double radius) {
			double theta = (2*PI/iteration)*count;
			cardiod(theta, radius); 
		}

		bool ongoingForward () {
			if (current_vel.linear.x != 0) {
				return true;
			}
			else {
				return false;
			}
		}

		void get_path () {
			
			// double x_dis = wp.x - current_pose.x
			// double y_dis = wp.y - current_pose.y
			// double displacement = pow(pow(x_dis, 2) + pow(y_dis, 2), 1/2); //sqrt[(x2-x1)**2-(y2-y1)**2]
			// geometry_msgs::Pose2D path;
			
			path.x = wp.x - current_pose.x;
			path.y = wp.y - current_pose.y;
			displacement = pow(pow(path.x, 2) + pow(path.y, 2), 0.5);
			double bearing = atan(path.y/path.x);
			path.theta = bearing - current_pose.theta; // heading
			ROS_INFO("Heading: %f, Dis: %f", path.theta, displacement);
			
		}

		bool arrived () {
			
			get_path();
			
			if (displacement < threshold) {
				 return true;
			 }
			 else {
				 return false;
			 }
		}

		void update_error() {
			p_error_x = displacement;
			p_error_z = path.theta;
			ROS_INFO("Errors Updated");
		}

		double PDController (double error_x, double error_z, double Kp_x, double Kd_x, double Kp_z, double Kd_z) {
			double dx = (error_x - p_error_x) / 0.1; //need confrimed 100 ms
			double dz =  (error_z - p_error_z) / 0.1;
			command.linear.x = Kp_x*error_x + Kd_x*dx;
			command.angular.z = -1 * Kp_z*error_z + Kd_z*dz;
			update_error();


		} 

		void get_command() {
				PDController (displacement, path.theta, Kp_x, Kd_x, Kd_z, Kd_z);
				ROS_INFO("Commands ready");
			}


};


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "motion_ctrl");

	ros::NodeHandle n;

	Control control;
	ros::Subscriber odom_readout = n.subscribe("/odom", 10, &Control::odomCallback, &control);
	ros::Publisher vel_command_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
	
	int count = 1;
	while (ros::ok()) {

		ros::Rate loop_rate(10);

		if (count > control.iteration) {
			geometry_msgs::Twist stop;
			stop.linear.x = 0.0;
			stop.angular.z = 0.0;
			vel_command_pub.publish(stop); // publish stop
			ROS_INFO("Drawing has completed");
			break;
		}

		control.get_wp(count, control.radius);
		
		if (! control.arrived()) {
			// control.get_path(); already done in .arrived()
			control.get_command();
			vel_command_pub.publish(control.command); //publish control.command.
			ROS_INFO("Command has published");
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		else {
			ROS_INFO("WP %d rached", count);
			count++;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}
