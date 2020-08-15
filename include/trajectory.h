#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>


using namespace geometry_msgs;
using namespace std;
using namespace ros;

const double deg2rad = M_PI/180;

// Subscribers

// Publishers
ros::Publisher ref_pos_pub;
ros::Publisher ref_pos_delay_pub;
ros::Publisher ref_vel_pub;
ros::Publisher ref_yaw_pub;
ros::Publisher setpoint_pos_pub;
ros::Publisher servo_pub;
ros::Publisher traj_on_pub;
ros::Publisher reg_on_pub;

mavros_msgs::State current_state_msg;
geometry_msgs::PoseStamped pos_ref_start_msg;
geometry_msgs::Vector3 reftrajectory_msg, reftrajectory_delay_msg, reftrajectory_vel_msg;
geometry_msgs::PoseStamped setpoint_pos_msg;
std_msgs::Bool traj_start_msg, reg_on_msg;
std_msgs::Float64 ref_yaw_msg;
std_msgs::Float64 servo_pos_msg;

tf::Quaternion setpoint_att_quat;

int count_start = 0;
double max_z, x_des, y_des, z_des, del_z, radius, absvel, rotvel, time_period, climb_rate, land_rate;
int pos_pub_delay, traj_type, drop_type, num_drops, const_z, count_ = count_start, yaw_des;
bool traj_start, max_z_start, reg_on, drop_flag, climb_flag, land_flag, change_z, pub_setpoint_pos;


double t, t_last, traj_time, traj_time_z, t_last_z, time_drop = 0;
bool traj_started_flag = 0, climbed_flag = 0, landed_flag = 0, drop_start = 0, drop_started_flag = 0;
int print_flag_traj_start = 0, print_flag_hover_origin = 1, print_flag_hover = 1, 
print_flag_circle = 1, print_flag_fig8 = 1, print_flag_square = 1,  print_flag_changez = 1;
int print_flag_climb = 0, print_flag_land = 0;
