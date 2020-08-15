/**
 * @file   trajectory_withDropReg.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <trajectory.h>
#include <trajectories/set_trajectory_withDropRegConfig.h>

double sampleTime = 0.01;

void dynamicReconfigureCallback(trajectories::set_trajectory_withDropRegConfig &config, uint32_t level)
{
    traj_start = config.traj_start;
    max_z_start = config.max_z_start;
    reg_on = config.reg_on;
    drop_flag = config.drop;
    climb_flag = config.climb;
    land_flag = config.land;
    change_z = config.change_z;
    pub_setpoint_pos = config.pub_on_setpoint_position;

    traj_type = config.traj_type;
    drop_type = config.drop_type;
    num_drops = config.num_simultaneousDrops_firstLap;
    pos_pub_delay = config.pos_pub_delay;
    max_z = config.max_z;
    x_des = config.x_des;
    y_des = config.y_des;
    z_des = config.z_des;
    yaw_des = config.yaw_des;
    del_z = config.del_z;
    radius = config.des_radius;
    absvel = config.des_velocity;

    rotvel = absvel/radius;
    time_period = 2*M_PI/rotvel;

    climb_rate = config.climb_rate;
    land_rate = config.land_rate;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<trajectories::set_trajectory_withDropRegConfig> server;
    dynamic_reconfigure::Server<trajectories::set_trajectory_withDropRegConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    ref_pos_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/pose", 1);
    ref_pos_delay_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/pose_delayed", 1);
    ref_vel_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/velocity", 1);
    ref_yaw_pub = nh.advertise<std_msgs::Float64>("ref_trajectory/yaw", 1);
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    servo_pub = nh.advertise<std_msgs::Float64>("servo_pos", 1);
    traj_on_pub = nh.advertise<std_msgs::Bool>("trajectory_on", 1);
    reg_on_pub = nh.advertise<std_msgs::Bool>("regression_on", 1);

    ros::Rate rate(1/sampleTime);

    pos_ref_start_msg.pose.position.x = 0;
    pos_ref_start_msg.pose.position.y = 0;
    if (max_z_start)
        pos_ref_start_msg.pose.position.z = max_z;
    else
        pos_ref_start_msg.pose.position.z = 0;

//    int delay = 3+5;

    servo_pos_msg.data = 0.0;

    double x,y,z;
    double x_last = 0, y_last = 0, z_last = 0;
    double x_atTrajStart = 0, y_atTrajStart = 0, z_atTrajStart = 0;
    double u,v,w;

    bool x_delay_started = false, y_delay_started = false, z_delay_started = false;
    double x_delay_start,y_delay_start,z_delay_start;
    double x_delay,y_delay,z_delay;

    traj_start_msg.data = 0.0;

    while(ros::ok())
    {
        traj_start_msg.data = traj_start;
        reg_on_msg.data = reg_on;

        if (max_z_start &&  pos_ref_start_msg.pose.position.z != max_z
                        && !landed_flag && traj_start != 1)
        {
            pos_ref_start_msg.pose.position.z = max_z;
            print_flag_traj_start = 0;

        }
        else if(!max_z_start && pos_ref_start_msg.pose.position.z != 0.0
                             && traj_start != 1)
        {
            pos_ref_start_msg.pose.position.z = 0.0;
            print_flag_traj_start = 0;
        }

        if(change_z == 1)
        {
            if (print_flag_changez == 1)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Changing z by %.2f m!",del_z);
                print_flag_changez = 2;
            }
            const_z = 1;
        }
        else
        {
            if (print_flag_changez == 2)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Constant z!");
                print_flag_changez = 1;
            }
            const_z = 0;
        }

        if(traj_start == 1 && !landed_flag)
        {
//            traj_start_msg.data = traj_start;
            if(!traj_started_flag)
                traj_started_flag = true;

            t = ros::Time::now().toSec();
            traj_time = t - t_last;

            if(climb_flag)
            {
                while(ros::ok() && !climbed_flag && z_delay < max_z)
                {
                    if (print_flag_climb == 0)
                    {
                        ROS_INFO("---------------------------------");
                        ROS_INFO("Climbing initialized!");
                        print_flag_climb = 1;

                        z_delay_start = z;
                    }
                    x = x;
                    y = y;
                    z = z < max_z ? z + climb_rate*sampleTime : max_z;

                    x_delay = x;
                    y_delay = y;
                    z_delay = std::abs(z - z_delay_start) < climb_rate*pos_pub_delay*sampleTime ? z_delay_start : z_delay + climb_rate*sampleTime;

                    u = (x - x_last)/sampleTime;
                    v = (y - y_last)/sampleTime;
                    w = (z - z_last)/sampleTime;

                    ref_yaw_msg.data = deg2rad*yaw_des;
                    if(!pub_setpoint_pos)
                    {
                        reftrajectory_msg.x = x;
                        reftrajectory_msg.y = y;
                        reftrajectory_msg.z = z;
                        ref_pos_pub.publish(reftrajectory_msg);
                        reftrajectory_delay_msg.x = x_delay;
                        reftrajectory_delay_msg.y = y_delay;
                        reftrajectory_delay_msg.z = z_delay;
                        ref_pos_delay_pub.publish(reftrajectory_delay_msg);

                        reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                        reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                        reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
                        ref_vel_pub.publish(reftrajectory_vel_msg);

                        ref_yaw_pub.publish(ref_yaw_msg);
                    }
                    else
                    {
                        setpoint_pos_msg.header.stamp = ros::Time::now();
                        setpoint_pos_msg.pose.position.x = x;
                        setpoint_pos_msg.pose.position.y = y;
                        setpoint_pos_msg.pose.position.z = z;

                        setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
                        setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
                        setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
                        setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
                        setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
                        setpoint_pos_pub.publish(setpoint_pos_msg);
                    }

                    if(z_delay >= max_z)
                    {
                        if (print_flag_climb == 1)
                        {
                            ROS_INFO("Climbing complete!");
                            print_flag_climb = 0;
                        }
                        pos_ref_start_msg.pose.position.z = max_z;
                        climbed_flag = true;
                        landed_flag = false;
                        t_last = ros::Time::now().toSec();
                    }

                    x_last = x;
                    y_last = y;
                    z_last = z;

                    servo_pub.publish(servo_pos_msg);
                    traj_on_pub.publish(traj_start_msg);
                    reg_on_pub.publish(reg_on_msg);
                    ros::spinOnce();
                    rate.sleep();
                }
            }

            if(climbed_flag && !landed_flag)
            {
                pos_ref_start_msg.pose.position.z = max_z;
            }

            if (print_flag_traj_start == 1)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Reference trajectory started!");
                print_flag_traj_start = 0;
            }

            if(traj_type == 0) // hover at origin
            {
                if (!drop_started_flag && drop_flag)
                {
                    drop_start = drop_flag;
                    ROS_INFO("drop_start --> 1");
                    drop_started_flag = true;
                    if(drop_type == 2)
                        count_ = count_ - 1;
                }
                if (print_flag_hover_origin == 1)
                {
                    ROS_INFO("--------Hover at origin selected!--------");
                    print_flag_hover_origin = 0;
                    print_flag_hover = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                }
                x = pos_ref_start_msg.pose.position.x;
                y = pos_ref_start_msg.pose.position.y;
//                z = pos_ref_start_msg.pose.position.z - del_z*(sin(traj_time*3/7)) * const_z;
                z = pos_ref_start_msg.pose.position.z;
                x_delay = x;
                y_delay = y;
                z_delay = z;

                t_last = ros::Time::now().toSec();
            }

            if(traj_type == 1) // hover
            {
                if (!drop_started_flag && drop_flag)
                {
                    drop_start = drop_flag;
                    ROS_INFO("drop_start --> 1");
                    drop_started_flag = true;
                    if(drop_type == 2)
                        count_ = count_ - 1;
                }
                if (print_flag_hover == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Hover selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 0;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;

                    x_atTrajStart = x;
                    y_atTrajStart = y;
                    z_atTrajStart = z;

                    z_delay_started = false;
                    z_delay_start = z;
                }
                x = x_atTrajStart + x_des;
                y = y_atTrajStart + y_des;
                z = z_atTrajStart + z_des - del_z*(sin(rotvel*traj_time)) * const_z;
                x_delay = x;
                y_delay = y;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z*(sin(rotvel*(pos_pub_delay*sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay = z_atTrajStart + z_des - del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
//                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay = z_atTrajStart + z_des - del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
            }

            if(traj_type == 2) // circle
            {
                if (print_flag_circle == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Circle selected!--------");
                    ROS_INFO("rotational velocity = %.2f rad/s", rotvel);
                    ROS_INFO("time period = %.2f s", time_period);
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_circle = 0;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;

                    x_atTrajStart = pos_ref_start_msg.pose.position.x + x_des;
                    y_atTrajStart = pos_ref_start_msg.pose.position.y + y_des;
                    z_atTrajStart = pos_ref_start_msg.pose.position.z + z_des;

                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;
                }
                x = x_atTrajStart + radius*sin(rotvel*traj_time);
                if (!x_delay_started)
                    if (std::abs(x - x_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                        x_delay = x_delay_start;
                    else
                    {
                        x_delay = x_atTrajStart + radius*sin(rotvel*(traj_time - pos_pub_delay*sampleTime));
//                        std::cout<<"x_delay_started gets true"<<"\n";
                        x_delay_started = true;
                    }
                else
                    x_delay = x_atTrajStart + radius*sin(rotvel*(traj_time - pos_pub_delay*sampleTime));

                if(traj_time < 0.25*time_period)
                {
                    y = y_atTrajStart;
                    z = z_atTrajStart;
                    y_delay = y;
                    z_delay = z;
                    t_last_z = ros::Time::now().toSec();
                }
                else
                {
                    traj_time_z = t - t_last_z;
                    if (!drop_started_flag && traj_time_z > 0.25*time_period && drop_flag)
                    {
                        drop_start = drop_flag;
                        ROS_INFO("drop_start --> 1");
                        drop_started_flag = true;
                        if(drop_type == 2)
                            count_ = count_ - 1;
                    }
                    y = y_atTrajStart + radius*cos(rotvel*traj_time);
                    z = z_atTrajStart - del_z*((sin(rotvel*traj_time_z))) * const_z;

                    if (!y_delay_started)
                        if (std::abs(y - y_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                            y_delay = y_delay_start;
                        else
                        {
                            y_delay = y_atTrajStart + radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime));
//                            std::cout<<"y_delay_started gets true"<<"\n";
                            y_delay_started = true;
                        }
                    else
                        y_delay = y_atTrajStart + radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime));

                    if (!z_delay_started)
                        if (std::abs(z - z_delay_start) < del_z*(sin(rotvel*(pos_pub_delay*sampleTime))) * const_z)
                            z_delay = z_delay_start;
                        else
                        {
                            z_delay = z_atTrajStart - del_z*(sin(rotvel*(traj_time_z - pos_pub_delay*sampleTime))) * const_z;
//                            std::cout<<"z_delay_started gets true"<<"\n";
                            z_delay_started = true;
                        }
                    else
                        z_delay = z_atTrajStart - del_z*(sin(rotvel*(traj_time_z - pos_pub_delay*sampleTime))) * const_z;
                }
            }

            if(traj_type == 3) // figure 8
            {
                if (print_flag_fig8 == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Figure8 selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 0;
                    print_flag_square = 1;

                    x_atTrajStart = pos_ref_start_msg.pose.position.x + x_des;
                    y_atTrajStart = pos_ref_start_msg.pose.position.y + y_des;
                    z_atTrajStart = pos_ref_start_msg.pose.position.z + z_des;

                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;
                }
                if (!drop_started_flag && traj_time > 0.25*time_period && drop_flag)
                {
                    drop_start = drop_flag;
                    ROS_INFO("drop_start --> 1");
                    drop_started_flag = true;
                    if(drop_type == 2)
                        count_ = count_ - 1;
                }
                x = x_atTrajStart - radius*cos(rotvel*traj_time + M_PI/2);
                if (!x_delay_started)
                    if (std::abs(x - x_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                        x_delay = x_delay_start;
                    else
                    {
                        x_delay = x_atTrajStart - radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime) + M_PI/2);
//                        std::cout<<"x_delay_started gets true"<<"\n";
                        x_delay_started = true;
                    }
                else
                    x_delay = x_atTrajStart - radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime) + M_PI/2);

                y = y_atTrajStart + (radius/2)*sin(2*rotvel*traj_time);
                if (!y_delay_started)
                    if (std::abs(y - y_delay_start) < (radius/2)*(sin(2*rotvel*(pos_pub_delay*sampleTime))))
                        y_delay = y_delay_start;
                    else
                    {
                        y_delay = y_atTrajStart + (radius/2)*sin(2*rotvel*(traj_time - pos_pub_delay*sampleTime));
//                        std::cout<<"y_delay_started gets true"<<"\n";
                        y_delay_started = true;
                    }
                else
                    y_delay = y_atTrajStart + (radius/2)*sin(2*rotvel*(traj_time - pos_pub_delay*sampleTime));

                z = z_atTrajStart + del_z*(sin(rotvel*traj_time)) * const_z;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z*(sin(rotvel*(pos_pub_delay*sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay = z_atTrajStart + del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
//                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay = z_atTrajStart + del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
            }

            if(traj_type == 4) // circle ground
            {
                if (print_flag_circle == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Circle selected!--------");
                    ROS_INFO("rotational velocity = %.2f rad/s", rotvel);
                    ROS_INFO("time period = %.2f s", time_period);
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_circle = 0;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;

                    x_atTrajStart = pos_ref_start_msg.pose.position.x + x_des;
                    y_atTrajStart = pos_ref_start_msg.pose.position.y + y_des;
                    z_atTrajStart = pos_ref_start_msg.pose.position.z + z_des;

                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;
                }
                x = x_atTrajStart + radius*sin(rotvel*traj_time);
                if (!x_delay_started)
                    if (std::abs(x - x_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                        x_delay = x_delay_start;
                    else
                    {
                        x_delay = x_atTrajStart + radius*sin(rotvel*(traj_time - pos_pub_delay*sampleTime));
//                        std::cout<<"x_delay_started gets true"<<"\n";
                        x_delay_started = true;
                    }
                else
                    x_delay = x_atTrajStart + radius*sin(rotvel*(traj_time - pos_pub_delay*sampleTime));

                if(traj_time < 0.25*time_period)
                {
                    y = y_atTrajStart;
                    z = z_atTrajStart;
                    y_delay = y;
                    z_delay = z;
                    t_last_z = ros::Time::now().toSec();
                }
                else
                {
                    traj_time_z = t - t_last_z;
                    if (!drop_started_flag && traj_time_z > 0.25*time_period  && drop_flag)
                    {
                        drop_start = drop_flag;
                        ROS_INFO("drop_start --> 1");
                        drop_started_flag = true;
                    }
                    y = y_atTrajStart + radius*cos(rotvel*traj_time);
                    if (!y_delay_started)
                        if (std::abs(y - y_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                            y_delay = y_delay_start;
                        else
                        {
                            y_delay = y_atTrajStart + radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime));
    //                        std::cout<<"y_delay_started gets true"<<"\n";
                            y_delay_started = true;
                        }
                    else
                        y_delay = y_atTrajStart + radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime));

                    if ((x > 0.2 && x <= 0.8 && y <= 0) && z_delay >= (max_z - del_z))
                    {
                        z = z > (max_z - del_z) ? z - land_rate*sampleTime*const_z : (max_z - del_z);
                        z_delay = std::abs(z - z_delay_start) < land_rate*pos_pub_delay*sampleTime ? z_delay_start : z_delay - land_rate*sampleTime*const_z;
                    }
                    else if( (x > 0.3 && x <= 0.9 && y >= 0) && z_delay <= max_z )
                    {
                        z = z < max_z ? z + climb_rate*sampleTime*const_z : max_z;
                        z_delay = std::abs(z - z_delay_start) < climb_rate*pos_pub_delay*sampleTime ? z_delay_start : z_delay + climb_rate*sampleTime*const_z;
                    }
                    else
                    {
                        z = z;
                        z_delay = z;
                        z_delay_start = z;
                    }
                }
            }
            if(traj_type == 5) // square (TO BE IMPROVED)
            {
                if (print_flag_square == 1)
                {
                    ROS_INFO("--------Square selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 0;

                    x_atTrajStart = pos_ref_start_msg.pose.position.x + x_des;
                    y_atTrajStart = pos_ref_start_msg.pose.position.y + y_des;
                    z_atTrajStart = pos_ref_start_msg.pose.position.z + z_des;

                    x_last = x_atTrajStart;
                    y_last = y_atTrajStart;
                    z_last = z_atTrajStart;

                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;
                }
                if (!drop_started_flag && traj_time > 0.25*time_period && drop_flag)
                {
                    drop_start = drop_flag;
                    ROS_INFO("drop_start --> 1");
                    drop_started_flag = true;
                    if(drop_type == 2)
                        count_ = count_ - 1;
                }
                if (std::abs(sin(rotvel*traj_time)-1) < 0.001 || std::abs(sin(rotvel*traj_time)+1) < 0.001)
                    x = x_atTrajStart + radius*(sin(rotvel*traj_time)<0 ? std::floor(sin(rotvel*traj_time)) : std::ceil(sin(rotvel*traj_time)));
                else
                    x = x_last;
                if(traj_time < 0.25*time_period)
                {
                    y = y_atTrajStart;
                    z = z_atTrajStart;
                    t_last_z = ros::Time::now().toSec();
                }
                else
                {
                    traj_time_z = t - t_last_z;
                    if (std::abs(cos(rotvel*traj_time)-1) < 0.001 || std::abs(cos(rotvel*traj_time)+1) < 0.001)
                        y = y_atTrajStart + radius*(cos(rotvel*traj_time)<0 ? std::floor(cos(rotvel*traj_time)) : std::ceil(cos(rotvel*traj_time)));
                    else
                        y = y_last;
                    if (std::abs(sin(rotvel*traj_time_z)-1) < 0.001 || std::abs(sin(rotvel*traj_time_z)+1) < 0.001)
                        z = z_atTrajStart - del_z*(sin(rotvel*traj_time_z)<0 ? std::floor(sin(rotvel*traj_time_z)) : std::ceil(sin(rotvel*traj_time_z))) * const_z;
                    else
                        z = z_last;
                }
                x_delay = x;
                y_delay = y;
                z_delay = z;
            }
            ref_yaw_msg.data = deg2rad*yaw_des;
            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
        }
        else if(traj_start == 1 || land_flag)
        {
            if (print_flag_traj_start == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Holding position at [x,y,z]: %.2f,%.2f,%.2f",x,y,z);
                print_flag_traj_start = 1;
            }
            x = x;
            y = y;
            z = z;

            x_delay = x;
            y_delay = y;
            z_delay = z;

            if (land_flag)
                ref_yaw_msg.data = ref_yaw_msg.data;
            else
                ref_yaw_msg.data = deg2rad*yaw_des;
            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);

            t_last = ros::Time::now().toSec();
        }
        else
        {
            if(traj_started_flag)
            {
//                pos_ref_start_msg.pose.position.z = 0.0;
                traj_started_flag = false;
            }

            if (print_flag_traj_start == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Default reference position only!");
                ROS_INFO("Position reference x =  %.2f m!",pos_ref_start_msg.pose.position.x);
                ROS_INFO("Position reference y =  %.2f m!",pos_ref_start_msg.pose.position.y);
                ROS_INFO("Position reference z =  %.2f m!",pos_ref_start_msg.pose.position.z);
                print_flag_traj_start = 1;
            }

            x = pos_ref_start_msg.pose.position.x;
            y = pos_ref_start_msg.pose.position.y;
            z = pos_ref_start_msg.pose.position.z;

            x_delay = x;
            y_delay = y;
            z_delay = z;

            ref_yaw_msg.data = ref_yaw_msg.data;
            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);

            t_last = ros::Time::now().toSec();
//            std::cout<<"t_last = "<<t_last<<"\n";

            climbed_flag = false;
            landed_flag = false;

            if (drop_started_flag)
            {
                drop_start = 0;
                ROS_INFO("drop_start --> 0");
                drop_started_flag = false;
            }
        }

        if(land_flag)
        {
            while(ros::ok() && !landed_flag && z_delay > 0.0)
            {
                if (print_flag_land == 0)
                {
                    ROS_INFO("---------------------------------");
                    ROS_INFO("Landing initialized!");
                    print_flag_land = 1;

                    z_delay_start = z;
                }
                x = x;
                y = y;
                z = z > 0 ? z - land_rate*sampleTime : 0;

                x_delay = x;
                y_delay = y;
                z_delay = std::abs(z - z_delay_start) < land_rate*pos_pub_delay*sampleTime ? z_delay_start : z_delay - land_rate*sampleTime;

                u = (x - x_last)/sampleTime;
                v = (y - y_last)/sampleTime;
                w = (z - z_last)/sampleTime;

                ref_yaw_msg.data = deg2rad*yaw_des;

                if(!pub_setpoint_pos)
                {
                    reftrajectory_msg.x = x;
                    reftrajectory_msg.y = y;
                    reftrajectory_msg.z = z;
                    ref_pos_pub.publish(reftrajectory_msg);
                    reftrajectory_delay_msg.x = x_delay;
                    reftrajectory_delay_msg.y = y_delay;
                    reftrajectory_delay_msg.z = z_delay;
                    ref_pos_delay_pub.publish(reftrajectory_delay_msg);

                    reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                    reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                    reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
                    ref_vel_pub.publish(reftrajectory_vel_msg);

                    ref_yaw_pub.publish(ref_yaw_msg);
                }
                else
                {
                    setpoint_pos_msg.header.stamp = ros::Time::now();
                    setpoint_pos_msg.pose.position.x = x;
                    setpoint_pos_msg.pose.position.y = y;
                    setpoint_pos_msg.pose.position.z = z;

                    setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
                    setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
                    setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
                    setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
                    setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
                    setpoint_pos_pub.publish(setpoint_pos_msg);
                }

                if(z_delay <= 0.0)
                {
                    if (print_flag_land == 1)
                    {
                        ROS_INFO("Landing complete!");
                        print_flag_land = 0;
                    }
//                        pos_ref_start_msg.pose.position.z = 0.0;
                    climbed_flag = false;
                    landed_flag = true;
                    t_last = ros::Time::now().toSec();
                }

                x_last = x;
                y_last = y;
                z_last = z;

                servo_pub.publish(servo_pos_msg);
                traj_on_pub.publish(traj_start_msg);
                reg_on_pub.publish(reg_on_msg);
                ros::spinOnce();
                rate.sleep();
            }
        }

        u = (x - x_last)/sampleTime;
        v = (y - y_last)/sampleTime;
        w = (z - z_last)/sampleTime;

//        std::cout<<"absolute vel along x & y = "<<sqrt(u*u + v*v)<<"\n";

        x_last = x;
        y_last = y;
        z_last = z;

        if(!pub_setpoint_pos)
        {
            reftrajectory_msg.x = x;
            reftrajectory_msg.y = y;
            reftrajectory_msg.z = z;
            ref_pos_pub.publish(reftrajectory_msg);
            reftrajectory_delay_msg.x = x_delay;
            reftrajectory_delay_msg.y = y_delay;
            reftrajectory_delay_msg.z = z_delay;
            ref_pos_delay_pub.publish(reftrajectory_delay_msg);

            reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
            reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
            reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
            ref_vel_pub.publish(reftrajectory_vel_msg);

            ref_yaw_pub.publish(ref_yaw_msg);
        }
        else
        {
            setpoint_pos_msg.header.stamp = ros::Time::now();
            setpoint_pos_msg.pose.position.x = x;
            setpoint_pos_msg.pose.position.y = y;
            setpoint_pos_msg.pose.position.z = z;

            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
            setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
            setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
            setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
            setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
            setpoint_pos_pub.publish(setpoint_pos_msg);
        }

        if(drop_start == 1 && traj_start)
        {
            time_drop = time_drop + 0.01;
//            float delay_drop = (3.15*7/3) * (2/drop_type);
            float delay_drop = (0.5*time_period) * (2/drop_type);

            if(time_drop > delay_drop && count_ < 4)
            {
                if (count_ == count_start)
                    count_ = count_ + num_drops;
                else
                    count_ = count_ + 1;
                if (count_ >= 0)
                {
                    servo_pos_msg.data = count_/4.0;
                    ROS_INFO("servo_pos_msg %f",servo_pos_msg.data);
                }
                time_drop = 0.0;
            }
            else
            {
                servo_pos_msg.data = servo_pos_msg.data;
//                ROS_INFO("servo_pos_msg %f",servo_pos_msg.data);
            }
        }
        else
        {
            servo_pos_msg.data = servo_pos_msg.data;
            time_drop = 0.0;
            if (count_ != count_start)
            {
                count_ = count_start;
                servo_pos_msg.data = 0.0;
            }
        }

        servo_pub.publish(servo_pos_msg);
        traj_on_pub.publish(traj_start_msg);
        reg_on_pub.publish(reg_on_msg);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}
