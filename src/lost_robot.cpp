#include <cstdlib>
#include <ctime>
#include <string>
#include <fstream>
#include "utility.h"
#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"



bool is_found_exit = false;

void pub_marker(const ros::Publisher &pub, double x, double y, double angle, std::string frame_name)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = frame_name;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);

    pub.publish(msg);
}

void lost_robot_callback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Ok, I'm near" << msg.data);
    is_found_exit = true;
}

int main(int argc, char **argv)
{
	std::srand((unsigned)std::time(0));
    ros::init(argc, argv, "lost_node");
    ros::NodeHandle nh;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
        nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin("/home/?vlad?/.gazebo/models/pioneer2dx/model.sdf");

    std::string model;
    std::string buf;
    while (std::getline(fin, buf))
    {
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "lost_robot";
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
    //Spawning finished

    ros::Publisher pub =
        nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10, true);

    ros::Subscriber sub = nh.subscribe("/pt_topic", 10, &lost_robot_callback);

    

    std::string base_frame_id = "world";
    std::string frame_id = "/the_lost";
    std::string helper_frame_id = "/the_helper";

    const double max_speed = 0.5;
    const int max_steps = 30;
    const double da = M_PI / 20;
    double speed_x = 0.0;
    double speed_y = 0.0;
    double x = 0.0;
    double y = 0.0;
    double distance = 0.0;
    double old_angle = 0.0;
    double new_angle = 0.0;
    int wait = 60;
    int steps = 0;
    isLost = true;

    ros::Rate r(30);
    double dt = (double)r.expectedCycleTime().toSec();

    while (ros::ok() && !is_found_exit)
    {
        double sx, sy;
        broadcast_pose(x, y, base_frame_id, frame_id);
        try
        {
            take_pose(frame_id, helper_frame_id, sx, sy);
        }
        catch (const tf::TransformException &e)
        {
            r.sleep();
            continue;
        }
		
        distance = calc_dist(x, y, sx + x, sy + y);
        old_angle = calc_angle(sx, sy);
		if (isLost)
		{
                steps = (steps + 1) % max_steps;
                if (steps == 0)
                {
                    speed_x = 0.3 * max_speed * (double)(std::rand() % 2);
                    speed_y = 0.3 * max_speed * (double)(std::rand() % 2);
                }
                old_angle = calc_angle(speed_x, speed_y);
                if (distance < 1.8)
                {
                    isLost = false;
                }             
        }
		else
        {
			if (wait > 0)
			{
				wait--;
				speed_x = 0;
				speed_y = 0;
			}
			else
			{
				speed_x = max_speed * cos(old_angle);
				speed_y = max_speed * sin(old_angle);
			}
        }

        if (fabs(new_angle - old_angle) < 4 * da)
        {
            x += speed_x * dt;
            y += speed_y * dt;
            new_angle = old_angle;
        }
        else
        {
			if (old_angle > new_angle)
			{
				new_angle = (new_angle + da > M_PI ? 0 : new_angle + da);
			}
			else
			{
				new_angle = (new_angle - da < -M_PI ? 0 : new_angle - da);
			}
        }
        // ROS_INFO_STREAM("a=" << a << ", old_angle=" << old_angle);
        pub_marker(pub, x, y, a, frame_id);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

