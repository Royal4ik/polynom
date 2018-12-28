#include <cstdlib>
#include <ctime>
#include <string>
#include <fstream>
#include "transformation.h"
#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"

bool is_exit = false;
bool is_found = false;

void pub_lost(const ros::Publisher &pub, double x, double y, double angle, std::string name)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = name;
    //msg.pose.position.x = x;
    //msg.pose.position.y = y;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);

    pub.publish(msg);
}

void found_topic_callback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("OK");
    is_exit = true;
}

enum class LostState
{
    Waiting,
    Chase
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lost_node");
    ros::NodeHandle nh;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
        nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin("/home/artem/.gazebo/models/pioneer2dx/model.sdf");

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

    ros::Subscriber sub = nh.subscribe("/ftopic", 10, &found_topic_callback);

    std::srand((unsigned)std::time(0));

    std::string base_frame_id = "world";
    std::string frame_id = "/lost";
    std::string helper_frame_id = "/helper";

    const double max_speed = 0.5;
    const int max_steps = 30;
    const double da = M_PI / 20;
    double speed_x = 0.0;
    double speed_y = 0.0;
    double x = 0.0;
    double y = 0.0;
    double dist = 0.0;
    double new_angle = 0.0;
    double old_angle = 0.0;
	int k = 0;
    int steps = 0;
    LostState state = LostState::Waiting;

    ros::Rate r(30);
    double dt = (double)r.expectedCycleTime().toSec();

    while (ros::ok() && !is_exit)
    {
        double help_x, help_y;
        broadcast_pose(x, y, base_frame_id, frame_id);

        try
        {
            take_pose(frame_id, helper_frame_id, help_x, help_y);
        }
        catch (const tf::TransformException &e)
        {
            r.sleep();
            continue;
        }

        dist = calc_distance(x, y, help_x + x, help_y + y);
        new_angle = calc_angle(help_x, help_y);
        if (!is_found)
		{
            steps = (steps + 1) % max_steps;
            if (steps == 1)
            {
                speed_x = 0.02 * max_speed * (double)(std::rand() % 5 - 2);
                speed_y = 0.02 * max_speed * (double)(std::rand() % 5 - 2);
            }
            new_angle = calc_angle(speed_x, speed_y);

            if (dist < 1.5)
            {
                is_found = true;
            }
        }
         else
        {
            speed_x = 0.1 * max_speed * cos(new_angle);
            speed_y = 0.1 * max_speed * sin(new_angle);

			if (k < 10)
			{	
				speed_x = 0;
            	speed_y = 0;
				k++;
			}
        }

        if (fabs(old_angle - new_angle) < 4 * da)
        {
            x += speed_x;
            y += speed_y;
            old_angle = new_angle;
        }
        else
        {
            if (new_angle > old_angle)
            {
                old_angle = (old_angle + da > M_PI ? 0 : old_angle + da);
            }
            else
            {
                old_angle = (old_angle - da < -M_PI ? 0 : old_angle - da);
            }
        }

        pub_lost(pub, x, y, old_angle, srv.request.model_name);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
