#include <string>
#include <fstream>
#include "ros/ros.h"
#include "utility.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <std_msgs/String.h>


void pub_helper(const ros::Publisher &pub,  double x,  double y, double angle,std::string frame_name)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = frame_name;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);
    pub.publish(msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "helper_node");
    ros::NodeHandle node;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
        node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin("/home/?vlad?/.gazebo/models/pioneer3at/model.sdf");

    std::string model;
    std::string buf;
    while (std::getline(fin, buf))
    {
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "helper_robot";
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
    //Spawning finished

    ros::Publisher pub =
        node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10, true);

    ros::Publisher pub_to_lost =
        node.advertise<std_msgs::String>("/pt_topic", 10);

    ROS_INFO_STREAM("Helper is started");

    std::string base_frame_id = "world";
    std::string frame_id = "/helper";
    std::string lost_frame_id = "/lost";

    const double max_speed = 0.5;
    const double exit_coord_x  = 3.0;
    const double exit_coord_y = -8.0;
    const double da = M_PI / 20;
    double speed_x = 0.0;
    double speed_y = 0.0;
    double old_angle = 0.0;
    double distance = 0.0;
    double x = exit_coord_x ;
    double y = exit_coord_y;
    double new_angle = 0.0;
    HelperState state = HelperState::Chase;

    ros::Rate r(30);
	isFound = false;
    while (ros::ok())
    {
        broadcast_pose(x, y, base_frame_id, frame_id);
        if (!isFound)
		{
			double lx, ly;
			try
			{
				take_pose(frame_id, lost_frame_id, lx, ly);
			}
			catch (const tf::TransformException &e)
			{
				r.sleep();
				continue;
			}

			distance = range(x, y, lx + x, ly + x);
			old_angle = calc_angle(lx, ly);
			if (distance > 1.5)
			{
				speed_x = 0.1 * max_speed * cos(old_angle);
				speed_y = 0.1 * max_speed * sin(old_angle);
			}
			else
			{
				isFound = true;
			}
		}
		else
		{
			distance = range(x, y, exit_coord_x , exit_coord_y );
			old_angle = calc_angle(exit_coord_x - x, exit_coord_y - y);
			if (distance < 2.0)
			{
				state = HelperState::AtExit;
				std_msgs::String msg;
				msg.data = std::string("This exit");
				pub_to_lost.publish(msg);
				break;
			}
			
			speed_x = 0.1 * max_speed * cos(old_angle);
			speed_y = 0.1 * max_speed * sin(old_angle);
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
        // ROS_INFO_STREAM("a=" << a << ", angle=" << angle);
        pub_marker(pub, x, y, new_angle, frame_id);
        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO_STREAM("Helper is finished");
    return 0;
}

