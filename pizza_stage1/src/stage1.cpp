// Based on https://github.com/ros-planning/moveit_tutorials/blob/master/doc/pick_place/src/pick_place_tutorial.cpp

// standard library
#include <vector>
#include <string>

// ROS
#include "ros/ros.h"

// MoveIt
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/PlaceLocation.h"
#include "moveit_msgs/Grasp.h"

#include "gazebo_msgs/GetModelState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"

// TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class Stage1Commander
{
    private:
    static const float manipulator_offset, manipulator_depth;
    tf2::Quaternion grab_orientation;
    ros::NodeHandle nh_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface group_;
    ros::ServiceServer task1_srv_;
    ros::ServiceClient model_state_srv_;
    geometry_msgs::Pose pizza_orig_pose_;

    geometry_msgs::Pose get_model_pose(std::string model_name)
    {
        gazebo_msgs::GetModelState req;
        req.request.model_name = model_name;
        req.request.relative_entity_name = "world";
        model_state_srv_.call(req);

        if (!req.response.success)
            ROS_FATAL("Get model state service failed!");
        return req.response.pose;
    }

    void add_collision_objects()
    {
        ROS_INFO("Adding objects to MoveIt scene...");

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(2);

        collision_objects[0].id = "table";
        collision_objects[0].header.frame_id = "world";

        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 1.0;
        collision_objects[0].primitives[0].dimensions[1] = 2.0;
        collision_objects[0].primitives[0].dimensions[2] = 0.2;

        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0] = get_model_pose("table");

        collision_objects[0].operation = collision_objects[0].ADD;

        collision_objects[1].id = "pizza_box";
        collision_objects[1].header.frame_id = "world";

        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
        collision_objects[1].primitives[0].dimensions.resize(3);
        collision_objects[1].primitives[0].dimensions[0] = 0.32;
        collision_objects[1].primitives[0].dimensions[1] = 0.32;
        collision_objects[1].primitives[0].dimensions[2] = 0.03;

        collision_objects[1].primitive_poses.resize(1);
        pizza_orig_pose_ = get_model_pose("pizza_box");
        collision_objects[1].primitive_poses[0] = pizza_orig_pose_;

        collision_objects[1].operation = collision_objects[1].ADD;

        planning_scene_interface_.applyCollisionObjects(collision_objects);

        ROS_INFO("Done adding objects!");
    }

    bool pick()
    {
        ROS_INFO("Executing pick...");

        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);

        grasps[0].grasp_pose.header.frame_id = "world";
        grasps[0].grasp_pose.pose = pizza_orig_pose_;
        grasps[0].grasp_pose.pose.position.x -= manipulator_offset;
        grasps[0].grasp_pose.pose.position.x -= 0.32 / 2;  // grab edge of box
        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(grab_orientation);

        open_gripper(grasps[0].pre_grasp_posture);
        grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
        grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
        grasps[0].pre_grasp_approach.min_distance = manipulator_depth - 0.01;
        grasps[0].pre_grasp_approach.desired_distance = manipulator_depth;

        closed_gripper(grasps[0].grasp_posture);
        grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.1;
        grasps[0].post_grasp_retreat.desired_distance = 0.25;

        grasps[0].max_contact_force = 10.;
        grasps[0].allowed_touch_objects = { "pizza_box" };

        group_.setSupportSurfaceName("table");
        bool success = (bool)group_.pick("pizza_box", grasps);

        ROS_INFO("%s executing pick!", success?"Done":"Failed");
        return success;
    }

    bool place()
    {
        ROS_INFO("Executing place...");

        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);

        place_location[0].place_pose.header.frame_id = "world";
        place_location[0].place_pose.pose = pizza_orig_pose_;
        place_location[0].place_pose.pose.position.y -= .5;  // move it half a meter in -y direction
        place_location[0].place_pose.pose.orientation = tf2::toMsg(grab_orientation);

        place_location[0].pre_place_approach.direction.header.frame_id = "world";
        place_location[0].pre_place_approach.direction.vector.z = -1.0;
        place_location[0].pre_place_approach.min_distance = manipulator_depth - 0.01;
        place_location[0].pre_place_approach.desired_distance = manipulator_depth;

        place_location[0].post_place_retreat.direction.header.frame_id = "world";
        place_location[0].post_place_retreat.direction.vector.x = -1.0;
        place_location[0].post_place_retreat.min_distance = 0.1;
        place_location[0].post_place_retreat.desired_distance = 0.25;

        place_location[0].allowed_touch_objects = { "pizza_box" };

        open_gripper(place_location[0].post_place_posture);

        // Set support surface as table2.
        group_.setSupportSurfaceName("table");
        // Call place to place the object using the place locations given.
        bool success = (bool)group_.place("pizza_box", place_location);

        ROS_INFO("%s executing place!", success?"Done":"Failed");
        return success;
    }


    void open_gripper(trajectory_msgs::JointTrajectory& posture)
    {
        /* Add both finger joints of panda robot. */
        posture.joint_names.resize(2);
        posture.joint_names[0] = "panda_finger_joint1";
        posture.joint_names[1] = "panda_finger_joint2";

        /* Set them as open, wide enough for the object to fit. */
        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.04;
        posture.points[0].positions[1] = 0.04;
        posture.points[0].time_from_start = ros::Duration(0.5);
    }

    void closed_gripper(trajectory_msgs::JointTrajectory& posture)
    {
        /* Add both finger joints of panda robot. */
        posture.joint_names.resize(2);
        posture.joint_names[0] = "panda_finger_joint1";
        posture.joint_names[1] = "panda_finger_joint2";

        /* Set them as closed. */
        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.00;
        posture.points[0].positions[1] = 0.00;
        posture.points[0].time_from_start = ros::Duration(0.5);
    }

    bool task1_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        bool success;
        success = pick();
        if (!success) return false;

        // ros::WallDuration(1.0).sleep();
        success = place();
        if (!success) return false;

        return true;
    }


    public:
    Stage1Commander(ros::NodeHandle nh): nh_(nh), group_("panda_arm")
    {
        group_.setPlanningTime(45.0);
        model_state_srv_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        ROS_INFO("End effector link: %s", group_.getEndEffectorLink().c_str());

        grab_orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
        tf2::Quaternion turn_end;
        turn_end.setRPY(0., 0., M_PI / 2);
        grab_orientation *= turn_end;

        add_collision_objects();

        task1_srv_ = nh_.advertiseService("stage1/task1", &Stage1Commander::task1_callback, this);
    }
};


const float Stage1Commander::manipulator_offset = 0.08;
const float Stage1Commander::manipulator_depth = 0.04;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stage1");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // wait for ros & gazebo initialization
    ros::WallDuration(1.0).sleep();

    Stage1Commander st1(nh);

    ros::waitForShutdown();
    return 0;
}
