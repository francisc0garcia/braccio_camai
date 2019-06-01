#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Int16MultiArray.h>

class MoveitJointInterface
{
public:
    std::string planning_group;
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group;

    // camera parameters
    int camera_width, camera_height, camera_radius;
    double center_x, center_y, box_area;
    double camera_current_x, camera_current_y;
    bool bounding_box_updated = false;

    // planning parameters
    double goal_tolerance;
    double planning_time;
    double p_gain_yaw, p_gain_pitch;

    double lim_x_up, lim_x_down, lim_y_up, lim_y_down, lim_z_up, lim_z_down;
    std::vector<double> initial_pose;

public:
    MoveitJointInterface()
    {
        ros::NodeHandle n;
        ros::NodeHandle private_node("~");

        // Read parameters
        private_node.param<std::string>("planning_group", planning_group, std::string("braccio_arm"));
        private_node.param<int>("camera_width", camera_width, 640);
        private_node.param<int>("camera_height", camera_height, 480);
        private_node.param<int>("camera_radius", camera_radius, 10);
        private_node.param<double>("goal_tolerance", goal_tolerance, 0.05);
        private_node.param<double>("planning_time", planning_time, 2.0);
        private_node.param<double>("p_gain_yaw", p_gain_yaw, 0.001);
        private_node.param<double>("p_gain_pitch", p_gain_pitch, 0.001);
        private_node.param<std::vector<double>>("initial_pose", initial_pose, std::vector<double>());

        ROS_INFO_STREAM("planning_group: " << planning_group);
        ROS_INFO_STREAM("camera_width: " << camera_width);
        ROS_INFO_STREAM("camera_height: " << camera_height);
        ROS_INFO_STREAM("camera_radius: " << camera_radius);
        ROS_INFO_STREAM("goal_tolerance: " << goal_tolerance);
        ROS_INFO_STREAM("planning_time: " << planning_time);
        ROS_INFO_STREAM("p_gain_yaw: " << p_gain_yaw);
        ROS_INFO_STREAM("p_gain_pitch: " << p_gain_pitch);

        move_group = new moveit::planning_interface::MoveGroupInterface(planning_group);
        joint_model_group = move_group->getCurrentState()->getJointModelGroup(planning_group);

        ROS_INFO_STREAM("Reference frame [" << move_group->getPlanningFrame().c_str() << "]");
        ROS_INFO_STREAM("End effector link: [" << move_group->getEndEffectorLink().c_str() << "]");

        center_x = camera_width / 2.0;
        center_y = camera_height / 2.0;

        // fix initial pose if defined
        if (initial_pose.size() > 0)
        {
            move_group->setJointValueTarget(initial_pose);
            move_group->move();
            ROS_INFO_STREAM("Initial position reached!");
        }

        ros::Subscriber sub_box = n.subscribe("bounding_box", 1, &MoveitJointInterface::callbackBoundingBox, this);

        ros::Rate loop_rate(10); // 10 hz loop
        while (ros::ok())
        {
            double delta_x = center_x - camera_current_x;
            double delta_y = center_y - camera_current_y;

            if (bounding_box_updated && (std::abs(delta_x) > camera_radius || std::abs(delta_y) > camera_radius))
            {
                planJointToDelta(delta_x, delta_y);
                bounding_box_updated = false;
            }

            loop_rate.sleep();
        }
    };

    ~MoveitJointInterface()
    {
        delete joint_model_group;
        delete move_group;
    };

    void callbackBoundingBox(const std_msgs::Int16MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() == 0)
            return;

        int label_id = msg->data[0];
        int startX = msg->data[1];
        int startY = msg->data[2];
        int endX = msg->data[3];
        int endY = msg->data[4];

        camera_current_x = (endX + startX) / 2.0;
        camera_current_y = (endY + startY) / 2.0;

        double image_area = camera_width * camera_height;

        box_area = ((endX - startX) * (endY - startY)) / image_area;
        bounding_box_updated = true;
    }

    void planJointToDelta(double delta_x, double delta_y)
    {
        // get current joint angles
        std::vector<double> group_variable_values;
        move_group->getCurrentState()->copyJointGroupPositions(move_group->getCurrentState()->getRobotModel()->getJointModelGroup(move_group->getName()), group_variable_values);

        //ROS_INFO_STREAM("joints");
        //for(double j: group_variable_values)
        //    ROS_INFO_STREAM(j);

        int index_joint_x = 0;
        int index_joint_y = 3;

        double p_x = p_gain_yaw * (1 - box_area); // the closer the slower
        double p_y = p_gain_pitch * (1 - box_area);

        // use a simple p controller to update new joint angles
        group_variable_values[index_joint_x] -= (delta_x * p_x);
        group_variable_values[index_joint_y] += (delta_y * p_y);

        // send goal joint to moveit
        move_group->clearPoseTargets();
        //move_group->setPlannerId("RRTConnectkConfigDefault");
        move_group->setGoalTolerance(goal_tolerance);
        move_group->setPlanningTime(planning_time);
        move_group->setJointValueTarget(group_variable_values);
        move_group->asyncMove();
    }

    void planToDelta(double delta_x, double delta_y)
    {
        ROS_INFO_STREAM("planToDelta: delta_x: " << delta_x << " delta_y " << delta_y);
        geometry_msgs::Pose current_position = move_group->getCurrentPose().pose;

        move_group->setGoalTolerance(0.05);
        move_group->setPlanningTime(10);

        double p = 0.002;

        double x = 0.05;
        double y = current_position.position.y + (delta_x * p);
        double z = current_position.position.z + (delta_y * p);

        ROS_INFO_STREAM("planToDelta: y: " << y << " z " << z);

        lim_x_up = 0.3;
        lim_x_down = -0.3;

        lim_y_up = 0.3;
        lim_y_down = -0.3;

        lim_z_up = 0.4;
        lim_z_down = 0.0;

        y = std::max(lim_y_down, y);
        y = std::min(lim_y_up, y);

        z = std::max(lim_z_down, z);
        z = std::min(lim_z_up, z);

        x = std::max(lim_x_down, x);
        x = std::min(lim_x_up, x);

        ROS_INFO_STREAM("planToDelta: new position: y: " << y << " z " << z
                                                         << " current.y: " << current_position.position.y << " current.z " << current_position.position.z);
        z = 0.1;

        geometry_msgs::Pose target_pose;
        tf2::Quaternion q;
        q.setRPY(1.48891, 0.0, 0.0);
        target_pose.orientation.x = q[0];
        target_pose.orientation.y = q[1];
        target_pose.orientation.z = q[2];
        target_pose.orientation.w = q[3];

        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.005;
        double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.0) // 10 % of poses are reached
        {
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            ROS_INFO_STREAM("Computed trajectory with [" << trajectory.joint_trajectory.points.size() << "] points, fraction: " << fraction);
            my_plan.trajectory_ = trajectory;

            bool success = (move_group->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success)
                ROS_INFO_STREAM("plan executed succesfully");
        }
        else
        {
            ROS_INFO_STREAM("Fail to get cartesian path: " << fraction);
        }

        /*
        //move_group->setPositionTarget(x, y, z);
        move_group->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group->execute(my_plan);
            ROS_INFO("Executed!");
        }*/
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MoveitJointInterface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveitJointInterface();

    //ros::spin();
    ros::waitForShutdown();

    return 0;
}