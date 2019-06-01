#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>

class DirectJointInterface
{
public:
    ros::Publisher pub_js;
    sensor_msgs::JointState current_js_state;

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
    DirectJointInterface()
    {
        ros::NodeHandle n;
        ros::NodeHandle private_node("~");

        // Read parameters
        private_node.param<int>("camera_width", camera_width, 640);
        private_node.param<int>("camera_height", camera_height, 480);
        private_node.param<int>("camera_radius", camera_radius, 10);
        private_node.param<double>("goal_tolerance", goal_tolerance, 0.05);
        private_node.param<double>("planning_time", planning_time, 2.0);
        private_node.param<double>("p_gain_yaw", p_gain_yaw, 0.001);
        private_node.param<double>("p_gain_pitch", p_gain_pitch, 0.001);
        private_node.param<std::vector<double>>("initial_pose", initial_pose, std::vector<double>());

        ROS_INFO_STREAM("camera_width: " << camera_width);
        ROS_INFO_STREAM("camera_height: " << camera_height);
        ROS_INFO_STREAM("camera_radius: " << camera_radius);
        ROS_INFO_STREAM("goal_tolerance: " << goal_tolerance);
        ROS_INFO_STREAM("planning_time: " << planning_time);
        ROS_INFO_STREAM("p_gain_yaw: " << p_gain_yaw);
        ROS_INFO_STREAM("p_gain_pitch: " << p_gain_pitch);

        center_x = camera_width / 2.0;
        center_y = camera_height / 2.0;

        pub_js = n.advertise<sensor_msgs::JointState>("joint_states", 1, false);
        ros::Subscriber sub_box = n.subscribe("bounding_box", 1, &DirectJointInterface::callbackBoundingBox, this);
        ros::Subscriber sub_js = n.subscribe("current_state", 1, &DirectJointInterface::callbackJointStateCurrent, this);

        // fix initial pose if defined
        if (initial_pose.size() > 0)
        {
            sensor_msgs::JointState js;
            js.position = initial_pose;

            ROS_INFO_STREAM("Waiting for reaching initial pose");

            while (ros::ok() && !goal_reached(current_js_state, js, goal_tolerance))
            {
                js.header.stamp = ros::Time::now();
                pub_js.publish(js);
                ros::spinOnce();
                ros::Duration(0.2).sleep();
            }

            ROS_INFO_STREAM("Initial position reached!");
        }

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

    ~DirectJointInterface(){};

    void callbackJointStateCurrent(const sensor_msgs::JointState::ConstPtr &msg)
    {
        current_js_state = *msg;
    }

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
        // copy current joint states
        sensor_msgs::JointState js;
        for (double p : current_js_state.position)
            js.position.push_back(p);

        js.header.stamp = ros::Time::now();

        int index_joint_x = 0;
        int index_joint_y = 3;

        double p_x = p_gain_yaw * (1 - box_area); // the closer the slower
        double p_y = p_gain_pitch * (1 - box_area);

        // use a simple p controller to update new joint angles
        js.position[index_joint_x] -= (delta_x * p_x);
        js.position[index_joint_y] += (delta_y * p_y);

        pub_js.publish(js);
    }

    bool goal_reached(sensor_msgs::JointState current, sensor_msgs::JointState goal, double threshold)
    {
        if (current.position.size() == 0)
            return false;

        for (int i = 0; i < current.position.size(); i++)
        {
            if (std::abs(goal.position[i] - current.position[i]) > threshold)
                return false;
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DirectJointInterface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    DirectJointInterface();

    //ros::spin();
    ros::waitForShutdown();

    return 0;
}