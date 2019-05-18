#include <BraccioRobot.h>
#include <Servo.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

Position initial_position(90, 90, 90, 90, 45,  GRIPPER_CLOSED);
Position current_position;
int speed_arm = 150;

int req_servo_angle[7];

void jointstates_callback( const sensor_msgs::JointState& joint);
void gripper_callback( const std_msgs::Bool& state);

ros::NodeHandle  nh;

ros::Subscriber<sensor_msgs::JointState> join_states_sub("joint_states", jointstates_callback);

void setup() 
{
    //init() will start the robot and move to the supplied initial position.
    BraccioRobot.init(initial_position);

    // wait for the arm to be ready
    delay(3000);

    // connect with ROS
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(join_states_sub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("Robot arm is Connected");
}

void loop() 
{ 
    nh.spinOnce();
    move_arm();
}

void move_arm()
{
    // Move the robot to the position with specified speed
    current_position.set(req_servo_angle[0], req_servo_angle[1], req_servo_angle[2], req_servo_angle[3],  req_servo_angle[4],  map(req_servo_angle[5], 70, 90, 0, 70));
    BraccioRobot.moveToPosition(current_position, speed_arm);
}

void jointstates_callback( const sensor_msgs::JointState& joint)
{
    for(int i = 0; i < 7; i++)
    {
        req_servo_angle[i] = joint.position[i] * 57.2958;
    }
}
