#include <Braccio.h>
#include <Servo.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

int req_servo_angle[7];

void jointstates_callback( const sensor_msgs::JointState& joint);

ros::NodeHandle  nh;
ros::Subscriber<sensor_msgs::JointState> joinstates_sub("joint_states", jointstates_callback);

void setup() 
{
    Braccio.begin();
    
    // wait for the arm to be ready
    delay(3000);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(joinstates_sub);
    nh.subscribe(gripper_sub);

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
    Braccio.ServoMovement(20, 
            req_servo_angle[0], 
            req_servo_angle[1], 
            req_servo_angle[2], 
            req_servo_angle[3], 
            req_servo_angle[4], 
        map(req_servo_angle[5], 70, 90, 0, 70)
    );  
}

void jointstates_callback( const sensor_msgs::JointState& joint)
{
    for(int i = 0; i < 7; i++)
    {
        req_servo_angle[i] = joint.position[i] * 57.2958;
    }
}