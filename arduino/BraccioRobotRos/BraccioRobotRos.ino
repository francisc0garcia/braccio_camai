#include <BraccioRobot.h>
#include <Servo.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

Position initial_position(90, 90, 90, 90, 45,  GRIPPER_CLOSED);
Position current_position;
int speed_arm = 180;

int req_servo_angle[7];

void jointstates_callback( const sensor_msgs::JointState& joint);
void gripper_callback( const std_msgs::Bool& state);

ros::NodeHandle  nh;

ros::Subscriber<sensor_msgs::JointState> join_states_sub("joint_states", jointstates_callback);

sensor_msgs::JointState current_js_request;
sensor_msgs::JointState js_msg;
ros::Publisher pub_joint_state("current_state", &js_msg);

bool new_js_request = false;
float pos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float velocity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float effort[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
char  *names[] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint"};

void setup() 
{
    //init() will start the robot and move to the supplied initial position.
    BraccioRobot.init(initial_position);

    js_msg.position = pos;
    js_msg.position_length=6;

    js_msg.name = names;
    js_msg.name_length=6;

    js_msg.effort = effort;
    js_msg.effort_length=6;

    js_msg.velocity = velocity;
    js_msg.velocity_length=6;

    // wait for the arm to be ready
    delay(3000);

    // connect with ROS
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_joint_state);
    nh.subscribe(join_states_sub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    // set initial pose and publish it
    pos[0] = initial_position.getBase() / 57.2958;
    pos[1] = initial_position.getShoulder() / 57.2958;
    pos[2] = initial_position.getElbow() / 57.2958;
    pos[3] = initial_position.getWristRotation() / 57.2958;
    pos[4] = initial_position.getWrist() / 57.2958;
    pos[5] = initial_position.getGripper() / 57.2958;
    js_msg.position = pos;
    js_msg.header.stamp = nh.now();
    js_msg.header.frame_id = "braccio_arm";
    pub_joint_state.publish(&js_msg);

    nh.loginfo("Robot arm is Connected");
}

void loop() 
{ 
    nh.spinOnce();

    if(new_js_request)
      move_arm();
    
    // return current joint state
    js_msg.header.stamp = nh.now();
    js_msg.position = pos;
    pub_joint_state.publish(&js_msg);
}

void move_arm()
{
    // Move the robot to the position with specified speed
    current_position.set(req_servo_angle[0], req_servo_angle[1], req_servo_angle[2], req_servo_angle[3],  req_servo_angle[4],  map(req_servo_angle[5], 70, 90, 0, 70));
    BraccioRobot.moveToPosition(current_position, speed_arm);
}

void jointstates_callback( const sensor_msgs::JointState& joint)
{
    for(int i = 0; i < 6; i++)
    {
        req_servo_angle[i] = joint.position[i] * 57.2958;
        pos[i] = joint.position[i];
    }
    new_js_request = true;
}
