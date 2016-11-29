#include <ros/ros.h>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <heel_toe_planner/Controller.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_controller");
    ros::NodeHandle nh;
    //create client
    Reemc_Trajectory_Control control(nh);
    control.Step();



    return 0;
}


///back_camera/camera_info
///back_camera/image
///back_camera/image/compressed
///back_camera/image/compressed/parameter_descriptions
///back_camera/image/compressed/parameter_updates
///back_camera/image/compressedDepth
///back_camera/image/compressedDepth/parameter_descriptions
///back_camera/image/compressedDepth/parameter_updates
///back_camera/image/theora
///back_camera/image/theora/parameter_descriptions
///back_camera/image/theora/parameter_updates
///back_camera/parameter_descriptions
///back_camera/parameter_updates
///base_link_wrench
///clock
///gains/arm_left_1_joint/parameter_descriptions
///gains/arm_left_1_joint/parameter_updates
///gains/arm_left_2_joint/parameter_descriptions
///gains/arm_left_2_joint/parameter_updates
///gains/arm_left_3_joint/parameter_descriptions
///gains/arm_left_3_joint/parameter_updates
///gains/arm_left_4_joint/parameter_descriptions
///gains/arm_left_4_joint/parameter_updates
///gains/arm_left_5_joint/parameter_descriptions
///gains/arm_left_5_joint/parameter_updates
///gains/arm_left_6_joint/parameter_descriptions
///gains/arm_left_6_joint/parameter_updates
///gains/arm_left_7_joint/parameter_descriptions
///gains/arm_left_7_joint/parameter_updates
///gains/arm_right_1_joint/parameter_descriptions
///gains/arm_right_1_joint/parameter_updates
///gains/arm_right_2_joint/parameter_descriptions
///gains/arm_right_2_joint/parameter_updates
///gains/arm_right_3_joint/parameter_descriptions
///gains/arm_right_3_joint/parameter_updates
///gains/arm_right_4_joint/parameter_descriptions
///gains/arm_right_4_joint/parameter_updates
///gains/arm_right_5_joint/parameter_descriptions
///gains/arm_right_5_joint/parameter_updates
///gains/arm_right_6_joint/parameter_descriptions
///gains/arm_right_6_joint/parameter_updates
///gains/arm_right_7_joint/parameter_descriptions
///gains/arm_right_7_joint/parameter_updates
///gains/head_1_joint/parameter_descriptions
///gains/head_1_joint/parameter_updates
///gains/head_2_joint/parameter_descriptions
///gains/head_2_joint/parameter_updates
///gains/leg_left_1_joint/parameter_descriptions
///gains/leg_left_1_joint/parameter_updates
///gains/leg_left_2_joint/parameter_descriptions
///gains/leg_left_2_joint/parameter_updates
///gains/leg_left_3_joint/parameter_descriptions
///gains/leg_left_3_joint/parameter_updates
///gains/leg_left_4_joint/parameter_descriptions
///gains/leg_left_4_joint/parameter_updates
///gains/leg_left_5_joint/parameter_descriptions
///gains/leg_left_5_joint/parameter_updates
///gains/leg_left_6_joint/parameter_descriptions
///gains/leg_left_6_joint/parameter_updates
///gains/leg_right_1_joint/parameter_descriptions
///gains/leg_right_1_joint/parameter_updates
///gains/leg_right_2_joint/parameter_descriptions
///gains/leg_right_2_joint/parameter_updates
///gains/leg_right_3_joint/parameter_descriptions
///gains/leg_right_3_joint/parameter_updates
///gains/leg_right_4_joint/parameter_descriptions
///gains/leg_right_4_joint/parameter_updates
///gains/leg_right_5_joint/parameter_descriptions
///gains/leg_right_5_joint/parameter_updates
///gains/leg_right_6_joint/parameter_descriptions
///gains/leg_right_6_joint/parameter_updates
///gazebo/link_states
///gazebo/model_states
///gazebo/parameter_descriptions
///gazebo/parameter_updates
///gazebo/set_link_state
///gazebo/set_model_state
///ground_truth_odom
///left_scan
///right_scan
///rosout
///rosout_agg
///sonar_torso
///stereo/left/camera_info
///stereo/left/image
///stereo/left/image/compressed
///stereo/left/image/compressed/parameter_descriptions
///stereo/left/image/compressed/parameter_updates
///stereo/left/image/compressedDepth
///stereo/left/image/compressedDepth/parameter_descriptions
///stereo/left/image/compressedDepth/parameter_updates
///stereo/left/image/theora
///stereo/left/image/theora/parameter_descriptions
///stereo/left/image/theora/parameter_updates
///stereo/left/parameter_descriptions
///stereo/left/parameter_updates
///stereo/right/camera_info
///stereo/right/image
///stereo/right/image/compressed
///stereo/right/image/compressed/parameter_descriptions
///stereo/right/image/compressed/parameter_updates
///stereo/right/image/compressedDepth
///stereo/right/image/compressedDepth/parameter_descriptions
///stereo/right/image/compressedDepth/parameter_updates
///stereo/right/image/theora
///stereo/right/image/theora/parameter_descriptions
///stereo/right/image/theora/parameter_updates
///stereo/right/parameter_descriptions
///stereo/right/parameter_updates
