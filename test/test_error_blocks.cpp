#include <urdf/model.h>
#include <ubr_calibration/optimizer.h>
#include <gtest/gtest.h>

std::string robot_description =
"<?xml version='1.0' ?>"
"<robot name='maxwell'>"
"  <link name='base_link'/>"
"  <joint name='torso_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='-0.00635 0 0.7914'/>"
"    <parent link='base_link'/>"
"    <child link='torso_link'/>"
"  </joint>"
"  <link name='torso_link'/>"
"  <joint name='torso_actuator_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='torso_link'/>"
"    <child link='torso_actuator_link'/>"
"  </joint>"
"  <link name='torso_actuator_link'/>"
"  <joint name='arm_lift_joint' type='prismatic'>"
"    <axis xyz='0 0 1'/>"
"    <limit effort='30' lower='-0.464' upper='0' velocity='0.0508'/>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='torso_link'/>"
"    <child link='arm_lift_link'/>"
"  </joint>"
"  <link name='arm_lift_link'/>"
"  <joint name='arm_base_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.0611 0 0'/>"
"    <parent link='arm_lift_link'/>"
"    <child link='arm_link'/>"
"  </joint>"
"  <link name='arm_link'/>"
"  <joint name='arm_shoulder_pan_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='arm_link'/>"
"    <child link='arm_shoulder_pan_servo_link'/>"
"  </joint>"
"  <link name='arm_shoulder_pan_servo_link'/>"
"  <joint name='arm_shoulder_pan_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0.0235'/>"
"    <axis xyz='0 0 1'/>"
"    <limit effort='30' lower='-1.57' upper='1.57' velocity='0.524'/>"
"    <parent link='arm_shoulder_pan_servo_link'/>"
"    <child link='arm_shoulder_pan_link'/>"
"  </joint>"
"  <link name='arm_shoulder_pan_link'/>"
"  <joint name='arm_shoulder_lift_servo_joint' type='fixed'>"
"    <origin rpy='0 -1.57 0' xyz='0 0 0.0526'/>"
"    <parent link='arm_shoulder_pan_link'/>"
"    <child link='arm_shoulder_lift_servo_link'/>"
"  </joint>"
"  <link name='arm_shoulder_lift_servo_link'/>"
"  <joint name='arm_shoulder_lift_joint' type='revolute'>"
"    <origin rpy='0 1.57 0' xyz='0 0 0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.77' upper='1.317' velocity='0.524'/>"
"    <parent link='arm_shoulder_lift_servo_link'/>"
"    <child link='arm_shoulder_lift_link'/>"
"  </joint>"
"  <link name='arm_shoulder_lift_link'/>"
"  <joint name='arm_upperarm_roll_servo_joint' type='fixed'>"
"    <origin rpy='1.57 1.57 0' xyz='0.0712978 0 0'/>"
"    <parent link='arm_shoulder_lift_link'/>"
"    <child link='arm_upperarm_roll_servo_link'/>"
"  </joint>"
"  <link name='arm_upperarm_roll_servo_link'/>"
"  <joint name='arm_upperarm_roll_joint' type='revolute'>"
"    <origin rpy='-1.57 0 1.57' xyz='0 0 0'/>"
"    <axis xyz='1 0 0'/>"
"    <limit effort='30' lower='-2' upper='2' velocity='0.524'/>"
"    <parent link='arm_upperarm_roll_servo_link'/>"
"    <child link='arm_upperarm_roll_link'/>"
"  </joint>"
"  <link name='arm_upperarm_roll_link'/>"
"  <joint name='arm_elbow_flex_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0.0869955 0 0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.57' upper='2.617' velocity='0.524'/>"
"    <parent link='arm_upperarm_roll_link'/>"
"    <child link='arm_elbow_flex_link'/>"
"  </joint>"
"  <link name='arm_elbow_flex_link'/>"
"  <joint name='arm_forearm_fixed_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <parent link='arm_elbow_flex_link'/>"
"    <child link='arm_forearm_link'/>"
"  </joint>"
"  <link name='arm_forearm_link'/>"
"  <joint name='arm_wrist_flex_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.125 0 0'/>"
"    <parent link='arm_forearm_link'/>"
"    <child link='arm_wrist_flex_servo_link'/>"
"  </joint>"
"  <link name='arm_wrist_flex_servo_link'/>"
"  <joint name='arm_wrist_flex_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0.0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.57' upper='1.57' velocity='0.785'/>"
"    <parent link='arm_wrist_flex_servo_link'/>"
"    <child link='arm_wrist_flex_link'/>"
"  </joint>"
"  <link name='arm_wrist_flex_link'/>"
"  <joint name='arm_wrist_roll_joint' type='revolute'>"
"    <axis xyz='1 0 0'/>"
"    <limit effort='30' lower='-2.617' upper='2.617' velocity='0.785'/>"
"    <origin rpy='0 0 0' xyz='0.031 0 0'/>"
"    <parent link='arm_wrist_flex_link'/>"
"    <child link='arm_wrist_roll_link'/>"
"  </joint>"
"  <link name='arm_wrist_roll_link'/>"
"  <joint name='gripper_joint' type='fixed'>"
"    <axis xyz='0 0 1'/>"
"    <origin rpy='0 0 0' xyz='0.15 0 -0.015'/>"
"    <parent link='arm_wrist_roll_link'/>"
"    <child link='gripper_link'/>"
"  </joint>"
"  <link name='gripper_link'/>"
"  <joint name='head_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0.512375'/>"
"    <parent link='torso_link'/>"
"    <child link='head_link'/>"
"  </joint>"
"  <link name='head_link'/>"
"  <joint name='head_pan_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.010 0 0.0254'/>"
"    <parent link='head_link'/>"
"    <child link='head_pan_servo_link'/>"
"  </joint>"
"  <link name='head_pan_servo_link'/>"
"  <joint name='head_pan_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0.019'/>"
"    <axis xyz='0 0 1'/>"
"    <limit effort='30' lower='-2.617' upper='2.617' velocity='1.0'/>"
"    <parent link='head_pan_servo_link'/>"
"    <child link='head_pan_link'/>"
"  </joint>"
"  <link name='head_pan_link'/>"
"  <joint name='head_tilt_servo_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0.0415'/>"
"    <parent link='head_pan_link'/>"
"    <child link='head_tilt_servo_link'/>"
"  </joint>"
"  <link name='head_tilt_servo_link'/>"
"  <joint name='head_tilt_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <axis xyz='0 1 0'/>"
"    <limit effort='30' lower='-1.57' upper='1.57' velocity='1.0'/>"
"    <parent link='head_tilt_servo_link'/>"
"    <child link='head_tilt_link'/>"
"  </joint>"
"  <link name='head_tilt_link'/>"
"  <joint name='head_camera_frame_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0 0 0.026'/>"
"    <parent link='head_tilt_link'/>"
"    <child link='head_camera_frame'/>"
"  </joint>"
"  <link name='head_camera_frame'/>"
"  <joint name='head_camera_ir_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='0.01905 -0.0269875 0.032075'/>"
"    <parent link='head_camera_frame'/>"
"    <child link='head_camera_ir_link'/>"
"  </joint>"
"  <link name='head_camera_ir_link'/>"
"  <joint name='head_camera_ir_optical_frame_joint' type='fixed'>"
"    <origin rpy='-1.57 0.0 -1.57' xyz='0 0 0'/>"
"    <parent link='head_camera_ir_link'/>"
"    <child link='head_camera_ir_optical_frame'/>"
"  </joint>"
"  <link name='head_camera_ir_optical_frame'/>"
"  <joint name='head_camera_rgb_joint' type='fixed'>"
"    <origin rpy='0 -0.08 0' xyz='0 0.0552875 0'/>"
"    <parent link='head_camera_ir_link'/>"
"    <child link='head_camera_rgb_link'/>"
"  </joint>"
"  <link name='head_camera_rgb_link'/>"
"  <joint name='head_camera_rgb_optical_frame_joint' type='fixed'>"
"    <origin rpy='-1.57 0.0 -1.57' xyz='0 0 0'/>"
"    <parent link='head_camera_rgb_link'/>"
"    <child link='head_camera_rgb_optical_frame'/>"
"  </joint>"
"  <link name='head_camera_rgb_optical_frame'/>"
"</robot>";

TEST(OptimizerTest, error_blocks_maxwell)
{
  Optimizer opt(robot_description, "base_link", "gripper_link");

  std::vector<ubr_calibration::CalibrationData> data;
  ubr_calibration::CalibrationData msg;
  msg.joint_states.name.resize(9);
  msg.joint_states.name[0] = "arm_lift_flex_joint";
  msg.joint_states.name[1] = "arm_shoulder_pan_joint";
  msg.joint_states.name[2] = "arm_shoulder_lift_joint";
  msg.joint_states.name[3] = "arm_upperarm_roll_joint";
  msg.joint_states.name[4] = "arm_elbow_flex_joint";
  msg.joint_states.name[5] = "arm_wrist_flex_joint";
  msg.joint_states.name[6] = "arm_wrist_roll_joint";
  msg.joint_states.name[7] = "head_pan_joint";
  msg.joint_states.name[8] = "head_tilt_joint";
  msg.joint_states.position.resize(9);
  msg.joint_states.position[0] = 0.0;
  msg.joint_states.position[1] = -0.814830;
  msg.joint_states.position[2] = -0.00022290000000002586;
  msg.joint_states.position[3] = 0.0;
  msg.joint_states.position[4] = -0.7087341;
  msg.joint_states.position[5] = 0.0;
  msg.joint_states.position[6] = 0.0;
  msg.joint_states.position[7] = -0.8280187999999999;
  msg.joint_states.position[8] = 0.6358500000000002;
  msg.rgbd_observations.resize(1);
  msg.rgbd_observations[0].header.frame_id = "head_camera_rgb_optical_frame";
  msg.rgbd_observations[0].point.x = -0.0143163670728;
  msg.rgbd_observations[0].point.y = 0.111304592065;
  msg.rgbd_observations[0].point.z = 0.522079317365;
  data.push_back(msg);

  msg.joint_states.position[1] = -0.019781999999999966;
  msg.joint_states.position[7] = 0.0;
  msg.rgbd_observations[0].point.x = 0.0365330705881;
  msg.rgbd_observations[0].point.y = 0.102609552493;
  msg.rgbd_observations[0].point.z = 0.536061220027;
  data.push_back(msg);

  msg.joint_states.position[1] = 0.883596;
  msg.joint_states.position[7] = 0.9442135999999999;
  msg.rgbd_observations[0].point.x = 0.0942445346646;
  msg.rgbd_observations[0].point.y = 0.11409172323;
  msg.rgbd_observations[0].point.z = 0.517497963716;
  data.push_back(msg);

  opt.optimize(data, false);

  EXPECT_LT(opt.summary()->initial_cost, 1e-20);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
