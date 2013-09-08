#include <urdf/model.h>
#include <ubr_calibration/update_urdf.h>
#include <gtest/gtest.h>

std::string robot_description =
"<?xml version='1.0' ?>"
"<robot name='test'>"
"  <link name='link_0'/>"
"  <joint name='first_joint' type='fixed'>"
"    <origin rpy='0 0 0' xyz='1 1 1'/>"
"    <parent link='link_0'/>"
"    <child link='link_1'/>"
"  </joint>"
"  <link name='link_1'/>"
"  <joint name='second_joint' type='revolute'>"
"    <origin rpy='0 0 0' xyz='0 0 0'/>"
"    <axis xyz='0 0 1'/>"
"    <limit effort='30' lower='-1.57' upper='1.57' velocity='0.524'/>"
"    <parent link='link_1'/>"
"    <child link='link_2'/>"
"  </joint>"
"  <link name='link_2'/>"
"  <joint name='third_joint' type='fixed'>"
"    <origin rpy='0 -1.5 0' xyz='0 0 0.0526'/>"
"    <parent link='link_2'/>"
"    <child link='link_3'/>"
"  </joint>"
"  <link name='link_3'/>"
"</robot>";

std::string robot_description_updated =
"<?xml version=\"1.0\" ?>\n"
"<robot name=\"test\">\n"
"  <link name=\"link_0\" />\n"
"  <joint name=\"first_joint\" type=\"fixed\">\n"
"    <origin rpy=\"0 0 0\" xyz=\"1 1 1\" />\n"
"    <parent link=\"link_0\" />\n"
"    <child link=\"link_1\" />\n"
"  </joint>\n"
"  <link name=\"link_1\" />\n"
"  <joint name=\"second_joint\" type=\"revolute\">\n"
"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\" />\n"
"    <axis xyz=\"0 0 1\" />\n"
"    <limit effort=\"30\" lower=\"-1.57\" upper=\"1.57\" velocity=\"0.524\" />\n"
"    <parent link=\"link_1\" />\n"
"    <child link=\"link_2\" />\n"
"    <calibration rising=\"0.245\" />\n"
"  </joint>\n"
"  <link name=\"link_2\" />\n"
"  <joint name=\"third_joint\" type=\"fixed\">\n"
"    <origin rpy=\"4 3.5 6\" xyz=\"1 2 3.0526\" />\n"
"    <parent link=\"link_2\" />\n"
"    <child link=\"link_3\" />\n"
"  </joint>\n"
"  <link name=\"link_3\" />\n"
"</robot>";

TEST(UpdateUrdfTest, test_urdf_update)
{
  std::map<std::string, double> offsets;

  offsets["second_joint"] = 0.245;  // should add a rising
  
  offsets["third_joint_x"] = 1;  // camera-like offset
  offsets["third_joint_y"] = 2;
  offsets["third_joint_z"] = 3;
  offsets["third_joint_roll"] = 4;
  offsets["third_joint_pitch"] = 5;
  offsets["third_joint_yaw"] = 6;

  std::string s = updateURDF(robot_description, offsets);
  
  // google test fails if we give it all of s/robot_description_updated, so break this up
  std::vector<std::string> s_pieces;
  std::vector<std::string> robot_pieces;

  boost::split(s_pieces, s, boost::is_any_of("\n"));
  boost::split(robot_pieces, robot_description_updated, boost::is_any_of("\n"));

  for (int i = 0; i < robot_pieces.size(); ++i)
  {
    ASSERT_STREQ(s_pieces[i].c_str(), robot_pieces[i].c_str());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
