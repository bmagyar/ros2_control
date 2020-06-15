// Copyright 2020 ros2_control development team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>
#include <string>
#include <vector>

#include "transmission_interface/transmission_loader.hpp"

namespace testing
{
class TransmissionLoaderTest : public Test
{
  public:
    TransmissionLoaderTest()  //: transmission_loader(robot_hw)
    {
        // // Populate actuators interface
        // for (unsigned int i = 0; i < dim; ++i)
        // {
        //     hardware_interface::ActuatorStateHandle state_handle(act_names[i], &act_pos[i], &act_vel[i],
        //     &act_eff[i]);
        //     act_state_iface.registerHandle(state_handle);

        //     hardware_interface::ActuatorHandle pos_cmd_handle(state_handle, &act_pos_cmd[i]);
        //     pos_act_iface.registerHandle(pos_cmd_handle);

        //     hardware_interface::ActuatorHandle vel_cmd_handle(state_handle, &act_vel_cmd[i]);
        //     vel_act_iface.registerHandle(vel_cmd_handle);

        //     hardware_interface::ActuatorHandle eff_cmd_handle(state_handle, &act_eff_cmd[i]);
        //     eff_act_iface.registerHandle(eff_cmd_handle);
        // }
        // robot_hw.registerInterface(&act_state_iface);
        // robot_hw.registerInterface(&pos_act_iface);
        // robot_hw.registerInterface(&vel_act_iface);
        // robot_hw.registerInterface(&eff_act_iface);
    }

  protected:
    const unsigned int dim = { 3 };
    std::vector<std::string> act_names = { "foo_actuator", "bar_actuator", "baz_actuator" };
    std::vector<double> act_pos = { 0.0, 0.0, 0.0 };
    std::vector<double> act_vel = { 0.0, 0.0, 0.0 };
    std::vector<double> act_eff = { 0.0, 0.0, 0.0 };
    std::vector<double> act_pos_cmd = { 0.0, 0.0, 0.0 };
    std::vector<double> act_vel_cmd = { 0.0, 0.0, 0.0 };
    std::vector<double> act_eff_cmd = { 0.0, 0.0, 0.0 };

    // hardware_interface::RobotHardware robot_hw;
    // transmission_interface::TransmissionLoader transmission_loader;
};

// using transmission_interface::parse_transmissions_from_urdf;

// TEST_F(TestTransmissionParser, successfully_parse_valid_urdf)
// {
//     const auto transmissions = parse_transmissions_from_urdf(valid_urdf_xml_);

//     ASSERT_THAT(transmissions, SizeIs(2));
//     EXPECT_EQ("rrbot_joint1", transmissions[0].joint_name);
//     EXPECT_EQ(hardware_interface::joint_control_type::POSITION, transmissions[0].joint_control_type);
//     EXPECT_EQ("rrbot_joint2", transmissions[1].joint_name);
//     EXPECT_EQ(hardware_interface::joint_control_type::VELOCITY, transmissions[1].joint_control_type);
// }

// TEST_F(TestTransmissionParser, empty_string_throws_error)
// {
//     ASSERT_THROW(parse_transmissions_from_urdf(""), std::runtime_error);
// }

// TEST_F(TestTransmissionParser, empty_urdf_returns_empty)
// {
//     const std::string empty_urdf = "<?xml version=\"1.0\"?><robot name=\"robot\"
//     xmlns=\"http://www.ros.org\"></robot>";
//     const auto transmissions = parse_transmissions_from_urdf(empty_urdf);
//     ASSERT_THAT(transmissions, IsEmpty());
// }

// TEST_F(TestTransmissionParser, wrong_urdf_throws_error)
// {
//     ASSERT_THROW(parse_transmissions_from_urdf(wrong_urdf_xml_), std::runtime_error);
// }

// TEST_F(TransmissionLoaderTest, UnsupportedHwInterfaceType)
// {
//     // Parse transmission info
//     std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
//     ASSERT_EQ(1, infos.size());

//     // Transmission has only one hardware interface, which is unsupported. Loading should fail
//     {
//         TransmissionInfo info = infos.front();
//         info.joints_.front().hardware_interfaces_.clear();
//         info.joints_.front().hardware_interfaces_.resize(1, "unsupported/hw_interface_type");

//         TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//         EXPECT_FALSE(trans_iface_loader.load(info));
//     }

//     // Transmission has multiple hardware interfaces, of which one is unsupported. Loading should succeed
//     // (best-effort policy)
//     {
//         TransmissionInfo info = infos.front();
//         info.joints_.front().hardware_interfaces_.push_back("unsupported/hw_interface_type");

//         TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//         EXPECT_TRUE(trans_iface_loader.load(info));
//     }
// }

// TEST_F(TransmissionLoaderTest, UnavailableInterface)
// {
//     // Parse transmission info
//     std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
//     ASSERT_EQ(1, infos.size());
//     const TransmissionInfo& info = infos.front();

//     hardware_interface::RobotHW empty_robot_hw;  // No actuator interfaces are registered to this robot

//     TransmissionLoader trans_iface_loader(&empty_robot_hw, &robot_transmissions);
//     ASSERT_FALSE(trans_iface_loader.load(info));
// }

// // We currently can't load a transmission where each joint requires a different set of hardware interfaces
// TEST_F(TransmissionLoaderTest, UnsupportedFeature)
// {
//     // Parse transmission info
//     std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/transmission_interface_loader_unsupported.urdf");
//     ASSERT_EQ(2, infos.size());

//     // Different hw interface counts
//     {
//         const TransmissionInfo& info = infos.front();
//         TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//         ASSERT_FALSE(trans_iface_loader.load(info));
//     }

//     // Same hw interface count, but different types
//     {
//         const TransmissionInfo& info = infos.back();
//         TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//         ASSERT_FALSE(trans_iface_loader.load(info));
//     }
// }

// TEST_F(TransmissionLoaderTest, HwIfacePermutation)
// {
//     // Parse transmission info
//     std::vector<TransmissionInfo> infos =
//         parseUrdf("test/urdf/transmission_interface_loader_hw_iface_permutation.urdf");
//     ASSERT_EQ(1, infos.size());
//     const TransmissionInfo& info = infos.front();

//     TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//     ASSERT_TRUE(trans_iface_loader.load(info));
// }

// TEST_F(TransmissionLoaderTest, AccessorValidation)
// {
//     // Parse transmission info
//     const std::string urdf_filename = "test/urdf/transmission_interface_loader_valid.urdf";
//     std::string urdf;
//     ASSERT_TRUE(readFile(urdf_filename, urdf));

//     // Load transmissions
//     TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//     ASSERT_TRUE(trans_iface_loader.load(urdf));  // NOTE: Using URDF loader

//     // Validate raw data accessor
//     TransmissionLoaderData* loader_data_ptr = trans_iface_loader.getData();
//     ASSERT_TRUE(nullptr != loader_data_ptr);
//     ASSERT_TRUE(&robot_hw == loader_data_ptr->robot_hw);
//     ASSERT_TRUE(&robot_transmissions == loader_data_ptr->robot_transmissions);
//     ASSERT_EQ(3, loader_data_ptr->raw_joint_data_map.size());
//     ASSERT_EQ(
//         6, loader_data_ptr->transmission_data.size());  // Each transmission is added as many times as joint
//         interfaces
// }

// TEST_F(TransmissionLoaderTest, DuplicateTransmissions)
// {
//     // Parse transmission info
//     const std::string urdf_filename = "test/urdf/transmission_interface_loader_duplicate.urdf";
//     std::string urdf;
//     ASSERT_TRUE(readFile(urdf_filename, urdf));

//     // Load transmissions
//     TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//     ASSERT_TRUE(trans_iface_loader.load(urdf));  // NOTE: Using URDF loader

//     // NOTE: We allow to overwrite an existing transmission in the same way a hardware interface can be overwritten.
//     // An informative warning message is printed notifying that a transmission handle has been overwritten.
// }

// TEST_F(TransmissionLoaderTest, SuccessfulLoad)
// {
//     // Parse transmission info
//     const std::string urdf_filename = "test/urdf/transmission_interface_loader_valid.urdf";
//     std::string urdf;
//     ASSERT_TRUE(readFile(urdf_filename, urdf));

//     std::vector<TransmissionInfo> infos = parseUrdf(urdf_filename);
//     ASSERT_EQ(2, infos.size());

//     // Get info for each transmission
//     const TransmissionInfo& info_red = infos.front();
//     ASSERT_EQ(1, info_red.actuators_.size());
//     ASSERT_EQ(1, info_red.joints_.size());

//     const TransmissionInfo& info_diff = infos.back();
//     ASSERT_EQ(2, info_diff.actuators_.size());
//     ASSERT_EQ(2, info_diff.joints_.size());

//     // Load transmissions
//     TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//     ASSERT_TRUE(trans_iface_loader.load(urdf));  // NOTE: Using URDF loader

//     using namespace hardware_interface;

//     // Actuator interfaces
//     PositionActuatorInterface* act_pos_cmd_iface = robot_hw.get<PositionActuatorInterface>();
//     VelocityActuatorInterface* act_vel_cmd_iface = robot_hw.get<VelocityActuatorInterface>();
//     EffortActuatorInterface* act_eff_cmd_iface = robot_hw.get<EffortActuatorInterface>();
//     ASSERT_TRUE(nullptr != act_pos_cmd_iface);
//     ASSERT_TRUE(nullptr != act_vel_cmd_iface);
//     ASSERT_TRUE(nullptr != act_eff_cmd_iface);

//     // Joint interfaces
//     PositionJointInterface* pos_jnt_iface = robot_hw.get<PositionJointInterface>();
//     VelocityJointInterface* vel_jnt_iface = robot_hw.get<VelocityJointInterface>();
//     EffortJointInterface* eff_jnt_iface = robot_hw.get<EffortJointInterface>();
//     ASSERT_TRUE(nullptr != pos_jnt_iface);
//     ASSERT_TRUE(nullptr != vel_jnt_iface);
//     ASSERT_TRUE(nullptr != eff_jnt_iface);

//     // Transmission interfaces
//     ActuatorToJointStateInterface* act_to_jnt_state = robot_transmissions.get<ActuatorToJointStateInterface>();
//     JointToActuatorPositionInterface* jnt_to_act_pos_cmd =
//     robot_transmissions.get<JointToActuatorPositionInterface>();
//     JointToActuatorVelocityInterface* jnt_to_act_vel_cmd =
//     robot_transmissions.get<JointToActuatorVelocityInterface>();
//     JointToActuatorEffortInterface* jnt_to_act_eff_cmd = robot_transmissions.get<JointToActuatorEffortInterface>();
//     ASSERT_TRUE(nullptr != act_to_jnt_state);
//     ASSERT_TRUE(nullptr != jnt_to_act_pos_cmd);
//     ASSERT_TRUE(nullptr != jnt_to_act_vel_cmd);
//     ASSERT_TRUE(nullptr != jnt_to_act_eff_cmd);

//     // Actuator handles
//     ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_));
//     ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_));
//     ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_));
//     ActuatorHandle act_pos_cmd_handle_red = act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_);
//     ActuatorHandle act_vel_cmd_handle_red = act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_);
//     ActuatorHandle act_eff_cmd_handle_red = act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_);

//     ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_));
//     ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_));
//     ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_));
//     ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_));
//     ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_));
//     ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_));
//     ActuatorHandle act_pos_cmd_handle_diff1 = act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_);
//     ActuatorHandle act_vel_cmd_handle_diff1 = act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_);
//     ActuatorHandle act_eff_cmd_handle_diff1 = act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_);
//     ActuatorHandle act_pos_cmd_handle_diff2 = act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_);
//     ActuatorHandle act_vel_cmd_handle_diff2 = act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_);
//     ActuatorHandle act_eff_cmd_handle_diff2 = act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_);

//     // Joint handles
//     ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_red.joints_.front().name_));
//     ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_red.joints_.front().name_));
//     ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_red.joints_.front().name_));
//     JointHandle pos_jnt_handle_red = pos_jnt_iface->getHandle(info_red.joints_.front().name_);
//     JointHandle vel_jnt_handle_red = vel_jnt_iface->getHandle(info_red.joints_.front().name_);
//     JointHandle eff_jnt_handle_red = eff_jnt_iface->getHandle(info_red.joints_.front().name_);

//     ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.front().name_));
//     ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.front().name_));
//     ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.front().name_));
//     ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.back().name_));
//     ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.back().name_));
//     ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.back().name_));
//     JointHandle pos_jnt_handle_diff1 = pos_jnt_iface->getHandle(info_diff.joints_.front().name_);
//     JointHandle vel_jnt_handle_diff1 = vel_jnt_iface->getHandle(info_diff.joints_.front().name_);
//     JointHandle eff_jnt_handle_diff1 = eff_jnt_iface->getHandle(info_diff.joints_.front().name_);
//     JointHandle pos_jnt_handle_diff2 = pos_jnt_iface->getHandle(info_diff.joints_.back().name_);
//     JointHandle vel_jnt_handle_diff2 = vel_jnt_iface->getHandle(info_diff.joints_.back().name_);
//     JointHandle eff_jnt_handle_diff2 = eff_jnt_iface->getHandle(info_diff.joints_.back().name_);

//     // Propagate state forward
//     act_pos.assign(3, 50.0);
//     act_vel.assign(3, -50.0);
//     act_eff.assign(3, 1.0);

//     act_to_jnt_state->propagate();

//     EXPECT_NEAR(1.5, pos_jnt_handle_red.getPosition(), EPS);
//     EXPECT_NEAR(1.5, pos_jnt_handle_diff1.getPosition(), EPS);
//     EXPECT_NEAR(0.5, pos_jnt_handle_diff2.getPosition(), EPS);

//     EXPECT_NEAR(-1.0, pos_jnt_handle_red.getVelocity(), EPS);
//     EXPECT_NEAR(-1.0, pos_jnt_handle_diff1.getVelocity(), EPS);
//     EXPECT_NEAR(0.0, pos_jnt_handle_diff2.getVelocity(), EPS);

//     EXPECT_NEAR(50.0, pos_jnt_handle_red.getEffort(), EPS);
//     EXPECT_NEAR(100.0, pos_jnt_handle_diff1.getEffort(), EPS);
//     EXPECT_NEAR(0.0, pos_jnt_handle_diff2.getEffort(), EPS);

//     // Propagate position commands forward
//     pos_jnt_handle_red.setCommand(1.5);
//     pos_jnt_handle_diff1.setCommand(1.5);
//     pos_jnt_handle_diff2.setCommand(0.5);

//     jnt_to_act_pos_cmd->propagate();

//     EXPECT_NEAR(50.0, act_pos_cmd_handle_red.getPosition(), EPS);
//     EXPECT_NEAR(50.0, act_pos_cmd_handle_diff1.getPosition(), EPS);
//     EXPECT_NEAR(50.0, act_pos_cmd_handle_diff2.getPosition(), EPS);

//     // Propagate velocity commands forward
//     vel_jnt_handle_red.setCommand(1.0);
//     vel_jnt_handle_diff1.setCommand(1.0);
//     vel_jnt_handle_diff2.setCommand(0.0);

//     jnt_to_act_vel_cmd->propagate();

//     EXPECT_NEAR(-50.0, act_vel_cmd_handle_red.getVelocity(), EPS);
//     EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff1.getVelocity(), EPS);
//     EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff2.getVelocity(), EPS);

//     // Propagate effort commands forward
//     eff_jnt_handle_red.setCommand(50.0);
//     eff_jnt_handle_diff1.setCommand(100.0);
//     eff_jnt_handle_diff2.setCommand(0.0);

//     jnt_to_act_eff_cmd->propagate();

//     EXPECT_NEAR(1.0, act_eff_cmd_handle_red.getEffort(), EPS);
//     EXPECT_NEAR(1.0, act_eff_cmd_handle_diff1.getEffort(), EPS);
//     EXPECT_NEAR(1.0, act_eff_cmd_handle_diff2.getEffort(), EPS);
// }

// TEST_F(TransmissionLoaderTest, SuccessfulLoadReversible)
// {
//     // Parse transmission info
//     const std::string urdf_filename = "test/urdf/transmission_interface_loader_bidirectional_valid.urdf";
//     std::string urdf;
//     ASSERT_TRUE(readFile(urdf_filename, urdf));

//     std::vector<TransmissionInfo> infos = parseUrdf(urdf_filename);
//     ASSERT_EQ(2, infos.size());

//     // Get info for each transmission
//     const TransmissionInfo& info_red = infos.front();
//     ASSERT_EQ(1, info_red.actuators_.size());
//     ASSERT_EQ(1, info_red.joints_.size());

//     const TransmissionInfo& info_diff = infos.back();
//     ASSERT_EQ(2, info_diff.actuators_.size());
//     ASSERT_EQ(2, info_diff.joints_.size());

//     // Load transmissions
//     TransmissionLoader trans_iface_loader(&robot_hw, &robot_transmissions);
//     ASSERT_TRUE(trans_iface_loader.load(urdf));  // NOTE: Using URDF loader

//     using namespace hardware_interface;

//     // Actuator interfaces
//     PositionActuatorInterface* act_pos_cmd_iface = robot_hw.get<PositionActuatorInterface>();
//     VelocityActuatorInterface* act_vel_cmd_iface = robot_hw.get<VelocityActuatorInterface>();
//     EffortActuatorInterface* act_eff_cmd_iface = robot_hw.get<EffortActuatorInterface>();
//     ActuatorStateInterface* act_state_iface = robot_hw.get<ActuatorStateInterface>();
//     ASSERT_TRUE(nullptr != act_pos_cmd_iface);
//     ASSERT_TRUE(nullptr != act_vel_cmd_iface);
//     ASSERT_TRUE(nullptr != act_eff_cmd_iface);
//     ASSERT_TRUE(nullptr != act_state_iface);

//     // Joint interfaces
//     PositionJointInterface* pos_jnt_iface = robot_hw.get<PositionJointInterface>();
//     VelocityJointInterface* vel_jnt_iface = robot_hw.get<VelocityJointInterface>();
//     EffortJointInterface* eff_jnt_iface = robot_hw.get<EffortJointInterface>();
//     JointStateInterface* state_jnt_iface = robot_hw.get<JointStateInterface>();
//     ASSERT_TRUE(nullptr != pos_jnt_iface);
//     ASSERT_TRUE(nullptr != vel_jnt_iface);
//     ASSERT_TRUE(nullptr != eff_jnt_iface);

//     // Forward Transmission interfaces
//     ActuatorToJointStateInterface* act_to_jnt_state = robot_transmissions.get<ActuatorToJointStateInterface>();
//     JointToActuatorPositionInterface* jnt_to_act_pos_cmd =
//     robot_transmissions.get<JointToActuatorPositionInterface>();
//     JointToActuatorVelocityInterface* jnt_to_act_vel_cmd =
//     robot_transmissions.get<JointToActuatorVelocityInterface>();
//     JointToActuatorEffortInterface* jnt_to_act_eff_cmd = robot_transmissions.get<JointToActuatorEffortInterface>();
//     ASSERT_TRUE(nullptr != act_to_jnt_state);
//     ASSERT_TRUE(nullptr != jnt_to_act_pos_cmd);
//     ASSERT_TRUE(nullptr != jnt_to_act_vel_cmd);
//     ASSERT_TRUE(nullptr != jnt_to_act_eff_cmd);

//     // Inverse Transmission interfaces
//     JointToActuatorStateInterface* jnt_to_act_state = robot_transmissions.get<JointToActuatorStateInterface>();
//     ActuatorToJointPositionInterface* act_to_jnt_pos_cmd =
//     robot_transmissions.get<ActuatorToJointPositionInterface>();
//     ActuatorToJointVelocityInterface* act_to_jnt_vel_cmd =
//     robot_transmissions.get<ActuatorToJointVelocityInterface>();
//     ActuatorToJointEffortInterface* act_to_jnt_eff_cmd = robot_transmissions.get<ActuatorToJointEffortInterface>();
//     ASSERT_TRUE(nullptr != jnt_to_act_state);
//     ASSERT_TRUE(nullptr != act_to_jnt_pos_cmd);
//     ASSERT_TRUE(nullptr != act_to_jnt_vel_cmd);
//     ASSERT_TRUE(nullptr != act_to_jnt_eff_cmd);

//     // Actuator handles
//     ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_));
//     ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_));
//     ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_));
//     ASSERT_NO_THROW(act_state_iface->getHandle(info_red.actuators_.front().name_));
//     ActuatorHandle act_pos_cmd_handle_red = act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_);
//     ActuatorHandle act_vel_cmd_handle_red = act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_);
//     ActuatorHandle act_eff_cmd_handle_red = act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_);

//     ActuatorStateHandle act_state_handle_red = act_state_iface->getHandle(info_red.actuators_.front().name_);

//     ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_));
//     ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_));
//     ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_));
//     ASSERT_NO_THROW(act_state_iface->getHandle(info_diff.actuators_.front().name_));
//     ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_));
//     ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_));
//     ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_));
//     ASSERT_NO_THROW(act_state_iface->getHandle(info_diff.actuators_.front().name_));
//     ActuatorHandle act_pos_cmd_handle_diff1 = act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_);
//     ActuatorHandle act_vel_cmd_handle_diff1 = act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_);
//     ActuatorHandle act_eff_cmd_handle_diff1 = act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_);
//     ActuatorHandle act_pos_cmd_handle_diff2 = act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_);
//     ActuatorHandle act_vel_cmd_handle_diff2 = act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_);
//     ActuatorHandle act_eff_cmd_handle_diff2 = act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_);

//     ActuatorStateHandle act_state_handle_diff1 = act_state_iface->getHandle(info_diff.actuators_.front().name_);
//     ActuatorStateHandle act_state_handle_diff2 = act_state_iface->getHandle(info_diff.actuators_.back().name_);

//     // Joint handles
//     ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_red.joints_.front().name_));
//     ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_red.joints_.front().name_));
//     ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_red.joints_.front().name_));
//     ASSERT_NO_THROW(state_jnt_iface->getHandle(info_red.joints_.front().name_));
//     JointHandle pos_jnt_handle_red = pos_jnt_iface->getHandle(info_red.joints_.front().name_);
//     JointHandle vel_jnt_handle_red = vel_jnt_iface->getHandle(info_red.joints_.front().name_);
//     JointHandle eff_jnt_handle_red = eff_jnt_iface->getHandle(info_red.joints_.front().name_);

//     JointStateHandle state_jnt_handle_red = state_jnt_iface->getHandle(info_red.joints_.front().name_);

//     ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.front().name_));
//     ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.front().name_));
//     ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.front().name_));
//     ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.back().name_));
//     ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.back().name_));
//     ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.back().name_));
//     JointHandle pos_jnt_handle_diff1 = pos_jnt_iface->getHandle(info_diff.joints_.front().name_);
//     JointHandle vel_jnt_handle_diff1 = vel_jnt_iface->getHandle(info_diff.joints_.front().name_);
//     JointHandle eff_jnt_handle_diff1 = eff_jnt_iface->getHandle(info_diff.joints_.front().name_);
//     JointHandle pos_jnt_handle_diff2 = pos_jnt_iface->getHandle(info_diff.joints_.back().name_);
//     JointHandle vel_jnt_handle_diff2 = vel_jnt_iface->getHandle(info_diff.joints_.back().name_);
//     JointHandle eff_jnt_handle_diff2 = eff_jnt_iface->getHandle(info_diff.joints_.back().name_);

//     // Propagate state forward
//     act_pos.assign(3, 50.0);
//     act_vel.assign(3, -50.0);
//     act_eff.assign(3, 1.0);

//     act_to_jnt_state->propagate();

//     EXPECT_NEAR(1.5, pos_jnt_handle_red.getPosition(), EPS);
//     EXPECT_NEAR(1.5, pos_jnt_handle_diff1.getPosition(), EPS);
//     EXPECT_NEAR(0.5, pos_jnt_handle_diff2.getPosition(), EPS);

//     EXPECT_NEAR(-1.0, pos_jnt_handle_red.getVelocity(), EPS);
//     EXPECT_NEAR(-1.0, pos_jnt_handle_diff1.getVelocity(), EPS);
//     EXPECT_NEAR(0.0, pos_jnt_handle_diff2.getVelocity(), EPS);

//     EXPECT_NEAR(50.0, pos_jnt_handle_red.getEffort(), EPS);
//     EXPECT_NEAR(100.0, pos_jnt_handle_diff1.getEffort(), EPS);
//     EXPECT_NEAR(0.0, pos_jnt_handle_diff2.getEffort(), EPS);

//     // Propagate position commands forward
//     pos_jnt_handle_red.setCommand(1.5);
//     pos_jnt_handle_diff1.setCommand(1.5);
//     pos_jnt_handle_diff2.setCommand(0.5);

//     jnt_to_act_pos_cmd->propagate();

//     EXPECT_NEAR(50.0, act_pos_cmd_handle_red.getPosition(), EPS);
//     EXPECT_NEAR(50.0, act_pos_cmd_handle_diff1.getPosition(), EPS);
//     EXPECT_NEAR(50.0, act_pos_cmd_handle_diff2.getPosition(), EPS);

//     // Propagate velocity commands forward
//     vel_jnt_handle_red.setCommand(1.0);
//     vel_jnt_handle_diff1.setCommand(1.0);
//     vel_jnt_handle_diff2.setCommand(0.0);

//     jnt_to_act_vel_cmd->propagate();

//     EXPECT_NEAR(-50.0, act_vel_cmd_handle_red.getVelocity(), EPS);
//     EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff1.getVelocity(), EPS);
//     EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff2.getVelocity(), EPS);

//     // Propagate effort commands forward
//     eff_jnt_handle_red.setCommand(50.0);
//     eff_jnt_handle_diff1.setCommand(100.0);
//     eff_jnt_handle_diff2.setCommand(0.0);

//     jnt_to_act_eff_cmd->propagate();

//     EXPECT_NEAR(1.0, act_eff_cmd_handle_red.getEffort(), EPS);
//     EXPECT_NEAR(1.0, act_eff_cmd_handle_diff1.getEffort(), EPS);
//     EXPECT_NEAR(1.0, act_eff_cmd_handle_diff2.getEffort(), EPS);

//     // Now propegate things in the reverse direction
//     RawJointDataMap* joint_data_map = &trans_iface_loader.getData()->raw_joint_data_map;
//     joint_data_map->operator[]("foo_joint").position = 1.5;
//     joint_data_map->operator[]("bar_joint").position = 1.5;
//     joint_data_map->operator[]("baz_joint").position = 0.5;

//     joint_data_map->operator[]("foo_joint").velocity = -1.0;
//     joint_data_map->operator[]("bar_joint").velocity = -1.0;
//     joint_data_map->operator[]("baz_joint").velocity = -2.0;

//     joint_data_map->operator[]("foo_joint").effort = 5.0;
//     joint_data_map->operator[]("bar_joint").effort = 10.0;
//     joint_data_map->operator[]("baz_joint").effort = -5.0;

//     jnt_to_act_state->propagate();

//     EXPECT_NEAR(50.0, act_state_handle_red.getPosition(), EPS);
//     EXPECT_NEAR(50.0, act_state_handle_diff1.getPosition(), EPS);
//     EXPECT_NEAR(50.0, act_state_handle_diff2.getPosition(), EPS);

//     EXPECT_NEAR(-50.0, act_state_handle_red.getVelocity(), EPS);
//     EXPECT_NEAR(-150.0, act_state_handle_diff1.getVelocity(), EPS);
//     EXPECT_NEAR(50.0, act_state_handle_diff2.getVelocity(), EPS);

//     EXPECT_NEAR(0.1, act_state_handle_red.getEffort(), EPS);
//     EXPECT_NEAR(0.05, act_state_handle_diff1.getEffort(), EPS);
//     EXPECT_NEAR(0.15, act_state_handle_diff2.getEffort(), EPS);

//     act_pos_cmd_handle_red.setCommand(3.0);
//     act_pos_cmd_handle_diff1.setCommand(3.0);
//     act_pos_cmd_handle_diff2.setCommand(3.0);

//     // reverse propegate position commands
//     act_to_jnt_pos_cmd->propagate();

//     EXPECT_NEAR(0.56, joint_data_map->operator[]("foo_joint").position_cmd, EPS);
//     EXPECT_NEAR(0.56, joint_data_map->operator[]("bar_joint").position_cmd, EPS);
//     EXPECT_NEAR(0.5, joint_data_map->operator[]("baz_joint").position_cmd, EPS);

//     // Propagate velocity commands forward
//     act_vel_cmd_handle_red.setCommand(1.0);
//     act_vel_cmd_handle_diff1.setCommand(1.0);
//     act_vel_cmd_handle_diff2.setCommand(0.0);

//     act_to_jnt_vel_cmd->propagate();

//     EXPECT_NEAR(0.02, joint_data_map->operator[]("foo_joint").velocity_cmd, EPS);
//     EXPECT_NEAR(0.01, joint_data_map->operator[]("bar_joint").velocity_cmd, EPS);
//     EXPECT_NEAR(0.01, joint_data_map->operator[]("baz_joint").velocity_cmd, EPS);

//     // Propagate effort commands forward
//     act_eff_cmd_handle_red.setCommand(50.0);
//     act_eff_cmd_handle_diff1.setCommand(1.0);
//     act_eff_cmd_handle_diff2.setCommand(0.0);

//     act_to_jnt_eff_cmd->propagate();

//     EXPECT_NEAR(2500.0, joint_data_map->operator[]("foo_joint").effort_cmd, EPS);
//     EXPECT_NEAR(50.0, joint_data_map->operator[]("bar_joint").effort_cmd, EPS);
//     EXPECT_NEAR(50.0, joint_data_map->operator[]("baz_joint").effort_cmd, EPS);
// }

}  // namespace testing
