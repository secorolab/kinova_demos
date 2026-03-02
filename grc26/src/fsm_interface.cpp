#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <atomic>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacdotsolver.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "robif2b/functions/kinova_gen3.h"
#include "grc26/fsm_interface.hpp"

#define NUM_JOINTS 7

#define KINOVA_TAU_CMD_LIMIT 30.0

FSMInterface::FSMInterface(robif2b_kinova_gen3_nbx& rob, SystemState& system_state)
  : rob(rob), system_state(system_state), in_comm_with_hw(false)
{

}

FSMInterface::~FSMInterface() {
}

int FSMInterface::get_current_state() const
{
    return fsm.currentStateIndex;
}

bool FSMInterface::is_in_comm_with_hw() const
{
    return in_comm_with_hw;
}

void FSMInterface::configure(events *eventData, SystemState& system_state){

  ArmKDLModel model;
  bool kdl_model_loaded =
  model.loadFromURDF("grc26",
                    "GEN3_URDF_V12.urdf",
                    "base_link",
                    "EndEffector_Link");
                
  if (!kdl_model_loaded) {
    printf("Failed to parse URDF file");
    produce_event(eventData, E_CONFIGURE_EXIT);
    return;
  }
  KDL::Chain kdl_chain = model.chain();
  int num_joints = model.num_joints();
  assert(num_joints == NUM_JOINTS && "Kinova arm has unexpected number of joints");

  // establish communication with arm
  robif2b_kinova_gen3_configure(&rob);
  if (!rob.success) {
      printf("Error during gen3_configure\n");
      robif2b_kinova_gen3_shutdown(&rob);
      if (!rob.success) {
          printf("Error during gen3_shutdown\n");
          produce_event(eventData, E_CONFIGURE_EXIT);
          return;
      }
  }

  robif2b_kinova_gen3_recover(&rob);
  if (!rob.success) {
      printf("Error during gen3_recover\n");
      robif2b_kinova_gen3_shutdown(&rob);
      if (!rob.success) {
          printf("Error during gen3_shutdown\n");
          produce_event(eventData, E_CONFIGURE_EXIT);
          return;
      }
  }

  printf("Starting\n");
  robif2b_kinova_gen3_start(&rob);
  if (!rob.success) {
      printf("Error during gen3_start\n");
      robif2b_kinova_gen3_stop(&rob);
      printf("Stopped\n");
  }

  in_comm_with_hw = true;
  // emit event to transition to idle state after configuration
  produce_event(eventData, E_CONFIGURED);
}

void FSMInterface::idle(events *eventData, const SystemState& system_state){

}

void FSMInterface::execute(events *eventData, SystemState& system_state){

  // solve for control commands

  // update control commands
}

void FSMInterface::touch_table_behavior_config(events *eventData, SystemState& system_state){
  sp.translation_mode = ControlMode::Velocity;
  sp.rotation_mode    = ControlMode::Position;
  sp.gripper_mode    = ControlMode::None;
  sp.linear_vel = Eigen::Vector3d(0.0, 0.0, -0.01); // m/s
  sp.orientation = Eigen::Vector3d(0.0, 0.0, 0.0); // roll, pitch, yaw

  // TODO: add post condition

  produce_event(eventData, E_M_SLIDE_ALONG_TABLE_CONFIGURED);
}

void FSMInterface::slide_on_table_behavior_config(events *eventData, SystemState& system_state){
  sp.translation_mode = ControlMode::Velocity;
  sp.rotation_mode    = ControlMode::Position;
  sp.gripper_mode    = ControlMode::None;
  sp.linear_vel = Eigen::Vector3d(0.05, 0.0, 0.0); // m/s
  sp.orientation = Eigen::Vector3d(0.0, 0.0, 0.0); // roll, pitch, yaw

  // TODO: add post condition

  produce_event(eventData, E_M_TOUCH_TABLE_CONFIGURED);
}

void FSMInterface::grasp_object_behavior_config(events *eventData, SystemState& system_state){
  sp.translation_mode = ControlMode::Velocity;
  sp.rotation_mode    = ControlMode::Position;
  sp.gripper_mode    = ControlMode::Position;
  sp.linear_vel = Eigen::Vector3d(0.0, 0.0, -0.01); // m/s
  sp.orientation = Eigen::Vector3d(0.0, 0.0, 0.0); // roll, pitch, yaw
  sp.gripper_position = 1.0; // fully closed

  // TODO: add post condition

  produce_event(eventData, E_M_GRASP_OBJECT_CONFIGURED);
}

void FSMInterface::collaborate_behavior_config(events *eventData, SystemState& system_state){
  sp.translation_mode = ControlMode::Effort;
  sp.rotation_mode    = ControlMode::Effort;
  sp.gripper_mode    = ControlMode::Position;
  sp.feedforward_force = Eigen::Vector3d(0.0, 0.0, 0.0); // Newton
  sp.feedforward_torque = Eigen::Vector3d(0.0, 0.0, 0.0); // Newton-meter
  sp.gripper_position = 1.0; // fully closed

  // TODO: add post condition
  
  produce_event(eventData, E_M_COLLABORATE_CONFIGURED);
}

void FSMInterface::release_object_behavior_config(events *eventData, SystemState& system_state){
  sp.translation_mode = ControlMode::None;
  sp.rotation_mode    = ControlMode::None;
  sp.gripper_mode    = ControlMode::Position;
  sp.gripper_position = 0.0; // fully open

  produce_event(eventData, E_M_RELEASE_OBJECT_CONFIGURED);
}

void FSMInterface::exit(events *eventData, SystemState& system_state){
  // stop the arm and shutdown communication
  robif2b_kinova_gen3_stop(&rob);
  if (!rob.success) {
      printf("Error during gen3_stop\n");
  }
  printf("Stopped\n");

  robif2b_kinova_gen3_shutdown(&rob);
  if (!rob.success) {
      printf("Error during gen3_shutdown\n");
  }
  printf("Shutdown\n");

  // check: 
}

// decision of which behavior to execute based on events and arm state
void FSMInterface::fsm_behavior(events *eventData, SystemState& system_state){

  if (consume_event(eventData, E_ENTER_IDLE)) {
      idle(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_CONFIGURE)) {
      configure(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_EXECUTE)) {
      execute(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_TOUCH_TABLE)) {
      touch_table_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_SLIDE_ALONG_TABLE)) {
      slide_on_table_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_GRASP_OBJECT)) {
      grasp_object_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_COLLABORATE)) {
      collaborate_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_RELEASE_OBJECT)) {
      release_object_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_EXIT)) {
      exit(eventData, system_state);
  }
}

void FSMInterface::compute_gravity_comp(events *eventData, SystemState& system_state){

}

void FSMInterface::compute_cartesian_ctrl(events *eventData, SystemState& system_state){

}

void FSMInterface::run_fsm(){
  produce_event(&eventData, E_STEP);
  fsm_behavior(&eventData, system_state);
  fsm_step_nbx(&fsm);
  reconfig_event_buffers(&eventData);
};