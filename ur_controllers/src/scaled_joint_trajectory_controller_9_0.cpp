// Copyright 2019, FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------

#include <memory>
#include <vector>
#include <sstream>

#include "ur_controllers/scaled_joint_trajectory_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace ur_controllers
{

ScaledJointTrajectoryController::~ScaledJointTrajectoryController()
{
  if (positions_output_file_.is_open()) {
    positions_output_file_.close();
    RCLCPP_INFO(rclcpp::get_logger("ScaledJointTrajectoryController"), "Positions output file closed");
  }
}

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init()
{
  // Create the parameter listener and get the parameters
  scaled_param_listener_ = std::make_shared<scaled_joint_trajectory_controller::ParamListener>(get_node());
  scaled_params_ = scaled_param_listener_->get_params();
  if (!scaled_params_.speed_scaling_interface_name.empty()) {
    RCLCPP_INFO(get_node()->get_logger(), "Using scaling state from the hardware from interface %s.",
                scaled_params_.speed_scaling_interface_name.c_str());
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "No scaling interface set. This controller will not use speed scaling.");
  }

  // Create subscription for realtime distance left
  realtime_distance_left_subscription_ = get_node()->create_subscription<std_msgs::msg::Int32>(
    "/realtime_distance_left",  // topic name
    10,  // QoS profile - queue size
    std::bind(&ScaledJointTrajectoryController::realtime_distance_left_callback, this, std::placeholders::_1)
  );

  // Create subscription for start distance correct
  start_distance_correct_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/start_correct_distance",  // topic name
    10,  // QoS profile - queue size
    std::bind(&ScaledJointTrajectoryController::start_distance_correct_callback, this, std::placeholders::_1)
  );

  joints_states_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",  // topic name
    10,  // QoS profile - queue size
    std::bind(&ScaledJointTrajectoryController::joints_states_callback, this, std::placeholders::_1)
  );

  start_expend_kalman_filter_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/start_expend_kalman_filter",  // topic name
    10,  // QoS profile - queue size
    std::bind(&ScaledJointTrajectoryController::start_expend_kalman_filter_callback, this, std::placeholders::_1)
  );

  // pressure_control_subscription_ = get_node()->create_subscription<std_msgs::msg::Int32>(
  //   "/pressure_control",  // topic name
  //   10,  // QoS profile - queue size
  //   std::bind(&ScaledJointTrajectoryController::pressure_control_callback, this, std::placeholders::_1)
  // );

  expend_kalman_filter_theta_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "/expend_kalman_filter_theta",  // topic name
    10  // QoS profile - queue size
  );

  return JointTrajectoryController::on_init();
}

controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::state_interface_configuration();

  if (!scaled_params_.speed_scaling_interface_name.empty()) {
    conf.names.push_back(scaled_params_.speed_scaling_interface_name);
  }

  return conf;
}

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  TimeData time_data;
  time_data.time = get_node()->now();
  time_data.period = rclcpp::Duration::from_nanoseconds(0);
  time_data.uptime = get_node()->now();
  time_data_.initRT(time_data);

  // Set scaling interfaces
  if (!scaled_params_.speed_scaling_interface_name.empty()) {
    auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](auto& interface) {
      return (interface.get_name() == scaled_params_.speed_scaling_interface_name);
    });
    if (it != state_interfaces_.end()) {
      scaling_state_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find speed scaling interface in state interfaces.");
    }
  }

  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type ScaledJointTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& period)
{
  if (scaling_state_interface_.has_value()) {
    scaling_factor_ = scaling_state_interface_->get().get_value();
  }

  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }
  auto logger = this->get_node()->get_logger();

  // update dynamic parameters
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(logger, params_);
  }

  auto compute_error_for_joint = [&](JointTrajectoryPoint& error, int index, const JointTrajectoryPoint& current,
                                     const JointTrajectoryPoint& desired) {
    // error defined as the difference between current and desired
    if (joints_angle_wraparound_[index]) {
      // if desired, the shortest_angular_distance is calculated, i.e., the error is
      //  normalized between -pi<error<pi
      error.positions[index] = angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    } else {
      error.positions[index] = desired.positions[index] - current.positions[index];
    }
    if (has_velocity_state_interface_ && (has_velocity_command_interface_ || has_effort_command_interface_)) {
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_) {
      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
    }
  };

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (current_external_msg != *new_external_msg && (*(rt_has_pending_goal_.readFromRT()) && !active_goal) == false) {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    traj_external_point_ptr_->update(*new_external_msg);
  }

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point = [&](auto& joint_interface, const std::vector<double>& trajectory_point_interface) {
    for (size_t index = 0; index < dof_; ++index) {
      joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  };

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_state_interfaces(state_current_);

  // currently carrying out a trajectory
  if (has_active_trajectory()) {
    // Main Speed scaling difference...
    // Adjust time with scaling factor
    TimeData time_data;
    time_data.time = time;
    rcl_duration_value_t t_period = (time_data.time - time_data_.readFromRT()->time).nanoseconds();
    time_data.period = rclcpp::Duration::from_nanoseconds(scaling_factor_ * t_period);
    time_data.uptime = time_data_.readFromRT()->uptime + time_data.period;
    rclcpp::Time traj_time = time_data_.readFromRT()->uptime + rclcpp::Duration::from_nanoseconds(t_period);
    time_data_.reset();
    time_data_.initRT(time_data);

    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!traj_external_point_ptr_->is_sampled_already()) {
      first_sample = true;
      if (params_.open_loop_control) {
        traj_external_point_ptr_->set_point_before_trajectory_msg(traj_time, last_commanded_state_,
                                                                  joints_angle_wraparound_);
      } else {
        traj_external_point_ptr_->set_point_before_trajectory_msg(traj_time, state_current_, joints_angle_wraparound_);
      }
    }

    // find segment for current timestamp
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point = traj_external_point_ptr_->sample(traj_time, interpolation_method_, state_desired_,
                                                              start_segment_itr, end_segment_itr);

    if (valid_point) {
      const rclcpp::Time traj_start = traj_external_point_ptr_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      const double time_difference = time_data.uptime.seconds() - segment_time_from_start.seconds();
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      const bool before_last_point = end_segment_itr != traj_external_point_ptr_->end();
      auto active_tol = active_tolerances_.readFromRT();

      // have we reached the end, are not holding position, and is a timeout configured?
      // Check independently of other tolerances
      if (!before_last_point && *(rt_is_holding_.readFromRT()) == false && cmd_timeout_ > 0.0 &&
          time_difference > cmd_timeout_) {
        RCLCPP_WARN(logger, "Aborted due to command timeout");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_hold_position());
      }

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index) {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        // print output per default, goal will be aborted afterwards
        if ((before_last_point || first_sample) && *(rt_is_holding_.readFromRT()) == false &&
            !check_state_tolerance_per_joint(state_error_, index, active_tol->state_tolerance[index],
                                             true /* show_errors */)) {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (!before_last_point && *(rt_is_holding_.readFromRT()) == false &&
            !check_state_tolerance_per_joint(state_error_, index, active_tol->goal_state_tolerance[index],
                                             false /* show_errors */)) {
          outside_goal_tolerance = true;

          if (active_tol->goal_time_tolerance != 0.0) {
            // if we exceed goal_time_tolerance set it to aborted
            if (time_difference > active_tol->goal_time_tolerance) {
              within_goal_time = false;
              // print once, goal will be aborted afterwards
              check_state_tolerance_per_joint(state_error_, index, default_tolerances_.goal_state_tolerance[index],
                                              true /* show_errors */);
            }
          }
        }
      }

      // if (start_expend_kalman_filter)
      // {
      //   // Check if we have valid state data before proceeding
      //   if (theta_from_joint_state.size() >= 6 && dtheta_from_joint_state.size() >= 6 && tau_from_joint_state.size() >= 6) {
          
      //     Eigen::MatrixXd theta_k = Eigen::MatrixXd::Zero(6, 1);
      //     theta_k << theta_from_joint_state[0], theta_from_joint_state[1], theta_from_joint_state[2], 
      //                theta_from_joint_state[3], theta_from_joint_state[4], theta_from_joint_state[5];
          
      //     expend_kalman_filter_result_k_1.X_posterior = expend_kalman_filter_result_k.X_posterior;
      //     expend_kalman_filter_result_k_1.P_posterior = expend_kalman_filter_result_k.P_posterior;
          
      //     calculateExpendKalmanFilter(torque_k_1, 0.008, theta_k);

      //     torque_k_1 << tau_from_joint_state[0], tau_from_joint_state[1], tau_from_joint_state[2], 
      //                  tau_from_joint_state[3], tau_from_joint_state[4], tau_from_joint_state[5];
          
      //     geometry_msgs::msg::Twist expend_kalman_filter_theta;
      //     expend_kalman_filter_theta.linear.x = expend_kalman_filter_result_k.X_posterior(0, 0);
      //     expend_kalman_filter_theta.linear.y = expend_kalman_filter_result_k.X_posterior(1, 0);
      //     expend_kalman_filter_theta.linear.z = expend_kalman_filter_result_k.X_posterior(2, 0);
      //     expend_kalman_filter_theta.angular.x = expend_kalman_filter_result_k.X_posterior(3, 0);
      //     expend_kalman_filter_theta.angular.y = expend_kalman_filter_result_k.X_posterior(4, 0);
      //     expend_kalman_filter_theta.angular.z = expend_kalman_filter_result_k.X_posterior(5, 0);
      //     if (first_joint_state_callback) {
      //       first_joint_state_callback = false;
      //       RCLCPP_INFO(logger, "expend_kalman_filter_theta: %f, %f, %f, %f, %f, %f", expend_kalman_filter_theta.linear.x, expend_kalman_filter_theta.linear.y, expend_kalman_filter_theta.linear.z, expend_kalman_filter_theta.angular.x, expend_kalman_filter_theta.angular.y, expend_kalman_filter_theta.angular.z);
      //     }
      //     expend_kalman_filter_theta_pub_->publish(expend_kalman_filter_theta);
      //   }
      // } else {
      //   // 为下次开启卡尔曼滤波做初值准备
      //   if (theta_from_joint_state.size() >= 6 && dtheta_from_joint_state.size() >= 6 && tau_from_joint_state.size() >= 6) {
      //     expend_kalman_filter_result_k.X_posterior.block<6, 1>(0, 0) << theta_from_joint_state[0], theta_from_joint_state[1], 
      //                                                                    theta_from_joint_state[2], theta_from_joint_state[3], 
      //                                                                    theta_from_joint_state[4], theta_from_joint_state[5];
      //     expend_kalman_filter_result_k.X_posterior.block<6, 1>(6, 0) << dtheta_from_joint_state[0], dtheta_from_joint_state[1], 
      //                                                                    dtheta_from_joint_state[2], dtheta_from_joint_state[3], 
      //                                                                    dtheta_from_joint_state[4], dtheta_from_joint_state[5];
      //     torque_k_1 << tau_from_joint_state[0], tau_from_joint_state[1], tau_from_joint_state[2], 
      //                  tau_from_joint_state[3], tau_from_joint_state[4], tau_from_joint_state[5];
      //     expend_kalman_filter_result_k.P_posterior = Eigen::MatrixXd::Identity(12, 12) * 0.0001;
      //     expend_kalman_filter_result_k.P_posterior.block<6, 6>(0, 0) << 0.00002234 * 0.00002234, 0, 0, 0, 0, 0,
      //                                                                  0, 0.00002192 * 0.00002192, 0, 0, 0, 0,
      //                                                                  0, 0, 0.00002274 * 0.00002274, 0, 0, 0,
      //                                                                  0, 0, 0, 0.0000184 * 0.0000184, 0, 0,
      //                                                                  0, 0, 0, 0, 0.000019 * 0.000019, 0,
      //                                                                  0, 0, 0, 0, 0, 0.00002184 * 0.00002184;
      //   }
      // }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time) {
        if (use_closed_loop_pid_adapter_) {
          // Update PIDs
          for (auto i = 0ul; i < dof_; ++i) {
            tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
                              pids_[i]->computeCommand(state_error_.positions[i], state_error_.velocities[i],
                                                       (uint64_t)period.nanoseconds());
          }
        }

        // 激光距离校正
        if (start_distance_correct && correct_distance_z != 0.0) {
          Eigen::MatrixXd J = calculateJacobian(state_current_.positions);
          Eigen::MatrixXd J_inv = calculateJacobianInverse(J);
          Eigen::MatrixXd eef_pose = Eigen::MatrixXd::Zero(6, 1);
 
          error_z = correct_distance_z;

          double correct_distance_z_PID = calculate_PID(error_z, error_z_last, error_z_last_last, correct_distance_z_PID_last, Kp, Ki, Kd);
          eef_pose(2) = correct_distance_z_PID;
          Eigen::MatrixXd eef_joint = J_inv * eef_pose;
          for (auto i = 0ul; i < dof_; ++i) {
            state_desired_.positions[i] = eef_joint(i) + state_desired_.positions[i];
          }
          // RCLCPP_INFO(logger, "eef_pose_z: %f, correct_distance_z: %f", eef_pose(2), correct_distance_z);
          // RCLCPP_INFO(logger, "state_current_[0]: %f, state_current_[1]: %f, state_current_[2]: %f, state_current_[3]: %f, state_current_[4]: %f, state_current_[5]: %f", state_current_.positions[0], state_current_.positions[1], state_current_.positions[2], state_current_.positions[3], state_current_.positions[4], state_current_.positions[5]);
        }

        // set values for next hardware write()
        if (has_position_command_interface_) {
          
          // 只在pressure_control值为0时才输出到文件
          if (pressure_control_value == 0) {
            // 初始化文件输出（如果还没有初始化）
            initialize_positions_output_file();
            
            // 将state_desired_.positions输出到文件
            write_positions_to_file(state_desired_.positions);
          }
          
          assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
        }
        if (has_velocity_command_interface_) {
          if (use_closed_loop_pid_adapter_) {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          } else {
            assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
          }
        }
        if (has_acceleration_command_interface_) {
          assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);
        }
        if (has_effort_command_interface_) {
          assign_interface_from_point(joint_command_interface_[3], tmp_command_);
        }

        // store the previous command. Used in open-loop control mode
        last_commanded_state_ = state_desired_;
      }

      if (active_goal) {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving) {
          auto result = std::make_shared<FollowJTrajAction::Result>();
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          result->set__error_string("Aborted due to path tolerance violation");
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          rt_has_pending_goal_.writeFromNonRT(false);

          RCLCPP_WARN(logger, "Aborted due to state tolerance violation");

          traj_msg_external_point_ptr_.reset();
          traj_msg_external_point_ptr_.initRT(set_hold_position());
        } else if (!before_last_point) {
          // check goal tolerance
          if (!outside_goal_tolerance) {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            result->set__error_string("Goal successfully reached!");
            active_goal->setSucceeded(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_.writeFromNonRT(false);

            RCLCPP_INFO(logger, "Goal reached, success!");

            traj_msg_external_point_ptr_.reset();
            traj_msg_external_point_ptr_.initRT(set_success_trajectory_point());
          } else if (!within_goal_time) {
            const std::string error_string =
                "Aborted due to goal_time_tolerance exceeding by " + std::to_string(time_difference) + " seconds";

            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            result->set__error_string(error_string);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_.writeFromNonRT(false);

            RCLCPP_WARN(logger, error_string.c_str());

            traj_msg_external_point_ptr_.reset();
            traj_msg_external_point_ptr_.initRT(set_hold_position());
          }
        }
      } else if (tolerance_violated_while_moving && *(rt_has_pending_goal_.readFromRT()) == false) {
        // we need to ensure that there is no pending goal -> we get a race condition otherwise
        RCLCPP_ERROR(logger, "Holding position due to state tolerance violation");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_hold_position());
      } else if (!before_last_point && !within_goal_time && *(rt_has_pending_goal_.readFromRT()) == false) {
        RCLCPP_ERROR(logger, "Exceeded goal_time_tolerance: holding position...");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_hold_position());
      }
      // else, run another cycle while waiting for outside_goal_tolerance
      // to be satisfied (will stay in this state until new message arrives)
      // or outside_goal_tolerance violated within the goal_time_tolerance
    }
  }

  publish_state(state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

void ScaledJointTrajectoryController::realtime_distance_left_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (start_distance_correct && msg->data != 88888888) {
    correct_distance_z = msg->data * 0.001; //雅各比矩阵的逆对应mm/s转换rad
  } else {
    correct_distance_z = 0.0;
  }
}

void ScaledJointTrajectoryController::start_distance_correct_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  start_distance_correct = msg->data;
}

double ScaledJointTrajectoryController::calculate_PID(double error_z, double error_z_last, double error_z_last_last, double correct_distance_z_PID_last, double Kp, double Ki, double Kd)
{
  double result;
  result = Kp * (error_z - error_z_last) + Ki * error_z + Kd * (error_z - 2 * error_z_last + error_z_last_last) + correct_distance_z_PID_last;
  correct_distance_z_PID_last = result;
  error_z_last_last = error_z_last;
  error_z_last = error_z;
  return result;
}

void ScaledJointTrajectoryController::start_expend_kalman_filter_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // start_expend_kalman_filter = msg->data;
}

void ScaledJointTrajectoryController::joints_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // theta_from_joint_state[0] = msg->position[5];
  // theta_from_joint_state[1] = msg->position[0];
  // theta_from_joint_state[2] = msg->position[1];
  // theta_from_joint_state[3] = msg->position[2];
  // theta_from_joint_state[4] = msg->position[3];
  // theta_from_joint_state[5] = msg->position[4];

  // dtheta_from_joint_state[0] = msg->velocity[5];
  // dtheta_from_joint_state[1] = msg->velocity[0];
  // dtheta_from_joint_state[2] = msg->velocity[1];
  // dtheta_from_joint_state[3] = msg->velocity[2];
  // dtheta_from_joint_state[4] = msg->velocity[3];
  // dtheta_from_joint_state[5] = msg->velocity[4];

  // tau_from_joint_state[0] = msg->effort[5];
  // tau_from_joint_state[1] = msg->effort[0];
  // tau_from_joint_state[2] = msg->effort[1];
  // tau_from_joint_state[3] = msg->effort[2];
  // tau_from_joint_state[4] = msg->effort[3];
  // tau_from_joint_state[5] = msg->effort[4];
}

void ScaledJointTrajectoryController::pressure_control_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  pressure_control_value = msg->data;
  RCLCPP_INFO(get_node()->get_logger(), "Received pressure_control value: %d", pressure_control_value);
  
  // 当接收到值为1时关闭文件
  if (pressure_control_value == 1 && positions_file_initialized_ && positions_output_file_.is_open()) {
    positions_output_file_.close();
    positions_file_initialized_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Positions output file closed due to pressure_control value = 1");
  }
}

void ScaledJointTrajectoryController::calculateExpendKalmanFilter(const Eigen::MatrixXd& torque_k_1, const double& dt, const Eigen::MatrixXd& theta_k)
{
    Eigen::MatrixXd X_k_prior = Eigen::MatrixXd::Zero(12, 1); //先验状态估计
    Eigen::MatrixXd X_k_posterior = Eigen::MatrixXd::Zero(12, 1); //后验状态估计
    Eigen::MatrixXd P_k_prior = Eigen::MatrixXd::Zero(12, 12); // 先验估计协方差矩阵
    Eigen::MatrixXd P_k_posterior = Eigen::MatrixXd::Zero(12, 12); // 后验估计协方差矩阵
    Eigen::MatrixXd K_k = Eigen::MatrixXd::Zero(12, 6); // 卡尔曼增益矩阵
    Eigen::MatrixXd H_k = Eigen::MatrixXd::Zero(6, 12); // 观测矩阵
    Eigen::MatrixXd R_k = Eigen::MatrixXd::Zero(6, 6); // 观测噪声协方差矩阵
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(12, 12); // 过程噪声协方差矩阵

    R_k.block<6, 6>(0, 0) << 0.00002234 * 0.00002234, 0, 0, 0, 0, 0,
                             0, 0.00002192 * 0.00002192, 0, 0, 0, 0,
                             0, 0, 0.00002274 * 0.00002274, 0, 0, 0,
                             0, 0, 0, 0.0000184 * 0.0000184, 0, 0,
                             0, 0, 0, 0, 0.000019 * 0.000019, 0,
                             0, 0, 0, 0, 0, 0.00002184 * 0.00002184;

    Q_k.block<6, 6>(0, 0) << 0.00002234 * 0.00002234, 0, 0, 0, 0, 0,
                             0, 0.00002192 * 0.00002192, 0, 0, 0, 0,
                             0, 0, 0.00002274 * 0.00002274, 0, 0, 0,
                             0, 0, 0, 0.0000184 * 0.0000184, 0, 0,
                             0, 0, 0, 0, 0.000019 * 0.000019, 0,
                             0, 0, 0, 0, 0, 0.00002184 * 0.00002184;

    Q_k.block<6, 6>(6, 6) << 0.0025 * 0.0025, 0, 0, 0, 0, 0,
                             0, 0.0025 * 0.0025, 0, 0, 0, 0,
                             0, 0, 0.0025 * 0.0025, 0, 0, 0,
                             0, 0, 0, 0.0025 * 0.0025, 0, 0,
                             0, 0, 0, 0, 0.0025 * 0.0025, 0,
                             0, 0, 0, 0, 0, 0.0025 * 0.0025;


    H_k.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    H_k.block<6, 6>(0, 6) = Eigen::MatrixXd::Zero(6, 6);

    // 计算先验状态估计
    X_k_prior.block<6, 1>(0, 0) = expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0) + expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0) * dt;
    X_k_prior.block<6, 1>(6, 0) = expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0) + NewtonEulerForwardDynamics(expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0), expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0), torque_k_1) * dt;


    // 计算离散时间状态转移矩阵Ad
    Eigen::MatrixXd Ad = DiscreteStateSpaceModel(expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0), expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0), torque_k_1, dt);

    // 计算先验估计协方差矩阵P_k_prior
    P_k_prior = Ad * expend_kalman_filter_result_k_1.P_posterior * Ad.transpose() + Q_k;

    // 计算卡尔曼增益矩阵
    Eigen::MatrixXd S = H_k * P_k_prior * H_k.transpose() + R_k;
    
    // 检查矩阵是否可逆
    Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
    if (lu.isInvertible()) {
        K_k = P_k_prior * H_k.transpose() * S.inverse();
    } else {
        // 如果矩阵不可逆，使用伪逆或设置增益为零
        // RCLCPP_WARN(rclcpp::get_logger("expend_kalman_filter"), "Measurement covariance matrix is singular, using pseudo-inverse");
        K_k = P_k_prior * H_k.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();
    }

    Eigen::MatrixXd H_k_theta_k = H_k * X_k_prior;
    // 计算后验状态估计
    X_k_posterior = X_k_prior + K_k * (theta_k - H_k * X_k_prior);
    
    // 计算后验估计协方差矩阵P_k_posterior
    P_k_posterior = (Eigen::MatrixXd::Identity(12, 12) - K_k * H_k) * P_k_prior;

    expend_kalman_filter_result_k.X_posterior = X_k_posterior;
    expend_kalman_filter_result_k.P_posterior = P_k_posterior;

    // // 验证结果是否包含NaN或无穷大
    // if (!expend_kalman_filter_result_k.X_posterior.allFinite()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("expend_kalman_filter"), "X_posterior contains NaN or infinite values, returning previous result");
    //     return expend_kalman_filter_result_k_1;
    // }
    
    // if (!expend_kalman_filter_result_k.P_posterior.allFinite()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("expend_kalman_filter"), "P_posterior contains NaN or infinite values, returning previous result");
    //     return expend_kalman_filter_result_k_1;
    // }
}

Eigen::MatrixXd ScaledJointTrajectoryController::calculateJacobianInverse(const Eigen::MatrixXd& jacobian)
{

  Eigen::MatrixXd jacobian_inv = jacobian.transpose() * ((jacobian * jacobian.transpose()).inverse());
  return jacobian_inv;
}

Eigen::MatrixXd ScaledJointTrajectoryController::calculateJacobian(const std::vector<double>& q)
{
  Eigen::MatrixXd J(6, 6);
  // UR3 机械臂参数
  const double a2 = -243.65;
  const double a3 = -213.0;
  const double d1 = 151.9;
  const double d2 = 119.85;
  const double d4 = -9.45;
  const double d5 = 83.4;
  const double d6 = 82.4;

  // 计算雅可比矩阵的每个元素
  J(0,0) = d6 * (cos(q[0]) * cos(q[4]) + cos(q[1] + q[2] + q[3]) * sin(q[0]) * sin(q[4])) + d2 * cos(q[0]) + d4 * cos(q[0]) - a3 * cos(q[1] + q[2]) * sin(q[0]) - a2 * cos(q[1]) * sin(q[0]) - d5 * sin(q[1] + q[2] + q[3]) * sin(q[0]);
  J(1,0) = d6 * (cos(q[4]) * sin(q[0]) - cos(q[1] + q[2] + q[3]) * cos(q[0]) * sin(q[4])) + d2 * sin(q[0]) + d4 * sin(q[0]) + a3 * cos(q[1] + q[2]) * cos(q[0]) + a2 * cos(q[0]) * cos(q[1]) + d5 * sin(q[1] + q[2] + q[3]) * cos(q[0]);
  J(2,0) = 0.0;
  J(3,0) = 0.0;
  J(4,0) = 0.0;
  J(5,0) = 1.0;

  J(0,1) = -cos(q[0]) * (a3 * sin(q[1] + q[2]) + a2 * sin(q[1]) - d5 * cos(q[1] + q[2] + q[3]) - d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
  J(1,1) = -sin(q[0]) * (a3 * sin(q[1] + q[2]) + a2 * sin(q[1]) - d5 * cos(q[1] + q[2] + q[3]) - d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
  J(2,1) = a3 * cos(q[1] + q[2]) + a2 * cos(q[1]) + d5 * (cos(q[1] + q[2]) * sin(q[3]) + sin(q[1] + q[2]) * cos(q[3])) - d6 * sin(q[4]) * (cos(q[1] + q[2]) * cos(q[3]) - sin(q[1] + q[2]) * sin(q[3]));
  J(3,1) = sin(q[0]);
  J(4,1) = -cos(q[0]);
  J(5,1) = 0.0;

  J(0,2) = cos(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) - a3 * sin(q[1] + q[2]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
  J(1,2) = sin(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) - a3 * sin(q[1] + q[2]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
  J(2,2) = a3 * cos(q[1] + q[2]) + d5 * sin(q[1] + q[2] + q[3]) - d6 * cos(q[1] + q[2] + q[3]) * sin(q[4]);
  J(3,2) = sin(q[0]);
  J(4,2) = -cos(q[0]);
  J(5,2) = 0.0;

  J(0,3) = cos(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
  J(1,3) = sin(q[0]) * (d5 * cos(q[1] + q[2] + q[3]) + d6 * sin(q[1] + q[2] + q[3]) * sin(q[4]));
  J(2,3) = d5 * sin(q[1] + q[2] + q[3]) - d6 * cos(q[1] + q[2] + q[3]) * sin(q[4]);
  J(3,3) = sin(q[0]);
  J(4,3) = -cos(q[0]);
  J(5,3) = 0.0;

  J(0,4) = -d6 * (sin(q[0]) * sin(q[4]) + cos(q[1] + q[2] + q[3]) * cos(q[0]) * cos(q[4]));
  J(1,4) = d6 * (cos(q[0]) * sin(q[4]) - cos(q[1] + q[2] + q[3]) * cos(q[4]) * sin(q[0]));
  J(2,4) = -d6 * sin(q[1] + q[2] + q[3]) * cos(q[4]);
  J(3,4) = sin(q[1] + q[2] + q[3]) * cos(q[0]);
  J(4,4) = sin(q[1] + q[2] + q[3]) * sin(q[0]);
  J(5,4) = -cos(q[1] + q[2] + q[3]);

  J(0,5) = 0.0;
  J(1,5) = 0.0;
  J(2,5) = 0.0;
  J(3,5) = cos(q[4]) * sin(q[0]) - cos(q[1] + q[2] + q[3]) * cos(q[0]) * sin(q[4]);
  J(4,5) = -cos(q[0]) * cos(q[4]) - cos(q[1] + q[2] + q[3]) * sin(q[0]) * sin(q[4]);
  J(5,5) = -sin(q[1] + q[2] + q[3]) * sin(q[4]);

  // 将小于阈值的元素设为0
  const double eps = 1e-5;
  for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
          if (std::abs(J(i,j)) < eps) {
              J(i,j) = 0.0;
          }
      }
  }
  return J;
}

void ScaledJointTrajectoryController::initialize_positions_output_file()
{
  if (!positions_file_initialized_) {
    // 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/xht/桌面/test_8_27/state_desired_positions_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".txt";
    
    positions_output_file_.open(ss.str(), std::ios::out);
    if (positions_output_file_.is_open()) {
      positions_file_initialized_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Positions output file opened: %s", ss.str().c_str());
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to open positions output file: %s", ss.str().c_str());
    }
  }
}

void ScaledJointTrajectoryController::write_positions_to_file(const std::vector<double>& positions)
{
  if (positions_file_initialized_ && positions_output_file_.is_open()) {
    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;
    
    positions_output_file_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") 
                          << "." << std::setfill('0') << std::setw(3) << millis << " ";
    
    // 输出所有关节位置
    for (size_t i = 0; i < positions.size(); ++i) {
      positions_output_file_ << positions[i];
      if (i < positions.size() - 1) {
        positions_output_file_ << " ";
      }
    }
    positions_output_file_ << std::endl;
    positions_output_file_.flush(); // 确保数据立即写入文件
  }
}

}    // namespace ur_controllers



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ur_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
