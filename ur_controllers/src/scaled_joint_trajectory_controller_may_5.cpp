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
  RCLCPP_INFO(get_node()->get_logger(), "Created subscription to realtime_distance_left topic");

  // Create subscription for start distance correct
  start_distance_correct_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/start_correct_distance",  // topic name
    10,  // QoS profile - queue size
    std::bind(&ScaledJointTrajectoryController::start_distance_correct_callback, this, std::placeholders::_1)
  );

  start_expend_kalman_filter_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/start_expend_kalman_filter",  // topic name
    10,  // QoS profile - queue size
    std::bind(&ScaledJointTrajectoryController::start_expend_kalman_filter_callback, this, std::placeholders::_1)
  );

  distance_platform_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "/distance_platform",  // topic name
    10,  // QoS profile - queue size
    std::bind(&ScaledJointTrajectoryController::distance_platform_callback, this, std::placeholders::_1)
  );

  expend_kalman_filter_z_pub_ = get_node()->create_publisher<std_msgs::msg::Int32>(
    "/expend_kalman_filter_z",  // topic name
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

    // 当前状态与前次状态


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
      if (start_expend_kalman_filter)
      {
        X_k_1_k_1 = X_k_k;
        P_k_1_k_1 = P_k_k;
      }else
      {
        // 为下次开启卡尔曼滤波做初值准备
        X_k_k(0, 0) = state_current_.positions[0];
        X_k_k(1, 0) = state_current_.velocities[0];
        X_k_k(2, 0) = state_current_.positions[1];
        X_k_k(3, 0) = state_current_.velocities[1];
        X_k_k(4, 0) = state_current_.positions[2];
        X_k_k(5, 0) = state_current_.velocities[2];
        X_k_k(6, 0) = state_current_.positions[3];
        X_k_k(7, 0) = state_current_.velocities[3];
        X_k_k(8, 0) = state_current_.positions[4];
        X_k_k(9, 0) = state_current_.velocities[4];
        X_k_k(10, 0) = state_current_.positions[5];
        X_k_k(11, 0) = state_current_.velocities[5];  
      }

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
 
          // 拓展卡尔曼滤波
          if (start_expend_kalman_filter)
          {
            Eigen::MatrixXd q(6, 1);
            q << state_current_.positions[0], state_current_.positions[1], state_current_.positions[2], state_current_.positions[3], state_current_.positions[4], state_current_.positions[5];

            double laser_distance_z = correct_distance_z + distance_platform; //单位为mm
            calculateExpendKalmanFilter(q, laser_distance_z);
            
            error_z = calculate_z(X_k_k) * 1000 - distance_platform; // 单位为mm
            auto expend_kalman_filter_z_msg = std_msgs::msg::Int32();
            expend_kalman_filter_z_msg.data = error_z * 1000;
            expend_kalman_filter_z_pub_->publish(expend_kalman_filter_z_msg);
            RCLCPP_INFO(logger, "expend_kalman_filter_z: %f", error_z * 1000);
            
          }else
          {
            error_z = correct_distance_z;
          }

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

double ScaledJointTrajectoryController::calculate_PID(double error_z, double error_z_last, double error_z_last_last, double correct_distance_z_PID_last, double Kp, double Ki, double Kd)
{
  double result;
  result = Kp * (error_z - error_z_last) + Ki * error_z + Kd * (error_z - 2 * error_z_last + error_z_last_last) + correct_distance_z_PID_last;
  correct_distance_z_PID_last = result;
  error_z_last_last = error_z_last;
  error_z_last = error_z;
  return result;
}

void ScaledJointTrajectoryController::calculateExpendKalmanFilter(const Eigen::MatrixXd& q, const double& laser_distance_z)
{
  // UR3 机械臂参数
  const double a2 = -243.65 * 0.001;
  const double a3 = -213.0 * 0.001;
  const double d1 = 151.9 * 0.001;
  const double d2 = 119.85 * 0.001;
  const double d4 = -9.45 * 0.001;
  const double d5 = 83.4 * 0.001;
  const double d6 = 82.4 * 0.001;

  // 状态转移矩阵
  Eigen::MatrixXd F(12, 12);
  double delta_t = 0.008;
  F = Eigen::MatrixXd::Identity(12, 12);
  for (int i = 0; i < 6; i++) {
    F(2*i, 2*i+1) = delta_t;
  }

  // 过程噪声
  Eigen::MatrixXd Q(12, 12);
  Q = Eigen::MatrixXd::Identity(12, 12);
  for (int i = 0; i < 6; i++) {
    Q(2*i, 2*i) = 0.1 * 0.1;     // 位置噪声
    Q(2*i+1, 2*i+1) = 0.1 * 0.1;   // 速度噪声
  }
  
  // 检查数值稳定性
  if (std::abs(F.determinant()) < 1e-10) {
    RCLCPP_WARN(get_node()->get_logger(), "F matrix is near singular");
  }
  
  if (std::abs(Q.determinant()) < 1e-10) {
    RCLCPP_WARN(get_node()->get_logger(), "Q matrix is near singular");
  }

  // 预测模型
  Eigen::MatrixXd X_k_k_1(12, 1);
  Eigen::MatrixXd P_k_k_1(12, 12);
  
  // 添加维度检查
  if (X_k_1_k_1.rows() != 12 || X_k_1_k_1.cols() != 1) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid X_k_1_k_1 dimensions: %dx%d", X_k_1_k_1.rows(), X_k_1_k_1.cols());
  }
  
  if (P_k_1_k_1.rows() != 12 || P_k_1_k_1.cols() != 12) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid P_k_1_k_1 dimensions: %dx%d", P_k_1_k_1.rows(), P_k_1_k_1.cols());
  }

  X_k_k_1 = F * X_k_1_k_1;
  P_k_k_1 = F * P_k_1_k_1 * F.transpose() + Q;

  // 确保协方差矩阵是对称的
  P_k_k_1 = (P_k_k_1 + P_k_k_1.transpose()) / 2.0;
  
  // 确保协方差矩阵是正定的
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P_k_k_1);
  if (solver.eigenvalues().minCoeff() < 0) {
    RCLCPP_WARN(get_node()->get_logger(), "P_k_k_1 is not positive definite");
  }

  // 观测向量
  Eigen::MatrixXd Z(7, 1);
  Z(0, 0) = q(0, 0);
  Z(1, 0) = q(1, 0);
  Z(2, 0) = q(2, 0);
  Z(3, 0) = q(3, 0);
  Z(4, 0) = q(4, 0);
  Z(5, 0) = q(5, 0);
  Z(6, 0) = laser_distance_z * 0.001; // 单位转换为m

  // 观测矩阵
  Eigen::MatrixXd H(7, 12);
  H(0, 0) = 1.0, H(1, 1) = 1.0, H(2, 2) = 1.0, H(3, 3) = 1.0, H(4, 4) = 1.0, H(5, 5) = 1.0;
  H(6, 0) = 0.0;
  H(6, 2) = a3 * cos(q(1, 0) + q(2, 0)) + a2 * cos(q(1, 0)) + d5 * (cos(q(1, 0) + q(2, 0)) * sin(q(3, 0)) + sin(q(1, 0) + q(2, 0)) * cos(q(3, 0))) - d6 * sin(q(4, 0)) * (cos(q(1, 0) + q(2, 0)) * cos(q(3, 0)) - sin(q(1, 0) + q(2, 0)) * sin(q(3, 0)));
  H(6, 4) = a3 * cos(q(1, 0) + q(2, 0)) + d5 * sin(q(1, 0) + q(2, 0) + q(3, 0)) - d6 * cos(q(1, 0) + q(2, 0) + q(3, 0)) * sin(q(4, 0));
  H(6, 6) = d5 * sin(q(1, 0) + q(2, 0) + q(3, 0)) - d6 * cos(q(1, 0) + q(2, 0) + q(3, 0)) * sin(q(4, 0));
  H(6, 8) = -d6 * sin(q(1, 0) + q(2, 0) + q(3, 0)) * cos(q(4, 0));
  H(6, 10) = 0.0;

  // 噪声矩阵
  Eigen::MatrixXd R(7, 7);
  R(0, 0) = 0.01 * 0.01; // 角度编码器误差0.1度
  R(1, 1) = 0.01 * 0.01; // 角度编码器误差0.1度
  R(2, 2) = 0.01 * 0.01; // 角度编码器误差0.1度
  R(3, 3) = 0.01 * 0.01; // 角度编码器误差0.1度
  R(4, 4) = 0.01 * 0.01; // 角度编码器误差0.1度
  R(5, 5) = 0.01 * 0.01; // 角度编码器误差0.1度
  R(6, 6) = (0.01 * 0.01 * 10) * (0.01 * 0.01 * 10); // 激光测距误差10um

  // 卡尔曼增益计算
  Eigen::MatrixXd K_k(12, 7);

  // 在计算卡尔曼增益之前添加检查
  Eigen::MatrixXd temp = H * P_k_k_1 * H.transpose() + R;
  if (!temp.determinant()) {
      RCLCPP_WARN(get_node()->get_logger(), "Matrix is singular, skipping update");
  }

  K_k = P_k_k_1 * (H.transpose()) * ((H * P_k_k_1 * H.transpose() + R).inverse());

  // 状态估计与协方差更新
  // 在更新状态之前添加检查
  if (X_k_k_1.hasNaN() || P_k_k_1.hasNaN()) {
    RCLCPP_WARN(get_node()->get_logger(), "NaN detected in prediction step");
    std::stringstream ss;
    ss << "X_k_1_k_1:\n" << X_k_1_k_1.transpose() << "\n";
    ss << "P_k_1_k_1:\n" << P_k_1_k_1 << "\n";
    ss << "F:\n" << F << "\n";
    ss << "Q:\n" << Q;
    RCLCPP_WARN(get_node()->get_logger(), "%s", ss.str().c_str());
  }
  
  X_k_k = X_k_k_1 + K_k * (Z - H * X_k_k_1);
  RCLCPP_INFO(get_node()->get_logger(), "delat1: %f, delat2: %f, delat3: %f, delat4: %f, delat5: %f, delat6: %f", X_k_k(0, 0)-X_k_1_k_1(0, 0), X_k_k(2, 0)-X_k_1_k_1(2, 0), X_k_k(4, 0)-X_k_1_k_1(4, 0), X_k_k(6, 0)-X_k_1_k_1(6, 0), X_k_k(8, 0)-X_k_1_k_1(8, 0), X_k_k(10, 0)-X_k_1_k_1(10, 0));

  P_k_k = (Eigen::MatrixXd::Identity(12, 12) - K_k * H) * P_k_k_1;  
}

double ScaledJointTrajectoryController::calculate_z(const Eigen::MatrixXd& X_k_k)
{
  // UR3 机械臂参数
  const double a_2 = -243.65;
  const double a_3 = -213.0;
  const double d_5 = 83.4;
  const double d_6 = 82.4;
  
  double s_2_3_4 = sin(X_k_k(2, 0) + X_k_k(4, 0) + X_k_k(6, 0));
  double s_2_3 = sin(X_k_k(2, 0) + X_k_k(4, 0));
  double s_2 = sin(X_k_k(2, 0));
  double s_5 = sin(X_k_k(8, 0));
  double c_2_3_4 = cos(X_k_k(2, 0) + X_k_k(4, 0) + X_k_k(6, 0));
  
  double z = s_2_3_4 * s_5 * d_6 - c_2_3_4 * d_5 + s_2_3 * a_3 + s_2 * a_2;
  return z;
}

void ScaledJointTrajectoryController::start_expend_kalman_filter_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data)
  {
    // // 初始化状态向量
    // X_k_k = Eigen::MatrixXd::Zero(12, 1);
    // X_k_1_k_1 = Eigen::MatrixXd::Zero(12, 1);
    
    // 初始化协方差矩阵
    P_k_k = Eigen::MatrixXd::Identity(12, 12) * 0.0001;
    P_k_1_k_1 = Eigen::MatrixXd::Identity(12, 12) * 0.0001;
    
    // 初始化状态转移矩阵
    F = Eigen::MatrixXd::Identity(12, 12);
    double delta_t = 0.008;
    for (int i = 0; i < 6; i++) {
      F(2*i, 2*i+1) = delta_t;
    }
    
    // 初始化过程噪声矩阵
    Q = Eigen::MatrixXd::Identity(12, 12);
    for (int i = 0; i < 6; i++) {
      Q(2*i, 2*i) = 0.001 * 0.001;     // 位置噪声
      Q(2*i+1, 2*i+1) = 0.01 * 0.01;   // 速度噪声
    }
    
    // 确保平台高度有效
    if (distance_platform <= 0.0) {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid platform height: %f", distance_platform);
      start_expend_kalman_filter = false;
      return;
    }
  }else
  {
    // 重置协方差矩阵和平台高度
    P_k_k = Eigen::MatrixXd::Zero(12, 12);
    P_k_1_k_1 = Eigen::MatrixXd::Zero(12, 12);
    X_k_k = Eigen::MatrixXd::Zero(12, 1);
    X_k_1_k_1 = Eigen::MatrixXd::Zero(12, 1);
    distance_platform = 0.0;
  }

  if (distance_platform == 0.0 && msg->data)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Distance platform is 0.0, please check the platform height input! Now change to close expend kalman filter!");
    start_expend_kalman_filter = false;
  }else
  {
    start_expend_kalman_filter = msg->data;
  }
}

void ScaledJointTrajectoryController::distance_platform_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  distance_platform = msg->data;
}

}    // namespace ur_controllers



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ur_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
