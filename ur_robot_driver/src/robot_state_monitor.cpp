#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/ur/version_information.h>
#include <ur_client_library/exceptions.h>

#include <memory>
#include <string>
#include <vector>
#include <array>

class RobotStateMonitor : public rclcpp::Node
{
public:
  RobotStateMonitor() : Node("robot_state_monitor")
  {
    // 获取参数
    this->declare_parameter("tf_prefix", "");
    this->declare_parameter("publish_rate", 125.0);
    this->declare_parameter("robot_ip", "192.168.56.101");
    this->declare_parameter("script_filename", "");
    this->declare_parameter("output_recipe_filename", "");
    this->declare_parameter("input_recipe_filename", "");
    
    tf_prefix_ = this->get_parameter("tf_prefix").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    robot_ip_ = this->get_parameter("robot_ip").as_string();
    script_filename_ = this->get_parameter("script_filename").as_string();
    output_recipe_filename_ = this->get_parameter("output_recipe_filename").as_string();
    input_recipe_filename_ = this->get_parameter("input_recipe_filename").as_string();
    
    // 创建发布者
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_copy", 10);
    
    // 初始化机器人数据
    initialize_robot_data();
    
    // 尝试连接机器人
    if (!connect_to_robot()) {
      RCLCPP_WARN(this->get_logger(), "Failed to connect to robot, using simulation mode");
      simulation_mode_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Successfully connected to robot at %s", robot_ip_.c_str());
      simulation_mode_ = false;
    }
    
    // 创建定时器
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&RobotStateMonitor::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Robot State Monitor initialized");
    RCLCPP_INFO(this->get_logger(), "Publish Rate: %f Hz", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "Mode: %s", simulation_mode_ ? "Simulation" : "Real Robot");
  }

private:
  void initialize_robot_data()
  {
    // 初始化关节数据
    joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_efforts_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // 初始化机器人状态
    speed_scaling_ = 1.0;
    robot_status_ = "Initializing";
    robot_mode_ = 0;
    safety_mode_ = 0;
    runtime_state_ = 0;
  }
  
  bool connect_to_robot()
  {
    try {
      // 创建UR驱动实例
      ur_driver_ = std::make_unique<urcl::UrDriver>(
          robot_ip_, script_filename_, output_recipe_filename_, input_recipe_filename_,
          std::bind(&RobotStateMonitor::handleRobotProgramState, this, std::placeholders::_1), 
          true,  // headless_mode
          nullptr,  // tool_comm_setup
          50002,  // reverse_port
          50003,  // script_sender_port
          100,    // servoj_gain
          0.008,  // servoj_lookahead_time
          false,  // non_blocking_read
          "",     // reverse_ip
          50004,  // trajectory_port
          50005   // script_command_port
      );
      
      // 启动RTDE通信
      ur_driver_->startRTDECommunication();
      rtde_comm_started_ = true;
      
      return true;
    } catch (const urcl::UrException& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot: %s", e.what());
      return false;
    }
  }
  
  void handleRobotProgramState(bool program_running)
  {
    robot_program_running_ = program_running;
    RCLCPP_INFO(this->get_logger(), "Robot program running: %s", program_running ? "true" : "false");
  }
  
  void timer_callback()
  {
    // 读取机器人数据
    read_robot_data();
    
    // 发布关节状态
    publish_joint_states();
    
    // 发布TCP位姿
    publish_tcp_pose();
    
    // 发布力传感器数据
    publish_ft_sensor();
    
    // 发布机器人状态
    publish_robot_status();
  }
  
  void read_robot_data()
  {
    if (simulation_mode_) {
      // 使用模拟数据
      read_simulation_data();
    } else {
      // 读取真实机器人数据
      read_real_robot_data();
    }
  }
  
  void read_simulation_data()
  {
    // 模拟关节数据
    static double time = 0.0;
    time += 1.0 / publish_rate_;
    
    joint_positions_ = {0.1 * sin(time), 0.1 * cos(time), 0.05 * sin(time * 2), 
                       0.05 * cos(time * 2), 0.02 * sin(time * 3), 0.02 * cos(time * 3)};
    joint_velocities_ = {0.1 * cos(time), -0.1 * sin(time), 0.1 * cos(time * 2), 
                         -0.1 * sin(time * 2), 0.06 * cos(time * 3), -0.06 * sin(time * 3)};
    joint_efforts_ = {0.01 * sin(time), 0.01 * cos(time), 0.005 * sin(time * 2), 
                      0.005 * cos(time * 2), 0.002 * sin(time * 3), 0.002 * cos(time * 3)};
    
    // 模拟TCP位姿
    tcp_position_ = {0.5 + 0.1 * sin(time), 0.1 * cos(time), 0.5};
    tcp_orientation_ = {0.0, 0.0, 0.0, 1.0};
    
    // 模拟力传感器数据
    ft_force_ = {0.1 * sin(time), 0.1 * cos(time), 5.0}; // 5N重力
    ft_torque_ = {0.01 * sin(time), 0.01 * cos(time), 0.0};
    
    // 模拟机器人状态
    speed_scaling_ = 0.8 + 0.2 * sin(time * 0.1);
    robot_status_ = "Robot is running normally (simulation)";
    robot_mode_ = 7; // RUNNING
    safety_mode_ = 1; // NORMAL
    runtime_state_ = 1; // PLAYING
  }
  
  void read_real_robot_data()
  {
    if (!ur_driver_ || !rtde_comm_started_) {
      RCLCPP_WARN(this->get_logger(), "Robot driver not available");
      return;
    }
    
    try {
      // 获取数据包
      std::unique_ptr<rtde::DataPackage> data_pkg = ur_driver_->getDataPackage();
      
      if (data_pkg) {
        // 读取关节数据
        readData(data_pkg, "actual_q", joint_positions_);
        readData(data_pkg, "actual_qd", joint_velocities_);
        readData(data_pkg, "actual_current", joint_efforts_);
        
        // 读取TCP位姿
        std::array<double, 6> tcp_pose;
        readData(data_pkg, "actual_TCP_pose", tcp_pose);
        tcp_position_ = {tcp_pose[0], tcp_pose[1], tcp_pose[2]};
        // 将欧拉角转换为四元数（简化版本）
        tcp_orientation_ = {0.0, 0.0, 0.0, 1.0};
        
        // 读取力传感器数据
        readData(data_pkg, "actual_TCP_force", ft_sensor_data_);
        ft_force_ = {ft_sensor_data_[0], ft_sensor_data_[1], ft_sensor_data_[2]};
        ft_torque_ = {ft_sensor_data_[3], ft_sensor_data_[4], ft_sensor_data_[5]};
        
        // 读取机器人状态
        readData(data_pkg, "speed_scaling", speed_scaling_);
        readData(data_pkg, "robot_mode", robot_mode_);
        readData(data_pkg, "safety_mode", safety_mode_);
        readData(data_pkg, "runtime_state", runtime_state_);
        
        // 更新机器人状态字符串
        update_robot_status_string();
        
        packet_read_ = true;
      } else {
        RCLCPP_WARN(this->get_logger(), "No data package received from robot");
        packet_read_ = false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error reading robot data: %s", e.what());
      packet_read_ = false;
    }
  }
  
  template <typename T>
  void readData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                const std::string& var_name, T& data)
  {
    if (!data_pkg->getData(var_name, data)) {
      std::string error_msg = "Did not find '" + var_name + "' in data sent from robot.";
      RCLCPP_WARN(this->get_logger(), "%s", error_msg.c_str());
    }
  }
  
  void update_robot_status_string()
  {
    std::string mode_str = "Unknown";
    switch (robot_mode_) {
      case 0: mode_str = "DISCONNECTED"; break;
      case 1: mode_str = "CONFIRM_SAFETY"; break;
      case 2: mode_str = "BOOTING"; break;
      case 3: mode_str = "POWER_OFF"; break;
      case 4: mode_str = "POWER_ON"; break;
      case 5: mode_str = "IDLE"; break;
      case 6: mode_str = "BACKDRIVE"; break;
      case 7: mode_str = "RUNNING"; break;
      case 8: mode_str = "UPDATING_FIRMWARE"; break;
    }
    
    std::string safety_str = "Unknown";
    switch (safety_mode_) {
      case 0: safety_str = "NORMAL"; break;
      case 1: safety_str = "REDUCED"; break;
      case 2: safety_str = "PROTECTIVE_STOP"; break;
      case 3: safety_str = "RECOVERY"; break;
      case 4: safety_str = "SAFEGUARD_STOP"; break;
      case 5: safety_str = "SYSTEM_EMERGENCY_STOP"; break;
      case 6: safety_str = "ROBOT_EMERGENCY_STOP"; break;
      case 7: safety_str = "VIOLATION"; break;
      case 8: safety_str = "FAULT"; break;
      case 9: safety_str = "AUTOMATIC_MODE_SAFEGUARD_STOP"; break;
      case 10: safety_str = "SYSTEM_THREE_POSITION_ENABLING_STOP"; break;
    }
    
    robot_status_ = "Mode: " + mode_str + ", Safety: " + safety_str + 
                   ", Speed: " + std::to_string(static_cast<int>(speed_scaling_ * 100)) + "%";
  }
  
  void publish_joint_states()
  {
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    msg->position = joint_positions_;
    msg->velocity = joint_velocities_;
    msg->effort = joint_efforts_;
    
    joint_state_pub_->publish(*msg);
  }
  
  void publish_tcp_pose()
  {
    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = tf_prefix_ + "base_link";
    
    msg->pose.position.x = tcp_position_[0];
    msg->pose.position.y = tcp_position_[1];
    msg->pose.position.z = tcp_position_[2];
    
    msg->pose.orientation.x = tcp_orientation_[0];
    msg->pose.orientation.y = tcp_orientation_[1];
    msg->pose.orientation.z = tcp_orientation_[2];
    msg->pose.orientation.w = tcp_orientation_[3];
    
    tcp_pose_pub_->publish(*msg);
  }
  
  void publish_ft_sensor()
  {
    auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = tf_prefix_ + "tool0_controller";
    
    msg->wrench.force.x = ft_force_[0];
    msg->wrench.force.y = ft_force_[1];
    msg->wrench.force.z = ft_force_[2];
    
    msg->wrench.torque.x = ft_torque_[0];
    msg->wrench.torque.y = ft_torque_[1];
    msg->wrench.torque.z = ft_torque_[2];
    
    ft_sensor_pub_->publish(*msg);
  }
  
  void publish_robot_status()
  {
    // 发布机器人状态
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data = robot_status_;
    robot_status_pub_->publish(*status_msg);
    
    // 发布速度缩放
    auto speed_msg = std::make_unique<std_msgs::msg::Float64>();
    speed_msg->data = speed_scaling_;
    speed_scaling_pub_->publish(*speed_msg);
  }

  // 成员变量
  std::string tf_prefix_;
  double publish_rate_;
  std::string robot_ip_;
  std::string script_filename_;
  std::string output_recipe_filename_;
  std::string input_recipe_filename_;
  
  // 机器人连接
  std::unique_ptr<urcl::UrDriver> ur_driver_;
  bool rtde_comm_started_ = false;
  bool simulation_mode_ = false;
  bool packet_read_ = false;
  bool robot_program_running_ = false;
  
  // 机器人数据
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::array<double, 3> tcp_position_;
  std::array<double, 4> tcp_orientation_;
  std::array<double, 3> ft_force_;
  std::array<double, 3> ft_torque_;
  std::array<double, 6> ft_sensor_data_;
  double speed_scaling_;
  std::string robot_status_;
  uint32_t robot_mode_;
  uint32_t safety_mode_;
  uint32_t runtime_state_;
  
  // 发布者
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sensor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_scaling_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStateMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
