#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_msgs/msg/c_points.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <uav_core/PID.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std::placeholders;
using namespace custom_msgs::msg;

class UAV : public rclcpp::Node
{
public:
	UAV();
	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	// Publishers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// Subscriptions
	rclcpp::Subscription<VehicleAttitude>::SharedPtr attit_sub_;
	rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
	rclcpp::Subscription<VehicleControlMode>::SharedPtr control_mode_sub_;
	rclcpp::Subscription<CPoints>::SharedPtr c_points_sub_;
	std::atomic<uint64_t> timestamp_; //!< common synced timestamped

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	// Publisher Callbacks
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float vx, float vy, float vz, float yawspeed);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	// Subscriber Callbacks
	void listen_joy(const Joy::SharedPtr msg);
	void listen_attitude(const VehicleAttitude::SharedPtr msg);
	void listen_control_mode(const VehicleControlMode::SharedPtr msg);
	void listen_c_points(const CPoints::SharedPtr msg);
	void timer_callback();

	PID<double> pid_x{0.9, 0.0, 0.25, 0.1, 0.0, 2.0};
	PID<double> pid_y{0.9, 0.0, 0.25, 0.1, 0.0, 2.0};
	bool aruco_flag = false;
	struct Velocity
	{
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		float yaw = 0.0;
	} velocity_;

	CPoints::SharedPtr c_points_msg_;
	double x = 0.0f;
	double y = 0.0f;
	double z = 0.0f;

	float yaw = 0.0;
	Joy::SharedPtr joy_msg_;
	bool is_armed_ = false;
};