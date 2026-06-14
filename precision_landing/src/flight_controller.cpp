#include <uav_core/uav.hpp>

UAV::UAV() : Node("offboard_control")
{
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
	offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
	vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
	joy_msg_ = std::make_shared<Joy>();

	joy_sub_ = this->create_subscription<Joy>(
		"/joy",
		qos,
		std::bind(&UAV::listen_joy, this, _1));

	attit_sub_ = this->create_subscription<VehicleAttitude>(
		"/fmu/out/vehicle_attitude",
		qos,
		std::bind(&UAV::listen_attitude, this, _1));

	control_mode_sub_ = this->create_subscription<VehicleControlMode>(
		"/fmu/out/vehicle_control_mode",
		qos,
		std::bind(&UAV::listen_control_mode, this, _1));

	c_points_sub_ = this->create_subscription<CPoints>(
		"/aruco_center",
		qos,
		std::bind(&UAV::listen_c_points, this, _1));

	offboard_setpoint_counter_ = 0;
	timer_ = this->create_wall_timer(100ms, std::bind(&UAV::timer_callback, this));
}

void UAV::timer_callback()
{

	if (offboard_setpoint_counter_ == 10)
	{
		// Change to Offboard mode after 10 setpoints
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	}

	if (this->joy_msg_->axes.size() >= 5 &&
		this->joy_msg_->axes[0] <= -0.5 && this->joy_msg_->axes[1] <= -0.5 &&
		this->joy_msg_->axes[3] >= 0.5 && this->joy_msg_->axes[4] <= 0.5)
	{
		this->arm();
	}

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();

	publish_trajectory_setpoint(
		velocity_.x + x,
		velocity_.y + y,
		velocity_.z + z,
		velocity_.yaw);

	// stop the counter after reaching 11
	if (offboard_setpoint_counter_ < 11)
	{
		offboard_setpoint_counter_++;
	}
}

void UAV::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void UAV::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void UAV::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void UAV::publish_trajectory_setpoint(float vx, float vy, float vz, float yawspeed)
{
	TrajectorySetpoint msg{};
	float cos_yaw = std::cos(yaw);
	float sin_yaw = std::sin(yaw);

	float world_vx = (vx * cos_yaw - vy * sin_yaw);
	float world_vy = (vx * sin_yaw + vy * cos_yaw);
	msg.velocity = {world_vx, world_vy, vz};
	msg.yawspeed = yawspeed; // [rad/s]

	// Disable position and acceleration to use velocity control
	msg.position = {NAN, NAN, NAN};
	msg.yaw = NAN; // [-PI:PI]
	msg.acceleration = {NAN, NAN, NAN};

	// publishing
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void UAV::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void UAV::listen_joy(const Joy::SharedPtr msg)
{
	this->joy_msg_ = msg;
	if (msg->axes.size() >= 5)
	{
		velocity_ = {
			static_cast<float>(msg->axes[4] * 12.0f),
			static_cast<float>(-msg->axes[3] * 12.0f),
			static_cast<float>(-msg->axes[1] * 12.0f),
			static_cast<float>(-msg->axes[0] * 3.14f),
		};
	}
}

void UAV::listen_attitude(const VehicleAttitude::SharedPtr msg)
{
	// Convert quaternion to yaw angle
	this->yaw = std::atan2(2.0f * (msg->q[0] * msg->q[3] + msg->q[1] * msg->q[2]),
						   1.0f - 2.0f * (msg->q[2] * msg->q[2] + msg->q[3] * msg->q[3]));
}

void UAV::listen_control_mode(const VehicleControlMode::SharedPtr msg)
{
	this->is_armed_ = msg->flag_armed;
}

void UAV::listen_c_points(const CPoints::SharedPtr msg)
{
	aruco_flag = true;

	if (msg->detected == false)
	{
		RCLCPP_INFO(this->get_logger(), "No ArUco marker detected.");
		aruco_flag = false;
		x = 0.0f;
		y = 0.0f;

		return;
	}

	double error_x = msg->box_cx - msg->cx;
	double error_y = msg->box_cy - msg->cy;

	pid_y.setSetpoint(msg->cy);
	pid_x.setSetpoint(msg->cx);

	x = pid_y.update(msg->box_cy);
	y = -pid_x.update(msg->box_cx);

	if (std::sqrt(error_x * error_x + error_y * error_y) < 0.01)
		z = 0.5f;

	RCLCPP_INFO(this->get_logger(), " %.2f, %.2f", error_x, error_y);

	RCLCPP_INFO(this->get_logger(), "Updated velocities: vx=%.2f, vy=%.2f (cx:%.2f, cy:%.2f box_cx:%.2f box_cy:%.2f)",
				velocity_.x + x, velocity_.y + y,
				msg->cx, msg->cy, msg->box_cx, msg->box_cy);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UAV>());

	rclcpp::shutdown();
	return 0;
}