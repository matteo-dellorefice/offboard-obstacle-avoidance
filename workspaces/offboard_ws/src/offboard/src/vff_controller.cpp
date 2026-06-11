#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <algorithm>
#include <queue>
#include <Eigen/Dense>

using namespace std::chrono_literals;

struct obstacle {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
};

static std::vector<obstacle> obstacles = 
{
    { { 5., -8., 0. }, { 5., 2., 0. } },
    { { 10., 5., 0. }, { -10., 5., 0. } },
    { { 10., 12., 0. }, { -10., 12., 0. } },
    { { -10., 5, 0. }, { -10, 21.5, 0. } }
};

struct vff_controller : public rclcpp::Node
{ 
    Eigen::Vector3d waypoint_;
    Eigen::Vector3d localpos_;
    Eigen::Vector3d localvel_;
    double heading_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr localpos_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    int offboard_setpoint_counter_;
    rclcpp::TimerBase::SharedPtr px4_comm_timer_;

    vff_controller() : 
		Node("vff_controller")
	{
        waypoint_= {0., 0., -5};
        offboard_setpoint_counter_ = 0;

        /** Subscribers */
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/waypoint", 10, 
			[this](const geometry_msgs::msg::Point::SharedPtr msg) {
                waypoint_ = { msg->x, msg->y, msg->z };
            });

        localpos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            rclcpp::SensorDataQoS(),
            std::bind(&vff_controller::vehicle_position_sub_cb, this, std::placeholders::_1));
        
        /** Publishers */
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
		vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);

        px4_comm_timer_ = this->create_wall_timer(100ms, [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
			    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
			}

            publish_offboard_control();

            if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
        });
    }

    void vehicle_position_sub_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_offboard_control();

    void pub_arrow_marker(int id, Eigen::Vector3d rgb, Eigen::Vector3d start, Eigen::Vector3d end);
    Eigen::Vector3d compute_vff();
};

void vff_controller::vehicle_position_sub_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    localpos_ = { msg->x, msg->y, msg->z };
    localvel_ = { msg->vx, msg->vy, msg->vz };
    heading_ = msg->heading;
}

void vff_controller::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};

	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 0;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_pub_->publish(msg);
}

void vff_controller::publish_offboard_control() 
{
    px4_msgs::msg::OffboardControlMode oc_msg {};
	oc_msg.position = false;
	oc_msg.velocity = true;
	oc_msg.acceleration = false;
	oc_msg.attitude = false;
	oc_msg.body_rate = false;
	oc_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_pub_->publish(oc_msg);

    px4_msgs::msg::TrajectorySetpoint ts_msg {};

    ts_msg.yaw = std::atan2(waypoint_[1] - localpos_[1], waypoint_[0] - localpos_[0]);

    auto next_pos = compute_vff();
    ts_msg.position = { (float) next_pos[0], (float) next_pos[1], (float) next_pos[2] };

    trajectory_setpoint_pub_->publish(ts_msg);
}

double gaussian(double sigma, Eigen::Vector3d c, Eigen::Vector3d x)
{
    return std::exp(-1. * std::pow((x - c).norm(), 2.) / (2. * sigma));
}

double inv_power(double ampl, Eigen::Vector3d c, Eigen::Vector3d x)
{
    return ampl * pow(1. / (x - c).norm(), 4.);
}

/**
 * Calculates the closest point on segment ab to point x.
 */
Eigen::Vector3d segment_distance(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d x)
{
    double l2 = pow((a - b).norm(), 2.);

    if (l2 == 0.) return a;

    double t = std::max(0., std::min(1., (x - a).dot(b - a) / l2));
    auto proj = a + t * (b - a);

    return proj;
}

Eigen::Vector3d vff_controller::compute_vff()
{
    double coeff = 10. * (1. - gaussian(200., waypoint_, localpos_));
    Eigen::Vector3d target_force = coeff * (waypoint_ - localpos_).normalized();
    for (size_t i = 0; i < obstacles.size(); i++) {
        Eigen::Vector3d c = segment_distance(obstacles[i].a, obstacles[i].b, localpos_);
        c[2] = -5.; // TODO generalize to 3D
        Eigen::Vector3d f = (localpos_ - c).normalized();
        coeff = inv_power(100., c, localpos_);
        target_force += coeff * f;
    }

    Eigen::Vector3d _test = 2.5 * target_force.normalized();
    Eigen::Vector3d result = localpos_ + _test;
    result[2] = -5.;
    
    return result;
}

void vff_controller::pub_arrow_marker(int id, Eigen::Vector3d rgb, Eigen::Vector3d start, Eigen::Vector3d end)
{
    geometry_msgs::msg::Point _start;
    _start.x = start[0];
    _start.y = start[1];
    _start.z = start[2];
    geometry_msgs::msg::Point _end;
    _end.x = end[0];
    _end.y = end[1];
    _end.z = end[2];

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "my_frame";
    marker.header.stamp = rclcpp::Clock().now();

    marker.ns = "gradients";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points = { _start, _end };

    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = rgb[0];
    marker.color.g = rgb[1];
    marker.color.b = rgb[2];
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_nanoseconds(1000);

    marker_pub_->publish(marker);
}

int main(int argc, char *argv[])
{
    std::cout << "Virtual Force Field Controller starting up" << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vff_controller>());
    rclcpp::shutdown();
	
    return 0;
}