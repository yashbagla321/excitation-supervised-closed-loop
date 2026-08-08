#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Measurements.hpp"
#include "adaptive_localization/World.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ros_result_helpers.hpp"

namespace {

visualization_msgs::msg::Marker make_line_marker(
    const std::string& frame_id,
    const std::string& ns,
    int id,
    const adaptive::Vec2& a,
    const adaptive::Vec2& b,
    float red,
    float green,
    float blue,
    float alpha) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.025;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    geometry_msgs::msg::Point p0;
    p0.x = a.x;
    p0.y = a.y;
    p0.z = 0.08;
    geometry_msgs::msg::Point p1;
    p1.x = b.x;
    p1.y = b.y;
    p1.z = 0.08;
    marker.points.push_back(p0);
    marker.points.push_back(p1);
    return marker;
}

}  // namespace

class MeasurementEmulatorNode final : public rclcpp::Node {
public:
    MeasurementEmulatorNode()
        : Node("measurement_emulator_node") {
        beacon_count_ = declare_parameter<int>("beacon_count", 1);
        path_steps_ = declare_parameter<int>("path_steps", 80);
        seed_ = declare_parameter<int>("seed", 42);
        range_sigma_ = declare_parameter<double>("range_sigma", 0.03);
        bearing_sigma_ = declare_parameter<double>("bearing_sigma", 0.006);
        dropout_probability_ = declare_parameter<double>("dropout_probability", 0.0);
        output_dir_ = declare_parameter<std::string>("output_dir", "results/ros_gz");
        trajectory_ = declare_parameter<std::string>("trajectory", "excited");
        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "cooperative_localization/measurement_markers", rclcpp::QoS(1).transient_local());
        timer_ = create_wall_timer(
            std::chrono::milliseconds(250),
            [this]() {
                run_once();
            });
    }

private:
    void run_once() {
        if (has_run_) {
            return;
        }
        has_run_ = true;

        const adaptive::World world = adaptive::make_world(beacon_count_);
        const auto path = adaptive::make_vehicle_path(path_steps_, trajectory_);
        adaptive::Noise noise{range_sigma_, bearing_sigma_};
        std::mt19937 rng(static_cast<unsigned int>(seed_));
        std::bernoulli_distribution dropout(dropout_probability_);

        const auto output_path = cooperative_localization_gz::output_path_from_param(
            output_dir_, "ros_gz_measurements.csv");
        if (!output_path.parent_path().empty()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        std::ofstream out(output_path);
        out << "time,beacon,robot_x,robot_y,rv,bv_local,rt,bt_local\n";
        out << std::fixed << std::setprecision(8);

        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        for (std::size_t t = 0; t < path.size(); ++t) {
            for (std::size_t beacon = 0; beacon < world.beacons.size(); ++beacon) {
                if (dropout(rng)) {
                    continue;
                }
                const auto measurement = adaptive::make_local_frame_measurement(
                    world, path[t], beacon, t, noise, rng);
                out << t << ',' << beacon << ',' << path[t].x << ',' << path[t].y << ','
                    << measurement.rv << ',' << measurement.bv_local << ','
                    << measurement.rt << ',' << measurement.bt_local << '\n';
                if (t % 8U == 0U) {
                    markers.markers.push_back(make_line_marker(
                        "world",
                        "range_vehicle",
                        marker_id++,
                        world.beacons[beacon],
                        path[t],
                        0.15F,
                        0.45F,
                        0.95F,
                        0.35F));
                    markers.markers.push_back(make_line_marker(
                        "world",
                        "range_target",
                        marker_id++,
                        world.beacons[beacon],
                        world.target,
                        0.95F,
                        0.35F,
                        0.15F,
                        0.25F));
                }
            }
        }
        marker_publisher_->publish(markers);
        RCLCPP_INFO(
            get_logger(),
            "Wrote emulated local-frame measurements to %s",
            output_path.string().c_str());
    }

    bool has_run_ = false;
    int beacon_count_ = 1;
    int path_steps_ = 80;
    int seed_ = 42;
    double range_sigma_ = 0.03;
    double bearing_sigma_ = 0.006;
    double dropout_probability_ = 0.0;
    std::string output_dir_;
    std::string trajectory_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeasurementEmulatorNode>());
    rclcpp::shutdown();
    return 0;
}
