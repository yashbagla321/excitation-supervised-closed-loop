#include <memory>
#include <random>
#include <string>
#include <chrono>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Simulation.hpp"
#include "adaptive_localization/World.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ros_result_helpers.hpp"

namespace {

visualization_msgs::msg::Marker make_sphere_marker(
    const std::string& frame_id,
    const std::string& ns,
    int id,
    const adaptive::Vec2& position,
    double radius,
    float red,
    float green,
    float blue,
    float alpha) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = 0.2;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    return marker;
}

}  // namespace

class HiddenTargetExperimentNode final : public rclcpp::Node {
public:
    HiddenTargetExperimentNode()
        : Node("hidden_target_experiment_node") {
        scenario_ = declare_parameter<int>("scenario", 1);
        beacon_count_ = declare_parameter<int>("beacon_count", 1);
        seed_ = declare_parameter<int>("seed", 42);
        output_dir_ = declare_parameter<std::string>("output_dir", "results/ros_gz");
        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "cooperative_localization/markers", rclcpp::QoS(1).transient_local());
        timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
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

        adaptive::SimulationConfig config;
        config.output_dir = output_dir_;
        std::mt19937 rng(static_cast<unsigned int>(seed_));
        const adaptive::TrialResult trial =
            adaptive::run_trial(scenario_, beacon_count_, 0, config, rng);
        cooperative_localization_gz::write_ros_trial_csv(
            cooperative_localization_gz::output_path_from_param(output_dir_, "ros_gz_single_trial.csv"),
            trial);

        publish_markers(trial);
        RCLCPP_INFO(
            get_logger(),
            "Wrote ROS/Gazebo validation trial to %s",
            cooperative_localization_gz::output_path_from_param(
                output_dir_, "ros_gz_single_trial.csv").string().c_str());
    }

    void publish_markers(const adaptive::TrialResult& trial) {
        const adaptive::World world = adaptive::make_world(beacon_count_);
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(make_sphere_marker(
            "world", "truth", 0, world.target, 0.24, 0.90F, 0.20F, 0.16F, 1.0F));
        markers.markers.push_back(make_sphere_marker(
            "world", "estimate", 1, trial.estimate, 0.20, 0.10F, 0.40F, 0.95F, 1.0F));
        for (std::size_t i = 0; i < world.beacons.size(); ++i) {
            markers.markers.push_back(make_sphere_marker(
                "world",
                "beacon_truth",
                static_cast<int>(10 + i),
                world.beacons[i],
                0.20,
                0.10F,
                0.75F,
                0.35F,
                1.0F));
        }
        marker_publisher_->publish(markers);
    }

    bool has_run_ = false;
    int scenario_ = 1;
    int beacon_count_ = 1;
    int seed_ = 42;
    std::string output_dir_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HiddenTargetExperimentNode>());
    rclcpp::shutdown();
    return 0;
}
