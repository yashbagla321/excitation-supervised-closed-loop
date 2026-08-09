// Excitation-supervised closed-loop control (Algorithm 1 of the CDC paper)
// executed software-in-the-loop through ROS 2 and Gazebo:
//
//   - the VEHICLE is a Gazebo model; its motion is integrated by the Gazebo
//     physics engine from velocity commands this node publishes on
//     /model/vehicle/cmd_vel through ros_gz_bridge (ZOH actuation with real
//     transport latency, not the explicit-Euler kinematics of the batch
//     simulator);
//   - the vehicle POSE consumed by the estimator is the odometry Gazebo
//     publishes back over the bridge;
//   - range--bearing packets are emulated from that pose with the same
//     noise model as the batch simulator, and are delivered to the
//     estimator after a configurable sensing delay (whole packets);
//   - estimation (Gauss--Newton on the scenario-1 residuals), the spread
//     certificate S_v, the supervision rule, and the excitation command are
//     the same library code the batch simulator runs.
//
// One launch = one experiment; the node logs a CSV with the same schema as
// closed_loop_local_1beacon.csv (plus sim-time and command columns) and
// shuts down when the packet budget is exhausted.

#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "adaptive_localization/Estimators.hpp"
#include "adaptive_localization/Math.hpp"
#include "adaptive_localization/Measurements.hpp"
#include "adaptive_localization/Simulation.hpp"
#include "adaptive_localization/Solver.hpp"
#include "adaptive_localization/Types.hpp"
#include "adaptive_localization/World.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

#include "ros_result_helpers.hpp"

namespace {

struct LogRow {
    int step = 0;
    adaptive::Vec2 robot;
    adaptive::Vec2 target_estimate;
    double target_error = 0.0;
    double goal_error = 0.0;
    double beacon_position_rmse = 0.0;
    double beacon_yaw_rmse = -1.0;
    double cost = 0.0;
    double spread = 0.0;
    double sigma_min = -1.0;
    double excitation_norm2 = 0.0;
    bool retriggered = false;
    bool estimate_ready = false;
    double sim_time = 0.0;
    adaptive::Vec2 command;
    adaptive::Vec2 beacon_estimate;
    double beacon_estimate_yaw = 0.0;
};

// A packet taken at the vehicle's true (Gazebo) pose, waiting out the
// configured sensing delay before the estimator may consume it.
struct PendingPacket {
    adaptive::Vec2 pose;
    adaptive::LocalFrameMeasurement measurement;
    int taken_at_step = 0;
};

}  // namespace

class SupervisedClosedLoopNode final : public rclcpp::Node {
public:
    SupervisedClosedLoopNode()
        : Node("supervised_closed_loop_node") {
        packet_count_ = declare_parameter<int>("packet_count", 120);
        packet_period_ = declare_parameter<double>("packet_period", 0.08);
        control_gain_ = declare_parameter<double>("control_gain", 1.2);
        exploration_amplitude_ = declare_parameter<double>("exploration_amplitude", 0.25);
        exploration_decay_ = declare_parameter<double>("exploration_decay", 0.5);
        exploration_frequency_ = declare_parameter<double>("exploration_frequency", 0.45);
        spread_threshold_ = declare_parameter<double>("spread_threshold", 0.16);
        range_sigma_ = declare_parameter<double>("range_sigma", 0.02);
        bearing_sigma_ = declare_parameter<double>("bearing_sigma", 0.004);
        seed_ = declare_parameter<int>("seed", 7);
        sensing_delay_packets_ = declare_parameter<int>("sensing_delay_packets", 1);
        solver_max_iterations_ = declare_parameter<int>("solver_max_iterations", 25);
        solver_initial_lambda_ = declare_parameter<double>("solver_initial_lambda", 1e-2);
        initial_target_estimate_ = {
            declare_parameter<double>("initial_target_estimate_x", 0.0),
            declare_parameter<double>("initial_target_estimate_y", 0.0),
        };
        initial_beacon_guess_radius_ = declare_parameter<double>("initial_beacon_guess_radius", 2.0);
        initial_beacon_guess_yaw_ = declare_parameter<double>("initial_beacon_guess_yaw", 0.0);
        output_dir_ = declare_parameter<std::string>("output_dir", "results/ros_gz");
        output_name_ = declare_parameter<std::string>("output_name", "closed_loop_gz_run.csv");
        update_estimate_markers_ = declare_parameter<bool>("update_estimate_markers", true);
        world_name_ = declare_parameter<std::string>("world_name", "hidden_target_validation");
        const auto cmd_vel_topic =
            declare_parameter<std::string>("cmd_vel_topic", "/model/vehicle/cmd_vel");
        const auto odometry_topic =
            declare_parameter<std::string>("odometry_topic", "/model/vehicle/odometry");

        world_ = adaptive::make_world(1);
        state_ = adaptive::initial_state_scenario1(
            1, initial_target_estimate_, initial_beacon_guess_radius_, initial_beacon_guess_yaw_);
        beacon_estimates_ = adaptive::beacon_estimates_from_scenario1_state(state_, 1);
        target_estimate_ = initial_target_estimate_;
        rng_.seed(static_cast<unsigned int>(seed_));

        command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry& msg) {
                latest_pose_ = {msg.pose.pose.position.x, msg.pose.pose.position.y};
                have_pose_ = true;
            });
        set_pose_client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/" + world_name_ + "/set_pose");

        // Node-clock timer: with use_sim_time this runs on the Gazebo /clock,
        // so the packet cadence is measured in simulation time and pauses
        // when the simulation pauses.
        timer_ = create_timer(
            std::chrono::duration<double>(packet_period_),
            [this]() {
                on_tick();
            });

        RCLCPP_INFO(
            get_logger(),
            "Supervised closed loop through Gazebo: %d packets at %.0f ms, S_bar=%.3f, "
            "sensing delay %d packet(s), seed %d",
            packet_count_, packet_period_ * 1e3, spread_threshold_, sensing_delay_packets_, seed_);
    }

private:
    void on_tick() {
        if (finished_) {
            return;
        }
        if (!have_pose_) {
            // Gazebo has not published odometry yet; keep waiting on sim time.
            return;
        }
        const adaptive::Vec2 robot = latest_pose_;
        const double sim_time = now().seconds();

        if (step_ == 0) {
            // Initial-condition row, logged before any command is issued --
            // the counterpart of the batch simulator's step-0 point.
            LogRow row;
            row.step = 0;
            row.robot = robot;
            row.target_estimate = target_estimate_;
            row.target_error = adaptive::norm(target_estimate_ - world_.target);
            row.goal_error = adaptive::norm(robot - world_.target);
            row.beacon_position_rmse = adaptive::beacon_position_rmse(world_, beacon_estimates_);
            row.beacon_yaw_rmse = adaptive::beacon_yaw_rmse(world_, beacon_estimates_);
            row.sim_time = sim_time;
            if (!beacon_estimates_.empty()) {
                row.beacon_estimate = beacon_estimates_[0].position;
                row.beacon_estimate_yaw = beacon_estimates_[0].yaw;
            }
            rows_.push_back(row);
            step_ = 1;
            return;
        }

        const int step = step_;
        const adaptive::Noise noise{range_sigma_, bearing_sigma_};

        // Emulate the range--bearing packet at the vehicle's true pose as
        // reported by Gazebo. The measurement's time index is its eventual
        // position in the estimator's pose window (delivery is FIFO, so the
        // taken order equals the delivered order).
        adaptive::LocalFrameMeasurement measurement = adaptive::make_local_frame_measurement(
            world_, robot, 0, static_cast<std::size_t>(packets_taken_), noise, rng_);
        ++packets_taken_;
        pending_.push_back({robot, measurement, step});

        // Deliver every packet whose sensing delay has matured.
        while (!pending_.empty() &&
               pending_.front().taken_at_step + sensing_delay_packets_ <= step) {
            path_.push_back(pending_.front().pose);
            measurements_.push_back(pending_.front().measurement);
            pending_.pop_front();
        }

        if (!estimator_initialized_) {
            std::vector<double> closed_form_seed;
            estimator_initialized_ = adaptive::two_view_closed_form_initial_state(
                1, path_, measurements_, closed_form_seed);
            if (estimator_initialized_) {
                state_ = std::move(closed_form_seed);
            }
        }
        if (estimator_initialized_) {
            const auto result = adaptive::gauss_newton(
                state_,
                [&](const std::vector<double>& state) {
                    return adaptive::residuals_scenario1(state, 1, path_, measurements_, noise);
                },
                solver_max_iterations_,
                solver_initial_lambda_,
                [&](const std::vector<double>& state) {
                    return adaptive::jacobian_scenario1(state, 1, path_, measurements_, noise);
                });
            state_ = result.x;
            target_estimate_ = {state_[0], state_[1]};
            current_cost_ = result.cost;
            beacon_estimates_ = adaptive::beacon_estimates_from_scenario1_state(state_, 1);
        }

        // Spread certificate over the delivered window (the poses the
        // estimator actually holds), plus the conditioning diagnostic --
        // both from the same library code as the batch simulator.
        const double window_spread = adaptive::path_spread(path_);
        const double sigma_min = measurements_.empty()
            ? -1.0
            : adaptive::local_observability_rank_and_sigma_min(
                  state_, path_, measurements_, noise).second;

        // Algorithm 1 supervision: retrigger the excitation epoch while the
        // window spread is below the accuracy-driven threshold S_bar.
        bool retriggered = false;
        if (window_spread < spread_threshold_) {
            excitation_epoch_ = step;
            retriggered = true;
        }

        const double current_time = static_cast<double>(step - 1) * packet_period_;
        const double epoch_time = static_cast<double>(excitation_epoch_ - 1) * packet_period_;
        const double exploration = exploration_amplitude_ *
            std::exp(-exploration_decay_ * (current_time - epoch_time));
        const adaptive::Vec2 excitation{
            exploration * std::cos(exploration_frequency_ * current_time),
            exploration * std::sin(exploration_frequency_ * current_time),
        };
        const adaptive::Vec2 command =
            (target_estimate_ - robot) * control_gain_ + excitation;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = command.x;
        twist.linear.y = command.y;
        command_publisher_->publish(twist);

        if (update_estimate_markers_) {
            publish_estimate_markers();
        }

        LogRow row;
        row.step = step;
        row.robot = robot;
        row.target_estimate = target_estimate_;
        row.target_error = adaptive::norm(target_estimate_ - world_.target);
        row.goal_error = adaptive::norm(robot - world_.target);
        row.beacon_position_rmse = adaptive::beacon_position_rmse(world_, beacon_estimates_);
        row.beacon_yaw_rmse = adaptive::beacon_yaw_rmse(world_, beacon_estimates_);
        row.cost = current_cost_;
        row.spread = window_spread;
        row.sigma_min = sigma_min;
        row.excitation_norm2 = adaptive::dot(excitation, excitation);
        row.retriggered = retriggered;
        row.estimate_ready = estimator_initialized_;
        row.sim_time = sim_time;
        row.command = command;
        if (!beacon_estimates_.empty()) {
            row.beacon_estimate = beacon_estimates_[0].position;
            row.beacon_estimate_yaw = beacon_estimates_[0].yaw;
        }
        rows_.push_back(row);

        if (step >= packet_count_) {
            finish();
            return;
        }
        ++step_;
    }

    // Moves the translucent estimate markers in the Gazebo scene to the
    // current estimates via the bridged set_pose service, so recordings of
    // the run show the estimator converging. Fire-and-forget: a missing or
    // slow service must never stall the control loop.
    void publish_estimate_markers() {
        if (!set_pose_client_->service_is_ready()) {
            if (!warned_no_set_pose_) {
                RCLCPP_WARN(
                    get_logger(),
                    "set_pose service not available; estimate markers will not move");
                warned_no_set_pose_ = true;
            }
            return;
        }
        const auto send = [this](const std::string& name, double x, double y, double z) {
            auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
            request->entity.name = name;
            request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
            request->pose.position.x = x;
            request->pose.position.y = y;
            request->pose.position.z = z;
            request->pose.orientation.w = 1.0;
            set_pose_client_->async_send_request(request);
        };
        send("target_estimate", target_estimate_.x, target_estimate_.y, 0.15);
        if (!beacon_estimates_.empty()) {
            send("beacon_estimate_0",
                 beacon_estimates_[0].position.x, beacon_estimates_[0].position.y, 0.16);
        }
    }

    void finish() {
        finished_ = true;
        geometry_msgs::msg::Twist stop;
        command_publisher_->publish(stop);
        write_csv();
        const auto& final_row = rows_.back();
        RCLCPP_INFO(
            get_logger(),
            "Finished %d packets: goal %.4f m, target %.4f m, beacon %.4f m, yaw %.4f rad, "
            "%d retriggered packet(s)",
            packet_count_, final_row.goal_error, final_row.target_error,
            final_row.beacon_position_rmse, final_row.beacon_yaw_rmse, count_retriggers());
        rclcpp::shutdown();
    }

    int count_retriggers() const {
        int count = 0;
        for (const auto& row : rows_) {
            if (row.retriggered) {
                ++count;
            }
        }
        return count;
    }

    void write_csv() const {
        const auto output_path =
            cooperative_localization_gz::output_path_from_param(output_dir_, output_name_);
        if (!output_path.parent_path().empty()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        std::ofstream out(output_path);
        out << "step,robot_x,robot_y,target_estimate_x,target_estimate_y,target_error,goal_error,"
               "beacon_position_rmse,beacon_yaw_rmse,cost,spread,sigma_min,excitation_norm2,"
               "retriggered,estimate_ready,sim_time,cmd_vx,cmd_vy,"
               "beacon_estimate_x,beacon_estimate_y,beacon_estimate_yaw\n";
        out << std::fixed << std::setprecision(8);
        for (const auto& row : rows_) {
            out << row.step << ',' << row.robot.x << ',' << row.robot.y << ','
                << row.target_estimate.x << ',' << row.target_estimate.y << ','
                << row.target_error << ',' << row.goal_error << ','
                << row.beacon_position_rmse << ',';
            if (row.beacon_yaw_rmse >= 0.0) {
                out << row.beacon_yaw_rmse;
            }
            out << ',' << row.cost << ',' << row.spread << ',';
            if (row.sigma_min >= 0.0) {
                out << row.sigma_min;
            }
            out << ',' << row.excitation_norm2 << ',' << (row.retriggered ? 1 : 0) << ','
                << (row.estimate_ready ? 1 : 0) << ',' << row.sim_time << ','
                << row.command.x << ',' << row.command.y << ','
                << row.beacon_estimate.x << ',' << row.beacon_estimate.y << ','
                << row.beacon_estimate_yaw << '\n';
        }
        RCLCPP_INFO(
            get_logger(), "Wrote Gazebo closed-loop log to %s", output_path.string().c_str());
    }

    // Experiment parameters.
    int packet_count_ = 120;
    double packet_period_ = 0.08;
    double control_gain_ = 1.2;
    double exploration_amplitude_ = 0.25;
    double exploration_decay_ = 0.5;
    double exploration_frequency_ = 0.45;
    double spread_threshold_ = 0.16;
    double range_sigma_ = 0.02;
    double bearing_sigma_ = 0.004;
    int seed_ = 7;
    int sensing_delay_packets_ = 1;
    int solver_max_iterations_ = 25;
    double solver_initial_lambda_ = 1e-2;
    adaptive::Vec2 initial_target_estimate_;
    double initial_beacon_guess_radius_ = 2.0;
    double initial_beacon_guess_yaw_ = 0.0;
    std::string output_dir_;
    std::string output_name_;
    std::string world_name_;
    bool update_estimate_markers_ = true;

    // Estimator and supervision state.
    adaptive::World world_;
    std::vector<double> state_;
    bool estimator_initialized_ = false;
    std::vector<adaptive::BeaconEstimate> beacon_estimates_;
    adaptive::Vec2 target_estimate_;
    std::mt19937 rng_;
    std::vector<adaptive::Vec2> path_;
    std::vector<adaptive::LocalFrameMeasurement> measurements_;
    std::deque<PendingPacket> pending_;
    int packets_taken_ = 0;
    int excitation_epoch_ = 1;
    double current_cost_ = 0.0;
    int step_ = 0;
    bool finished_ = false;

    // Gazebo interface state.
    adaptive::Vec2 latest_pose_;
    bool have_pose_ = false;
    bool warned_no_set_pose_ = false;
    std::vector<LogRow> rows_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SupervisedClosedLoopNode>());
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
