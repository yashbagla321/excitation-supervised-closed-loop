#include <filesystem>
#include <memory>
#include <string>
#include <chrono>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Output.hpp"
#include "adaptive_localization/Simulation.hpp"
#include "rclcpp/rclcpp.hpp"

class HiddenTargetBatchRunner final : public rclcpp::Node {
public:
    HiddenTargetBatchRunner()
        : Node("hidden_target_batch_runner") {
        output_dir_ = declare_parameter<std::string>("output_dir", "results/ros_gz_batch");
        expanded_trials_per_case_ = declare_parameter<int>("expanded_trials_per_case", 20);
        run_full_suite_ = declare_parameter<bool>("run_full_suite", false);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(300),
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
        config.expanded_trials_per_case = expanded_trials_per_case_;
        std::filesystem::create_directories(config.output_dir);

        const auto information_rows = adaptive::run_information_conditioning_sweep(config);
        adaptive::write_information_conditioning_csv(
            config.output_dir / "information_conditioning.csv", information_rows);

        const auto near_degenerate_rows = adaptive::run_near_degenerate_trajectory_sweep(config);
        adaptive::write_trajectory_sweep_csv(
            config.output_dir / "near_degenerate_trajectory_sweep.csv", near_degenerate_rows);

        const auto intermittent_rows = adaptive::run_intermittent_measurement_sweep(config);
        adaptive::write_intermittent_measurement_sweep_csv(
            config.output_dir / "intermittent_measurement_sweep.csv", intermittent_rows);

        const auto outlier_rows = adaptive::run_outlier_robustness_sweep(config);
        adaptive::write_outlier_robustness_sweep_csv(
            config.output_dir / "outlier_robustness_sweep.csv", outlier_rows);

        const auto vehicle_noise_rows = adaptive::run_vehicle_localization_noise_sweep(config);
        adaptive::write_vehicle_localization_noise_sweep_csv(
            config.output_dir / "vehicle_localization_noise_sweep.csv", vehicle_noise_rows);

        const auto poor_initialization_rows = adaptive::run_poor_initialization_sweep(config);
        adaptive::write_poor_initialization_sweep_csv(
            config.output_dir / "poor_initialization_sweep.csv", poor_initialization_rows);

        const auto baseline_rows = adaptive::run_expanded_baseline_comparison(config);
        adaptive::write_expanded_baseline_summary_csv(
            config.output_dir / "expanded_baseline_summary.csv", baseline_rows);

        if (run_full_suite_) {
            const auto trials = adaptive::run_monte_carlo(config);
            adaptive::write_trial_csv(config.output_dir / "monte_carlo_trials.csv", trials);
            std::vector<adaptive::SummaryRow> summaries;
            for (int scenario : config.monte_carlo_scenarios) {
                for (int beacon_count : config.monte_carlo_beacon_counts) {
                    summaries.push_back(adaptive::summarize(scenario, beacon_count, trials));
                }
            }
            adaptive::write_summary_csv(config.output_dir / "monte_carlo_summary.csv", summaries);
        }

        RCLCPP_INFO(
            get_logger(),
            "Wrote ROS/Gazebo batch validation CSVs to %s",
            config.output_dir.string().c_str());
    }

    bool has_run_ = false;
    std::string output_dir_;
    int expanded_trials_per_case_ = 20;
    bool run_full_suite_ = false;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HiddenTargetBatchRunner>());
    rclcpp::shutdown();
    return 0;
}
