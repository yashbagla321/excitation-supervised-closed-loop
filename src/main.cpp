#include <filesystem>
#include <iostream>
#include <exception>
#include <random>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Output.hpp"
#include "adaptive_localization/Simulation.hpp"

// This entry point is trimmed to the closed-loop excitation-supervision
// study cited by the CDC paper "Excitation-Supervised Closed-Loop
// Self-Calibration and Target Seeking for an Unknown-Pose Range-Bearing
// Relay". The estimator core (include/, src/*.cpp other than this file) is
// shared verbatim with the companion ACC repository, which runs the
// open-loop identifiability/robustness sweeps this file does not exercise.

namespace {

void write_all_outputs(
    const std::filesystem::path& output_dir,
    const std::vector<adaptive::SupervisedExcitationComparisonRow>& supervised_excitation_rows,
    const std::vector<adaptive::SupervisedLambdaSweepRow>& supervised_lambda_rows,
    const adaptive::ClosedLoopResult& local_single_beacon) {
    adaptive::write_supervised_excitation_comparison_csv(
        output_dir / "supervised_excitation_comparison.csv", supervised_excitation_rows);
    adaptive::write_supervised_lambda_sweep_csv(
        output_dir / "supervised_lambda_sweep.csv", supervised_lambda_rows);
    adaptive::write_closed_loop_csv(output_dir / "closed_loop_local_1beacon.csv", local_single_beacon);
    adaptive::write_beacon_estimate_csv(output_dir / "beacon_estimates_local_1beacon.csv", local_single_beacon);
    adaptive::write_svg_plot(output_dir / "closed_loop_local_1beacon.svg", local_single_beacon);
    adaptive::write_error_curve_svg(output_dir / "closed_loop_errors_local_1beacon.svg", local_single_beacon);
    adaptive::write_beacon_error_svg(output_dir / "beacon_errors_local_1beacon.svg", local_single_beacon);
}

void print_run_summary(const adaptive::SimulationConfig& config, const std::filesystem::path& config_path) {
    std::cout << "CDC closed-loop simulation complete.\n";
    std::cout << "Config: " << config_path.string() << "\n";
    std::cout << "Output directory: " << config.output_dir.string() << "\n\n";
    std::cout << "Wrote " << (config.output_dir / "supervised_excitation_comparison.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "supervised_lambda_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "closed_loop_local_1beacon.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "beacon_estimates_local_1beacon.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "closed_loop_local_1beacon.svg").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "closed_loop_errors_local_1beacon.svg").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "beacon_errors_local_1beacon.svg").string() << "\n";
}

}  // namespace

int run_main(int argc, char** argv) {
    const std::filesystem::path config_path = argc > 1 ? argv[1] : "config/simulation.ini";
    adaptive::write_default_config_if_missing(config_path);
    const auto config = adaptive::load_config(config_path);
    std::filesystem::create_directories(config.output_dir);

    const auto supervised_excitation_rows = adaptive::run_supervised_excitation_comparison(config);
    const auto supervised_lambda_rows = adaptive::run_supervised_lambda_sweep(config);

    std::mt19937 closed_loop_rng(config.closed_loop_seed);
    // The single-beacon flagship run is this paper's Algorithm 1 artifact,
    // so it must actually run the excitation-supervised mode.
    const auto closed_loop_s1_single = adaptive::run_closed_loop_comparison(
        1, 1, config, closed_loop_rng, adaptive::ClosedLoopExcitationMode::Supervised);

    write_all_outputs(config.output_dir, supervised_excitation_rows, supervised_lambda_rows, closed_loop_s1_single);
    print_run_summary(config, config_path);
    return 0;
}

int main(int argc, char** argv) {
    try {
        return run_main(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
