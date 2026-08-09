#include <filesystem>
#include <iostream>
#include <exception>
#include <random>
#include <vector>

#include "adaptive_localization/Config.hpp"
#include "adaptive_localization/Output.hpp"
#include "adaptive_localization/Simulation.hpp"

// This entry point is trimmed to the closed-loop excitation-supervision
// study cited by the paper "Excitation-Supervised Closed-Loop
// Self-Calibration and Target Seeking for an Unknown-Pose Range-Bearing
// Relay". The estimator core (include/, src/*.cpp other than this file) is
// shared verbatim with the companion trajectory-induced-self-calibration
// repository, which runs the open-loop identifiability/robustness sweeps
// this file does not exercise.

namespace {

// Writes every artifact this program's run produces, in the same order the
// closed-loop paper's tables/figures reference them:
//   - `supervised_excitation_comparison.csv`: rows from
//     run_supervised_excitation_comparison(), i.e. the fixed decaying-circular
//     schedule vs. the excitation-supervised controller (Algorithm 1), for
//     both the nominal and understimulated scenarios.
//   - `supervised_lambda_sweep.csv`: rows from run_supervised_lambda_sweep(),
//     sweeping the fixed schedule's decay rate lambda over a paired Monte
//     Carlo batch and recording how each side of the comparison fares at
//     every value.
//   - `supervised_seeking_comparison.csv`: rows from
//     run_supervised_seeking_comparison(), the nontrivial target-seeking
//     Monte Carlo comparison where only the excitation policy can supply
//     the spread needed to calibrate.
//   - `supervised_threshold_ablation.csv`: rows from
//     run_supervised_threshold_ablation(), the Monte Carlo ablation over
//     the supervisor's spread threshold S_bar testing the accuracy-driven
//     design rule against measured yaw RMSE and excitation cost.
//   - `closed_loop_local_1beacon.csv` / `beacon_estimates_local_1beacon.csv`
//     plus three SVGs: the single flagship excitation-supervised, one-beacon,
//     local-frame closed-loop run (`local_single_beacon`) that the paper's
//     trajectory and convergence figures are drawn from.
//
// Parameters:
//   output_dir: directory (already created by the caller) to write into.
//   supervised_excitation_rows: fixed-vs-supervised comparison rows for the
//     nominal and understimulated scenarios.
//   supervised_lambda_rows: fixed-vs-supervised Monte Carlo comparison rows,
//     one per swept decay-rate lambda.
//   supervised_seeking_rows: target-seeking Monte Carlo comparison rows, one
//     per excitation policy.
//   supervised_threshold_rows: spread-threshold ablation rows, one per
//     candidate S_bar.
//   local_single_beacon: the flagship supervised, one-beacon closed-loop run
//     whose trajectory/error/beacon-estimate history is exported to CSV/SVG.
// Returns: nothing; all outputs are written to disk as a side effect.
void write_all_outputs(
    const std::filesystem::path& output_dir,
    const std::vector<adaptive::SupervisedExcitationComparisonRow>& supervised_excitation_rows,
    const std::vector<adaptive::SupervisedLambdaSweepRow>& supervised_lambda_rows,
    const std::vector<adaptive::SupervisedSeekingComparisonRow>& supervised_seeking_rows,
    const std::vector<adaptive::SupervisedThresholdAblationRow>& supervised_threshold_rows,
    const adaptive::ClosedLoopResult& local_single_beacon) {
    adaptive::write_supervised_excitation_comparison_csv(
        output_dir / "supervised_excitation_comparison.csv", supervised_excitation_rows);
    adaptive::write_supervised_lambda_sweep_csv(
        output_dir / "supervised_lambda_sweep.csv", supervised_lambda_rows);
    adaptive::write_supervised_seeking_comparison_csv(
        output_dir / "supervised_seeking_comparison.csv", supervised_seeking_rows);
    adaptive::write_supervised_threshold_ablation_csv(
        output_dir / "supervised_threshold_ablation.csv", supervised_threshold_rows);
    adaptive::write_closed_loop_csv(output_dir / "closed_loop_local_1beacon.csv", local_single_beacon);
    adaptive::write_beacon_estimate_csv(output_dir / "beacon_estimates_local_1beacon.csv", local_single_beacon);
    adaptive::write_svg_plot(output_dir / "closed_loop_local_1beacon.svg", local_single_beacon);
    adaptive::write_error_curve_svg(output_dir / "closed_loop_errors_local_1beacon.svg", local_single_beacon);
    adaptive::write_beacon_error_svg(output_dir / "beacon_errors_local_1beacon.svg", local_single_beacon);
}

// Prints a short human-readable summary of the run to stdout: which config
// file was loaded, where outputs landed, and the path of every file
// write_all_outputs() just wrote. Purely informational (no return value);
// lets a reader confirm a run succeeded without opening the output
// directory, and gives CI logs a stable, greppable list of produced files.
//   config: the loaded run configuration (used for its output_dir).
//   config_path: the config file path that was passed to load_config(),
//     echoed back so the printed summary is self-describing.
void print_run_summary(const adaptive::SimulationConfig& config, const std::filesystem::path& config_path) {
    std::cout << "Closed-loop simulation complete.\n";
    std::cout << "Config: " << config_path.string() << "\n";
    std::cout << "Output directory: " << config.output_dir.string() << "\n\n";
    std::cout << "Wrote " << (config.output_dir / "supervised_excitation_comparison.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "supervised_lambda_sweep.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "supervised_seeking_comparison.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "supervised_threshold_ablation.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "closed_loop_local_1beacon.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "beacon_estimates_local_1beacon.csv").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "closed_loop_local_1beacon.svg").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "closed_loop_errors_local_1beacon.svg").string() << "\n";
    std::cout << "Wrote " << (config.output_dir / "beacon_errors_local_1beacon.svg").string() << "\n";
}

}  // namespace

// Drives the five studies this entry point exists to produce, then
// writes their outputs. Split out from main() so exceptions can be caught in
// one place (see main()) while this function's control flow stays linear and
// easy to read top-to-bottom.
//
// Parameters:
//   argc, argv: standard C main() argument count/vector. argv[1], if given,
//     overrides the default config file path.
// Returns: 0 on success. Exceptions propagate to the caller (main()), which
//   turns them into a non-zero exit code.
//
// Sections, in the order they run:
//   1. Config bootstrap: resolve the config path (defaulting to
//      "config/simulation.ini" when no CLI argument is given), write a fully
//      commented default config file if none exists yet (first-run
//      convenience so the binary is runnable with no setup), then load it
//      and ensure the output directory exists.
//   2. Fixed-vs-supervised excitation comparison
//      (run_supervised_excitation_comparison): runs the fixed decaying-swirl
//      excitation schedule ("decaying_circular") back-to-back against the
//      excitation-supervised controller of Algorithm 1
//      ("excitation_supervised") in two scenarios — nominal (far initial
//      pose, moderate decay, the same defaults used throughout the rest of
//      the paper) and understimulated (robot starts exactly at the true
//      target and the fixed schedule decays fast, so only an excitation
//      controller that can retrigger — on the trajectory-spread certificate
//      S_v described in Simulation.cpp — keeps the geometry identifiable).
//      This is the head-to-head fixed-circular vs. supervised comparison the
//      paper's results table is built from.
//   3. Decay-rate lambda sweep (run_supervised_lambda_sweep): repeats the
//      understimulated-style, no-transient comparison across a range of
//      fixed-schedule decay rates lambda over a paired Monte Carlo batch, to
//      show the supervised controller is robust to not knowing the "right"
//      lambda in advance, rather than only beating one hand-picked worst
//      case.
//   4. Target-seeking comparison (run_supervised_seeking_comparison): the
//      nontrivial Monte Carlo scenario where the vehicle starts at its own
//      (wrong) initial target estimate, so the seeking control term is
//      initially quiescent and target-seeking success depends entirely on
//      the excitation policy supplying the calibrating spread.
//   5. Spread-threshold ablation (run_supervised_threshold_ablation): Monte
//      Carlo ablation over the supervisor's threshold S_bar, testing the
//      accuracy-driven design rule eps_psi = sigma / sqrt(S_bar) against
//      the measured yaw RMSE and the excitation cost of each threshold.
//   6. Flagship single-beacon run: seeds a dedicated RNG from
//      config.closed_loop_seed and runs one local-frame, one-beacon
//      closed-loop trial explicitly in
//      adaptive::ClosedLoopExcitationMode::Supervised. This is the run whose
//      trajectory and convergence curves the paper's Algorithm 1 figures and
//      the Gazebo replay video (see scripts/render_gazebo_panel.py and
//      scripts/render_gazebo_validation_video.py) are drawn from, so it must
//      use the supervised controller rather than the fixed baseline.
//   7. Output: hand all five studies' results to write_all_outputs() and
//      print_run_summary().
int run_main(int argc, char** argv) {
    const std::filesystem::path config_path = argc > 1 ? argv[1] : "config/simulation.ini";
    adaptive::write_default_config_if_missing(config_path);
    const auto config = adaptive::load_config(config_path);
    std::filesystem::create_directories(config.output_dir);

    const auto supervised_excitation_rows = adaptive::run_supervised_excitation_comparison(config);
    const auto supervised_lambda_rows = adaptive::run_supervised_lambda_sweep(config);
    const auto supervised_seeking_rows = adaptive::run_supervised_seeking_comparison(config);
    const auto supervised_threshold_rows = adaptive::run_supervised_threshold_ablation(config);

    std::mt19937 closed_loop_rng(config.closed_loop_seed);
    // The single-beacon flagship run is this paper's Algorithm 1 artifact,
    // so it must actually run the excitation-supervised mode.
    const auto closed_loop_s1_single = adaptive::run_closed_loop_comparison(
        1, 1, config, closed_loop_rng, adaptive::ClosedLoopExcitationMode::Supervised);

    write_all_outputs(
        config.output_dir,
        supervised_excitation_rows,
        supervised_lambda_rows,
        supervised_seeking_rows,
        supervised_threshold_rows,
        closed_loop_s1_single);
    // Deterministic per-step traces behind the papers' policy-comparison
    // figure (see run_closed_loop_showcase in Simulation.hpp).
    for (const auto& run : adaptive::run_closed_loop_showcase(config)) {
        adaptive::write_closed_loop_csv(
            config.output_dir / ("closed_loop_showcase_" + run.name + ".csv"), run.result);
    }
    print_run_summary(config, config_path);
    return 0;
}

// Program entry point. Delegates all real work to run_main() and converts
// any exception it throws (e.g. a malformed config value, or an I/O failure
// writing outputs) into a one-line message on stderr and exit code 1,
// instead of letting the exception propagate out of main() and terminate
// the process via std::terminate() with a less useful message.
//   argc, argv: forwarded unchanged to run_main().
// Returns: run_main()'s return code (0 on success), or 1 if an exception
//   was caught.
int main(int argc, char** argv) {
    try {
        return run_main(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
