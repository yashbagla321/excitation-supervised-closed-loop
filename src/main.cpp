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

// Writes every artifact this program's run produces, in the same order the
// CDC closed-loop paper's tables/figures reference them:
//   - `supervised_excitation_comparison.csv`: rows from
//     run_supervised_excitation_comparison(), i.e. the fixed decaying-circular
//     schedule vs. the excitation-supervised controller (Algorithm 1), for
//     both the nominal and understimulated scenarios.
//   - `supervised_lambda_sweep.csv`: rows from run_supervised_lambda_sweep(),
//     sweeping the fixed schedule's decay rate lambda and recording how each
//     side of the comparison fares at every value.
//   - `closed_loop_local_1beacon.csv` / `beacon_estimates_local_1beacon.csv`
//     plus three SVGs: the single flagship excitation-supervised, one-beacon,
//     local-frame closed-loop run (`local_single_beacon`) that the paper's
//     trajectory and convergence figures are drawn from.
//
// Parameters:
//   output_dir: directory (already created by the caller) to write into.
//   supervised_excitation_rows: fixed-vs-supervised comparison rows for the
//     nominal and understimulated scenarios.
//   supervised_lambda_rows: fixed-vs-supervised comparison rows, one per
//     swept decay-rate lambda.
//   local_single_beacon: the flagship supervised, one-beacon closed-loop run
//     whose trajectory/error/beacon-estimate history is exported to CSV/SVG.
// Returns: nothing; all outputs are written to disk as a side effect.
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

// Prints a short human-readable summary of the run to stdout: which config
// file was loaded, where outputs landed, and the path of every file
// write_all_outputs() just wrote. Purely informational (no return value);
// lets a reader confirm a run succeeded without opening the output
// directory, and gives CI logs a stable, greppable list of produced files.
//   config: the loaded run configuration (used for its output_dir).
//   config_path: the config file path that was passed to load_config(),
//     echoed back so the printed summary is self-describing.
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

// Drives the three studies this CDC-paper entry point exists to produce, then
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
//      controller that can retrigger — via the trajectory-spread margin and
//      observability-conditioning check described in Simulation.cpp — keeps
//      the geometry identifiable). This is the head-to-head fixed-circular
//      vs. supervised comparison the paper's results table is built from.
//   3. Decay-rate lambda sweep (run_supervised_lambda_sweep): repeats the
//      understimulated-style, no-transient comparison across a range of
//      fixed-schedule decay rates lambda, to show the supervised controller
//      is robust to not knowing the "right" lambda in advance, rather than
//      only beating one hand-picked worst case.
//   4. Flagship single-beacon run: seeds a dedicated RNG from
//      config.closed_loop_seed and runs one local-frame, one-beacon
//      closed-loop trial explicitly in
//      adaptive::ClosedLoopExcitationMode::Supervised. This is the run whose
//      trajectory and convergence curves the paper's Algorithm 1 figures and
//      the Gazebo replay video (see scripts/render_gazebo_panel.py and
//      scripts/render_gazebo_validation_video.py) are drawn from, so it must
//      use the supervised controller rather than the fixed baseline.
//   5. Output: hand all three studies' results to write_all_outputs() and
//      print_run_summary().
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
