#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

#include "adaptive_localization/Types.hpp"

namespace cooperative_localization_gz {

inline void write_ros_trial_csv(
    const std::filesystem::path& path,
    const adaptive::TrialResult& trial) {
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path);
    out << "scenario,beacons,trial,target_x,target_y,estimate_x,estimate_y,error,"
           "beacon_position_rmse,beacon_yaw_rmse,cost,iterations,runtime_ms,converged\n";
    out << std::fixed << std::setprecision(8);
    out << trial.scenario << ',' << trial.beacons << ',' << trial.trial << ','
        << trial.truth.x << ',' << trial.truth.y << ','
        << trial.estimate.x << ',' << trial.estimate.y << ','
        << trial.error << ',' << trial.beacon_position_rmse << ','
        << trial.beacon_yaw_rmse << ',' << trial.cost << ','
        << trial.iterations << ',' << trial.runtime_ms << ','
        << (trial.converged ? 1 : 0) << '\n';
}

inline std::filesystem::path output_path_from_param(
    const std::string& output_dir,
    const std::string& file_name) {
    return std::filesystem::path(output_dir) / file_name;
}

}  // namespace cooperative_localization_gz
