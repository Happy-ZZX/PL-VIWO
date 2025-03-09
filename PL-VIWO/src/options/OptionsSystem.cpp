#include "OptionsSystem.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"
#include <ros/package.h>

void viw::OptionsSystem::load_print(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_system";
    parser->parse_external(f, "sys", "verbosity", verbosity);
    viw::Print_Logger::setPrintLevel(verbosity);
    ov_core::Printer::setPrintLevel(Print_Logger::get_ov_verbosity(verbosity));
    parser->parse_external(f, "sys", "save_timing", save_timing);
    parser->parse_external(f, "sys", "path_timing", path_timing);
    parser->parse_external(f, "sys", "save_state", save_state);
    parser->parse_external(f, "sys", "path_state", path_state);
    parser->parse_external(f, "sys", "save_trajectory", save_trajectory);
    parser->parse_external(f, "sys", "path_trajectory", path_trajectory);
    parser->parse_external(f, "sys", "save_prints", save_prints);
    parser->parse_external(f, "sys", "exp_id", exp_id);
    parser->parse_external(f, "sys", "path_bag", path_bag);
    parser->parse_external(f, "sys", "bag_start", bag_start);
    parser->parse_external(f, "sys", "bag_durr", bag_durr);
    parser->parse_external(f, "sys", "path_gt", path_gt, false);
    // Replace MINS_DIR if we have it
    path_timing.substr(0, 8) == "MINS_DIR" ? path_timing.replace(0, 8, ros::package::getPath("viw")) : std::string();
    path_state.substr(0, 8) == "MINS_DIR" ? path_state.replace(0, 8, ros::package::getPath("viw")) : std::string();
    path_trajectory.substr(0, 8) == "MINS_DIR" ? path_trajectory.replace(0, 8, ros::package::getPath("viw")) : std::string();
  }
  PRINT1(BOLDBLUE "Options - System\n" RESET);
  PRINT1("\t- save_timing: %s\n", save_timing ? "true" : "false");
  save_timing ? PRINT1("\t- path_timing: %s\n", path_timing.c_str()) : PRINT1("");
  PRINT1("\t- save_state: %s\n", save_state ? "true" : "false");
  save_state ? PRINT1("\t- path_state: %s\n", path_state.c_str()) : PRINT1("");
  PRINT1("\t- save_trajectory: %s\n", save_trajectory ? "true" : "false");
  save_trajectory ? PRINT1("\t- path_trajectory: %s\n", path_trajectory.c_str()) : PRINT1("");
  PRINT1("\t- save_prints: %s\n", save_prints ? "true" : "false");
  PRINT1("\t- exp_id: %d\n", exp_id);
  PRINT1("\t- path_bag: %s\n", path_bag.c_str());
  PRINT1("\t- bag_start: %.1f\n", bag_start);
  PRINT1("\t- bag_durr: %.1f\n", bag_durr);
  path_gt.empty() ? PRINT1("") : PRINT1("\t- path_gt: %s\n", path_gt.c_str());
}
