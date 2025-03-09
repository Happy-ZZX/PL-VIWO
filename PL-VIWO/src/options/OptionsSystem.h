#ifndef VIW_OPTIONSSYSTEM_H
#define VIW_OPTIONSSYSTEM_H

#include <memory>
#include <string>
namespace ov_core {
class YamlParser;
}

namespace viw {

struct OptionsSystem {

  void load_print(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  /// Bag info
  double bag_start = 0;
  double bag_durr = -1;
  std::string path_bag;

  /// ground truth file path
  std::string path_gt;

  /// If we should record the timing performance to file
  bool save_timing = false;

  /// If we should record full state estimation to file
  bool save_state = false;

  /// If we should record the trajectory (pose) estimation to file
  bool save_trajectory = false;

  /// Log outputs
  bool save_prints = false;

  /// The path to the file we will record the state information into
  std::string path_state;

  /// The path to the file we will record the timing into
  std::string path_timing;

  /// The path to the file we will record the trajectory into
  std::string path_trajectory;

  /// Experiment number
  int exp_id = 0;

  /// Message verbosity
  int verbosity = 2;
};
} // namespace viw

#endif // VIW_OPTIONSSYSTEM_H
