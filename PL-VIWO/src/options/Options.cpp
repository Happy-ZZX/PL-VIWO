#include "Options.h"
#include "OptionsEstimator.h"
#include "OptionsGPS.h"
#include "OptionsInit.h"
#include "OptionsSimulation.h"
#include "OptionsSystem.h"
#include "OptionsWheel.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

viw::Options::Options() {
  sys = std::make_shared<OptionsSystem>();
  sim = std::make_shared<OptionsSimulation>();
  est = std::make_shared<OptionsEstimator>();
};

void viw::Options::load_print(const std::shared_ptr<ov_core::YamlParser> &parser) {
  // Load parameters
  sys->load_print(parser);
  est->load_print(parser);
  sim->load_print(parser);

  // force nonholonomic constraint to simulated trajectory when using wheel measurements
  if (est->wheel->enabled && sim->const_holonomic && boost::filesystem::exists(parser->get_config_folder() + "config_simulation.yaml")) {
    PRINT3(YELLOW "Enable non-holonomic constraint for wheel simulation.\n" RESET);
    sim->const_holonomic = false;
  }

  // Match the ground truth file path if we are initializing from ground truth
  if (est->init->use_gt)
    est->init->path_gt = sys->path_gt;
}