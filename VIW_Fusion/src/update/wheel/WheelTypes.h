#ifndef VIW_WHEELTYPES_H
#define VIW_WHEELTYPES_H

namespace viw {

/// Types of wheel measurements supported
/// Wheel2DAng: left/right wheel angular velocities. Additionally supports intrinsic (left/right wheel radii & base length) calibration
/// Wheel2DLin: left/right wheel linear velocities.
/// Wheel2DCen: angular/linear wheel velocities of the wheel odometry frame.
/// Wheel3DAng: left/right wheel angular velocities + planar motion constraint. Additionally supports intrinsic calibration
/// Wheel3DLin: left/right wheel linear velocities + planar motion constraint.
/// Wheel3DCen: angular/linear wheel velocities of the wheel odometry frame + planar motion constraint.

struct WheelData {

  /// Timestamp of the reading
  double time = -1;

  /// Sensor reading 1 (left wheel reading or angular velocity)
  double m1 = 0;

  /// Sensor reading 2 (right wheel reading or linear velocity)
  double m2 = 0;

  /// Sort function to allow for using of STL containers
  bool operator<(const WheelData &other) const { return time < other.time; }
};
} // namespace viw

#endif // VIW_WHEELTYPES_H