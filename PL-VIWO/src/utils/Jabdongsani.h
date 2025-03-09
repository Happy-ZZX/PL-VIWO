#ifndef VIW_PACKAGE_JABDONGSANI_H
#define VIW_PACKAGE_JABDONGSANI_H

#include "memory"
#include "vector"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;
namespace ov_type {
class Type;
}
namespace viw {

/// odds and ends. Mostly used to analyze matrix
class JDSN {
public:
  static void print(MatrixXd m);
  static void print(MatrixXd m, string name);
  static void print(string name, MatrixXd m);
  static void print(MatrixXd m, string name, int precision);
  static void print(string name, MatrixXd m, int precision);
  static void print(vector<int> vec, string name);
  static void print(string name, vector<int> vec);
  static void size(MatrixXd m, string name);
  static void size(string name, MatrixXd m);
  static bool symmetric(MatrixXd m, bool verbose = false);
};

struct Dummy {
  Matrix3d R;
  Vector3d p;
  vector<MatrixXd> VM;
  vector<shared_ptr<ov_type::Type>> VT;
};

/// computes the statistics of values (residual, chi, etc) incrementally.
struct STAT {
  /// Mean value
  float mean = 0;

  /// Variance of the value
  float var = 0;

  /// Counter
  float cnt = 0;

  /// Add object. Will be added scalar form
  void add_stat(const float &val);
  void add_stat(const VectorXd &vec);
  void add_stat(const MatrixXd &mat);

  /// Returns standard deviation
  float std();

  /// Reset
  void reset();
};

} // namespace viw

#endif // VIW_PACKAGE_JABDONGSANI_H