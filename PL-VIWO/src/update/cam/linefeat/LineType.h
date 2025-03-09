#ifndef VIW_LINETYPES_H
#define VIW_LINETYPES_H

#include <Eigen/Eigen>
#include <memory>
#include <unordered_map>
#include <vector>

namespace ov_type {
class Type;
}
namespace viw {

struct LineLinSys {
  Eigen::MatrixXd Hf;
  Eigen::MatrixXd Hx;
  Eigen::MatrixXd R;
  Eigen::VectorXd res;
  std::vector<std::shared_ptr<ov_type::Type>> Hx_order;
  std::unordered_map<std::shared_ptr<ov_type::Type>, size_t> Hx_mapping;

  void print();

  void print_matrix(Eigen::MatrixXd H);

  void print_matrix(Eigen::VectorXd r);
};
}// namespace viw

#endif // VIW_CAMTYPES_H