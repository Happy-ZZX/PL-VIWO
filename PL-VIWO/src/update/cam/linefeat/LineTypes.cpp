#include "LineTypes.h"
#include "utils/Print_Logger.h"

void viw::LineLinSys::print() {
  PRINT0("Hf\n");
  print_matrix(Hf);
  PRINT0("Hx\n");
  print_matrix(Hx);
  PRINT0("R\n");
  print_matrix(R);
  PRINT0("res\n");
  print_matrix(res);
}

void viw::LineLinSys::print_matrix(Eigen::MatrixXd H) {
  // if Print level is not 0 and we do not save prints, just return. It is heavy
  if (viw::Print_Logger::current_print_level != 0 && viw::Print_Logger::pFile == NULL)
    return;

  for (int i = 0; i < H.rows(); i++) {
    for (int j = 0; j < H.cols(); j++) {
      PRINT0("%.2f ", H(i, j));
      assert(!std::isnan(H(i, j)));
    }
    printf("\n");
  }
}

void viw::LineLinSys::print_matrix(Eigen::VectorXd r) {
  // if Print level is not 0 and we do not save prints, just return. It is heavy
  if (viw::Print_Logger::current_print_level != 0 && viw::Print_Logger::pFile == NULL)
    return;

  for (int i = 0; i < r.rows(); i++) {
    PRINT0("%.2f ", r(i));
    assert(!std::isnan(r(i)));
  }
  printf("\n");
}