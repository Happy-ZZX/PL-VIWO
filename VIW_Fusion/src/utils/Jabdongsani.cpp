#include "Jabdongsani.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace viw;

void STAT::add_stat(const float &val) {
  cnt++;
  // compute only if we have more than 1 samples
  if (cnt > 1)
    var = (cnt - 2) / (cnt - 1) * var + 1.0 / cnt * pow((val - mean), 2);
  mean = (val + (cnt - 1) * mean) / cnt;
}

void STAT::add_stat(const VectorXd &vec) {
  for (int i = 0; i < (int)vec.size(); i++)
    add_stat(vec[i]);
}

void STAT::add_stat(const MatrixXd &mat) {
  int min_dim = mat.rows() < mat.cols() ? mat.rows() : mat.cols();
  for (int i = 0; i < min_dim; i++)
    add_stat(mat(i, i));
}

float STAT::std() { return sqrt(var); }

void STAT::reset() {
  mean = 0;
  var = 0;
  cnt = 0;
}