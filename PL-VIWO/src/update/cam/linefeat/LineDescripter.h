#ifndef LINE_DESCRPTER_H
#define LINE_DESCRPTER_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

Class LineDescripter {
public:
  static void vector<int> computeLBD(const Mat &image, const Point2f &start, const Point2f &end, int numBands = 8, int samplesPerBand = 10);
  
private:
  static void computeGradient(const Mat &image, Mat &gradientX, Mat &gradientY); 
};
#endif