#include "LineDescripter.h"

// Function to compute the gradient of the image
void computeGradient(const Mat &image, Mat &gradientX, Mat &gradientY) {
    Sobel(image, gradientX, CV_32F, 1, 0, 3);
    Sobel(image, gradientY, CV_32F, 0, 1, 3);
}

// Function to compute the LBD descriptor for a single line
vector<int> computeLBD(const Mat &image, const Point2f &start, const Point2f &end, int numBands = 8, int samplesPerBand = 10) {
    // Calculate unit vector along the line
    Point2f lineVector = end - start;
    float lineLength = norm(lineVector);
    Point2f unitLineVector = lineVector / lineLength;

    // Calculate perpendicular vector
    Point2f perpVector(-unitLineVector.y, unitLineVector.x);

    // Compute gradient
    Mat gradientX, gradientY;
    computeGradient(image, gradientX, gradientY);

    // LBD descriptor
    vector<int> descriptor;

    // Iterate through bands
    for (int band = 0; band < numBands; band++) {
        float bandStart = band * lineLength / numBands;
        float bandEnd = (band + 1) * lineLength / numBands;

        int positiveGradient = 0;
        int negativeGradient = 0;

        // Sample points within the band
        for (int sample = 0; sample < samplesPerBand; sample++) {
            float alpha = static_cast<float>(sample) / samplesPerBand;
            Point2f samplePoint = start + alpha * (bandEnd - bandStart) * unitLineVector;

            // Extend sampling to both sides of the line
            for (int side = -1; side <= 1; side += 2) {
                Point2f sidePoint = samplePoint + side * 3 * perpVector;

                // Get gradient at the sampling point
                int x = cvRound(sidePoint.x);
                int y = cvRound(sidePoint.y);

                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    float gx = gradientX.at<float>(y, x);
                    float gy = gradientY.at<float>(y, x);
                    float magnitude = std::sqrt(gx * gx + gy * gy);
                    float direction = std::atan2(gy, gx);

                    // Simplify the descriptor to binary
                    if (magnitude > 0) {
                        if (direction > 0) {
                            positiveGradient++;
                        } else {
                            negativeGradient++;
                        }
                    }
                }
            }
        }

        // Add band descriptor to the result
        descriptor.push_back(positiveGradient > negativeGradient ? 1 : 0);
    }

    return descriptor;
}