#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <BasicLinearAlgebra.h>

class KalmanFilter
{
public:
  KalmanFilter(size_t samplingPeriod);
  void Estimate(float accelerationMeasurement, float distanceMeasurement);
  float GetEstimate() const;

private:
  float estimate { 0.f };
  const size_t samplingPeriod { 50 };
  BLA::Matrix<4,2> kalmanGain {0.8974f, 2.1932f, -0.0022f, 1.2311f, -0.0003f, 0.3136f, 0.0994f, -2.1933f};
  BLA::Matrix<4,4> stateMatrix {0.99f, 0.f, 0.f, 0.f, static_cast<float>(samplingPeriod)/1000.f, 1.f, 0.f, 0.f, 0.f, static_cast<float>(samplingPeriod)/1000.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
  BLA::Matrix<2,4> outputMatrix {1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f};
  BLA::Matrix<4,1> statePrediction {0.f, 0.f, 0.f, 0.f};
  BLA::Matrix<4,1> stateEstimation {0.f, 0.f, 0.f, 0.f};
  BLA::Matrix<2,1> outputPrediction {0.f, 0.f};
  BLA::Matrix<2,1> outputError {0.f, 0.f};
};

#endif