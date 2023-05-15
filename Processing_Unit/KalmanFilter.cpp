#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(size_t samplingPeriod):
  samplingPeriod{samplingPeriod}
{
  stateMatrix = {0.99f, 0.f, 0.f, 0.f, static_cast<float>(samplingPeriod)/1000.f, 1.f, 0.f, 0.f, 0.f, static_cast<float>(samplingPeriod)/1000.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
}

void KalmanFilter::Estimate(float accelerationMeasurement, float distanceMeasurement)
{
  BLA::Matrix<2,1> outputMeasured {accelerationMeasurement, distanceMeasurement};
  outputPrediction = outputMatrix * statePrediction;
  outputError = outputMeasured - outputPrediction;
  stateEstimation = statePrediction + kalmanGain * outputError;
  statePrediction = stateMatrix * stateEstimation;
}

float KalmanFilter::GetEstimate() const
{
  return stateEstimation(1);
}