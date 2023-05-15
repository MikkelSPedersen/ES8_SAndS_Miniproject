#ifndef SAMPLING_H
#define SAMPLING_H

#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include <FreeRTOS_SAMD21.h>

class Sampling
{
public:
  Sampling(size_t samplingPeriod);
  void Begin();
  bool Sample();
  float GetAcceleration() const;
  float GetDistance() const;

private:
  const size_t samplingPeriod { 50 };
  float acceleration { 0.f };
  float distance { 0.f };
  float x { 0.f };
  float y { 0.f };
  float z { 0.f };
  SFEVL53L1X distanceSensor;
};

#endif