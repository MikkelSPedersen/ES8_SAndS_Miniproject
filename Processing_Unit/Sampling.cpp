#include "Sampling.h"

Sampling::Sampling(size_t samplingPeriod):
  samplingPeriod{ samplingPeriod }
{

}

void Sampling::Begin()
{
  Wire.begin();

  if (!IMU.begin()) 
  {
    while (1);
  }
  if (distanceSensor.begin() != 0)
  {
    while (1);
  }

  // Changes distance sensor to sample at set period
  distanceSensor.setDistanceModeShort();
  distanceSensor.setIntermeasurementPeriod(samplingPeriod);
  distanceSensor.startRanging();
}

bool Sampling::Sample()
{
  if (IMU.accelerationAvailable()) 
  {
    IMU.readAcceleration(x, y, z);
    acceleration = x * 9.82f;
  }

  while (!distanceSensor.checkForDataReady())
  {
    vTaskDelay(1);
  }
  distance = static_cast<float>(distanceSensor.getDistance()) / 1000.f; //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
}

float Sampling::GetAcceleration() const
{
  return acceleration;
}

float Sampling::GetDistance() const
{
  return distance;
}