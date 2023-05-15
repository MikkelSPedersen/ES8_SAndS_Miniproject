#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <ArduinoBLE.h>

class Communication
{
public:
  void Begin();
  void Transmit(float speed);

private:
  float speed {0.f};
  BLEService processingService {"0000181A-0000-1000-8000-00805F9B34FB"};
  BLEFloatCharacteristic speedCharacteristic {"0000181A-0000-1000-8000-00805F9B34FB",  // standard 16-bit characteristic UUID
      BLERead | BLENotify}; // remote clients will be able to get notifications if this characteristic changes
};

#endif