#include "Communication.h"

void Communication::Begin()
{
  if (!BLE.begin()) {
    while(1);
  }

  BLE.setLocalName("ProcessingUnit");
  BLE.setAdvertisedService(processingService);
  processingService.addCharacteristic(speedCharacteristic);
  BLE.addService(processingService);
  speedCharacteristic.writeValue(speed);

  BLE.advertise();
}

void Communication::Transmit(float speed)
{
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central)
  {
    speedCharacteristic.writeValue(speed);
    Serial.println(speed);
  }
}