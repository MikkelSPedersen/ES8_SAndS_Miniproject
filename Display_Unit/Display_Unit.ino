#include <ArduinoBLE.h>
#include <Nextion.h>

constexpr char* Service{ "0000181A-0000-1000-8000-00805F9B34FB" };
constexpr char* Characteristic{ "0000181A-0000-1000-8000-00805F9B34FB" };
// Nextion variable ("PageID", "ObjID", "ObjName")
NexNumber NumberSpeed {0, 4, "SpeedNum"};
NexNumber NumberSpeedGauge {0, 1, "SpeedGauge"};
NexText ConnectionStatusText {0, 3, "Status"};
NexText unit {0, 5, "t1"};

void setup() 
{
  Serial.begin(115200);
  bool DisplayReady = nexInit();
  if (!DisplayReady) 
  {
    while (true);
  }
  ConnectionStatusText.setText("Connecting...");
  BLE.begin();
  BLE.scanForUuid(Service);
}

void loop() 
{
  BLEDevice peripheral = BLE.available();
  if (peripheral) 
  {
    if (peripheral.localName() != "ProcessingUnit") 
    {
      return;
    }
    BLE.stopScan();
    BLECharacteristic characteristic;
    if (ConnectToDevice(peripheral, characteristic)) 
    {
      ConnectionStatusText.setText("Connected");
      while (peripheral.connected()) 
      {
        if (characteristic.valueUpdated()) 
        {
          char buffer[6];
          Serial.println(unit.getText(buffer, 6));
          float value = 0;
          characteristic.readValue(&value, 4);
          NumberSpeed.setValue(ConvertToDisplayValue(value));
          NumberSpeedGauge.setValue(CalculateGaugeValue(value));
        }
      }
      ConnectionStatusText.setText("Connecting...");
    }
    // peripheral disconnected, start scanning again
    BLE.scanForUuid(Service);
  }
}

bool ConnectToDevice(BLEDevice peripheral, BLECharacteristic& characteristic) 
{
  if (!peripheral.connect()) 
  {
    return false;
  }
  if (!peripheral.discoverAttributes()) 
  {
    peripheral.disconnect();
    return false;
  }
  characteristic = peripheral.characteristic(Characteristic);
  characteristic.subscribe();
  if (!characteristic) 
  {
    peripheral.disconnect();
    return false;
  }
  return true;
}

uint32_t ConvertToDisplayValue(float value)
{
  float ConvertionFactorToDisplay { 100.f };
  float floatVal = abs(value) * ConvertionFactorToDisplay;
  return static_cast<uint32_t>(floatVal);
}

uint32_t CalculateGaugeValue(float Value) 
{
  uint32_t ReturnValue{ 0 };
  uint32_t GaugeOffset{ 318 };
  float TotalGaugeRange{ 264.f };
  float MaxAssumedVelocity{ 3.f };

  float MappedValue = abs(Value) * (TotalGaugeRange / MaxAssumedVelocity);
  
  ReturnValue = ((GaugeOffset + static_cast<uint32_t>(MappedValue)) % 360);
  return ReturnValue;
}