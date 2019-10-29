# Arduino-WaterFlow

Water flow in Arduino.

## Support platform

- esp8226
- esp32

## Usage

```cpp
#include <WaterFlow.h>

void setup() {

  uint8_t pin[2] = {35, 34};
  Waterflow water_flow = Waterflow(pin, 2);
  water_flow.setCalibrationFactor(0, 4.5);
  water_flow.setCalibrationFactor(1, 7.5);
  water_flow.setMaxMillliter(1, 50);
  water_flow.setMaxMillliter(1, 100);
  water_flow.setMaxSeconds(1, 60);
  water_flow.getMaxSeconds(1, 30);

  // If you want to use EEPROM
  water_flow.setEEPROMAddress(10);
  water_flow.loadData();
  ...
  water_flow.saveDate();
}
void loop() {
  long liter_1 = water_flow.getMillliter(0) / 1000;
  long liter_2 = water_flow.getMillliter(0) / 1000;
}
```
