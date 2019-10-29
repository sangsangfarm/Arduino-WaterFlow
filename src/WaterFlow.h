/**
 * @file WaterFlow.h
 * @brief Measuring Water flow.
 * @date 2019-10-24
 * @author Janghun Lee (jhlee@sangsang.farm)
 */

#ifndef WATER_FLOW_H_
#define WATER_FLOW_H_

extern "C" {
#include <driver/rmt.h>
#include <esp32-hal-gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
}
#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 4096
#define ESP_INTR_FLAG_DEFAULT 0
/**
 * @struct WaterFlowBaseInfo
 * @brief Water flow sensor base info which will be saved in EEPROM.
 * @date 2019-10-24
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
typedef struct WaterFlowBaseInfo {
  /** Water flow calibration factor */
  double calibration_factor;
  /** The maximum amount of water  */
  long max_millliter;
  /** The maximum watering time */
  long max_seconds;
} WaterFlowBaseInfo;
/**
 * @struct WaterFlowInfo
 * @brief Water flow sensor info.
 * @date 2019-10-24
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
typedef struct WaterFlowInfo : WaterFlowBaseInfo {
  /** GPIO pin number */
  uint8_t pin;
  /** Water flow sensor pulse */
  long pulse;
  /** The amount of water flow  */
  long millliter;
} WaterFlowInfo;

class WaterFlow {
 private:
  /** The number of water flow sensors */
  size_t _water_flow_num;
  /** EEPROM address */
  int _eeprom_address = 0;

  static void flow(void* arg);
  static void IRAM_ATTR isr_handler(void* arg);

 public:
  WaterFlow(uint8_t pins[], size_t water_flow_num,
            int intr_alloc_flags = ESP_INTR_FLAG_DEFAULT);
  ~WaterFlow();
  void reset(int index);

  void setEEPROMAddress(int eeprom_address);
  void loadData(void);
  void saveData(void);

  void setCalibrationFactor(int index, double calibration_factor);
  double getCalibrationFactor(int index);

  void setMaxMillliter(int index, long max_millliter);
  long getMaxMillliter(int index);

  void setMaxSeconds(int index, long max_seconds);
  long getMaxSeconds(int index);

  long getMillliter(int index);
};

#endif /** WATER_FLOW_H_ */