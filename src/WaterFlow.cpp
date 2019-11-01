/**
 * @file WaterFlow.cpp
 * @brief Measuring Water flow.
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */

#include <WaterFlow.h>

unsigned long getMillis(void);

/** Water flow info */
static WaterFlowInfo* _water_flows = NULL;
/** ISR queue */
static xQueueHandle _event_queue = NULL;

/**
 * @fn WaterFlow::WaterFlow(uint8_t pins[], size_t water_flow_num,
                     int intr_alloc_flags)
 * @brief WaterFlow Constructor
 * @param pins water flow sensors' GPIO
 * @param water_flow_num water flow sensor number
 * @param intr_alloc_flags Flags used to allocate the interrupt
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
WaterFlow::WaterFlow(uint8_t pins[], size_t water_flow_num,
                     int intr_alloc_flags) {
  _water_flow_num = water_flow_num;
  _water_flows =
      (WaterFlowInfo*)malloc(sizeof(WaterFlowInfo) * _water_flow_num);
  uint64_t pin_bit_mask = 0;
  for (int i = 0; i < _water_flow_num; i++) {
    // bit masking for setting GPIO pin;
    pin_bit_mask |= (1ULL << pins[i]);
    _water_flows[i].pin = pins[i];
    _water_flows[i].pulse = _water_flows[i].milliliter = 0;
    _water_flows[i].calibration_factor = 7.5;
    _water_flows[i].max_milliliter = 1000;
    _water_flows[i].max_milliseconds = 10;
    _water_flows[i].start_time = 0;
  }

  // Set these GPIO pins to input mode and when GPIO pin is Low to High, it
  // occur interrupt.
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.pin_bit_mask = pin_bit_mask;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Create Queue
  _event_queue = xQueueCreate(10 * water_flow_num, sizeof(uint32_t));
  xTaskCreate(flow, "flow", 2048, this, 10, NULL);
  gpio_install_isr_service(intr_alloc_flags);

  // Add isr handler
  // Pass index value(i) to know which GPIO pin is occured interrupt
  for (int i = 0; i < _water_flow_num; i++) {
    gpio_isr_handler_add((gpio_num_t)_water_flows[i].pin, isr_handler,
                         (void*)i);
  }
}

/**
 * @fn WaterFlow::~WaterFlow()
 * @brief WaterFlow Destructor
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
WaterFlow::~WaterFlow() { free(_water_flows); }
/**
 * @fn void WaterFlow::IRAM_ATTR isr_handler(void* arg)
 * @brief ISR handler
 * @param arg parameter that will be placed in queue;
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void IRAM_ATTR WaterFlow::isr_handler(void* arg) {
  int index = (int)arg;
  xQueueSendFromISR(_event_queue, &index, NULL);
}

/**
 * @fn void WaterFlow::setEEPROMAddress(int eeprom_address)
 * @brief Set EEPROM address
 * @param eeprom_address EEPROM address
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::setEEPROMAddress(int eeprom_address) {
  _eeprom_address = eeprom_address;
}

/**
 * @fn void WaterFlow::loadData(void)
 * @brief Load water flow base info data from EEPROM
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::loadData(void) {
  WaterFlowBaseInfo info;
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < _water_flow_num; i++) {
    EEPROM.get(_eeprom_address + i * sizeof(WaterFlowBaseInfo), info);
    _water_flows[i].calibration_factor = info.calibration_factor;
    _water_flows[i].max_milliliter = info.max_milliliter;
    _water_flows[i].max_milliseconds = info.max_milliseconds;
  }
  EEPROM.end();
}

/**
 * @fn void WaterFlow::saveData(void)
 * @brief Save water flow base info data from EEPROM
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::saveData(void) {
  WaterFlowBaseInfo info;

  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < _water_flow_num; i++) {
    info.calibration_factor = _water_flows[i].calibration_factor;
    info.max_milliliter = _water_flows[i].max_milliliter;
    info.max_milliseconds = _water_flows[i].max_milliseconds;

    EEPROM.put(_eeprom_address + i * sizeof(WaterFlowBaseInfo), info);
    EEPROM.commit();
  }
  EEPROM.end();
}
/**
 * @fn void WaterFlow::flow(void* arg)
 * @brief Get signal from water flow sensor
 * @param arg pointer that received from xTaskCreate
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::flow(void* arg) {
  int index;
  for (;;) {
    if (xQueueReceive(_event_queue, &index, portMAX_DELAY)) {
      if (_water_flows[index].start_time == 0) {
        _water_flows[index].start_time = getMillis();
      }
      _water_flows[index].pulse++;
      _water_flows[index].milliliter =
          (_water_flows[index].pulse * 1000 /
           (60 * _water_flows[index].calibration_factor));
    }
  }
}

/**
 * @fn void WaterFlow::reset(int index)
 * @brief Reset specific water flow sensor's pulse and milliliter
 * @param index water flow sensor index
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::reset(int index) {
  _water_flows[index].pulse = _water_flows[index].milliliter = 0;
  _water_flows[index].start_time = 0;
}

/**
 * @fn void WaterFlow::setCalibrationFactor(int index, double
                                            calibration_factor)
 * @brief Set specific water flow sensor's calibration factor
 * @param index water flow sensor index
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::setCalibrationFactor(int index, double calibration_factor) {
  _water_flows[index].calibration_factor = calibration_factor;
}

/**
 * @fn double WaterFlow::getCalibrationFactor(int index)
 * @brief Get specific water flow sensor's calibration factor
 * @param index water flow sensor index
 * @return water flow sensor's calibration factor
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
double WaterFlow::getCalibrationFactor(int index) {
  return _water_flows[index].calibration_factor;
}
/**
 * @fn void WaterFlow::setMaxMilliliter(int index, unsigned long max_milliliter)
 * @brief Set specific water flow sensor's max milliliter
 * @param index water flow sensor index
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::setMaxMilliliter(int index, unsigned long max_milliliter) {
  _water_flows[index].max_milliliter = max_milliliter;
}
/**
 * @fn unsigned long WaterFlow::getMaxMilliliter(int index)
 * @brief Get specific water flow sensor's max milliliter
 * @param index water flow sensor index
 * @return water flow sensor's max milliliter
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
unsigned long WaterFlow::getMaxMilliliter(int index) {
  return _water_flows[index].max_milliliter;
}
/**
 * @fn void WaterFlow::setMaxMillisecond(int index, unsigned long
 * max_milliseconds)
 * @brief Set specific water flow sensor's max seconds
 * @param index water flow sensor index
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::setMaxMillisecond(int index, unsigned long max_milliseconds) {
  _water_flows[index].max_milliseconds = max_milliseconds;
}

/**
 * @fn unsigned long WaterFlow::getMaxMillisecond(int index)
 * @brief Get specific water flow sensor's max seconds
 * @param index water flow sensor index
 * @return water flow sensor's max seconds
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
unsigned long WaterFlow::getMaxMillisecond(int index) {
  return _water_flows[index].max_milliseconds;
}

/**
 * @fn unsigned long WaterFlow::getMilliliter(int index)
 * @brief Get specific water flow sensor's milliliter
 * @param index water flow sensor index
 * @return water flow sensor's milliliter
 * @date 2019-10-25
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
unsigned long WaterFlow::getMilliliter(int index) {
  return _water_flows[index].milliliter;
}

/**
 * @fn bool WaterFlow::isTimeOut(int index)
 * @brief Check watering timeout in selected water flow sensor.
 * @param index water flow sensor index
 * @return Time-out status
 * @date 2019-10-29
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
bool WaterFlow::isTimeOut(int index) {
  if (_water_flows[index].start_time != 0 &&
      getMillis() - _water_flows[index].start_time >=
          _water_flows[index].max_milliseconds) {
    return true;
  } else {
    return false;
  }
}

/**
 * @fn void WaterFlow::setStartTime(int index)
 * @brief Set specific water flow sensor's start time
 * @param index water flow sensor index
 * @date 2019-10-30
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
void WaterFlow::setStartTime(int index) {
  _water_flows[index].start_time = getMillis();
}
/**
 * @fn unsigned long getMillis(void)
 * @brief Get milliseconds
 * @date 2019-10-30
 * @author Janghun Lee (jhlee@sangsang.farm)
 */
unsigned long getMillis(void) {
  struct timeval now;
  unsigned long millis;
  gettimeofday(&now, NULL);
  millis = now.tv_sec * 1000 + now.tv_usec / 1000;
  return millis;
}