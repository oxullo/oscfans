/*
 * fanscontroller.h
 *
 *  Created on: 14 Sep 2021
 *      Author: xi
 */

#ifndef SRC_FANSCONTROLLER_H_
#define SRC_FANSCONTROLLER_H_

#include "fan.h"

namespace {
const uint8_t FANS_COUNT = 2;
const uint8_t MODBUS_ADDR_MAP[] = {1, 2};
const uint8_t HIGHEST_ADDR = 2;
const uint8_t MODBUS_ADDR_TO_INDEX[] = {255, 0, 1};
const uint32_t UPDATE_DELAY_MS = 10;
}

class FansController {
public:
    FansController();

    void begin();
    void update();
    void reset();
    void enable(bool run);
    bool set_setpoint_percent(uint8_t address, float percent);

private:
    Fan fans[FANS_COUNT];
};

#endif /* SRC_FANSCONTROLLER_H_ */
