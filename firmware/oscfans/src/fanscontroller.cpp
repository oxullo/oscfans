/*
 * fanscontroller.cpp
 *
 *  Created on: 14 Sep 2021
 *      Author: xi
 */

#include <fanscontroller.h>

FansController::FansController()
{
}

void FansController::begin()
{
    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].begin(MODBUS_ADDR_MAP[i]);
        fans[i].dump_opregs();
    }
}

void FansController::update()
{
    static uint8_t current_fan = 0;
    static uint32_t ts_last_update = 0;

    if (millis() - ts_last_update > UPDATE_DELAY_MS) {
        fans[current_fan].update();
        ts_last_update = millis();
        current_fan = (current_fan + 1) % FANS_COUNT;
    }
}

void FansController::reset()
{
    delay(10);
    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].reset();
    }
}

void FansController::enable(bool run)
{
    delay(10);
    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].set_enabled(run);
    }
}

bool FansController::set_setpoint_percent(uint8_t address, float percent)
{
    uint16_t setpoint = min(max(percent, 0), 100) / 100.0 * 16383;

    if (address > HIGHEST_ADDR) {
        return false;
    }

    uint8_t index = MODBUS_ADDR_TO_INDEX[address];
    fans[index].set_setpoint(setpoint);

    return true;
}
