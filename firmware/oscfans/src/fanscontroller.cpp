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
    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].update();
        delay(UPDATE_DELAY_MS);
    }
}

void FansController::reset()
{
    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].reset();
    }
}

void FansController::enable(bool run)
{
    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].set_enabled(run);
    }
}

void FansController::set_setpoint_percent(uint8_t address, float percent)
{
    uint16_t setpoint = min(max(percent, 0), 100) / 100.0 * 16383;
    // TODO: something better than this shit
    fans[address - 1].set_setpoint(setpoint);
}
