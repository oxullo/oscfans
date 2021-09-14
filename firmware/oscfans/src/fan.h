/*
 * fan.h
 *
 *  Created on: 14 Sep 2021
 *      Author: xi
 */

#ifndef SRC_FAN_H_
#define SRC_FAN_H_

#include <ModbusMaster.h>

namespace {

typedef enum SV20Reg {
    S2R_WD_TIME = 0,
    S2R_FREQ_SETPOINT = 2,
    S2R_RUN_ENABLE = 3,
    S2R_FWD_REV = 4,
    S2R_START_CMD = 5,
    S2R_FAULT_ACK = 6,
    S2R_CURRENT_LIMIT = 9,
    S2R_ACCEL_TIME = 10,
    S2R_DECEL_TIME = 11,
    S2R_REF_FREQ = 15,
    S2R_TORQUE = 26,
    S2R_ACTUAL_POWER = 27,
    S2R_DCBUS_VOLTAGE = 29,
    S2R_ENABLED = 37,
    S2R_READY_TO_RUN = 38,
    S2R_FAULT = 53,
    S2R_WARNING = 58,
    S2R_VFD_VERSION = 60,
    S2R_VFD_MODEL = 61,
    S2R_STW = 99,
    S2R_HSW = 100,
    S2R_ZSW = 109,
    S2R_HIW = 110
} SV20Reg;

}

class Fan {
public:
    Fan();

    void begin(uint8_t addr);
    void set_enabled(bool run_);
    void set_setpoint(uint16_t setpoint_);
    bool update();
    bool reset();
    void dump_opregs();

private:
    bool read_reg(SV20Reg regaddr, uint16_t* value);
    bool write_reg(SV20Reg regaddr, uint16_t value);
    void dump_reg(SV20Reg regaddr, const char* identifier);

    ModbusMaster bus;
    uint8_t addr;
    bool run;
    uint16_t setpoint;
    uint32_t tsu_last_telegram;
};

#endif
