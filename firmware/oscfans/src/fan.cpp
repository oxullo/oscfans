/*
 * fan.cpp
 *
 *  Created on: 14 Sep 2021
 *      Author: xi
 */

#include <Arduino.h>
#include <AIO.h>

#include "fan.h"

#define DEBUG


namespace {

const uint32_t TELEGRAM_SEND_DELAY_US = 10000;

void pre_tx()
{
    digitalWrite(AIO::RS485_RX_EN_PIN, HIGH);
    digitalWrite(AIO::RS485_TX_EN_PIN, HIGH);
    delay(2);
}

void post_tx()
{
    digitalWrite(AIO::RS485_RX_EN_PIN, LOW);
    digitalWrite(AIO::RS485_TX_EN_PIN, LOW);
    delay(2);
}

}


Fan::Fan() :
        addr(0),
        run(false),
        setpoint(0),
        tsu_last_telegram(0)
{
}

void Fan::begin(uint8_t addr_)
{
    addr = addr_;
    bus.begin(addr_, Serial1);
    bus.preTransmission(pre_tx);
    bus.postTransmission(post_tx);
}

void Fan::set_enabled(bool run_)
{
    run = run_;
}

void Fan::set_setpoint(uint16_t setpoint_)
{
    setpoint = min(setpoint_, 16383);
}

bool Fan::update()
{
//    if (micros() - tsu_last_telegram > TELEGRAM_SEND_DELAY_US) {
//        Serial.print(".");
        // 0x047E: disable offs, enable operations
        // 0x047F: as above, plus run bit
        bus.setTransmitBuffer(0, run ? 0x047F : 0x047E);
        bus.setTransmitBuffer(1, setpoint);
        uint8_t rc = bus.writeMultipleRegisters(S2R_STW, 2);

        tsu_last_telegram = micros();

#ifdef DEBUG
        if (rc != bus.ku8MBSuccess) {
            Serial.print("E01 addr=");
            Serial.println(addr);
        }
#endif
        return rc == bus.ku8MBSuccess;
//    } else {
//        return true;
//    }
}

bool Fan::reset()
{
    bool rc = true;

    rc &= write_reg(S2R_FAULT_ACK, 0);
    rc &= write_reg(S2R_FAULT_ACK, 1);
    rc &= write_reg(S2R_FAULT_ACK, 0);

    rc &= write_reg(S2R_WD_TIME, 0);

    return rc;
}

void Fan::dump_opregs()
{
    Serial.print("** REGS unit addr=");
    Serial.println(addr);
    dump_reg(S2R_WD_TIME, "S2R_WD_TIME");
    dump_reg(S2R_FREQ_SETPOINT, "S2R_FREQ_SETPOINT");
    dump_reg(S2R_RUN_ENABLE, "S2R_RUN_ENABLE");
    dump_reg(S2R_FWD_REV, "S2R_FWD_REV");
    dump_reg(S2R_START_CMD, "S2R_START_CMD");
    dump_reg(S2R_FAULT_ACK, "S2R_FAULT_ACK");
    dump_reg(S2R_CURRENT_LIMIT, "S2R_CURRENT_LIMIT");
    dump_reg(S2R_ACCEL_TIME, "S2R_ACCEL_TIME");
    dump_reg(S2R_DECEL_TIME, "S2R_DECEL_TIME");
    dump_reg(S2R_REF_FREQ, "S2R_REF_FREQ");
    dump_reg(S2R_TORQUE, "S2R_TORQUE");
    dump_reg(S2R_ACTUAL_POWER, "S2R_ACTUAL_POWER");
    dump_reg(S2R_DCBUS_VOLTAGE, "S2R_DCBUS_VOLTAGE");
    dump_reg(S2R_ENABLED, "S2R_ENABLED");
    dump_reg(S2R_READY_TO_RUN, "S2R_READY_TO_RUN");
    dump_reg(S2R_FAULT, "S2R_FAULT");
    dump_reg(S2R_WARNING, "S2R_WARNING");
    dump_reg(S2R_VFD_VERSION, "S2R_VFD_VERSION");
    dump_reg(S2R_VFD_MODEL, "S2R_VFD_MODEL");
    dump_reg(S2R_STW, "S2R_STW");
    dump_reg(S2R_HSW, "S2R_HSW");
    dump_reg(S2R_ZSW, "S2R_ZSW");
    dump_reg(S2R_HIW, "S2R_HIW");
}

bool Fan::write_reg(SV20Reg regaddr, uint16_t value)
{
    uint8_t rc;

    for (uint8_t i=0 ; i < 3 ; ++i) {
        wait_deadtime();
        rc = bus.writeSingleRegister(regaddr, value);
        if (rc == bus.ku8MBSuccess) {
            return true;
        }
#ifdef DEBUG
        else {
            Serial.print("E wr a=");
            Serial.print(regaddr);
            Serial.print(" v=");
            Serial.print(value);
            Serial.print(" rc=");
            Serial.println(rc);
        }
#endif
    }

    return false;
}

bool Fan::read_reg(SV20Reg regaddr, uint16_t* value)
{
    wait_deadtime();
    uint8_t rc = bus.readHoldingRegisters(regaddr, 1);

    if (rc == bus.ku8MBSuccess) {
        *value = bus.getResponseBuffer(0);
    }
#ifdef DEBUG
        else {
            Serial.print("E rr a=");
            Serial.print(regaddr);
            Serial.print(" rc=");
            Serial.println(rc);
        }
#endif

    return rc;
}

void Fan::dump_reg(SV20Reg regaddr, const char* identifier)
{
    uint16_t value;

    uint8_t rc = read_reg(regaddr, &value);

    Serial.print(identifier);
    Serial.print("=");

    if (rc == bus.ku8MBSuccess) {
        Serial.println(value);
    } else {
        Serial.println("ERR");
    }
}

void Fan::wait_deadtime()
{
    delay(10);
//    if (micros() - tsu_last_telegram < TELEGRAM_SEND_DELAY_US) {
//        delayMicroseconds(TELEGRAM_SEND_DELAY_US - (micros() - tsu_last_telegram));
//    }
}
