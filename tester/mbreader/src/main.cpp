#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <AIO.h>
#include <ModbusMaster.h>


const char* VERSION = "v0.1.0";
const uint32_t MODBUS_BAUDRATE = 115200;
const uint8_t FANS_COUNT = 2;
const uint16_t UNIT_ADDR = 1;


ModbusMaster vfd;

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


void pre_tx()
{
    digitalWrite(AIO::RS485_RX_EN_PIN, HIGH);
    digitalWrite(AIO::RS485_TX_EN_PIN, HIGH);
}

void post_tx()
{
    digitalWrite(AIO::RS485_RX_EN_PIN, LOW);
    digitalWrite(AIO::RS485_TX_EN_PIN, LOW);
}

uint8_t read_single_reg(SV20Reg regaddr, uint16_t* value)
{
    uint8_t result = vfd.readHoldingRegisters(regaddr, 1);

    if (result == vfd.ku8MBSuccess) {
        *value = vfd.getResponseBuffer(0);
    }
    return result;
}

void read_and_print(SV20Reg regaddr, const char* label)
{
    uint16_t value;

    uint8_t rc = read_single_reg(regaddr, &value);

    Serial.print(label);
    Serial.print("=");

    if (rc == vfd.ku8MBSuccess) {
        Serial.println(value);
    } else {
        Serial.println("ERR");
    }
    delay(2);
}

void read_regs()
{
    read_and_print(S2R_WD_TIME, "S2R_WD_TIME");
    read_and_print(S2R_FREQ_SETPOINT, "S2R_FREQ_SETPOINT");
    read_and_print(S2R_RUN_ENABLE, "S2R_RUN_ENABLE");
    read_and_print(S2R_FWD_REV, "S2R_FWD_REV");
    read_and_print(S2R_START_CMD, "S2R_START_CMD");
    read_and_print(S2R_FAULT_ACK, "S2R_FAULT_ACK");
    read_and_print(S2R_CURRENT_LIMIT, "S2R_CURRENT_LIMIT");
    read_and_print(S2R_ACCEL_TIME, "S2R_ACCEL_TIME");
    read_and_print(S2R_DECEL_TIME, "S2R_DECEL_TIME");
    read_and_print(S2R_REF_FREQ, "S2R_REF_FREQ");
    read_and_print(S2R_TORQUE, "S2R_TORQUE");
    read_and_print(S2R_ACTUAL_POWER, "S2R_ACTUAL_POWER");
    read_and_print(S2R_DCBUS_VOLTAGE, "S2R_DCBUS_VOLTAGE");
    read_and_print(S2R_ENABLED, "S2R_ENABLED");
    read_and_print(S2R_READY_TO_RUN, "S2R_READY_TO_RUN");
    read_and_print(S2R_FAULT, "S2R_FAULT");
    read_and_print(S2R_WARNING, "S2R_WARNING");
    read_and_print(S2R_VFD_VERSION, "S2R_VFD_VERSION");
    read_and_print(S2R_VFD_MODEL, "S2R_VFD_MODEL");
    read_and_print(S2R_STW, "S2R_STW");
    read_and_print(S2R_HSW, "S2R_HSW");
    read_and_print(S2R_ZSW, "S2R_ZSW");
    read_and_print(S2R_HIW, "S2R_HIW");
}

void write_reg(SV20Reg regaddr, uint16_t value)
{
    uint8_t rc = vfd.writeSingleRegister(regaddr, value);

    Serial.print("Write regaddr=");
    Serial.print(regaddr);
    Serial.print(" value=");
    Serial.print(value);

    if (rc == vfd.ku8MBSuccess) {
        Serial.println(" ok");
    } else {
        Serial.println(" error");
    }
}

void setup()
{
    Serial.begin(115200);

    Serial.print("MBReader ");
    Serial.println(VERSION);

    Serial1.begin(MODBUS_BAUDRATE);
    AIO::baseboard_init();

    pinMode(AIO::RS485_RX_EN_PIN, OUTPUT);
    pinMode(AIO::RS485_TX_EN_PIN, OUTPUT);

    vfd.begin(UNIT_ADDR, Serial1);
    vfd.preTransmission(pre_tx);
    vfd.postTransmission(post_tx);

    write_reg(S2R_WD_TIME, 2000);

    Serial.println("Init completed");
}

void update_control(bool start, uint16_t setpoint)
{
    vfd.setTransmitBuffer(0, start ? 0x047F : 0x047E);
    vfd.setTransmitBuffer(1, setpoint);
    uint8_t rc = vfd.writeMultipleRegisters(S2R_STW, 2);

    if (rc != vfd.ku8MBSuccess) {
        Serial.println("UC err");
    }
}

void loop()
{
    static bool update = false;
    static bool enable = false;
    static uint16_t setpoint = 0;
    static uint32_t ts_last_setpoint = 0;

    if (Serial.available()) {
        char entry = Serial.read();

        if (entry >= '0' && entry <= '9') {
            setpoint = (entry - '0') / 10.0 * 16383.0;
            Serial.print("Setpoint=");
            Serial.println(setpoint);
        } else {
            switch (entry) {
                case 'p':
                    Serial.println("Dumping regs:");
                    read_regs();
                    Serial.println("----");
                    break;

                case 'S':
                    write_reg(S2R_START_CMD, 1);
                    break;

                case 's':
                    write_reg(S2R_START_CMD, 0);
                    break;

                case 'R':
                    write_reg(S2R_RUN_ENABLE, 1);
                    break;

                case 'r':
                    write_reg(S2R_RUN_ENABLE, 0);
                    break;

                case '0':
                    write_reg(S2R_FREQ_SETPOINT, 0);
                    break;

                case 't':
                    write_reg(S2R_STW, 1151);
                    break;

                case 'E':
                    Serial.println("Enabling");
                    enable = true;
                    break;

                case 'e':
                    Serial.println("Disabling");
                    enable = false;
                    break;

                case 'w':
                    Serial.println("Starting");
                    enable = false;
                    break;

                case 'U':
                    Serial.println("Start updating STW");
                    update = true;
                    write_reg(S2R_FAULT_ACK, 0);
                    delay(10);
                    write_reg(S2R_FAULT_ACK, 1);
                    delay(5);
                    break;

                case 'u':
                    Serial.println("Stop updating STW");
                    update = false;
                    break;

                case 'a':
                    // Acknowledge faults
                    write_reg(S2R_FAULT_ACK, 0);
                    delay(100);
                    write_reg(S2R_FAULT_ACK, 1);
                    break;

                default:
                    Serial.println("p=print regs");
            }
        }
    }

    if (update && millis() - ts_last_setpoint > 100) {
        // write_reg(S2R_FREQ_SETPOINT, setpoint);
        update_control(enable, setpoint);
        ts_last_setpoint = millis();
    }
}
