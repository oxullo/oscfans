#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <AIO.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>
#include <OSCBoards.h>


#include "fan.h"


const char* VERSION = "v0.1.0";
const uint32_t MODBUS_BAUDRATE = 115200;
const uint8_t FANS_COUNT = 2;
const uint8_t MODBUS_ADDR_MAP[] = {1, 2};
const uint16_t MODBUS_FREQSETPOINT_REGADDR = 2; // cvt=2 modbus=40003 -> HSW
const uint16_t MODBUS_RUN_REGADDR = 3; // cvt=3 -> STW:3
const uint16_t OSC_PORT = 8888;
const uint32_t HEARTBEAT_PERIOD = 200;


typedef enum State {
    STATE_INIT,
    STATE_WAIT_FOR_ETHLINK,
    STATE_READY
} State;

Fan fans[FANS_COUNT];

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 20);
EthernetUDP osc_socket;
State state = STATE_INIT;


void set_fan1(OSCMessage& msg)
{
    float speed = msg.getFloat(0);

    Serial.print("set_fan id=1");
    Serial.print(" speed=");
    Serial.println(speed);

    // Clamp to range [0..100]
    speed = min(max(speed, 0), 100) / 100.0 * 16383;

    fans[0].set_setpoint(speed);

//    update_fan_speed(0, speed);
}

void set_fan2(OSCMessage& msg)
{
    float speed = msg.getFloat(0);

    Serial.print("set_fan id=2");
    Serial.print(" speed=");
    Serial.println(speed);

    // Clamp to range [0..100]
    speed = min(max(speed, 0), 100) / 100.0 * 16383;

    fans[1].set_setpoint(speed);

//    update_fan_speed(1, speed);
}

void enable(OSCMessage& msg)
{
    bool run = msg.getInt(0) == 1;

    Serial.print("Set run=");
    Serial.println(run);

    fans[0].set_enabled(run);
    fans[1].set_enabled(run);
}


void change_state(State new_state)
{
    if (new_state != state) {
        Serial.print("Changing state: ");
        Serial.print(state);
        Serial.print(" -> ");
        Serial.println(new_state);
        state = new_state;
    }
}

void heartbeat()
{
    static uint32_t ts_last_beat = 0;

    if (millis() - ts_last_beat > HEARTBEAT_PERIOD) {
        AIO::heartbeat();
        ts_last_beat = millis();
    }
}

void handle_state()
{
    switch (state) {
        case STATE_INIT:
            break;

        case STATE_WAIT_FOR_ETHLINK:
            if (Ethernet.linkStatus() == LinkON) {
                change_state(STATE_READY);
            }
            fans[0].reset();
            fans[1].reset();

            break;

        case STATE_READY:
            {
                OSCMessage msg;
                int packet_len = osc_socket.parsePacket();
                if (packet_len > 0) {
                    while (packet_len--) {
                        msg.fill(osc_socket.read());
                    }
                    if (msg.hasError()) {
                        // Assert error
                        digitalWrite(AIO::LED_RED, HIGH);
                    } else {
                        // Deassert error and toggle OSC packet receive
                        digitalWrite(AIO::LED_RED, LOW);
                        digitalWrite(AIO::LED_BLUE, !digitalRead(AIO::LED_BLUE));
                        msg.dispatch("/s/1", set_fan1);
                        msg.dispatch("/s/2", set_fan2);
                        msg.dispatch("/enable", enable);
                    }
                }
                if (Ethernet.linkStatus() == LinkOFF) {
                    change_state(STATE_WAIT_FOR_ETHLINK);
                }
            }
            break;
    }
}

void setup()
{
    Serial.begin(115200);

    Serial.print("OSCfans ");
    Serial.println(VERSION);

    Serial1.begin(MODBUS_BAUDRATE);
    AIO::baseboard_init();

    pinMode(AIO::RS485_RX_EN_PIN, OUTPUT);
    pinMode(AIO::RS485_TX_EN_PIN, OUTPUT);

    Serial.println("Initializing ethernet hardware");
    Ethernet.init(AIO::ETH_CS_PIN);
    Ethernet.begin(mac, ip);

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("FATAL: Cannot initialize ethernet hardware");
        for (;;);
    } else {
        Serial.print("Ethernet hardware type: ");
        Serial.println(Ethernet.hardwareStatus());
    }

    Serial.print("Device IP: ");
    Serial.print(Ethernet.localIP());
    Serial.print("/");
    Serial.println(Ethernet.subnetMask());

    osc_socket.begin(OSC_PORT);

    Serial.print("OSC endpoint port: ");
    Serial.println(OSC_PORT);

    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].begin(MODBUS_ADDR_MAP[i]);
        fans[i].dump_opregs();
    }

    change_state(STATE_WAIT_FOR_ETHLINK);
}

void loop()
{
    handle_state();
    heartbeat();
    fans[0].update();
//    fans[1].update();
}
