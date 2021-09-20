#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <AIO.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>
#include <OSCBoards.h>


#include "fanscontroller.h"


const char* VERSION = "v0.1.0";
const uint32_t MODBUS_BAUDRATE = 115200;
const uint16_t OSC_PORT = 8888;
const uint32_t HEARTBEAT_PERIOD = 200;


typedef enum State {
    STATE_INIT,
    STATE_WAIT_FOR_ETHLINK,
    STATE_READY
} State;

Fan fans[FANS_COUNT];
FansController fans_controller;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 20);
EthernetUDP osc_socket;
State state = STATE_INIT;


void set_setpoint(OSCMessage& msg, int addr_offset)
{
    uint8_t addr;
    if (msg.match("/1", addr_offset)) {
        addr = 1;
    } else if (msg.match("/2", addr_offset)) {
        addr = 2;
    } else {
        Serial.println("Invalid OSC address");
        digitalWrite(AIO::LED_RED, HIGH);
        return;
    }
    digitalWrite(AIO::LED_RED, LOW);

    float speed = msg.getFloat(0);

#ifdef DEBUG
    Serial.print("setpoint addr=");
    Serial.print(addr);
    Serial.print(" speed=");
    Serial.println(speed);
#endif

    fans_controller.set_setpoint_percent(addr, speed);
}

void set_all_setpoints(OSCMessage& msg)
{
    float speed1 = msg.getFloat(0);
    float speed2 = msg.getFloat(1);

#ifdef DEBUG
    Serial.print("all setpoint ");
    Serial.print(" speeds 1=");
    Serial.print(speed1);
    Serial.print(" 2=");
    Serial.println(speed2);
#endif

    fans_controller.set_setpoint_percent(1, speed1);
    fans_controller.set_setpoint_percent(2, speed2);
}

void enable(OSCMessage& msg)
{
    bool run = msg.getInt(0) == 1;

    Serial.print("Set run=");
    Serial.println(run);

    digitalWrite(AIO::LED_BLUE, run);

    fans_controller.enable(run);
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
                fans_controller.reset();
            }

            break;

        case STATE_READY:
            {
                OSCMessage msg;
                int packet_len;

                while ((packet_len = osc_socket.parsePacket()) > 0) {
                    while (packet_len--) {
                        msg.fill(osc_socket.read());
                    }
                    if (msg.hasError()) {
                        // Assert error
                        digitalWrite(AIO::LED_RED, HIGH);
                    } else {
                        // Deassert error and toggle OSC packet receive
                        msg.route("/s", set_setpoint);
                        msg.dispatch("/sa", set_all_setpoints);
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
        digitalWrite(AIO::LED_RED, HIGH);
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

    digitalWrite(AIO::LED_BLUE, HIGH);
    digitalWrite(AIO::LED_RED, HIGH);
    delay(2000);
    digitalWrite(AIO::LED_BLUE, LOW);
    digitalWrite(AIO::LED_RED, LOW);

    Serial.println("Initializing fans controllers");
    fans_controller.begin();

    Serial.println("Initialization done");

    change_state(STATE_WAIT_FOR_ETHLINK);
}

void loop()
{
    handle_state();
    fans_controller.update();
    heartbeat();
}
