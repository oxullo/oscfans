#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <AIO.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ModbusMaster.h>
#include <OSCBundle.h>
#include <OSCBoards.h>


const char* VERSION = "v0.1.0";
const uint32_t MODBUS_BAUDRATE = 9600;
const uint8_t FANS_COUNT = 2;
const uint8_t MODBUS_ADDR_MAP[] = {1, 2};
const uint16_t MODBUS_FREQSETPOINT_REGADDR = 40003; // cvt=2 modbus=40003 -> HSW
const uint16_t MODBUS_RUN_REGADDR = 40004; // cvt=3 -> STW:3
const uint16_t OSC_PORT = 8888;

typedef enum State {
    STATE_INIT,
    STATE_WAIT_FOR_ETHLINK,
    STATE_READY
} State;

ModbusMaster fans[FANS_COUNT];

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 20);
EthernetUDP osc_socket;
State state = STATE_INIT;


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

void update_fan_speed(uint8_t index, uint8_t speed)
{
    uint8_t rc = fans[index].writeSingleRegister(MODBUS_FREQSETPOINT_REGADDR, speed);

    if (rc) {
        Serial.print("ERR: cannot update fan speed idx=");
        Serial.print(index);
        Serial.print(" speed=");
        Serial.print(speed);
        Serial.print(" rc=");
        Serial.println(rc);
    }
}

void set_fan1(OSCMessage& msg)
{
    int speed = msg.getInt(0);

    Serial.print("set_fan id=1");
    Serial.print(" speed=");
    Serial.println(speed);

    // Clamp to range [0..100]
    speed = min(max(speed, 0), 100);

    update_fan_speed(0, speed);
}

void set_fan2(OSCMessage& msg)
{
    int speed = msg.getInt(0);

    Serial.print("set_fan id=2");
    Serial.print(" speed=");
    Serial.println(speed);

    // Clamp to range [0..100]
    speed = min(max(speed, 0), 100);

    update_fan_speed(1, speed);
}

void enable(OSCMessage& msg)
{
    int enabled = msg.getInt(0);
    Serial.print("Enable: ");
    Serial.println(enabled);

    for (uint8_t i=0 ; i < FANS_COUNT ; ++i) {
        uint8_t rc = fans[i].writeSingleRegister(MODBUS_RUN_REGADDR, enabled);
        if (rc) {
            Serial.print("ERR: cannot set enable=");
            Serial.print(enabled);
            Serial.print(" fan idx=");
            Serial.print(i);
            Serial.print(" rc=");
            Serial.println(rc);
        }
    }
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

void setup()
{
    Serial.begin(115200);

    Serial.print("OSCfans ");
    Serial.println(VERSION);

    Serial1.begin(MODBUS_BAUDRATE);
    AIO::baseboard_init();

    pinMode(AIO::RS485_RX_EN_PIN, OUTPUT);
    pinMode(AIO::RS485_TX_EN_PIN, OUTPUT);

    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].begin(MODBUS_ADDR_MAP[i], Serial1);
        fans[i].preTransmission(pre_tx);
        fans[i].postTransmission(post_tx);
    }

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

    change_state(STATE_WAIT_FOR_ETHLINK);
}

void loop()
{
    switch (state) {
        case STATE_WAIT_FOR_ETHLINK:
            if (Ethernet.linkStatus() == LinkON) {
                change_state(STATE_READY);
            }
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
                    Serial.println("OSC Packet error");
                } else {
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

    AIO::heartbeat();
}
