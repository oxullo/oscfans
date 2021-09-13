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
const uint16_t OSC_PORT = 8888;

typedef enum State {
    STATE_INIT,
    STATE_WAIT_FOR_LINK,
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

void set_fan(OSCMessage& msg, int address)
{
    float speed = msg.getFloat(0);
    Serial.print("set_fan id=");
    Serial.print(address);
    Serial.print(" speed=");
    Serial.println(speed);

    // Clamp to range [0..100]
    speed = min(max(speed, 0), 100);

    fans[address].writeSingleRegister(MODBUS_FREQSETPOINT_REGADDR, speed);
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

    change_state(STATE_WAIT_FOR_LINK);
}

void loop()
{
    switch (state) {
        case STATE_WAIT_FOR_LINK:
            if (Ethernet.linkStatus() == LinkON) {
                change_state(STATE_READY);
            }
            break;

        case STATE_READY:
        {
            OSCBundle bundle;

            int packet_len = osc_socket.parsePacket();

            if (packet_len > 0) {
                Serial.println(packet_len);
                while (packet_len--) {
                    bundle.fill(osc_socket.read());
                }

                if (bundle.hasError()) {
                    Serial.println("OSC Packet error");
                } else {
                    bundle.route("/s", set_fan);
                }
            }

            if (Ethernet.linkStatus() == LinkOFF) {
                change_state(STATE_WAIT_FOR_LINK);
            }
        }
        break;
    }

    AIO::heartbeat();
}
