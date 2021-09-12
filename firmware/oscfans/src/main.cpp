#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <AIO.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include <ModbusMaster.h>
#include <OSCBundle.h>
#include <OSCBoards.h>


const char* VERSION = "v0.1.0";
const uint32_t MODBUS_BAUDRATE = 9600;
const uint8_t FANS_COUNT = 2;
const uint8_t MODBUS_ADDR_MAP[] = {1, 2};
const uint16_t MODBUS_FREQSETPOINT_REGADDR = 40003; // cvt=2 modbus=40003 -> HSW
const uint16_t OSC_PORT = 8888;


ModbusMaster fans[FANS_COUNT];

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 20);
EthernetUDP osc_socket;


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

    speed = min(max(speed, 0), 100);

    fans[address].writeSingleRegister(MODBUS_FREQSETPOINT_REGADDR, speed);
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(MODBUS_BAUDRATE);
    AIO::baseboard_init();

    pinMode(AIO::RS485_RX_EN_PIN, OUTPUT);
    pinMode(AIO::RS485_TX_EN_PIN, OUTPUT);

    for (uint8_t i = 0; i < FANS_COUNT; ++i) {
        fans[i].begin(MODBUS_ADDR_MAP[i], Serial1);
        fans[i].preTransmission(pre_tx);
        fans[i].postTransmission(post_tx);
    }

    Ethernet.begin(mac, ip);
    osc_socket.begin(OSC_PORT);

    Serial.print("OSCfans ");
    Serial.println(VERSION);
}

void loop()
{
    OSCBundle bundle;
    
    int packet_len = osc_socket.parsePacket();

    if (packet_len) {
        while (packet_len--) {
            bundle.fill(osc_socket.read());
        }

        if (bundle.hasError()) {
            Serial.println("OSC Packet error");
        } else {
            bundle.route("/s", set_fan);
        }
    }

    AIO::heartbeat();
}
