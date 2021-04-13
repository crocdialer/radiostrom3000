#ifndef RFM95_CONFIG_
#define RFM95_CONFIG_

#include <RH_RF95.h>
#include <RHMesh.h>

// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
/// - RH_RF95 message:
/// - 8 symbol PREAMBLE
/// - Explicit header with header CRC (handled internally by the radio)
/// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
/// - 0 to 251 octets DATA
/// - CRC (handled internally by the radio)

namespace lora{

#if defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_FEATHER_M4)
#define CS_PIN 6
#elif defined(ARDUINO_ITSYBITSY_M0) || defined(ARDUINO_ITSYBITSY_M4)
#define CS_PIN 7
#endif

struct config_t
{
    //! chip select pin (<6> on adafruit_feather_m4, <7> on adafruit_itsybitsy_m4)
    uint8_t pin_cs = CS_PIN;

    //! interrupt request pin
    uint8_t pin_irq = 10;

    //! reset pin
    uint8_t pin_reset = 12;

    float frequency = 868.f;

    //! default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:
    uint8_t tx_power = 15;

    uint8_t address = 0;

    // carrier-access detection (ms)
    uint32_t cad_timeout = 10;
};

enum NodeStructType
{
    // NodeType is used type field by Node structs
    NodeType = 1 << 0,

    // NodeCommandType is used type field by NodeCommand structs
    NodeCommandType = 1 << 1,

    // NodeCommandACKType is used type field by NodeCommandACK structs
    NodeCommandACKType = 1 << 2

};

struct driver_struct_t
{
    RH_RF95 *driver = nullptr;
    RHReliableDatagram *manager = nullptr;
};

static bool setup(const config_t &the_config, driver_struct_t &out_driver)
{
    Serial.printf("lora-config: %d Mhz - %d dB\n", int(the_config.frequency), the_config.tx_power);

    // delete stuff
    if(out_driver.driver){ delete out_driver.driver; }
    if(out_driver.manager){ delete out_driver.manager; }

    out_driver.driver = new RH_RF95(the_config.pin_cs, the_config.pin_irq);

    pinMode(the_config.pin_reset, OUTPUT);
    digitalWrite(the_config.pin_reset, HIGH);

    // manual reset
    digitalWrite(the_config.pin_reset, LOW);
    delay(10);
    digitalWrite(the_config.pin_reset, HIGH);
    delay(10);

    // init the driver layer
    if(!out_driver.driver->init()){ return false; }

    // set tx-power
    delay(100);
    out_driver.driver->setTxPower(the_config.tx_power, false);
    
    // TODO: has no effect
    delay(100);
    if(!out_driver.driver->setFrequency(the_config.frequency)){ return false; }

    // 10ms carrier-sensing before transmit
    out_driver.driver->setCADTimeout(the_config.cad_timeout);

    // init manager
    out_driver.manager = new RHReliableDatagram(*out_driver.driver, the_config.address);
    return out_driver.manager->init();
}

}// lora namespace
#endif
