#include "ArduinoLowPower.h"

#include "utils.h"
#include "Timer.hpp"

// lora / radiohead
#include "rfm95_config.h"
#include "NodeTypes.h"

// voltage/current sensor
#include <Adafruit_INA219.h>

const uint8_t g_lora_address = 33;

// analog-digital-converter (ADC)
#define ADC_BITS 10
constexpr uint32_t ADC_MAX = (1 << ADC_BITS) - 1U;

const int g_update_interval = 33;
char g_serial_buf[512];

//! time management
const int g_update_interval_params = 2000;
int g_time_accum = 0, g_time_accum_params = 0;
long g_last_time_stamp;

enum TimerEnum
{
  TIMER_LORA_SEND = 0,
  TIMER_BATTERY_MEASURE = 1,
  TIMER_SENSOR_MEASURE = 2,
  NUM_TIMERS
};
kinski::Timer g_timer[NUM_TIMERS];

// battery
#if defined(ARDUINO_SAMD_ZERO)
constexpr uint8_t g_battery_pin = A7;
#elif defined(ARDUINO_FEATHER_M4)
constexpr uint8_t g_battery_pin = A6;
#endif

uint8_t g_battery_val = 0;

// INA219 I2C sensor
Adafruit_INA219 g_ina219;

radiostrom3000_t g_radiostrom3000 = {};

////////////////////////////////////////////////////////////////////////////////

void blink_status_led();

////////////////////////////////////////////////////////////////////////////////

// lora assets
lora::config_t g_lora_config = {};

// bundle radio-driver, datagram-manager and crypto assets
lora::driver_struct_t m_rfm95 = {};

//! lora message buffer
uint8_t g_lora_buffer[RH_RF95_MAX_MESSAGE_LEN];

float g_lora_send_interval = 5.f;

////////////////////////////////////////////////////////////////////////////////

void set_address(uint8_t address)
{
    g_lora_config.address = address;

    // init RFM95 module
    if(lora::setup(g_lora_config, m_rfm95))
    {
        Serial.print("LoRa radio init complete -> now listening on adress: 0x");
        Serial.println(g_lora_config.address, HEX);
    }
    else
    {
       Serial.println("LoRa radio init failed");
       while(true){ blink_status_led(); }
    }
}

////////////////////////////////////////////////////////////////////////////////
void lora_receive()
{
    uint8_t len = sizeof(g_lora_buffer);
    uint8_t from, to, msg_id, flags;

    // check for messages addressed to this node
    if(m_rfm95.manager->recvfromAck(g_lora_buffer, &len, &from, &to, &msg_id, &flags))
    {
        // received something
    }
}

////////////////////////////////////////////////////////////////////////////////

template<typename T> bool lora_send_status(const T &data)
{
    // data + checksum
    constexpr size_t num_bytes = sizeof(T) + 1;

    uint8_t crc_data[3 + sizeof(T)];
    crc_data[0] = g_lora_config.address;
    crc_data[1] = RH_BROADCAST_ADDRESS;

    memcpy(crc_data + 2, &data, sizeof(T));
    crc_data[sizeof(crc_data) - 1] = crc8(crc_data, 2 + sizeof(T));

    // send a broadcast-message
    return m_rfm95.manager->sendto(crc_data + 2, num_bytes, RH_BROADCAST_ADDRESS);
}

////////////////////////////////////////////////////////////////////////////////

void blink_status_led()
{
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
}

void setup()
{
    // drives our status LED
    pinMode(13, OUTPUT);

    // indicate "not ready"
    digitalWrite(13, HIGH);

    // while(!Serial){ blink_status_led(); }
    Serial.begin(9600);

    // battery measuring
    g_timer[TIMER_BATTERY_MEASURE].set_callback([]()
    {
        // voltage is divided by 2, so multiply back
        constexpr float voltage_divider = 2.f;
        auto raw_bat_measure = analogRead(g_battery_pin);

        float voltage = 3.3f * (float)raw_bat_measure * voltage_divider / (float)ADC_MAX;
        g_battery_val = static_cast<uint8_t>(map_value<float>(voltage, 3.6f, 4.2f, 0.f, 255.f));
        Serial.printf("battery-: %d%%\n", 100 * g_battery_val / 255);
        // Serial.printf("raw_bat_measure: %d\n", raw_bat_measure);
    });
    g_timer[TIMER_BATTERY_MEASURE].set_periodic();
    g_timer[TIMER_BATTERY_MEASURE].expires_from_now(10.f);

    // lora config
    set_address(g_lora_address);

    g_timer[TIMER_LORA_SEND].set_callback([]()
    {
        lora_send_status(g_radiostrom3000);
    });
    g_timer[TIMER_LORA_SEND].set_periodic();
    g_timer[TIMER_LORA_SEND].expires_from_now(g_lora_send_interval);

    // sensor setup
    if(!g_ina219.begin()){ while(true){ blink_status_led(); } }

    // sensor measuring
    g_timer[TIMER_SENSOR_MEASURE].set_callback([]()
    {
        float shuntvoltage = 0;
        float busvoltage = 0;
        float current_mA = 0;
        float loadvoltage = 0;
        float power_mW = 0;

        shuntvoltage = g_ina219.getShuntVoltage_mV();
        busvoltage = g_ina219.getBusVoltage_V();
        current_mA = g_ina219.getCurrent_mA();
        power_mW = g_ina219.getPower_mW();
        loadvoltage = busvoltage + (shuntvoltage / 1000);
        
        Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
        Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
        Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
        Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
        Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
        Serial.println("");

        // fill data-struct
        g_radiostrom3000.voltage = max(busvoltage * 1000, 0);
        g_radiostrom3000.current = max(current_mA, 0);
        g_radiostrom3000.power = max(power_mW, 0);
        g_radiostrom3000.battery = g_battery_val;

    });
    g_timer[TIMER_SENSOR_MEASURE].set_periodic();
    g_timer[TIMER_SENSOR_MEASURE].expires_from_now(2.f);

    digitalWrite(13, LOW);
}

void loop()
{
    // time measurement
    uint32_t delta_time = millis() - g_last_time_stamp;
    g_last_time_stamp = millis();
    g_time_accum += delta_time;
    g_time_accum_params += delta_time;

    float next_timer_event = 10.f;

    // poll Timer objects
    for(uint32_t i = 0; i < NUM_TIMERS; ++i)
    {
         g_timer[i].poll();
         next_timer_event = min(next_timer_event, g_timer[i].expires_from_now());
    }

    // wake up 2ms before next event
    int sleep_duration = max(next_timer_event * 1000 - 2, 0);
    LowPower.idle(sleep_duration);
}
