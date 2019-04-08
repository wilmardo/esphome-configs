#include "esphome.h"
using namespace esphome;

class MyCustomComponent : public Component
{
  public:
    sensor::Sensor *consumption_low_tarif = new sensor::Sensor();
    sensor::Sensor *consumption_high_tarif = new sensor::Sensor();
    sensor::Sensor *actual_consumption = new sensor::Sensor();
    sensor::Sensor *instant_power_usage = new sensor::Sensor();
    sensor::Sensor *instant_power_current = new sensor::Sensor();
    sensor::Sensor *gas_meter_m3 = new sensor::Sensor();
    sensor::Sensor *actual_tarif_group = new sensor::Sensor();
    sensor::Sensor *short_power_outages = new sensor::Sensor();
    sensor::Sensor *long_power_outages = new sensor::Sensor();
    sensor::Sensor *short_power_drops = new sensor::Sensor();
    sensor::Sensor *short_power_peaks = new sensor::Sensor();

    MyCustomSensor() : PollingComponent(15000) {}

    void setup() override
    {
    }
    void loop() override
    {
    }
    void update() override
    {
        float temperature = bmp.readTemperature();
        temperature_sensor->publish_state(temperature);

        int pressure = bmp.readPressure();
        pressure_sensor->publish_state(pressure / 100.0);
    }
}