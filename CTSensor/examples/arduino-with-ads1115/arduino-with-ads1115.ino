#include <Wire.h>
#include <CTSensor.h>
#include <Adafruit_ADS1015.h>

#define CHANNEL_COUNT 4

Adafruit_ADS1115 ads(0x48);

class ArduinoCTSensor : public CTSensor
{
public:
  ArduinoCTSensor(uint8_t n) : CTSensor(n) {}

  float get_raw_value_for(uint8_t channel)
  {
    int16_t instant = ads.readADC_SingleEnded(channel);
    return (instant * 0.1875) / 1000;
  }

  void print_log(const char *msg)
  {
    Serial.println(msg);
  }

  void delay(uint16_t miliseconds)
  {
    delay(miliseconds);
  }
};

ArduinoCTSensor *sensor;

void setup(void)
{
  Wire.setClock(400000);
  Serial.begin(9600);
  ads.begin();

  sensor = new ArduinoCTSensor(CHANNEL_COUNT);
  sensor->set_ref_voltage_on_zero_current_for(0, 1.659);
  sensor->set_ref_voltage_on_zero_current_for(1, 1.6025);
  sensor->set_ref_voltage_on_zero_current_for(2, 1.654);
  sensor->set_ref_voltage_on_zero_current_for(3, 1.6888);
  sensor->init();
  sensor->print_debug_info();
}

void loop(void)
{
  sensor->reset();
  sensor->start_sampling();
  sensor->print_channels_info();
}
