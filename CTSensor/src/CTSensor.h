#ifndef CT_SENSOR_H
#define CT_SENSOR_H

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <math.h>

#define DEBUG_MODE 1

class CTSensor
{
public:
  CTSensor(uint8_t n);

  bool init();
  void start_sampling();
  void reset();

  double get_min_current_rms_in_amps_for(uint8_t channel);
  double get_avg_current_rms_in_amps_for(uint8_t channel);
  double get_max_current_rms_in_amps_for(uint8_t channel);
  double get_energy_in_watts_hour_for(uint8_t channel);

  void set_ref_voltage_on_zero_current_for(uint8_t channel, float ref_value);

  void print_debug_info();
  void print_channels_info();

  virtual void print_log(const char *msg) = 0;
  virtual float get_raw_value_for(uint8_t channel) = 0;
  virtual void delay(uint16_t miliseconds) = 0;

  ~CTSensor();

protected:
  bool is_debuging_mode = true;

  uint8_t _sampling_period_in_seconds = 1;
  uint16_t _sampling_frequency_in_hertz = 100;
  uint8_t _max_current_input = 100;
  uint8_t _ref_voltage_input = 230;
  float _current_noise_level = 0.05;
  bool _skip_sampling_delay = true;
  double _ref_voltage_on_zero_current_per_channel[4] = {1.65, 1.65, 1.65, 1.65};
  uint8_t _current_rms_data_points_count = 1;

  uint8_t _channels_count;
  uint16_t _sampling_data_points_count;
  uint16_t _sampling_delay_ms;
  
  double _peak_current;

  double *_sum_of_current_rms_per_channel = NULL;
  double *_min_current_rms_per_channel = NULL;
  double *_avg_current_rms_per_channel = NULL;
  double *_max_current_rms_per_channel = NULL;

private:
  void _add_current_rms_params_for(uint8_t channel, double current_rms);
  double _get_current_rms_for_instants(double sum_of_squared_instants);

  void _log(const char *format, ...);

  void _new_buffers();
  void _delete_buffers();
};

#endif