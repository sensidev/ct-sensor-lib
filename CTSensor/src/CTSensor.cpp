#include "CTSensor.h"

CTSensor::CTSensor(uint8_t n)
{
    _channels_count = n;
}

bool CTSensor::init()
{
    _peak_current = sqrt(2) * _max_current_input;
    _sampling_data_points_count = _sampling_period_in_seconds * _sampling_frequency_in_hertz;
    _sampling_delay_ms = (uint16_t)roundf(1000 / _sampling_frequency_in_hertz);

    return true;
}

void CTSensor::calibrate(float *ref_values)
{
    for (uint8_t c = 0; c < _channels_count; c++)
    {
        _ref_voltage_on_zero_current_per_channel[c] = ref_values[c];
    }
}

void CTSensor::print_calibration_info()
{
    float sum_of_raw_values_per_channel[_channels_count] = {0, 0, 0, 0};
    uint8_t points = CALIBRATION_DATA_POINTS;

    while (points)
    {
        for (uint8_t c = 0; c < _channels_count; c++)
        {
            sum_of_raw_values_per_channel[c] += get_raw_value_for(c);
            delay(DELAY_BETWEEN_I2C_READS_MS);
        }
        points--;
    }

    _log("Current calibration set: {%f, %f, %f, %f};",
         _ref_voltage_on_zero_current_per_channel[0],
         _ref_voltage_on_zero_current_per_channel[1],
         _ref_voltage_on_zero_current_per_channel[2],
         _ref_voltage_on_zero_current_per_channel[3]);
    _log("Desired calibration set: {%f, %f, %f, %f};",
         sum_of_raw_values_per_channel[0] / CALIBRATION_DATA_POINTS,
         sum_of_raw_values_per_channel[1] / CALIBRATION_DATA_POINTS,
         sum_of_raw_values_per_channel[2] / CALIBRATION_DATA_POINTS,
         sum_of_raw_values_per_channel[3] / CALIBRATION_DATA_POINTS);
}

CTSensor::~CTSensor()
{
    _delete_buffers();
}

void CTSensor::start_sampling()
{
    double instant;
    double sum_of_squared_instants[_channels_count] = {};

    _log("[SAMPLING] START #%d!", _current_rms_data_points_count);

    for (uint16_t p = 0; p < _sampling_data_points_count; p++)
    {
        for (uint8_t i = 0; i < _channels_count; i++)
        {
            float _ref_voltage_on_zero_current = _ref_voltage_on_zero_current_per_channel[i];
            instant = (get_raw_value_for(i) - _ref_voltage_on_zero_current) / _ref_voltage_on_zero_current;
            sum_of_squared_instants[i] += instant * instant;
            delay(DELAY_BETWEEN_I2C_READS_MS);
        }
        if (!_skip_sampling_delay)
        {
            delay(_sampling_delay_ms);
        }
    }

    for (uint8_t channel = 0; channel < _channels_count; channel++)
    {
        double rms = _get_current_rms_for_instants(sum_of_squared_instants[channel]);

        _add_current_rms_params_for(channel, rms);
    }

    _current_rms_data_points_count++;
    _log("[SAMPLING] data point count so far: %d", _current_rms_data_points_count);
    _log("------------------------------------");
}

void CTSensor::_add_current_rms_params_for(uint8_t channel, double current_rms)
{
    _log("[CHANNEL %d] Add current RMS: %f Amps to data points", channel, current_rms);

    _rms_current_data_points_per_channel[channel][_current_rms_data_points_count - 1] = current_rms;

    _sum_of_current_rms_per_channel[channel] += current_rms;

    if (_min_current_rms_per_channel[channel] > current_rms)
    {
        _min_current_rms_per_channel[channel] = current_rms;
    }

    if (_max_current_rms_per_channel[channel] < current_rms)
    {
        _max_current_rms_per_channel[channel] = current_rms;
    }

    _avg_current_rms_per_channel[channel] = _sum_of_current_rms_per_channel[channel] / _current_rms_data_points_count;
}

double CTSensor::get_min_current_rms_in_amps_for(uint8_t channel)
{
    return _min_current_rms_per_channel[channel];
}

double CTSensor::get_avg_current_rms_in_amps_for(uint8_t channel)
{
    return _avg_current_rms_per_channel[channel];
}

double CTSensor::get_max_current_rms_in_amps_for(uint8_t channel)
{
    return _max_current_rms_per_channel[channel];
}

double CTSensor::get_energy_in_watts_hour_for(uint8_t channel)
{
    return _ref_voltage_input * _sum_of_current_rms_per_channel[channel] * _sampling_period_in_seconds / 3600;
}

double CTSensor::get_voltage_in_volts_for(uint8_t channel)
{
    return (double)_ref_voltage_input;
}

double CTSensor::get_avg_power_in_watts_for(uint8_t channel)
{
    return _ref_voltage_input * get_avg_current_rms_in_amps_for(channel);
}

double *CTSensor::get_rms_current_data_points_for(uint8_t channel)
{
    return _rms_current_data_points_per_channel[channel];
}

double CTSensor::_get_current_rms_for_instants(double sum_of_squared_instants)
{
    _log("Sum of squared instants: %f", sum_of_squared_instants);
    _log("Sampling data points count: %d", _sampling_data_points_count);
    _log("Peak current: %f", _peak_current);

    double rms = _peak_current * sqrt(sum_of_squared_instants / _sampling_data_points_count);
    bool should_filter_noise_out = rms < _current_noise_level;

    if (should_filter_noise_out)
    {
        rms = 0.0;
    }

    return rms;
}

void CTSensor::setup()
{
    _new_buffers();

    for (uint8_t channel = 0; channel < _channels_count; channel++)
    {
        _sum_of_current_rms_per_channel[channel] = 0.0;
        _max_current_rms_per_channel[channel] = 0.0;
        _avg_current_rms_per_channel[channel] = 0.0;
        _min_current_rms_per_channel[channel] = _max_current_input;
    }

    _current_rms_data_points_count = 1;
}

void CTSensor::teardown()
{
    _delete_buffers();
}

void CTSensor::_delete_buffers()
{
    for (uint8_t channel = 0; channel < _channels_count; channel++)
    {
        if (_rms_current_data_points_per_channel[channel])
        {
            delete[] _rms_current_data_points_per_channel[channel];
        }
    }
    delete[] _rms_current_data_points_per_channel;

    delete[] _sum_of_current_rms_per_channel;
    delete[] _min_current_rms_per_channel;
    delete[] _avg_current_rms_per_channel;
    delete[] _max_current_rms_per_channel;
}

void CTSensor::_new_buffers()
{
    _rms_current_data_points_per_channel = new double *[_channels_count];
    for (uint8_t channel = 0; channel < _channels_count; channel++)
    {
        _rms_current_data_points_per_channel[channel] = new double[MAX_NUMER_OF_RMS_CURRENT_DATA_POINTS_PER_CHANNEL];
    }
    _sum_of_current_rms_per_channel = new double[_channels_count];
    _min_current_rms_per_channel = new double[_channels_count];
    _avg_current_rms_per_channel = new double[_channels_count];
    _max_current_rms_per_channel = new double[_channels_count];
}

void CTSensor::print_debug_info()
{
    _log("Sampling period in seconds: %d", _sampling_period_in_seconds);
    _log("Sampling frequency in hertz: %d", _sampling_frequency_in_hertz);
    _log("Sampling data points count: %d", _sampling_data_points_count);
    _log("Sampling delay: %d", _sampling_delay_ms);
    _log("Skip sampling delay: %d", _skip_sampling_delay);
    _log("Max current input from sensors: %d", _max_current_input);
    _log("Max peak current: %f", _peak_current);
    _log("Ref constant voltage input from grid: %d", _ref_voltage_input);
    _log("Current RMS data points count: %d", _current_rms_data_points_count);
    _log("Current RMS noise level: %f", _current_noise_level);
    _log("------------------------------------");
}

void CTSensor::print_channels_info()
{
    for (uint8_t c = 0; c < _channels_count; c++)
    {
        _log("[CHANNEL %d] Raw value: %f V", c, get_raw_value_for(c));
        _log("[CHANNEL %d] Ref voltage on zero current: %f V", c, _ref_voltage_on_zero_current_per_channel[c]);
        _log("[CHANNEL %d] Min Current RMS: %f Amps", c, get_min_current_rms_in_amps_for(c));
        _log("[CHANNEL %d] Avg Current RMS: %f Amps", c, get_avg_current_rms_in_amps_for(c));
        _log("[CHANNEL %d] Max Current RMS: %f Amps", c, get_max_current_rms_in_amps_for(c));
        _log("[CHANNEL %d] Energy: %f Watts-Hour", c, get_energy_in_watts_hour_for(c));
        _log("------------------------------------");
    }
}

void CTSensor::_log(const char *format, ...)
{
#if DEBUG_MODE
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    print_log(buffer);
#endif
}