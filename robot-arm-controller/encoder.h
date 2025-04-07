#pragma once

class Encoder{
private:
  int m_sensor_a_pin;
  int m_sensor_b_pin;

  int m_prev_pattern_idx;
public:
  // there may be overflow errors...
  int reading;

  Encoder(int sensor_a_pin, int sensor_b_pin)
    : m_sensor_a_pin(sensor_a_pin),
      m_sensor_b_pin(sensor_b_pin),
      reading(0),
      m_prev_pattern_idx(0) {};
  
  void init();
  // must be called regularly to ensure correct readings
  void tick();
  int getReading();
};