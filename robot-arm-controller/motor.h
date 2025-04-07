#pragma once
#include <Arduino.h>

enum MotorDirection{
  FORWARD,
  REVERSE,
  STOP
};

class Motor{
private:
  const int m_power_pin;
  const int m_forward_pin;
  const int m_reverse_pin;

  void setDirection(MotorDirection direction);
public:
  void setRawPower(unsigned int power);
  Motor(int power_pin, int forward_pin, int reverse_pin)
    : m_power_pin(power_pin),
      m_forward_pin(forward_pin),
      m_reverse_pin(reverse_pin) {};

  void init();
  // power should be an integer ranging from -255 to 255,
  // anything higher will be capped
  void setPower(int power);
};