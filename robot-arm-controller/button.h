#pragma once

enum ButtonState{
  DOWN,
  UP,
  HELD,
  FREED,
};

class Button{
private:
  const int m_button_pin;
  
  inline bool buttonPressed(){
    return !digitalRead(m_button_pin);
  }
public:
  bool m_held;

  Button(int pin): m_button_pin(pin) {};
  void init();

  ButtonState getButtonState();
};
