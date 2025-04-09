#include <Arduino.h>
#include "button.h"

void Button::init(){
   pinMode(m_button_pin, INPUT_PULLUP);
}

ButtonState Button::getButtonState(){
  if(buttonPressed()){
    if(!m_held){
      m_held = true;
      return DOWN;
    }

    m_held = true;
    return HELD;
  }else{
    if(m_held){
      m_held = false;
      return UP;
    }

    m_held = false;
    return FREED;
  }
}
