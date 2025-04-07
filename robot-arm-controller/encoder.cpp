#include <Arduino.h>
#include "encoder.h"

void Encoder::init(){
  // state depends whether we will be using pullup resistors or not
  pinMode(m_sensor_a_pin, INPUT_PULLUP);
  pinMode(m_sensor_b_pin, INPUT_PULLUP);

  tick();
}

// a cyclic array of encoder pattern states.
// if the index of a new pattern is higher then 
// the one of an old one (with looping back), 
// it means a positive reading has occured.
const char enc_patterns_array[4] = {
  0b00,
  0b01,
  0b11,
  0b10
};

void Encoder::tick(){
  // get pattern reading
  int pattern = digitalRead(m_sensor_a_pin) << 1 | digitalRead(m_sensor_b_pin);
  
  int cur_idx;
  for(cur_idx = 0; cur_idx < 4; cur_idx++){
    if(pattern == enc_patterns_array[cur_idx]){
      break;
    }
  }

  // check whether we went 1+ or 1- from the previous index
  if((m_prev_pattern_idx + 1) % 4 == cur_idx){
    reading++;
  }else{
    int dec_idx = m_prev_pattern_idx - 1;
    if(dec_idx < 0){
      dec_idx = 3;
    }

    if(dec_idx == cur_idx){
      reading--;
    }
  }

  m_prev_pattern_idx = cur_idx;
}

int Encoder::getReading(){
  return reading;
}