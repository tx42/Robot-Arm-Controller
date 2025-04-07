#include <Arduino.h>
#include "pid.h"

float PIDController::calculate(float target, float current){
  float error = target - current;
  
  m_accum_error += error;

  float potential = error * p_gain;
  float integral = m_accum_error * i_gain;
  float derivative = (error - m_prev_error) * d_gain;

  return potential + integral + derivative;
}