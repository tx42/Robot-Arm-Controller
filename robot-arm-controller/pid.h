#pragma once

class PIDController{
private:
  float m_accum_error;
  float m_prev_error;

  // TODO: potentially add time interpolation
  //long m_prev_time;

public:
  float p_gain;
  float i_gain;
  float d_gain;

  float calculate(float target, float current);
};