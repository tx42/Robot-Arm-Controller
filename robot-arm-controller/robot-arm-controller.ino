#define USE_TIMER_2 true
#define UPDATE_TIME 1
#include <TimerInterrupt.h>

#include "encoder.h"
#include "motor.h"
#include "pid.h"

/*** HARDWARE CONSTANTS ***/
#define MOTOR_A_PWR 9
#define MOTOR_A_FWD 10
#define MOTOR_A_REV 8

#define MOTOR_B_PWR 10
#define MOTOR_B_FWD 10
#define MOTOR_B_REV 10

#define SENSOR_A1 2
#define SENSOR_A2 3

#define SENSOR_B1 2
#define SENSOR_B2 3


Motor motor_a(MOTOR_A_PWR, MOTOR_A_FWD, MOTOR_A_REV);
Motor motor_b(MOTOR_B_PWR, MOTOR_B_FWD, MOTOR_B_REV);

Encoder encoder_a(SENSOR_A1, SENSOR_A2);
Encoder encoder_b(SENSOR_B1, SENSOR_B2);

PIDController pid;

float target = 3.0;

volatile long delta;

void updateEncodersISR(void){
  static long prev_time = 0;
  delta = micros() - prev_time;
  prev_time = micros();

  encoder_a.tick();
  encoder_b.tick();
}

void setup(){
  // Hardware components setup
  motor_a.init();
  motor_b.init();

  encoder_a.init();
  encoder_b.init();

  // PID setup
  pid.p_gain = 1.0;
  pid.i_gain = 0.0;
  pid.d_gain = 0.0;

  // Serial init
  Serial.begin(9600);
  while(!Serial);

  // Interrupt init
  ITimer2.init();
  ITimer2.attachInterruptInterval(UPDATE_TIME, updateEncodersISR);
}

void loop(){
  if(Serial.available()){
    target = Serial.readStringUntil('\n').toFloat();
  }

  // collect the readings
  int reading_a = encoder_a.reading;
  int reading_b = encoder_b.reading;

  // convert from ticks to angles
  float angle_a = (float)reading_a / (30 * 4) * 360;
  float angle_b = (float)reading_b / (30 * 4) * 360;

  // shove it all in PID
  float res = pid.calculate(target, angle_a);
  
  // perform correction
  motor_a.setPower(-res);

  Serial.print("R1:");
  Serial.print(angle_a);
  Serial.print(",");
  Serial.print("R2:");
  Serial.print(angle_b);
  Serial.print(",");
  Serial.print("target:");
  Serial.print(target);
  Serial.print(",");
  Serial.print("response:");
  Serial.print(res);
  Serial.println();
}
