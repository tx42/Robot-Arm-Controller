#include <Servo.h>

#define USE_TIMER_2 true
#define SAMPLE_FREQUENCY 2000
#include <TimerInterrupt.h>

#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "button.h"

#define printval(name, val) Serial.print(name ":"); Serial.print(val); Serial.print(',')

#define CLUTCH_THRESHOLD 5.0

/*** HARDWARE CONSTANTS ***/
#define MOTOR_A_PWR 8
#define MOTOR_A_FWD 24
#define MOTOR_A_REV 25

#define MOTOR_B_PWR 7
#define MOTOR_B_FWD 23
#define MOTOR_B_REV 22

#define SENSOR_A1 2
#define SENSOR_A2 3

#define SENSOR_B1 5
#define SENSOR_B2 4

#define ANGLE_SENSOR A0
#define P_SENSOR A1
#define I_SENSOR A2
#define D_SENSOR A3

#define ARM_SERVO_PIN 11
#define LED_PIN 26
#define SELECT_BTN_PIN 27
#define PID_SUBMIT_BTN_PIN 29
#define SERVO_BTN_PIN 31

Servo arm_servo;

Motor motor_a(MOTOR_A_PWR, MOTOR_A_FWD, MOTOR_A_REV);
Motor motor_b(MOTOR_B_PWR, MOTOR_B_FWD, MOTOR_B_REV);

Encoder encoder_a(SENSOR_A1, SENSOR_A2);
Encoder encoder_b(SENSOR_B1, SENSOR_B2);

PIDController pid_a;
PIDController pid_b;

Button motor_select_btn(SELECT_BTN_PIN);
Button pid_submit_btn(PID_SUBMIT_BTN_PIN);
Button arm_ctrl_btn(SERVO_BTN_PIN);

float target_a;
float target_b;
bool motor_select;

void updateEncodersISR(void){
  encoder_a.tick();
  encoder_b.tick();
}

void setup(){
  // Hardware components setup
  motor_a.init();
  motor_b.init();

  encoder_a.init();
  encoder_b.init();

  pinMode(ANGLE_SENSOR, INPUT);
  pinMode(P_SENSOR, INPUT);
  pinMode(I_SENSOR, INPUT);
  pinMode(D_SENSOR, INPUT);

  pinMode(LED_PIN, OUTPUT);
  motor_select_btn.init();
  pid_submit_btn.init();
  arm_ctrl_btn.init();

  // Arm setup
  arm_servo.attach(ARM_SERVO_PIN);

  // PID default values
  pid_a.p_gain = 1.0;
  pid_a.i_gain = 0.1;
  pid_a.d_gain = 2.0;

  pid_b.p_gain = 30.5;
  pid_b.i_gain = 0.10;
  pid_b.d_gain = 7.28;

  // Serial init
  Serial.begin(115200);
  while(!Serial);

  // Interrupt init
  ITimer2.init();
  ITimer2.attachInterrupt(SAMPLE_FREQUENCY, updateEncodersISR);
}

bool arm_opened;
bool clutch;

void loop(){
  // manage arm
  switch(arm_ctrl_btn.getButtonState()){
  case DOWN:
    arm_servo.write(255);
    break;
  case UP:
    arm_servo.write(0);  
    break;
  }

  // manage motor selection
  digitalWrite(LED_PIN, motor_select);

  if(motor_select_btn.getButtonState() == DOWN){
    // release clutch if switching motors
    clutch = false;
    motor_select = !motor_select;
  }

  // update PID values
  float p_gain = ((float) analogRead(P_SENSOR)) / 1023.0 * 50.0;
  float i_gain = ((float) analogRead(I_SENSOR)) / 1023.0 * 10.0;
  float d_gain = ((float) analogRead(D_SENSOR)) / 1023.0 * 50.0;

  // update values only if submit button was pressed
  PIDController& target_pid = motor_select ? pid_a : pid_b;
  if(pid_submit_btn.getButtonState() == DOWN){
    target_pid.p_gain = p_gain;
    target_pid.i_gain = i_gain;
    target_pid.d_gain = d_gain;
  }

  // get new target
  float knob_input = ((float) analogRead(ANGLE_SENSOR) / 1023.0);
  float new_target;

  // setting the limits for motors
  if(motor_select){
    new_target = knob_input * 720.0;
  }else{
    new_target = knob_input * 180.0;
  }

  float& cur_target_ref = motor_select ? target_a : target_b;

  if(abs(cur_target_ref - new_target) < CLUTCH_THRESHOLD){
    // if the target is close to cur target then clutch
    clutch = true;
  }

  if(clutch){
    // only follow new target if clutched
    cur_target_ref = new_target;
  }

  // collect encoder readings
  int reading_a = encoder_a.reading;
  int reading_b = encoder_b.reading;

  // convert from ticks to angles
  // why /530? calibration factor. Dividing by it gave us ~ ok angles 
  float angle_a = (float)reading_a / (30 * 4) * 360 / 530 * 180;
  float angle_b = (float)reading_b / (30 * 4) * 360 / 530 * 180;

  // shove it all in PID
  float res_a = pid_a.calculate(target_a, angle_a);
  float res_b = pid_b.calculate(target_b, angle_b);
  
  // perform correction
  motor_a.setPower(res_a);
  motor_b.setPower(res_b);

  // statistics printing
  // basic parameters
  printval("R1", angle_a);
  printval("R2", angle_b);
  printval("t1", target_a);
  printval("t2", target_b);

  // pid responses
  printval("res1", res_a);
  printval("res2", res_b);

  // pid values
  printval("cur_p", p_gain);
  printval("cur_i", i_gain);
  printval("cur_d", d_gain);

  // pid for motor a
  printval("p1", pid_a.p_gain);
  printval("i1", pid_a.i_gain);
  printval("d1", pid_a.d_gain);

  // pid for motor b
  printval("p2", pid_b.p_gain);
  printval("i2", pid_b.i_gain);
  Serial.print("d2:");
  Serial.print(pid_b.d_gain);

  Serial.println();
}
