/*** HARDWARE CONSTANTS ***/
#define SENSOR_1 2
#define SENSOR_2 3

#define INPUT_KNOB A0
#define REVERSE_PIN 4

#define MAX_RPS 400

const char enc_patterns_array[4] = {
  0b00,
  0b01,
  0b11,
  0b10
};

int enc_idx = 0;
unsigned long last_time = 0;

void setup(){
  pinMode(SENSOR_1, OUTPUT);
  pinMode(SENSOR_2, OUTPUT);
  
  pinMode(INPUT_KNOB, INPUT);
  pinMode(REVERSE_PIN, INPUT);
  // Serial.begin(9600);
}

void loop(){
  float speed = analogRead(INPUT_KNOB);
  speed /= 1023.0;
  if(digitalRead(REVERSE_PIN)){
    speed *= -1.0;
  }

  float rps = speed * MAX_RPS;
  if(rps == 0){
    return;
  }

  unsigned long delay_us = abs((1 / rps) * 1000000);

  unsigned long delta = micros() - last_time;
  if(delta > delay_us){
    last_time = micros();

    // emulate new tick
    if(rps > 0){
      enc_idx++;
    }else{
      enc_idx--;
    }

    if(enc_idx < 0){
      enc_idx = 3;
    }
    enc_idx = enc_idx % 4;
    int pattern = enc_patterns_array[enc_idx];
    digitalWrite(SENSOR_1, pattern & (1 << 0));
    digitalWrite(SENSOR_2, pattern & (1 << 1));
  }
}
