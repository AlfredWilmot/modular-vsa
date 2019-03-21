#include <Arduino.h>

/* Fcn for controlling motor given joystick input */
void move_motor(int duty, int IN_A, int IN_B, int EN)
{
  /* Joystick is in resting position when analogRead(joystick_x_pin) equals ~ 512 +/- 5
      If below this value, go bkwrds (IN_1 = LO ; IN_2 = HI).
      If above this value, go fwds   (IN_1 = HI ; IN_2 = LO).
      Else, turn PWM off gently      (EN_A = LO).
  */
  int joy_rest      = 0;  // Value of joystick when untouched.
  int thresh        = 0.1;    // Hysteresis around joystick resting value
  int thresh_lower  = joy_rest - thresh;
  int thresh_upper  = joy_rest + thresh;


  
  if(duty >= thresh_upper)
  {
    /* Go fwd */
    digitalWrite(IN_A, HIGH);
    digitalWrite(IN_B, LOW);

    /* Map range [thresh_upper, 1023] to [0, 255] */
    duty = map(duty, thresh_upper, 1023, 0, 255);
    analogWrite(EN, duty);

  }
  else if(duty <= thresh_lower)
  {
    /* Go bkwds */
    digitalWrite(IN_A, LOW);
    digitalWrite(IN_B, HIGH);

    /* Map range [0, thresh_lower] to [255, 0] */
    duty = map(duty, 0, thresh_lower, 255, 0);
    analogWrite(EN, duty);

  }
  /* Gentle stop => PWM_duty = 0% */
  else
  {
    analogWrite(EN, 0);
  }
  
}


  /// Arduino Micro resources...
  //  PWM duty mapping : https://www.theengineeringprojects.com/2017/03/use-arduino-pwm-pins.html
  //  pin-out:           https://www.arduino.cc/en/uploads/Main/ArduinoMicro_Pinout3.png


/* L298N control pins (Set these LO to perform free-running stop) */
const int EN_A = 13;  // (pwm) 
const int EN_B = 11;  // (pwm)


/* Pin pairs define motor rotation direction (Set pairs equal, if corresponding EN is HI, to perform hard-stop) */
const int IN_1 = 12;
const int IN_2 = A0;
const int IN_3 = A1;
const int IN_4 = A2;

const int EN_A_DUTY = 0;
const int EN_B_DUTY = 0;

void setup() {

  // PWM pins
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  // Digital pins
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);


  
}

void loop() {

  /* Need to add joystick subscriber code, and pass value as duty to move_motor fcn*/
  int duty_from_joystick_roll = 500;
  int duty_from_joystick_yaw  = 500;
  
  /* Main control loop */
  while (1)
  {

    /* Move motors according to joystick position (Antagonist pairs simply move in opposite direction for now) */
    move_motor(duty_from_joystick_roll, IN_1, IN_2, EN_A);
    move_motor(duty_from_joystick_yaw , IN_3, IN_4, EN_B);

    _delay_ms(20);
  }
  
}
