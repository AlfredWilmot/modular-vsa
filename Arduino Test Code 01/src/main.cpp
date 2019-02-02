#include <Arduino.h>

/* Fcn for controlling motor given joystick input */
void move_motor(int duty, int IN_A, int IN_B, int EN)
{
  /* Joystick is in resting position when analogRead(joystick_x_pin) equals ~ 512 +/- 5
      If below this value, go bkwrds (IN_1 = LO ; IN_2 = HI).
      If above this value, go fwds   (IN_1 = HI ; IN_2 = LO).
      Else, turn PWM off gently      (EN_A = LO).
  */
  int joy_rest      = 512;  // Value of joystick when untouched.
  int thresh        = 20;    // Hysteresis around joystick resting value
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

    /* Map range [0, thresh_lower] to [0, 255] */
    duty = map(duty, 0, thresh_lower, 255, 0);
    analogWrite(EN, duty);

  }
  /* Gentle stop => PWM_duty = 0% */
  else
  {
    analogWrite(EN, 0);
  }
  
}

void test_joy_stick()
{

  // Arduino UNO resources...
  // PWM duty mapping : https://www.theengineeringprojects.com/2017/03/use-arduino-pwm-pins.html
  // pin-out:           https://www.circuito.io/blog/arduino-uno-pinout/

  /* Allocating Joystick pins */
  int joystick_pitch_pin = 14;
  int joystick_yaw_pin   = 15;
  int joystick_SW_pin    = 2;

  pinMode(joystick_pitch_pin, INPUT);
  pinMode(joystick_yaw_pin,   INPUT);
  pinMode(joystick_SW_pin,    INPUT);

  /* L298N control pins (Set these LO to perform free-running stop) */
  int EN_A = 11;  // (pwm) 
  int EN_B = 10;  // (pwm)

  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  /* Pin pairs define motor rotation direction (Set pairs equal, if corresponding EN is HI, to perform hard-stop) */
  int IN_1 = 9;
  int IN_2 = 8;
  int IN_3 = 7;
  int IN_4 = 6;

  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  int EN_A_DUTY = 0;
  int EN_B_DUTY = 0;

  /* Main control loop */
  while (1)
  {

    /* Move motors according to joystick position (Antagonist pairs simply move in opposite direction for now) */
    move_motor(analogRead(joystick_yaw_pin),    IN_1, IN_2, EN_A);
    move_motor(analogRead(joystick_pitch_pin),  IN_3, IN_4, EN_B);

    _delay_ms(20);
  }
}


void setup() {
/* This is required for arduino sketch to work */
}

void loop() {
  test_joy_stick();
}