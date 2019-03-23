#include <Arduino.h>

#define USE_USBCON 

#include <ros.h>
#include <sensor_msgs/Joy.h>



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


/* Joystick subscriber callback for controlling motors of one joint */
void test_proximal_joint(const sensor_msgs::Joy& msg)
{
    bool A_btn  = msg.buttons[0];

    bool B_btn  = msg.buttons[1];
    bool X_btn  = msg.buttons[2];

    bool LB_btn = msg.buttons[4];
    bool RB_btn = msg.buttons[5];

    int duty = 100; //duty: 0,255

    //Right motor rotates:  CW if A is pressed or if B is pressed:                 A || B
    //                      CCW if A && RB are pressed or if B && RB are pressed:  RB && (A || B)

    //Left motor roates:    CW if A is pressed or if X is pressed:                 A || X
    //                      CCW if A && LB are pressed or if X && LB are pressed:  LB && (A || X)



    /* CONTROL RIGHT MOTOR */
    if(A_btn || B_btn)
    {
      if(RB_btn)
      {
        // FWD?
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);
      }
      else
      {
        //BKWD?
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);
      }

      analogWrite(EN_A, duty);
    }
    else
    {
        /* STOP */
        analogWrite(EN_A, 0);
    }

    /* CONTROL LEFT MOTOR */
    if(A_btn || X_btn)
    {

      if(LB_btn)
      {
        // FWD?
        digitalWrite(IN_3, HIGH);
        digitalWrite(IN_4, LOW);
      }
      else
      {
        //BKWD?
        digitalWrite(IN_3, LOW);
        digitalWrite(IN_4, HIGH);
      }

      analogWrite(EN_B, duty);
    }
    else
    {
        /* STOP */
        analogWrite(EN_B, 0);
    }
}



ros::NodeHandle nh;
ros::Subscriber<sensor_msgs::Joy> sub("joy", &test_proximal_joint);

void setup() {

  // PWM pins
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  // Digital pins
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  Serial.begin(57600);

}

void loop() {

  nh.spinOnce();
  delay(10);

}
