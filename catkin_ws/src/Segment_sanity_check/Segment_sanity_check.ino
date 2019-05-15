/* A simple sanity-check script that makes sure that the motors on a single segment connected to the base are working properly with the L293D motor driver ICs.*/
/* Slightly moves each motor in either direction for a brief period of time, and then back again over the same period. */



/*******************************************************/
/* Define digital IO pins being used on Arduino mirco */
/*****************************************************/


// Motor direction pins 

// Right-hand-side motor: tilt distal joitn left [5= HIGH, 6 = LOW]
const int distal_right_motor_pinA = 5; 
const int distal_right_motor_pinB = 6;

// Left-hand motor: tilt distal joint left [7= HIGH, 8= LOW]
const int distal_left_motor_pinA = 7; 
const int distal_left_motor_pinB = 8;


// bottom motor: tilt proximal joint up [9=HIGH, 10=LOW]
const int proximal_bottom_motor_pinA = 9; 
const int proximal_bottom_motor_pinB = 10;

// top motor: tilt proximal joint up [9=HIGH, 10=LOW]
const int proximal_top_motor_pinA = 12; 
const int proximal_top_motor_pinB = 11;


void setup() 
{
  // Digital pins
  pinMode(distal_left_motor_pinA, OUTPUT);
  pinMode(distal_left_motor_pinB, OUTPUT);
  pinMode(distal_right_motor_pinA, OUTPUT);
  pinMode(distal_right_motor_pinB, OUTPUT);
  pinMode(proximal_bottom_motor_pinA, OUTPUT);
  pinMode(proximal_bottom_motor_pinB, OUTPUT);
  pinMode(proximal_top_motor_pinA, OUTPUT);
  pinMode(proximal_top_motor_pinB, OUTPUT);

  // Set all digital pin outputs to LOW to start with
  pinMode(distal_left_motor_pinA, OUTPUT);
  pinMode(distal_left_motor_pinB, OUTPUT);
  pinMode(distal_right_motor_pinA, OUTPUT);
  pinMode(distal_right_motor_pinB, OUTPUT);
  pinMode(proximal_top_motor_pinA, OUTPUT);
  pinMode(proximal_top_motor_pinB, OUTPUT);


}

/**********************/
/* Main control loop */
/********************/

void loop() 
{
  digitalWrite(proximal_top_motor_pinA, HIGH);
  digitalWrite(proximal_top_motor_pinB, LOW);
  digitalWrite(proximal_bottom_motor_pinA, HIGH);
  digitalWrite(proximal_bottom_motor_pinB, LOW);

  digitalWrite(distal_right_motor_pinA, HIGH);
  digitalWrite(distal_right_motor_pinB, LOW);
  digitalWrite(distal_left_motor_pinA, HIGH);
  digitalWrite(distal_left_motor_pinB, LOW);


  delay(500);
//
  digitalWrite(proximal_top_motor_pinA, LOW);
  digitalWrite(proximal_top_motor_pinB, HIGH);
  digitalWrite(proximal_bottom_motor_pinA, LOW);
  digitalWrite(proximal_bottom_motor_pinB, HIGH);;

  digitalWrite(distal_right_motor_pinA, LOW);
  digitalWrite(distal_right_motor_pinB, HIGH);
  digitalWrite(distal_left_motor_pinA, LOW);
  digitalWrite(distal_left_motor_pinB, HIGH);



  delay(500);


  
}

