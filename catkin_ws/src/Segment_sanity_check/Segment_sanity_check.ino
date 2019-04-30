/* A simple sanity-check script that makes sure that the motors on a single segment connected to the base are working properly with the L293D motor driver ICs.*/
/* Slightly moves each motor in either direction for a brief period of time, and then back again over the same period. */



/*******************************************************/
/* Define digital IO pins being used on Arduino mirco */
/*****************************************************/


// Motor direction pins 

const int proximal_motor_1a = 5;
const int proximal_motor_1b = 6;
const int proximal_motor_2a = 9;
const int proximal_motor_2b = 10;

const int distal_motor_1a = 7;
const int distal_motor_1b = 8;
const int distal_motor_2a = 12;
const int distal_motor_2b = 11;


// I-sense ADC pins 
const int I_sense_1 = A3;
const int I_sense_2 = A2;
const int I_sense_3 = A1;
const int I_sense_4 = A0;


void setup() 
{
}

/************************************/
/* HARDWARE SANITY CHECK FUNCTIONS */
/**********************************/

/* Sanity check of individual motor Open-Loop (OL) control */
void move_single_motor(int pin_A, int pin_B, int _ms)
{
  digitalWrite(pin_A, LOW);
  digitalWrite(pin_B, HIGH);
  delay(_ms);
  
  digitalWrite(pin_A, HIGH);
  digitalWrite(pin_B, LOW);
  delay(_ms);

  digitalWrite(pin_B, LOW);
  digitalWrite(pin_A, LOW);
}

/* Sanity check of antagonist-motor-pair OL control */
void move_antagonist_pair(int pin_A1, int pin_B1, int pin_A2, int pin_B2, int _ms)
{
  /* Move antagonist pair in positive dir */
  digitalWrite(pin_A1, LOW);
  digitalWrite(pin_B1, HIGH);
  digitalWrite(pin_A2, LOW);
  digitalWrite(pin_B2, HIGH);
  
  delay(_ms);

  /* Move antagonist pair in negative dir */
  digitalWrite(pin_A1, HIGH);
  digitalWrite(pin_B1, LOW);
  digitalWrite(pin_A2, HIGH);
  digitalWrite(pin_B2, LOW);
  
  delay(_ms);

  /* Stop all motors */
  digitalWrite(pin_A1, LOW);
  digitalWrite(pin_B1, LOW);
  digitalWrite(pin_A2, LOW);
  digitalWrite(pin_B2, LOW); 
}



/*****************************************/
/* Initialization and main control loop */
/***************************************/

void loop() 
{
  // Digital pins
  pinMode(proximal_motor_1a, OUTPUT);
  pinMode(proximal_motor_1b, OUTPUT);
  pinMode(proximal_motor_2a, OUTPUT);
  pinMode(proximal_motor_2b, OUTPUT);

  pinMode(distal_motor_1a, OUTPUT);
  pinMode(distal_motor_1b, OUTPUT);
  pinMode(distal_motor_2a, OUTPUT);
  pinMode(distal_motor_2b, OUTPUT);
  

  // Set all digital pin outputs to LOW to start with
  digitalWrite(proximal_motor_1a, LOW);
  digitalWrite(proximal_motor_1b, LOW);
  digitalWrite(proximal_motor_2a, LOW);
  digitalWrite(proximal_motor_2b, LOW);

  digitalWrite(distal_motor_1a, LOW);
  digitalWrite(distal_motor_1b, LOW);
  digitalWrite(distal_motor_2a, LOW);
  digitalWrite(distal_motor_2b, LOW);

  

  while(1)
  {
    /* Slightly move each individual motor in either direction in OL fashion.*/
//    move_single_motor(proximal_motor_1a,  proximal_motor_1b,  1000);
//    move_single_motor(proximal_motor_2a,  proximal_motor_2b,  1000);
//    move_single_motor(distal_motor_1a,    distal_motor_1b,    1000);
//    move_single_motor(distal_motor_2a,    distal_motor_2b,    1000);

    /* Slightly move each antagonist motor pair in OL fashion .*/
//    move_antagonist_pair(proximal_motor_1a,  proximal_motor_1b,  proximal_motor_2a, proximal_motor_2b, 500);
//    move_antagonist_pair(distal_motor_1a,  distal_motor_1b,  distal_motor_2a, distal_motor_2b, 500);
//    delay(500);
  }


  
}

