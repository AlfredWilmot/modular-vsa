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
  Serial.begin(115200);

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
  int max_sample_cycles = 10;
  int sample_cycles = max_sample_cycles;
  
  /* Move antagonist pair in positive dir */
  digitalWrite(pin_A1, LOW);
  digitalWrite(pin_B1, HIGH);
  digitalWrite(pin_A2, LOW);
  digitalWrite(pin_B2, HIGH);
  
  while(sample_cycles > 0)
  {
    Serial.print(analogRead(I_sense_1));
    Serial.print(" ");
    Serial.print(analogRead(I_sense_2));
    Serial.print(" ");
    Serial.print(analogRead(I_sense_3));
    Serial.print(" ");
    Serial.println(analogRead(I_sense_4));
    
    delay(_ms/max_sample_cycles);
    sample_cycles--;
  }
  sample_cycles = max_sample_cycles;
 

  /* Move antagonist pair in negative dir */
  digitalWrite(pin_A1, HIGH);
  digitalWrite(pin_B1, LOW);
  digitalWrite(pin_A2, HIGH);
  digitalWrite(pin_B2, LOW);
  
  while(sample_cycles > 0)
  {
    Serial.print(analogRead(I_sense_1));
    Serial.print(" ");
    Serial.print(analogRead(I_sense_2));
    Serial.print(" ");
    Serial.print(analogRead(I_sense_3));
    Serial.print(" ");
    Serial.println(analogRead(I_sense_4));
    
    delay(_ms/max_sample_cycles);
    sample_cycles--;
  }
  sample_cycles = max_sample_cycles;

  /* Stop all motors */
  digitalWrite(pin_A1, LOW);
  digitalWrite(pin_B1, LOW);
  digitalWrite(pin_A2, LOW);
  digitalWrite(pin_B2, LOW); 
}

/* Sanity check of I-sense PCBs: move single motor and display I-sense-output at rate of 10Hz. and stop if threshold is exceeded */
void check_single_I_sense(int pin_A, int pin_B, int _ms)
{
    digitalWrite(pin_A, HIGH);
    digitalWrite(pin_B, LOW);
    
    int max_cycles = 100;
    int cycle_count = max_cycles;
    
    while(max_cycles > 0)
    {
      Serial.println(analogRead(I_sense_1));
      delay(_ms/max_cycles);
      cycle_count--;
      
    }
  
    digitalWrite(pin_A, LOW);
    digitalWrite(pin_B, LOW);
    delay(_ms/2);
}



/**********************/
/* Main control loop */
/********************/

void loop() 
{

  check_single_I_sense(proximal_motor_1a, proximal_motor_1b, 1000);
 
    /* Slightly move each individual motor in either direction in OL fashion.*/
//    move_single_motor(proximal_motor_1a,  proximal_motor_1b,  1000);
//    move_single_motor(proximal_motor_2a,  proximal_motor_2b,  1000);
//    move_single_motor(distal_motor_1a,    distal_motor_1b,    1000);
//    move_single_motor(distal_motor_2a,    distal_motor_2b,    1000);

    /* Slightly move each antagonist motor pair in OL fashion .*/
//    move_antagonist_pair(proximal_motor_1a,  proximal_motor_1b,  proximal_motor_2a, proximal_motor_2b, 1000);
//    move_antagonist_pair(distal_motor_1a,  distal_motor_1b,  distal_motor_2a, distal_motor_2b, 1000);
//    delay(500);

//    check_single_I_sense(proximal_motor_1a, proximal_motor_1b);
//    Serial.println(analogRead(I_sense_1));


  


  
}

