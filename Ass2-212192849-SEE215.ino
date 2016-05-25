/* Author: Ayush Kumar 212192849 (2014)
   SUBJECT: SEE215 - Microcontroller Principles, Assignment 2 (Maze solving and line following autonomous vehicle)

   This project will follow a black line around the maze. When it hits a cross intersection (+) the robot will always aim to turn left.
   When it hits a dead end the robot will complete a U-turn and continue the maze. Whenever the maze turn right at a right angle the robot
   will also turn right at a right angle. Whenever the maze turns left at a right angle the robot will turn right at a right angle.
   If the robot has the options to turn right or left, it will take left as number one priority. When the robot get to the end of the maze
   and reaches a state where there is a black square the robot will stop and will not be able to complete any other movements unless
   the Arduino Mega 2560 is reset.
*/

#include <util/delay.h>
#define MAXSPEED 900

//initialise a variable used for PWM
long dutyCycle = 0;
//initialise boolean varaibles
bool count = true;
bool robotMove = true;
bool turningleft = false;
//initialise integers used to define the sensor pins
int left_sensor_middle = 0;
int right_sensor_middle = 0;
int middle_sensor = 0;
int left_sensor = 0;
int right_sensor = 0;





void setup() {
  DDRE = (1 << DDE3) | (1 << DDE5); // set right motor as output (PIN 5 , PIN 3)
  DDRH = (1 << DDH4) | (1 << DDH5); // set left motor as output (PIN 7, PIN 8 )
  DDRF = (1 << DDF7) | (1 << DDF6) | (1 << DDF5); // set PIN A7, PIN A6, PIN A5 as outputs for LED's



  //initialise Timer/Counter 4 B and C used for left motor to turn it forward and backward
  TCCR4A = (1 << COM4C1) | (1 << WGM41) | (1 << COM4B1); // fast PWM mode 14, output compare mode 2 for channel A and C
  TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS42); //Fast PWM mode and prescaler of 256
  TCCR4C = 0b00000000;
  TIMSK4 = 0b00000000; //no interuptt set for timer and counter
  OCR4CH = 0 >> 8; //high byte of 0
  OCR4CL = 0 & 0xFF; //low byte of 0
  OCR4AH = 0 >> 8; //high byte of 0
  OCR4AL = 0 & 0xFF; //low byte of 0
  ICR4H = 1250 >> 8; //high byte of 1250
  ICR4L = 1250 & 0xFF; // low byte of 1250
  TCNT4H = 0x00; //high byte of counting register 4
  TCNT4L = 0x00; // low byte of counting register 4

  //initialise Timer/Counter 3 A and C used for left motor to turn it forward and backward
  TCCR3A = (1 << COM3A1) | (1 << WGM41) | (1 << COM3C1); // fast PWM mode 14, output compare mode 2 for channel A and C
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS32); //Fast PWM mode and prescaler of 256
  TCCR3C = 0b00000000;
  TIMSK3 = 0b00000000; //no interupt set for timer and counter
  OCR3AH = 0 >> 8; //high byte of 0
  OCR3AL = 0 & 0xFF; //low byte of 0
  OCR3CH = 0 >> 8; //high byte of 0
  OCR3CL = 0 & 0xFF; //low byte of 0
  ICR3H = 1250 >> 8; //high byte of 1250
  ICR3L = 1250 & 0xFF; // low byte of 1250
  TCNT3H = 0x00; //high byte of counter register 3
  TCNT3L = 0x00; // low byte of counter register 3

  //enable interrupts
  EICRA = 0b00001000; //set interupt one to activate on a falling edge
  EIMSK = 0b00000010; //activate interupt 1
  sei(); // activate global interrupt

}

//This function will be used to operate the motors. 4 parameters are passed, each for a different count which will control the speed on the motors.
static void runMotors(int rightMotorB, int rightMotorF, int leftMotorB, int leftMotorF)
{
  //left motor backwards
  OCR4CH = leftMotorB >> 8;
  OCR4CL = leftMotorB & 0xFF;
  //left motor forward
  OCR4BH = leftMotorF >> 8;
  OCR4BL = leftMotorF & 0xFF;
  //right motor backwards
  OCR3AH = rightMotorB >> 8;
  OCR3AL = rightMotorB & 0xFF;
  //right Motor forwards
  OCR3CH = rightMotorF >> 8;
  OCR3CL = rightMotorF & 0xFF;


}

//The function will be called as soon as power is given to the board and is linked to the bottom 3 middle sensors. Whenever a sensor is HIGH it will cause the LED to go HIGH.
//This will be used so the robot is centred before it begins.
static void configure()
{
  //turn all LED off
  PORTF &= 0b00000000;
  //If left sensor is not on the line turn on left LED
  if ((PINC & 0b00100000))
  {
    PORTF |= 0b00100000;
  }
  //If middle sensor is not on the line turn on middle LED
  if ((PINB & 0b00001000))
  {
    PORTF |= 0b01000000;
  }
  //If right sensor is not on the line turn on right LED
  if ((PINA & 0b00000001))
  {
    PORTF |= 0b10000000;
  }
  //This will be false only when the pushbutton is pressed causing the robot to start moving and tracking the line.
  if (PINK & 0b00000001)
  {}
  else
  {
    count = false;
  }

}


void loop()
{
  //Wait for the push button to be pressed after the centre of the line has been found.
  while (count == true)
  {
    configure();
  }
  //while this variable is true check the sensors. This will only go false when the end of the maze has been found, forcing the code to skip all the logic.
  while (robotMove)
  {
    //re-initialise varaibles
    left_sensor_middle = PINC & 0b00100000;
    right_sensor_middle = PINA & 0b00000001;
    middle_sensor = PINB & 0b00001000;
    left_sensor = PIND & 0b00000010;
    right_sensor = PIND & 0b00000100;

    if ((left_sensor_middle) && (!(middle_sensor)) && (right_sensor_middle)) // check if left and right sensor are off the line, go straight at max speed
    {
      runMotors(0, MAXSPEED, 0, MAXSPEED);
    }
    else if ((!(left_sensor_middle)) && (middle_sensor) && (right_sensor_middle)) // check if only left sensor is on the line, turn left alot
    {
      runMotors(0, 0, 0, (MAXSPEED - 300));
    }
    else if ((!(left_sensor_middle)) && (!(middle_sensor)) && (right_sensor_middle)) // check if the left and middle sensor are on the line. turn left a little bit
    {
      runMotors(0, (MAXSPEED - 500), 0, (MAXSPEED - 300));

    }
    else if ((left_sensor_middle) && (middle_sensor) && ((!right_sensor_middle))) // check if only right sensor is on the line, turn right alot.
    {
      runMotors(0, (MAXSPEED - 300), 0, 0);
    }
    else if ((left_sensor_middle) && (!(middle_sensor)) && ((!right_sensor_middle))) // check if the right and middle sensor is on the line, turn right a little bit.
    {
      runMotors(0, (MAXSPEED - 300), 0, (MAXSPEED - 500));
    }
    else if ((!(left_sensor)) && (!(left_sensor_middle)) && (!(middle_sensor)) && ((!right_sensor_middle)) && (!(right_sensor))) // if all 5 sensors are on a line, that means the robot has reached the end so turn the robot off and turn on all LE. (If ther is time link it to a buzzer)
    {
      runMotors(0, 0, 0, 0);
      robotMove = false;
      PORTF = 0b11100000;
    }

    else //if no lines are detected start turning left, and this will also incoperate u-turns.
    {
      runMotors(0, (MAXSPEED - 150), (MAXSPEED - 150), 0);
    }


    while (turningleft)
    {
      runMotors(0, 0, 0, 700);
      _delay_ms(450);
      if ((!(PINB & 0b00001000)))

      {
        turningleft = false;
        break;
      }
      else
      {
        runMotors(0, 0, 0, (MAXSPEED - 250));
      }

    }


  }

}

//Interupt will be triggered when left sensor hits the line and becomes LOW
ISR(INT1_vect)
{

  turningleft = true;
}

