/*Example sketch to control a stepper motor with A4988 stepper motor driver, AccelStepper library and Arduino: continuous rotation. More info: https://www.makerguides.com */
// Include the AccelStepper library:
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#include "pinDefinitions.h"

#define dirPin1 8
#define stepPin1 7
#define dirPin2 6
#define stepPin2 5
#define doubleStepPin 9
#define quadStepPin 10

#define dirPin1Gpio 21
#define stepPin1Gpio 23
#define dirPin2Gpio 14
#define stepPin2Gpio 13
#define doubleStepPinGpio 23
#define quadStepPinGpio 2

String command;

uint32_t p_priority;

volatile uint16_t stepComp;
volatile uint16_t remain;
volatile uint16_t speed1, speed2;
volatile int nextMotor;

uint16_t remainNorm;
long speed1Norm, speed2Norm;
int nextMotorNorm;

bool stepTimerOn = false;

void setSpeed(double s1, double s2) {
  if (s1 != 0.0 and s2 == 0.0) {
    s2 = 0.014 * s1 / abs(s1);
  };

  if (s1 == 0.0 and s2 != 0.0) {
    s1 = 0.014 * s2 / abs(s2);
  };
  
  if (s1 != 0.0 and s2 != 0.0) {
    
    if (s1 > 0) {
      digitalWrite(dirPin1, LOW);
    };
    
    if (s1 < 0) {
      digitalWrite(dirPin1, HIGH);
      s1 = abs(s1);
    };
    
    if (s2 > 0) {
      digitalWrite(dirPin2, LOW);
    };
    
    if (s2 < 0) {
      digitalWrite(dirPin2, HIGH);
      s2 = abs(s2);
    };
    
    speed1Norm = constrain(long(981.746875 / s1 * 2.0), 70, 65535); //Unit convertion
    speed2Norm = constrain(long(981.746875 / s2 * 2.0), 70, 65535); //Unit convertion
    
    if (speed1Norm < speed2Norm) {
      nextMotorNorm = 1;
      if (stepTimerOn == false) {
        NRF_TIMER2->CC[0] = speed1Norm;
        remain = speed2Norm;
      };
    }

    else if (speed2Norm < speed1Norm) {
      nextMotorNorm = 2;
      if (stepTimerOn == false) {
        NRF_TIMER2->CC[0] = speed2Norm;
        remain = speed1Norm;
      };
    }
    
    else {
      nextMotorNorm = 0;
      if (stepTimerOn == false) {
        NRF_TIMER2->CC[0] = speed1Norm;
        remain = speed1Norm;
      };
    };

    NRF_TIMER2->TASKS_START = 1;
    stepTimerOn = true;
    nextMotor = nextMotorNorm;
    speed1 = speed1Norm;
    speed2 = speed2Norm;
  }

  else if (s1 == 0 and s2 == 0) {
      NRF_TIMER2->TASKS_STOP = 1; //TIMSK1 = 0;
      stepTimerOn = false;
  };
};

void setup() {
  //set step pin to be output
  NRF_P0->PIN_CNF[stepPin1Gpio] = 1 << 0;
  NRF_P0->PIN_CNF[dirPin1Gpio] = 1 << 0;
  //pinMode(dirPin1, OUTPUT);
  //pinMode(stepPin1, OUTPUT);
  NRF_P1->PIN_CNF[stepPin2Gpio] = 1 << 0;
  NRF_P1->PIN_CNF[dirPin2Gpio] = 1 << 0;
  NRF_P0->PIN_CNF[doubleStepPinGpio] = 1 << 0;
  //NRF_P0->OUTSET |= 1 << doubleStepPinGpio;
  NRF_P1->OUTSET |= 1 << quadStepPinGpio;

  NRF_TIMER2->TASKS_STOP = 1; //stop timer
  NRF_TIMER2->BITMODE = 0; //16-bit timer
  NRF_TIMER2->MODE = 0; //timer mode
  NRF_TIMER2->PRESCALER = 8;//8; //prescaler of 256
  NRF_TIMER2->TASKS_CLEAR = 1; //clear timer
  NRF_TIMER2->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos; //enable interrupt on CC[0]

  NVIC_EnableIRQ(TIMER2_IRQn);
  
  Serial.begin(115200);
  while (true) {
    if (Serial.available()) {
      command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "go") {
        break;
      };
    };
  };

  Serial.println(NVIC_GetPriority(TIMER2_IRQn));
  NVIC_SetPriority(TIMER2_IRQn, 1);
  Serial.println(NVIC_GetPriority(TIMER2_IRQn));

  Serial.println("end");

  setSpeed(6.28, -6.28);
};

void loop () {
}
extern "C"
{
    void TIMER2_IRQHandler_v() {
      if (NRF_TIMER2->EVENTS_COMPARE[0] == 1) {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;
        NRF_TIMER2->TASKS_STOP = 1;
        switch (nextMotor) {
          case 1:
            NRF_P0->OUTSET |= 1 << stepPin1Gpio; //write HIGH
            NRF_P0->OUTCLR |= 1 << stepPin1Gpio; //write LOW
            //digitalWrite(stepPin1, HIGH);
            //digitalWrite(stepPin1, LOW);

            if (remain < speed1) {
              nextMotor = 2;
              stepComp = remain;
              remain = speed1 - remain;
              
            }

            else if (speed1 < remain) {
              stepComp = speed1;
              remain -= speed1;
            }

            else {
              nextMotor = 0;
              stepComp = remain;
            };
            break;
            
          case 2:
            NRF_P1->OUTSET |= 1 << stepPin2Gpio; //write HIGH
            NRF_P1->OUTCLR |= 1 << stepPin2Gpio; //write LOW
            
            if (remain < speed2) {
              nextMotor = 1;
              stepComp = remain;
              remain = speed2 - remain;
            }

            else if (speed2 < remain) {
              stepComp = speed2;
              remain -= speed2;
            }
            
            else {
              nextMotor = 0;
              stepComp = remain;
            };
            break;
            
          default:
            //NRF_P0->OUTSET |= 1 << stepPin1Gpio; //write HIGH
            //NRF_P0->OUTCLR |= 1 << stepPin1Gpio; //write LOW
            NRF_P0->OUT |= 1 << stepPin1Gpio; //write HIGH
            NRF_P0->OUT &= ~(1 << stepPin1Gpio); //write LOW
            //digitalWrite(stepPin1, HIGH);
            //digitalWrite(stepPin1, LOW);
            NRF_P1->OUTSET |= 1 << stepPin2Gpio; //write HIGH
            NRF_P1->OUTCLR |= 1 << stepPin2Gpio; //write LOW
            
            if (speed1 < speed2) {
              nextMotor = 1;
              remain = speed2 - speed1;
            }

            else if (speed2 < speed1) {
              nextMotor = 2;
              remain = speed1 - speed2;
            }

            else {
              remain = speed1;
              stepComp = remain;
            };
            break;
        };
        
        NRF_TIMER2->TASKS_CLEAR = 1;

        NRF_TIMER2->CC[0] = stepComp;
        NRF_TIMER2->TASKS_START = 1;
      };
    };
}
