#include <Arduino_LSM9DS1.h>
#include <BasicLinearAlgebra.h>
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

#define standbyLEDGpio 12
#define activeLEDGpio 15

#define startButtonDig 2

using namespace BLA;

//robot status
volatile int status = 0; //(0 : standby, 1 : stablizing, 2 : active)
volatile bool activeLEDState = 0;
volatile bool standbyLEDState = 0;

//startup tolarences
const float startingTheta = 0.2;
const float startAcc = 1.0;

//angle correction
const float angleBias = -0.015;

//system delay
const float motDelay = 0.016;

//target frames per second (Hz)
const float freq = 119.0;

//physical attrabutes
const float R_beam = 0.06;
const float R_wheel = 0.041;
const float R_acc = 0.024;
const float M_beam = 0.997;
const float g = 9.81;
const float I_beam = 0.007483920614471;
const float pi = 3.1415926;

float St_acc = 0.0;

float x_target = 0.0;
float x_offLim = 4.0;

long count = 0;

volatile float Time = 0.0;

String command;
//sensor arrays
BLA::Matrix<4> y;
BLA::Matrix<4> Y;


//create states and reference states
BLA::Matrix<7> states = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
auto dynamicStates(states.Submatrix<4, 1>(0, 0));
auto biasStates(states.Submatrix<3, 1>(4, 0));

void buttonISR() {
  
  if (status == 0) {
  
    status = 1;
    NRF_TIMER2->TASKS_START = 1;
    
  };
};

//for all large equations
class Eq {
    private:
        
        float Bx, By, Btheta;
        float theta, thetaD, x;
        float nps, npc, nps2, npc2;
        float standardD[3] = {0.5, 0.5, 0.34};
        int tau[3] = {200, 200, 250};
        long fact = 0;

        BLA::Matrix<4> sensors;
        BLA::Matrix<7, 7> expm;
        BLA::Matrix<7, 7> a_mul;

        BLA::Matrix<4, 4> A = {0, 1, 0, 0,
                               0, 0, (M_beam*pow(R_beam, 2)*g + M_beam*R_beam*R_wheel*g)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0,
                               0, 0, 0, 1,
                               0, 0, M_beam*R_beam*g/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0};
        
        BLA::Matrix<4> B = {0,
                            R_wheel*I_beam/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam),
                            0,
                            M_beam*(-R_beam*R_wheel - pow(R_wheel, 2))/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam)};

        BLA::Matrix<4> K = {-1.7320508075689223, -11.293153767736229, -306.22507163543025, -52.01039735208999};

        //BLA::Matrix<4> K_ds = -~K - ~K * A * motDelay;
        //BLA::Matrix<4> K_dm = -~K * (B * motDelay);

    public:
        float accX;
        float accY;

        BLA::Matrix<7, 7, Eye<float>> diag;
        
        BLA::Matrix<4, 7> H = {0, 0, (-M_beam*R_beam*R_wheel*g + M_beam*R_beam*g*R_acc - M_beam*pow(R_wheel, 2)*g - g*I_beam)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 0, 0, 0,
                               0, 0, (M_beam*pow(R_beam, 2)*g + M_beam*R_beam*R_wheel*g)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 0, 0, 0,
                               0, 0, 0, 1, 0, 0, 0,
                               0, 1/R_wheel, 0, -R_beam/R_wheel - 1, 0, 0, 0};

        BLA::Matrix<7, 7> T = {0, 1, 0, 0, 0, 0, 0,
                               0, 0, (M_beam*pow(R_beam, 2)*g + M_beam*R_beam*R_wheel*g)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 0, 0, 0,
                               0, 0, 0, 1, 0, 0, 0,
                               0, 0, M_beam*R_beam*g/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 0, 0, 0,
                               0, 0, 0, 0, -1/tau[0], 0, 0,
                               0, 0, 0, 0, 0, -1/tau[1], 0,
                               0, 0, 0, 0, 0, 0, -1/tau[2]};

        BLA::Matrix<7, 7> q = {pow(0.0, 2), 0, 0, 0, 0, 0, 0,
                               0, pow(0.1, 2), 0, 0, 0, 0, 0,
                               0, 0, pow(0.4, 2), 0, 0, 0, 0,
                               0, 0, 0, pow(0.4, 2), 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0,//pow(2 * standardD[0], 2) / tau[0], 0, 0,
                               0, 0, 0, 0, 0, 0, 0,//pow(2 * standardD[1], 2) / tau[1], 0,
                               0, 0, 0, 0, 0, 0, 0};//pow(2 * standardD[2], 2) / tau[2]};

        BLA::Matrix<4, 4> Y = {pow(0.03, 2), 0, 0, 0,
                               0, pow(0.03, 2), 0, 0,
                               0, 0, pow((0.01), 2), 0,
                               0, 0, 0, pow(0.0001, 2)};

        
        BLA::Matrix<7, 7> P = {0.1, 0, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0, 0,
                               0, 0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0, pow(standardD[0], 2), 0, 0,
                               0, 0, 0, 0, 0, pow(standardD[1], 2), 0,
                               0, 0, 0, 0, 0, 0, pow(standardD[2], 2)};
        
        //solve dynamics with linear approximation
        void linSolve(BLA::Matrix<4, 1, BLA::Reference<BLA::Array<7, 1, float>>> dynamic, BLA::Matrix<3, 1, BLA::Reference<BLA::Array<7, 1, float>>> bias, float St_acc, float T_delta) {
            //linear solve of dynamics
            dynamic += (A * dynamic + B * St_acc) * T_delta;

            //propagating biases
            Bx = bias(0);
            By = bias(1);
            Btheta = bias(2);
            bias(0) += -Bx / tau[0] * T_delta;
            bias(1) += -By / tau[1] * T_delta;
            bias(2) += -Btheta / tau[2] * T_delta;
        };

        //solve for sensor guesses
        BLA::Matrix<4> sensorSolve(BLA::Matrix<4> dynamic, BLA::Matrix<3> bias, float St_acc, float stepSpeed) {
            //set equation values
            nps = sin(dynamic(2));
            npc = cos(dynamic(2));
            nps2 = sin(dynamic(2) * 2);
            npc2 = cos(dynamic(2) * 2);
            theta = dynamic(2);
            thetaD = dynamic(3);

            accX = (M_beam*pow(R_beam, 2)*R_wheel*pow(thetaD, 2)*nps + M_beam*R_beam*pow(R_wheel, 2)*St_acc*npc2/2 - M_beam*R_beam*pow(R_wheel, 2)*St_acc/2 + M_beam*R_beam*pow(R_wheel, 2)*pow(thetaD, 2)*nps2/2 - M_beam*R_beam*R_wheel*g*nps2/2 - M_beam*R_beam*R_wheel*R_acc*St_acc*npc + M_beam*R_beam*R_wheel*R_acc*pow(thetaD, 2)*nps + M_beam*R_beam*g*R_acc*nps - M_beam*pow(R_wheel, 2)*g*nps - M_beam*pow(R_wheel, 2)*R_acc*St_acc + R_wheel*I_beam*St_acc*npc - g*I_beam*nps)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + I_beam);
            accY = (R_acc*pow(thetaD, 2)*(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + I_beam) + (R_beam*(M_beam*(R_beam*R_wheel*St_acc*npc - R_beam*R_wheel*pow(thetaD, 2)*nps - R_beam*g*nps + pow(R_wheel, 2)*St_acc)*nps - pow(thetaD, 2)*(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + I_beam)*npc) + g*(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + I_beam))*npc - (2*M_beam*pow(R_beam, 3)*pow(thetaD, 2)*nps + M_beam*pow(R_beam, 2)*R_wheel*St_acc*npc2 - M_beam*pow(R_beam, 2)*R_wheel*St_acc + M_beam*pow(R_beam, 2)*R_wheel*pow(thetaD, 2)*nps2 - M_beam*pow(R_beam, 2)*g*nps2 - 2*M_beam*R_beam*R_wheel*g*nps + 2*R_beam*I_beam*pow(thetaD, 2)*nps - 2*R_wheel*I_beam*St_acc)*npc/2)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + I_beam);

            float phi_D = (dynamic(1) - R_beam * npc * dynamic(3)) / R_wheel - dynamic(3);

            //make sensor guess(y)
            sensors = {accX, accY, thetaD, phi_D};

            //add biases
            sensors.Submatrix<3, 1>(0, 0) += bias;
            
            return sensors;
        };

        //solve St_acc according to the goal states
        float solveControl(BLA::Matrix<4> states_goal, float St_acc) {
            //operate controller

            St_acc = (-~K * (states_goal + (A * states_goal + B * St_acc) * motDelay))(0);
            //St_acc = K_ds * states_goal + K_dm * St_acc;
            return St_acc;
        };

        //matrix exponential
        BLA::Matrix<7, 7> Expm(Matrix<7, 7> t, byte order, float T_delta) {
            Matrix<7, 7> a = t * T_delta;
            expm = diag;
            a_mul = diag;
            fact = 1;

            for (byte i = 1; i <= order; i++) {
              fact = fact * i;
              a_mul *= a;
              expm += a_mul / fact;
            }
            return expm;
        };
};

//time, and coordinate the propagation
class TimeControl {
    private:
        uint16_t clockCount;
        float T_delta;
    public:
        volatile unsigned int overflow = 0;

        void timeInit() {
            NRF_TIMER0->TASKS_STOP = 1; //stop timer
            NRF_TIMER0->BITMODE = 1; //8-bit timer
            NRF_TIMER0->MODE = 0; //timer mode
            NRF_TIMER0->PRESCALER = 8; //256 prescaler
            NRF_TIMER0->TASKS_CLEAR = 1; //clear timer
            NRF_TIMER0->CC[0] = 255; //set compare register
            NRF_TIMER0->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos;

            NVIC_EnableIRQ(TIMER0_IRQn); //enable interrupts on TIMER0
            NVIC_SetPriority(TIMER0_IRQn, 2); //counting overflows 2nd priority

            NRF_TIMER0->TASKS_START = 1; //start timer
        };

        //get time since last call
        float getTDelta() {
            NRF_TIMER0->TASKS_CAPTURE[1] = 1; //save timer value
            clockCount = NRF_TIMER0->CC[1]; //get timer value
            NRF_TIMER0->TASKS_CLEAR = 1;
            T_delta = (clockCount + 256 * overflow) * 16.0e-6;
            overflow = 0;
            return T_delta;
        };

};

TimeControl Clock;

//communicate with hardware
class Hardware {
    private:
        //Norm variables for motor stepping
        uint16_t remainNorm;
        long speed1Norm, speed2Norm;
        byte nextMotorNorm;
        float xAcc, yAcc, zAcc;
        float xGyro, yGyro, zGyro;

    public:
        
        //variables for motor stepping interupt
        volatile uint16_t stepComp;
        volatile uint16_t remain;
        volatile long speed1, speed2;
        volatile byte nextMotor;
        bool stepTimerOn = false;
        float Wh_meters = 0.0; //speed of wheel in meters per second
        float Wh_rad = 0.0; //speed of wheel in radians per second

        //variable for sensor opperation
        volatile bool sensorFlag = false;

        //accumulater constant
        float accumulater[3] = {1.0, 1.0, 1.0};

        void hardwareInit() {

            //set LED pins as output and timers
            NRF_P1->PIN_CNF[standbyLEDGpio] = 1 << 0;
            NRF_P1->PIN_CNF[activeLEDGpio] = 1 << 0;

            NRF_P1->OUT |= (1 << standbyLEDGpio);
            standbyLEDState = true;

            pinMode(startButtonDig, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(startButtonDig), buttonISR, FALLING);

            NRF_TIMER2->TASKS_STOP = 1; //stop timer
            NRF_TIMER2->BITMODE = 0; //16-bit timer
            NRF_TIMER2->MODE = 0; //timer mode
            NRF_TIMER2->PRESCALER = 8; //prescaler of 256
            NRF_TIMER2->TASKS_CLEAR = 1; //clear timer
            NRF_TIMER2->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos; //enable interrupt on CC[0]
            NRF_TIMER2->CC[0] = 25000; //0.4 second period

            NVIC_EnableIRQ(TIMER2_IRQn);
            NVIC_SetPriority(TIMER2_IRQn, 4); //make blinking light 4th priority

            IMU.begin();
            Wire1.setClock(100000);

            //set up sensor connection
            NRF_TIMER3->TASKS_STOP = 1; //stop timer
            NRF_TIMER3->BITMODE = 0; //16-bit timer
            NRF_TIMER3->MODE = 0; //timer mode
            NRF_TIMER3->PRESCALER = 2; //prescaler of 4
            NRF_TIMER3->TASKS_CLEAR = 1; //clear timer
            NRF_TIMER3->CC[0] = 17000; //clock runs at 120Hz
            NRF_TIMER3->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos; //enable interrupt on CC[0]

            NVIC_EnableIRQ(TIMER3_IRQn);
            NVIC_SetPriority(TIMER3_IRQn, 3); //setting sensor flag 3rd priority

            //set step pins to be output
            NRF_P0->PIN_CNF[stepPin1Gpio] = 1 << 0;
            NRF_P0->PIN_CNF[dirPin1Gpio] = 1 << 0;
            //pinMode(stepPin1, OUTPUT);
            //pinMode(dirPin1, OUTPUT);
            NRF_P1->PIN_CNF[stepPin2Gpio] = 1 << 0;
            NRF_P1->PIN_CNF[dirPin2Gpio] = 1 << 0;
            //pinMode(quadStepPin, OUTPUT);
            //pinMode(doubleStepPin, OUTPUT);
            //digitalWrite(quadStepPin, HIGH);
            //digitalWrite(doubleStepPin, HIGH);

            NRF_TIMER4->TASKS_STOP = 1; //stop timer
            NRF_TIMER4->BITMODE = 0; //16-bit timer
            NRF_TIMER4->MODE = 0; //timer mode
            NRF_TIMER4->PRESCALER = 8;//8; //prescaler of 256
            NRF_TIMER4->TASKS_CLEAR = 1; //clear timer
            NRF_TIMER4->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos; //enable interrupt on CC[0]

            NVIC_EnableIRQ(TIMER4_IRQn);
            NVIC_SetPriority(TIMER4_IRQn, 1); //make moving motors 1st priority

            //wait until IMU is ready            
            while (!IMU.accelerationAvailable()) {
            };
            
            //start clock
            NRF_TIMER3->TASKS_START = 1;

            IMU.readAcceleration(xAcc, Y(1), Y(0));
            IMU.readGyroscope(Y(2), yGyro, zGyro);

            Y(3) = Wh_meters;
            Y(0) *= -g;
            Y(1) *= -g;
            Y(2) *= -pi / 180.0;
            //Y(0) *= -g;
            //Y(1) *= -g;
            //Y(2) *= -pi / 180.0;
        };
        
        //quary the onboard IMU (call only when (sensorFlag = true))
        void getSensors() {
            //read from three spesific sensors
            while(true) {
              if (IMU.accelerationAvailable()) {
                sensorFlag = false;
                NRF_TIMER3->TASKS_START = 1;
                IMU.readAcceleration(xAcc, yAcc, zAcc);
                IMU.readGyroscope(xGyro, yGyro, zGyro);

                Y(3) = Wh_meters;
                //Y(0) = (Y(0) + accumulater[0] * zAcc * -g) / (1 + accumulater[0]);
                //Y(1) = (Y(1) + accumulater[1] * yAcc * -g) / (1 + accumulater[1]);
                //Y(2) = (Y(2) + accumulater[2] * xGyro * -pi / 180.0) / (1 + accumulater[2]);
                Y(0) = (cos(angleBias) * zAcc - sin(angleBias) * yAcc) * -g;
                Y(1) = (cos(angleBias) * yAcc - sin(angleBias) * zAcc) * -g;
                Y(2) = xGyro * -pi / 180.0;
                break;
              };
            };
        };

        //handle motor logistics to set new speeds for the motor
        void setSpeed(float s1, float s2) {
            Wh_meters = (s2 - s1) / 2;
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
                
                speed1Norm = constrain(long(981.746875 / (s1 * 2.0)), 20, 65535); //Unit convertion
                speed2Norm = constrain(long(981.746875 / (s2 * 2.0)), 20, 65535); //Unit convertion

                NRF_TIMER4->TASKS_CAPTURE[1] = 1;
                
                if (speed1Norm < speed2Norm) {
                  nextMotorNorm = 1;
                  NRF_TIMER4->CC[0] = max(speed1Norm, NRF_TIMER4->CC[1] + 1);
                  //remain = speed2Norm;
                  if (stepTimerOn == false) {
                      //NRF_TIMER4->CC[0] = speed1Norm;
                      remain = speed2Norm;
                  };
                }

                else if (speed2Norm < speed1Norm) {
                  nextMotorNorm = 2;
                  NRF_TIMER4->CC[0] = max(speed2Norm, NRF_TIMER4->CC[1] + 1);
                  //remain = speed1Norm;
                  if (stepTimerOn == false) {
                      //NRF_TIMER4->CC[0] = speed2Norm;
                      remain = speed1Norm;
                  };
                }
                
                else {
                  nextMotorNorm = 0;
                  NRF_TIMER4->CC[0] = max(speed1Norm, NRF_TIMER4->CC[1] + 1);
                  //remain = speed1Norm;
                  if (stepTimerOn == false) {
                      //NRF_TIMER4->CC[0] = speed1Norm;
                      remain = speed1Norm;
                  };
                };

                if (stepTimerOn == false) {
                  NRF_TIMER4->TASKS_START = 1;
                  stepTimerOn = true;
                };
                nextMotor = nextMotorNorm;
                speed1 = speed1Norm;
                speed2 = speed2Norm;
                //Serial.println(NRF_TIMER4->CC[0]);
                //Serial.println(speed1Norm);
                //Serial << "\n";
            }

            else if (s1 == 0 and s2 == 0) {
                NRF_TIMER4->TASKS_STOP = 1; //TIMSK1 = 0;
                stepTimerOn = false;
            };
        };

};

Hardware hardware;
Eq equation;

void setup() {
  Serial.begin(115200);
  /**
  while (true) {
    if (Serial.available()) {
      command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "go") {
        break;
      };
    };
  };
  **/
  //Serial.println("start1");
  Clock.timeInit();
  hardware.hardwareInit();
};

void loop() {

  //robot is waiting to be stabalized
  if (status == 1) {

    if (abs(pow(Y(0), 2) + pow(Y(1), 2) - pow(g, 2)) < pow(startAcc, 2) and Y(1) > 0) {
      states(2) = -atan2(Y(0), Y(1));
      //Serial.println(states(2));
      if (abs(states(2)) < startingTheta) {
        
        status = 2;
        
        //reset LEDs
        NRF_TIMER2->TASKS_STOP = 1;
        NRF_TIMER2->TASKS_CLEAR = 1;
        standbyLEDState = false;
        NRF_P1->OUT &= ~(1 << standbyLEDGpio);
        activeLEDState = true;
        NRF_P1->OUT |= (1 << activeLEDGpio);

        Clock.getTDelta();

        NRF_P0->OUT |= 1 << stepPin1Gpio; //write HIGH
        NRF_P0->OUT &= ~(1 << stepPin1Gpio); //write LOW
        NRF_P1->OUT |= 1 << stepPin2Gpio; //write HIGH
        NRF_P1->OUT &= ~(1 << stepPin2Gpio); //write LOW

        //Serial.println("start2");
      };
    };

    hardware.getSensors();
  }

  //robot is active
  if (status == 2) {

    Serial.println(Y(0));
    
    hardware.getSensors();
  };
};

extern "C" void TIMER0_IRQHandler_v() {
    if (NRF_TIMER0->EVENTS_COMPARE[0] == 1) {
        
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        Clock.overflow ++;
    };
};

extern "C" void TIMER2_IRQHandler_v() {
    if (NRF_TIMER2->EVENTS_COMPARE[0] == 1) {

      NRF_TIMER2->EVENTS_COMPARE[0] = 0;
      NRF_TIMER2->TASKS_CLEAR = 1;

      //if the standby LED state is on
      if (standbyLEDState == true) {
        NRF_P1->OUT &= ~(1 << standbyLEDGpio);
        standbyLEDState = false;
      }

      else if (standbyLEDState == false) {
        NRF_P1->OUT |= (1 << standbyLEDGpio);
        standbyLEDState = true;
      };
    };
};

extern "C" void TIMER3_IRQHandler_v() {
  if (NRF_TIMER3->EVENTS_COMPARE[0] == 1) {
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;
    hardware.sensorFlag = true;
  };
};

extern "C" void TIMER4_IRQHandler_v() {
  if (NRF_TIMER4->EVENTS_COMPARE[0] == 1) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    NRF_TIMER4->TASKS_STOP = 1;
    switch (hardware.nextMotor) {
      case 1:
        
        NRF_P0->OUT |= 1 << stepPin1Gpio; //write HIGH
        NRF_P0->OUT &= ~(1 << stepPin1Gpio); //write LOW
        
        if (hardware.remain < hardware.speed1) {
          hardware.nextMotor = 2;
          hardware.stepComp = hardware.remain;
          hardware.remain = hardware.speed1 - hardware.remain;
          
        }

        else if (hardware.speed1 < hardware.remain) {
          hardware.stepComp = hardware.speed1;
          hardware.remain -= hardware.speed1;
        }

        else {
          hardware.nextMotor = 0;
          hardware.stepComp = hardware.remain;
        };
        break;
        
      case 2:
        
        NRF_P1->OUT |= 1 << stepPin2Gpio; //write HIGH
        NRF_P1->OUT &= ~(1 << stepPin2Gpio); //write LOW
        
        if (hardware.remain < hardware.speed2) {
          hardware.nextMotor = 1;
          hardware.stepComp = hardware.remain;
          hardware.remain = hardware.speed2 - hardware.remain;
        }

        else if (hardware.speed2 < hardware.remain) {
          hardware.stepComp = hardware.speed2;
          hardware.remain -= hardware.speed2;
        }
        
        else {
          hardware.nextMotor = 0;
          hardware.stepComp = hardware.remain;
        };
        break;
        
      default:
        
        NRF_P0->OUT |= 1 << stepPin1Gpio; //write HIGH
        NRF_P0->OUT &= ~(1 << stepPin1Gpio); //write LOW
        //digitalWrite(stepPin1, HIGH);
        //digitalWrite(stepPin1, LOW);
        NRF_P1->OUT |= 1 << stepPin2Gpio; //write HIGH
        NRF_P1->OUT &= ~(1 << stepPin2Gpio); //write LOW

        if (hardware.speed1 < hardware.speed2) {
          hardware.nextMotor = 1;
          hardware.remain = hardware.speed2 - hardware.speed1;
        }

        else if (hardware.speed2 < hardware.speed1) {
          hardware.nextMotor = 2;
          hardware.remain = hardware.speed1 - hardware.speed2;
        }

        else {
          hardware.remain = hardware.speed1;
          hardware.stepComp = hardware.remain;
        };
        break;
    };
    
    NRF_TIMER4->TASKS_CLEAR = 1;

    NRF_TIMER4->CC[0] = hardware.stepComp;
    NRF_TIMER4->TASKS_START = 1;
  };
};
