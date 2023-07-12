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

/**
cycle time -- (1 / freq) sec -- 0.0084 sec

funtion time benchmarks:
    hardware.hardwareInit -- 0.013 sec
    hardware.getSensors -- 0.003 sec
    hardware.setSpeed -- 0.0001 sec

    robot.propagate -- 0.0002 sec !!!
    robot.update -- 0.0012 sec    !!!
    robot.impControl -- negligable

    stepTimerVect -- 0.000016+-
**/

//robot status
volatile int status = 0; //(0 : standby, 1 : stablizing, 2 : active)
volatile bool activeLEDState = 0;
volatile bool standbyLEDState = 0;
const int motorMode = 2; //(1 : double step, 2 : quad step)

//startup tolarences
const float startingTheta = 0.2;
const float startAcc = 1.0;

//angle correction
const float angleBias = 0.0;

//system delay
const float motDelay = 0.0;

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

String command;
//sensor arrays
BLA::Matrix<4> y;
BLA::Matrix<4> Y;


//create states and reference states
BLA::Matrix<7> states = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
auto dynamicStates(states.Submatrix<4, 1>(0, 0));
auto biasStates(states.Submatrix<3, 1>(4, 0));

//ISR for pressing the "Start" button
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
        float standardD[3] = {0.03, 0.03, 0.01};
        int tau[3] = {200, 200, 250};
        long fact = 0;

        BLA::Matrix<4> sensors;
        BLA::Matrix<7, 7> expm;
        BLA::Matrix<7, 7> a_mul;

        //linear dynamics for states
        BLA::Matrix<4, 4> A = {0, 1, 0, 0,
                               0, 0, (M_beam*pow(R_beam, 2)*g + M_beam*R_beam*R_wheel*g)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0,
                               0, 0, 0, 1,
                               0, 0, M_beam*R_beam*g/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0};
        
        //linear dynamics for inputs
        BLA::Matrix<4> B = {0,
                            R_wheel*I_beam/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam),
                            0,
                            M_beam*(-R_beam*R_wheel - pow(R_wheel, 2))/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam)};

        //LQG control coefficients (different for every robot)
        BLA::Matrix<4> K = {-17.320508075688366, -34.91702052158594, -347.2425410793382, -56.993577121134116}; //{-173.20508075685575, -147.0332024815403, -505.8355519985184, -75.86182038749384};

    public:

        float accX;
        float accY;

        BLA::Matrix<7, 7, Eye<float>> diag;
        
        //partial divs of sensors with resepct to states
        BLA::Matrix<4, 7> H = {0, 0, (-M_beam*R_beam*R_wheel*g + M_beam*R_beam*g*R_acc - M_beam*pow(R_wheel, 2)*g - g*I_beam)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 1, 0, 0,
                               0, 0, (M_beam*pow(R_beam, 2)*g + M_beam*R_beam*R_wheel*g)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 0, 1, 0,
                               0, 0, 0, 1, 0, 0, 1,
                               0, 1/R_wheel, 0, -R_beam/R_wheel - 1, 0, 0, 0};

        //linear dynamics for states (bias added)
        BLA::Matrix<7, 7> T = {0, 1, 0, 0, 0, 0, 0,
                               0, 0, (M_beam*pow(R_beam, 2)*g + M_beam*R_beam*R_wheel*g)/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 0, 0, 0,
                               0, 0, 0, 1, 0, 0, 0,
                               0, 0, M_beam*R_beam*g/(M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + I_beam), 0, 0, 0, 0,
                               0, 0, 0, 0, -1/tau[0], 0, 0,
                               0, 0, 0, 0, 0, -1/tau[1], 0,
                               0, 0, 0, 0, 0, 0, -1/tau[2]};

        //state uncertainty in propegation
        BLA::Matrix<7, 7> q = {pow(0.0, 2), 0, 0, 0, 0, 0, 0,
                               0, pow(0.00001, 2), 0, 0, 0, 0, 0,
                               0, 0, pow(0.00012, 2), 0, 0, 0, 0,
                               0, 0, 0, pow(0.00004, 2), 0, 0, 0,
                               0, 0, 0, 0, pow(2 * standardD[0], 2) / tau[0], 0, 0,
                               0, 0, 0, 0, 0, pow(2 * standardD[1], 2) / tau[1], 0,
                               0, 0, 0, 0, 0, 0, pow(2 * standardD[2], 2) / tau[2]};

        //sensor standard deviations
        BLA::Matrix<4, 4> Y = {pow(0.03, 2), 0, 0, 0,
                               0, pow(0.03, 2), 0, 0,
                               0, 0, pow((0.01), 2), 0,
                               0, 0, 0, pow(0.00001, 2)};

        //initial covariance matrix
        BLA::Matrix<7, 7> P = {0.001, 0, 0, 0, 0, 0, 0,
                               0, 0.001, 0, 0, 0, 0, 0,
                               0, 0, 0.001, 0, 0, 0, 0,
                               0, 0, 0, 0.001, 0, 0, 0,
                               0, 0, 0, 0, pow(standardD[0] / 100, 2), 0, 0,
                               0, 0, 0, 0, 0, pow(standardD[1] / 100, 2), 0,
                               0, 0, 0, 0, 0, 0, pow(standardD[2] / 100, 2)};
        
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
            
            return St_acc;
        };

        //matrix exponential func
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
        
        //placeholders for sensor readings
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
        volatile bool sensorFlag = false; //when sensorFlag is thrown by the interrupt, it indicates that a sensor measurement will be ready soon.

        //accumulater constant
        float accumulater[3] = {1.0, 1.0, 1.0}; //for smoothing out sensor measurements (not used right now)

        void hardwareInit() {

            //set LED pins as output and timers
            NRF_P1->PIN_CNF[standbyLEDGpio] = 1 << 0;
            NRF_P1->PIN_CNF[activeLEDGpio] = 1 << 0;

            NRF_P1->OUT |= (1 << standbyLEDGpio);
            standbyLEDState = true;

            pinMode(startButtonDig, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(startButtonDig), buttonISR, FALLING);

            //light blink timer
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

            //sensor flag timer
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
            NRF_P1->PIN_CNF[stepPin2Gpio] = 1 << 0;
            NRF_P1->PIN_CNF[dirPin2Gpio] = 1 << 0;
            
            //set motor config pins
            switch (motorMode) {

              case 1:
                pinMode(doubleStepPin, OUTPUT);
                digitalWrite(doubleStepPin, HIGH);
              case 2:
                pinMode(quadStepPin, OUTPUT);
                digitalWrite(quadStepPin, HIGH);
            }

            //motor step timer
            NRF_TIMER4->TASKS_STOP = 1; //stop timer
            NRF_TIMER4->BITMODE = 0; //16-bit timer
            NRF_TIMER4->MODE = 0; //timer mode
            NRF_TIMER4->PRESCALER = 8; //prescaler of 256
            NRF_TIMER4->TASKS_CLEAR = 1; //clear timer
            NRF_TIMER4->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos; //enable interrupt on CC[0]

            NVIC_EnableIRQ(TIMER4_IRQn);
            NVIC_SetPriority(TIMER4_IRQn, 1); //make moving motors 1st priority

            //wait until IMU is ready            
            while (!IMU.accelerationAvailable()) {
            };
            
            //start sensor flag timer
            NRF_TIMER3->TASKS_START = 1;

            //get initial sensor values
            IMU.readAcceleration(xAcc, Y(1), Y(0));
            IMU.readGyroscope(Y(2), yGyro, zGyro);

            Y(3) = Wh_meters;
            Y(0) *= -g;
            Y(1) *= -g;
            Y(2) *= -pi / 180.0;
        };
        
        //quary the onboard IMU (call when (sensorFlag = true))
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

            //read the average speed of the motors
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

                speed1Norm = constrain(long(981.746875 / (s1 * motorMode)), 10, 65535); //Unit convertion
                speed2Norm = constrain(long(981.746875 / (s2 * motorMode)), 10, 65535); //Unit convertion
                
                NRF_TIMER4->TASKS_CAPTURE[1] = 1;
                
                if (speed1Norm < speed2Norm) {
                  nextMotorNorm = 1;
                  NRF_TIMER4->CC[0] = max(speed1Norm, NRF_TIMER4->CC[1] + 1);
                  if (stepTimerOn == false) {
                      remain = speed2Norm;
                  };
                }

                else if (speed2Norm < speed1Norm) {
                  nextMotorNorm = 2;
                  NRF_TIMER4->CC[0] = max(speed2Norm, NRF_TIMER4->CC[1] + 1);
                  if (stepTimerOn == false) {
                      remain = speed1Norm;
                  };
                }
                
                else {
                  nextMotorNorm = 0;
                  NRF_TIMER4->CC[0] = max(speed1Norm, NRF_TIMER4->CC[1] + 1);
                  if (stepTimerOn == false) {
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
            }

            else if (s1 == 0 and s2 == 0) {
                NRF_TIMER4->TASKS_STOP = 1;
                stepTimerOn = false;
            };
        };

};

Hardware hardware;
Eq equation;

//manage states and control calculation
class Robot {

    private:

        BLA::Matrix<4, 4> inverted;

    public:

        BLA::Matrix<7, 7> a = equation.Expm(equation.T, 5, 1.0 / freq);
        BLA::Matrix<4> statesGoal = {0.0, 0.0, 0.0, 0.0};

        //propagate via reference states and calculate new covariance matrix
        void propegate(BLA::Matrix<4, 1, BLA::Reference<BLA::Array<7, 1, float>>> dynamic, BLA::Matrix<3, 1, BLA::Reference<BLA::Array<7, 1, float>>> bias, float Wh_acc, float T_prop) {

            equation.linSolve(dynamic, bias, Wh_acc, T_prop);
            
            equation.P = a * equation.P * ~a + equation.q * T_prop;
        };
        
        //get sensor values and update states
        void update(BLA::Matrix<4, 1, BLA::Reference<BLA::Array<7, 1, float>>> dynamic, BLA::Matrix<3, 1, BLA::Reference<BLA::Array<7, 1, float>>> bias, float Wh_acc, float Wh_rad) {

            //generate sensor guesses based on states
            y = equation.sensorSolve(dynamic, bias, Wh_acc, Wh_rad);

            Invert(equation.H * equation.P * ~equation.H + equation.Y, inverted);
            auto Kf = equation.P * ~equation.H * inverted;
            auto statesAdd = Kf * (Y - y);
            states += statesAdd;
            equation.P = (equation.diag - (Kf * equation.H)) * equation.P;
        };

        //calculate new St_acc
        float impControl(BLA::Matrix<4, 1, BLA::Reference<BLA::Array<7, 1, float>>> dynamic, float x_off, float St_acc) {
            if (abs(x_off) <= x_offLim) {
              statesGoal = dynamic - BLA::Matrix<4> {x_off, 0, 0, 0};
            }
            else if (x_off / abs(x_off) > 0) {
              statesGoal = dynamic - BLA::Matrix<4> {x_offLim, 0, 0, 0};
            }
            
            else if (x_off / abs(x_off) < 0) {
              statesGoal = dynamic - BLA::Matrix<4> {-x_offLim, 0, 0, 0};
            };
            return equation.solveControl(statesGoal, St_acc);
        };

        //set new speed according to St_acc and the time since called last
        void speedPropegate(float acc, float diff) {
            float T_delta = 1.0 / freq;
            hardware.Wh_rad += acc * T_delta;
            hardware.setSpeed(-(hardware.Wh_rad + diff), (hardware.Wh_rad - diff));
        };
};

Robot robot;

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

  Clock.timeInit(); //the clock is not actually used, but might be helpful for future additions to the project
  hardware.hardwareInit();
};

void loop() {

  //robot is waiting to be stabalized
  if (status == 1) {

    //if the robot is experiencing only the acceleration of gravity...
    if (abs(pow(Y(0), 2) + pow(Y(1), 2) - pow(g, 2)) < pow(startAcc, 2) and Y(1) > 0) {

      //...set the angle according to the sensors
      states(2) = -atan2(Y(0), Y(1));

      //if the angle is close to zero
      if (abs(states(2)) < startingTheta) {
        
        //go to active mode
        status = 2;
        
        //reset LEDs
        NRF_TIMER2->TASKS_STOP = 1;
        NRF_TIMER2->TASKS_CLEAR = 1;
        standbyLEDState = false;
        NRF_P1->OUT &= ~(1 << standbyLEDGpio);
        activeLEDState = true;
        NRF_P1->OUT |= (1 << activeLEDGpio);

        Clock.getTDelta();
      };
    };

    hardware.getSensors();
  }

  //robot is active
  if (status == 2) {
    count ++;

    //update the states
    robot.propegate(dynamicStates, biasStates, St_acc, 1.0 / freq);
    robot.update(dynamicStates, biasStates, St_acc, hardware.Wh_rad);

    //calculate and apply an acceleration to the motors
    St_acc = robot.impControl(dynamicStates, x_target, St_acc);
    if (int(count / freq) % 7 > 5) {
      robot.speedPropegate(St_acc, 2.0);
    }

    else {
      robot.speedPropegate(St_acc, 0.0);
    }

    //check whether the robot has fallen over (or is unrecoverable)
    if (abs(states(2)) > (pi / 3.0)) {
      
      //reset stuff
      status = 0;

      NRF_TIMER2->TASKS_STOP = 1;
      standbyLEDState = true;
      NRF_P1->OUT |= (1 << standbyLEDGpio);
      activeLEDState = false;
      NRF_P1->OUT &= ~(1 << activeLEDGpio);
      
      hardware.setSpeed(0.0, 0.0);
      
    };

    //if (int(count / (7 * freq)) % 2 < 1) {
    //  x_target = 0;
    //}

    //else {
    //  x_target = 2;
    //}
    
    hardware.getSensors();
  };
};

//clock overflow interrupt
extern "C" void TIMER0_IRQHandler_v() {
    if (NRF_TIMER0->EVENTS_COMPARE[0] == 1) {
        
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        Clock.overflow ++;
    };
};

//LED blink interrupt
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

//sensor flag interrupt
extern "C" void TIMER3_IRQHandler_v() {
  if (NRF_TIMER3->EVENTS_COMPARE[0] == 1) {
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;
    hardware.sensorFlag = true;
  };
};

//motor step interrupt
extern "C" void TIMER4_IRQHandler_v() {
  if (NRF_TIMER4->EVENTS_COMPARE[0] == 1) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    NRF_TIMER4->TASKS_STOP = 1;

    //step respective motor and set up the interrupt for the next step
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
    
    //clear the timer
    NRF_TIMER4->TASKS_CLEAR = 1;

    //set new compare value and restart timer
    NRF_TIMER4->CC[0] = hardware.stepComp;
    NRF_TIMER4->TASKS_START = 1;
  };
};
