#include <Arduino_LSM9DS1.h>
#include <Wire.h>

//registers taken from Ardino_LSM9DS1 docs

//standard deviation for x axis : 0.0005274861607 Gs

float yAcc;
float xGyro;
float zGyro;
float Y[3];
bool sensorFlag = false;
String command;

/**
class TimeControl {
    private:
        int T_curr, T_past;
        double timeDelta;


    public:
        double runTime = 0;

        
        void initTime() {
            //do whatever needs to happen to start keeping track of time
            T_curr = millis();

        };

        double getTDelta() {
            //check the time since last call of getTDelta() or initition
            //determine if update is nessesary
            T_past = T_curr;
            T_curr = millis();
            timeDelta = (T_curr - T_past);
            runTime += timeDelta;
            return timeDelta;
        };


};
**/


void getSensors() {
  int count = 0;
  while(true) {
    count++;
    if (IMU.accelerationAvailable()) {
      NRF_TIMER3->TASKS_START = 1;
      IMU.readAcceleration(Y[1], yAcc, Y[0]);
      IMU.readGyroscope(xGyro, Y[2], zGyro);
      break;
    };
  };
  Serial.println(count);
  sensorFlag = false;
};

void setup(){
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
  
  Serial.println(IMU.begin());
  Wire1.setClock(400000);

  NRF_TIMER3->TASKS_STOP = 1; //stop timer
  NRF_TIMER3->BITMODE = 0; //16-bit timer
  NRF_TIMER3->MODE = 0; //timer mode
  NRF_TIMER3->PRESCALER = 2; //prescaler of 4
  NRF_TIMER3->TASKS_CLEAR = 1; //clear timer
  NRF_TIMER3->CC[0] = 33500; //clock runs at 120Hz
  NRF_TIMER3->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos; //enable interrupt on CC[0]
  NRF_TIMER3->TASKS_START = 1;
  NVIC_EnableIRQ(TIMER3_IRQn);
}
void loop(){
  if (sensorFlag == true) {
    getSensors();
  };
  Serial.println("X Axis:");Serial.println(Y[0]);
  Serial.println("Y Axis:");Serial.println(Y[1]);
  Serial.println("Z Gyro:");Serial.println(Y[2]);
}

extern "C"
{
    void TIMER3_IRQHandler_v() {
      if (NRF_TIMER3->EVENTS_COMPARE[0] == 1) {
        NRF_TIMER3->EVENTS_COMPARE[0] = 0;
        NRF_TIMER3->TASKS_STOP = 1;
        NRF_TIMER3->TASKS_CLEAR = 1;
        sensorFlag = true;
      };
    };
}    

