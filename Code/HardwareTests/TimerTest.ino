String command;
long check = 0;
double Time;
class TimeControl {
    private:
        uint16_t clockCount;
        double T_delta;
    public:
        volatile long overflow = 0;

        void timeInit() {
            NRF_TIMER0->TASKS_STOP = 1; //stop timer
            NRF_TIMER0->BITMODE = 1; //8-bit timer
            NRF_TIMER0->MODE = 0; //timer mode
            NRF_TIMER0->PRESCALER = 8; //256 prescaler
            NRF_TIMER0->TASKS_CLEAR = 1; //clear timer
            NRF_TIMER0->CC[0] = 255; //set compare register
            NRF_TIMER0->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos;

            NVIC_EnableIRQ(TIMER0_IRQn); //enable interrupts on TIMER0
            NRF_TIMER0->TASKS_START = 1; //start timer
        };

        double getTDelta() {
            NRF_TIMER0->TASKS_CAPTURE[1] = 1; //save timer value
            clockCount = NRF_TIMER0->CC[1]; //get timer value
            NRF_TIMER0->TASKS_CLEAR = 1;
            T_delta = (clockCount + 256 * overflow) * 16.0e-6; //clockCount * 4.0e-6 + overflow * 1.024e-3;
            Serial.println(overflow);
            overflow = 0;
            return T_delta;
        };

};

TimeControl Clock;

void setup() {
  Serial.begin(9600);
  while (true) {
    if (Serial.available()) {
      command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "go") {
        break;
      };
    };
  };
  Serial.println("start");
  Clock.timeInit();
  NRF_TIMER0->TASKS_CAPTURE[1] = 1;
  int value = NRF_TIMER0->CC[1];
  Serial.println(value);
  delay(3000);
  Time = Clock.getTDelta();
  Serial.println(Time);
}

void loop() {
}

extern "C"
{
    void TIMER0_IRQHandler_v() {
        if (NRF_TIMER0->EVENTS_COMPARE[0] == 1) {   
            
            NRF_TIMER0->EVENTS_COMPARE[0] = 0;
            Clock.overflow ++;
        };
    };
};
