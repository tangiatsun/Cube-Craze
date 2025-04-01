#define STOP 0
#define TURN_R 1
#define TURN_L 2
#define FWD 3
#define BWD 4
#define CCW 130
#define NEU 150
#define CW 170
#define DT 2000

// ----- For Color Sensor -----
int s0 = 3, s1 = 4, s2 = 5, s3 = 6;
int out = 2;
int flag = 0;
volatile int counter = 0;
int countR = 0, countG = 0, countB = 0;
volatile int current_color = 0;  // 1 is yellow, 2 is blue, 3 is black, 0 is unclear

int QTI_sig_R = 14;
int QTI_sig_L = 15;

// ----- For wheels -----
volatile int robot_movement = FWD;
volatile int timing_wheel_R = NEU;  // How long between 20 ms to pulse
volatile int timing_wheel_L = NEU;  // How long between 20 ms to pulse
volatile int output_R = 0;
volatile int output_L = 0;

#define L_turn_time 1100
#define R_turn_time 1100

int counterR = 0;
int counterL = 0;
// int dummy = 0;
//storage variables
int ServoR = 9;
int ServoL = 10;


// ----- Setup -----
void setup() {
  pinMode(ServoR, OUTPUT);
  pinMode(ServoL, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  Serial.begin(115200);
  //set timer1 interrupt at 1kHz (0.1 ms)
  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  TCNT1 = 0;   //initialize counter value to 0
  // match register = [16,000,000/(8*(100kHz))] - 1
  OCR1A = 19;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);  // 64 prescaler
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();  //allow interrupts
}  //end setup

void TCS() {
  flag = 0;
  digitalWrite(s1, LOW);
  digitalWrite(s0, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  attachInterrupt(0, ISR_INTO, CHANGE);
  timer0_init();
}

void ISR_INTO() {
  counter++;
  // Serial.println("counter");
}

void timer0_init(void) {
  TCCR2A = 0x00;
  TCCR2B = 0x07;  //the clock frequency source 1024 points
  TCNT2 = 100;    //10 ms overflow again
  TIMSK2 = 0x01;  //allow interrupt
}

ISR(TIMER2_OVF_vect)  //the timer 2, 10ms interrupt overflow again. Internal overflow interrupt executive function
{
  TCNT2 = 100;
  flag++;
  // Serial.println("Color");
  if (flag == 1) {
    countR = counter;
    // Serial.print("red=");
    // Serial.print(countR);
    digitalWrite(s2, HIGH);
    digitalWrite(s3, HIGH);
  } else if (flag == 2) {
    countG = counter;
    // Serial.print(" green=");
    // Serial.print(countG);
    digitalWrite(s2, LOW);
    digitalWrite(s3, HIGH);
  } else if (flag == 3) {
    countB = counter;
    // Serial.print(" blue=");
    // Serial.print(countB);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);

  } else if (flag == 4) {
    if (countR < 100 && countG < 100 && countB > 120) {
      current_color = 2;  // blue
    } else if (countR > 240 && countG > 160 && countB < 150 && countB > 100) {
      current_color = 1;  // yellow
    } else if (countR < 50 && countG < 50 && countB < 50) {
      current_color = 3;  // black
    } else {
      current_color = 0;
    }
    flag = 0;
  }
  counter = 0;
}

void print_color() {
  if (current_color == 1) {
    Serial.println("yellow");
  } else if (current_color == 2) {
    Serial.println("blue");
  } else if (current_color == 3) {
    Serial.println("black");
  }
}
// Interrupts
ISR(TIMER1_COMPA_vect) {
  counterR++;
  counterL++;
  // If the 0.1ms counter reaches passed the threshold
  if (counterR >= timing_wheel_R) {
    if (output_R == LOW) {
      output_R = HIGH;
      if (robot_movement == STOP) {
        timing_wheel_R = NEU;
      } else if (robot_movement == FWD || robot_movement == TURN_L) {
        timing_wheel_R = CCW;
      } else if (robot_movement == BWD || robot_movement == TURN_R) {
        timing_wheel_R = CW;
      }
    } else if (output_R == HIGH) {
      output_R = LOW;
      timing_wheel_R = DT;
    }
    counterR = 0;
    digitalWrite(ServoR, output_R);
  }
  if (counterL >= timing_wheel_L) {
    if (output_L == LOW) {
      output_L = HIGH;
      if (robot_movement == STOP) {
        timing_wheel_L = NEU;
      } else if (robot_movement == BWD || robot_movement == TURN_L) {
        timing_wheel_L = CCW;
      } else if (robot_movement == FWD || robot_movement == TURN_R) {
        timing_wheel_L = CW;
      }
    } else if (output_L == HIGH) {
      output_L = LOW;
      timing_wheel_L = DT;
    }
    counterL = 0;
    digitalWrite(ServoL, output_L);
  }
}

int state = 0;
void loop() {
  // Serial.println("Start");
  TCS();
  robot_movement = FWD;
  // delay(10);
  int starting_color = current_color;
  while (1) {
    // robot_movement = BWD;
    // delay(10);
    robot_movement = FWD;
    delay(1000);
    robot_movement = TURN_R;
    delay(R_turn_time);
    robot_movement = FWD;

    delay(1000);
    robot_movement = TURN_L;
    delay(L_turn_time * 2);
    robot_movement = FWD;
    delay(2000);
    robot_movement = BWD;
    delay(1000);
    robot_movement = TURN_L;
    delay(L_turn_time);
    robot_movement = FWD;
    delay(1500);
    robot_movement = TURN_R;
    delay(R_turn_time * 2);
    robot_movement = STOP;
    while (1)
      ;
    // delay(10);
    // print_color();
    // if (current_color != starting_color) {
    //   robot_movement = TURN_R;
    //   delay(L_turn_time);
    //   robot_movement = FWD;
    // }
    // if (current_color = 3) {
    //   robot_movement = TURN_R;
    //   delay(L_turn_time * 2);
    //   robot_movement = FWD;
    // }
    // print_color();
  }
}
