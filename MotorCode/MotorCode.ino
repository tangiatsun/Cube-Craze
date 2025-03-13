#define RIGHT 0
#define STOP 1
#define LEFT 2
#define CCW 13
#define NEU 15
#define CW 17
#define DT 200

// ----- For Color Sensor -----
int s0 = 3, s1 = 4, s2 = 5, s3 = 6;
int out = 2;
int flag = 0;
volatile int counter = 0;
int countR = 0, countG = 0, countB = 0;
int current_color = 0;  // 1 is yellow, 2 is blue, 3 is black, 0 is unclear

int QTI_sig_R = 14;
int QTI_sig_L = 15;

// ----- For wheels -----
volatile int direction_wheel_R = RIGHT;
volatile int timing_wheel_R = NEU;
volatile int direction_wheel_L = LEFT;
volatile int timing_wheel_L = NEU;
volatile int output_R = 0;
volatile int output_L = 0;
int counterR = 0;
int counterL = 0;
// int dummy = 0;
//storage variables
int Servo1 = 9;
int Servo2 = 10;

// ----- Setup -----
void setup() {
  pinMode(Servo1, OUTPUT);
  pinMode(Servo2, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  Serial.begin(115200);
  //set timer1 interrupt at 1kHz (0.1 ms)
  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  TCNT1 = 0;   //initialize counter value to 0
  // match register = [16,000,000/(8*(1kHz))] - 1
  OCR1A = 199;
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
    // Serial.print("\n");
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);

  } else if (flag == 4) {
    if (countR < 100 && countG < 100 && countB > 150) {
      current_color = 2;  // blue
    } else if (countR > 300 && countG > 250 && countB < 200 && countB > 150) {
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
      if (direction_wheel_R == STOP) {
        timing_wheel_R = NEU;
      } else if (direction_wheel_R == RIGHT) {
        timing_wheel_R = CW;
      } else if (direction_wheel_R == LEFT) {
        timing_wheel_R = CCW;
      }
    } else if (output_R == HIGH) {
      output_R = LOW;
      timing_wheel_R = DT;
    }
    counterR = 0;
    digitalWrite(Servo1, output_R);
  }
  if (counterL >= timing_wheel_L) {
    if (output_L == LOW) {
      output_L = HIGH;
      if (direction_wheel_L == STOP) {
        timing_wheel_L = NEU;
      } else if (direction_wheel_L == RIGHT) {
        timing_wheel_L = CW;
      } else if (direction_wheel_L == LEFT) {
        timing_wheel_L = CCW;
      }
    } else if (output_L == HIGH) {
      output_L = LOW;
      timing_wheel_L = DT;
    }
    counterL = 0;
    digitalWrite(Servo2, output_L);
  }
}


void loop() {
  // Serial.println("Start");
  TCS();
  print_color();
}
// delay(5);
// TIMSK0 |= (0 << OCIE0A);
// TIMSK1 |= (0 << OCIE1A);
// TIMSK2 |= (0 << OCIE2A);
// //do other things here