#define STOP 0
#define TURN_R 1
#define TURN_L 2
#define FWD 3
#define BWD 4
#define CCW 130
#define NEU 150
#define CW 170
#define DT 2000
#define YELLOW 1
#define BLUE 2
#define BLACK 3
#define OTHER 4

// ----- For Color Sensor -----
int s0 = 3, s1 = 4, s2 = 5, s3 = 6;
int out = 2;
int flag = 0;
volatile int counter = 0;
int countR = 0, countG = 0, countB = 0;
volatile int current_color = OTHER;  // 1 is yellow, 2 is blue, 3 is black, 0 is unclear

// ----- For QTI -----
int QTI_sig_R = 12;
int QTI_sig_L = 13;

// ----- For wheels -----
volatile int robot_movement = FWD;
volatile int timing_wheel_R = NEU;  // How long between 20 ms to pulse
volatile int timing_wheel_L = NEU;  // How long between 20 ms to pulse
volatile int output_R = 0;
volatile int output_L = 0;
int counterR = 0;
int counterL = 0;


// ----- For starting motor -----


//storage variables
int ServoR = 9;
int ServoL = 10;
int ServoTop = 11;
#define L_turn_time 643
#define R_turn_time 643

// ----- Setup -----
void setup() {
  // top_servo.attach(top_servo_pin);
  // top_servo.write(90);
  // top_servo.detach();
  Serial.begin(115200);
  Serial.println("Begin");
  pinMode(ServoR, OUTPUT);
  pinMode(ServoL, OUTPUT);
  pinMode(ServoTop, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  motor_init();
  TCS();

}  //end setup

void motor_init() {
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
}

// ===== All setup for color sensor =====
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
    // UNCOMMENT FOR CALIBRATION:
    // Serial.print("red=");
    // Serial.print(countR);
    digitalWrite(s2, HIGH);
    digitalWrite(s3, HIGH);
  } else if (flag == 2) {
    countG = counter;
    // UNCOMMENT FOR CALIBRATION:
    // Serial.print(" green=");
    // Serial.print(countG);
    digitalWrite(s2, LOW);
    digitalWrite(s3, HIGH);
  } else if (flag == 3) {
    countB = counter;
    // UNCOMMENT FOR CALIBRATION:
    // Serial.print(" blue=");
    // Serial.println(countB);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
  } else if (flag == 4) {  // Adjust thresholds for calibration
    if (countR < 100 && countG < 100 && countB > 120) {
      current_color = BLUE;  // blue
    } else if (countR > 250 && countG > 160 && countB < 160 && countB > 100) {
      current_color = YELLOW;  // yellow
    } else if (countR < 100 && countG < 100 && countB < 100) {
      current_color = BLACK;  // black
    } else {
      current_color = OTHER;
    }
    flag = 0;
  }
  counter = 0;
}

void print_color() {
  if (current_color == YELLOW) {
    Serial.println("yellow");
  } else if (current_color == BLUE) {
    Serial.println("blue");
  } else if (current_color == BLACK) {
    Serial.println("black");
  }
}
int counterTop = 0;
volatile int output_top = LOW;
volatile int timing_top = 0;
bool top_init = 1;
int init_count = 0;
// ===== All setup for motor code =====
ISR(TIMER1_COMPA_vect) {
  counterR++;
  counterL++;
  counterTop++;
  if (counterTop >= timing_top && init_count != 0) {
    if (init_count < 50) {
      // Serial.println("1");
      if (output_top == LOW) {
        output_top = HIGH;
        timing_top = 100;
      } else if (output_top == HIGH) {
        output_top = LOW;
        timing_top = 1900;
        init_count += 1;
      }
    } else if ((init_count >= 50) && (init_count < 60)) {
      // Serial.println("2");
      if (output_top == LOW) {
        output_top = HIGH;
        timing_top = 200;
      } else if (output_top == HIGH) {
        output_top = LOW;
        timing_top = 1800;
        init_count += 1;
      }
    } else {
      output_top = LOW;
    }
    // Serial.println(init_count);
    digitalWrite(ServoTop, output_top);
    counterTop = 0;
  }
  // If the 0.1ms counter reaches pass the defined threshold
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

void turn_R_degree(int degree) {
  robot_movement = TURN_R;
  delay(R_turn_time * degree / 90);
  Serial.println("Right turn");
}

void turn_L_degree(int degree) {
  robot_movement = TURN_L;
  delay(L_turn_time * degree / 90);
  Serial.println("Left turn");
}

// Returns 0 if both white, 1 if left, 2 if right, 3 if both
int read_QTI() {
  int output = 0;
  long durationR = 0;
  long durationL = 0;
  pinMode(QTI_sig_R, OUTPUT);
  pinMode(QTI_sig_L, OUTPUT);
  digitalWrite(QTI_sig_R, HIGH);
  digitalWrite(QTI_sig_L, HIGH);
  delay(1);
  pinMode(QTI_sig_R, INPUT);
  pinMode(QTI_sig_L, INPUT);
  digitalWrite(QTI_sig_R, LOW);
  digitalWrite(QTI_sig_L, LOW);
  int QTI_reading_R = digitalRead(QTI_sig_R);
  int QTI_reading_L = digitalRead(QTI_sig_L);
  while (QTI_reading_R || QTI_reading_L) {
    QTI_reading_R = digitalRead(QTI_sig_R);
    QTI_reading_L = digitalRead(QTI_sig_L);
    durationR += QTI_reading_R;
    durationL += QTI_reading_L;
  }
  if (durationR < 10) {
    output += 2;
  }
  if (durationL < 10) {
    output += 1;
  }
  // Serial.print("Right: ");
  // Serial.println(durationR);
  // Serial.print("Left: ");
  // Serial.println(durationL);
  return output;
}

// ===== Main Loop =====
// The color sensor variable (current_color) will continuously update automatically as part of the background code, so reading this variable will yield the color under the sensor.
//    Calibration may need to be done depending on the room lighting conditions. This will have to be done manually by uncommenting the print statements in ISR(TIMER2_OVR_vect)

// The movement of the robot will be automatically adjusted by setting the variable "robot_movement." This should be used in conjunction with "delay()", timed loops, or
// while() conditions to control the robot
//    FWD will move both wheels forward, BWD will move both wheels backwards, TURN_R and TURN_L will cause the wheels to move opposite directions to turn. Use the defined number
//    R_turn_time and L_turn_time to delay the movement enough for it to make a 90 degree turn. STOP will cause the wheels to stop - however, due to imprecise signals the wheels may jiggle a little bit.
//    This can be fixed by sending a constant voltage. This will require an adjustment that is not included yet.

// If the wheels turn at different speeds which is causing the robot to not go straight, you can adjust the #define statements at the top of file to adjust the speed for CCW/CW.

// Movement pseudo code for pointing at an edge:
// start
// move forward until black on QTI
// repeat until black on QTI
//    turn 90 degrees to face partially into the opponents field
//    walk forward until hitting oppoonents field, then go a little more
//    turn 90 degrees
//    walk forward until hitting allied field, then go a little more
// turn 90 degrees, then repeat
//    walk until black on QTI
//    turn 90 degrees

// Movement for the initial arm release is not included
// QTI sensors are not included yet

int start_color = OTHER;
void loop() {
  // Serial.println("Start");
  // delay(30);
  // start_color = current_color;
  // robot_movement = FWD;
  // Serial.println(start_color);
  // while (1)
  //   ;
  // // move forward until black on QTI
  // // repeat until black on QTI
  // //    turn 90 degrees to face partially into the opponents field
  // //    walk forward until hitting oppoonents field, then go a little more
  // //    turn 90 degrees
  // //    walk forward until hitting allied field, then go a little more
  // // turn 90 degrees, then repeat
  // //    walk until black on QTI
  // //    turn 90 degrees
  // //   int direction = 0;
  // while (!read_QTI()) {
  //   robot_movement = FWD;
  // }
  // int QTI = read_QTI();
  // if (QTI == 1) {
  //   turn_R_degree(90);
  // }
  // if (QTI == 2) {
  //   turn_L_degree(90);
  // }
  // if (QTI == 3) {
  //   turn_R_degree(180);
  // }
  // robot_movement = FWD;
  // delay(100);

  //   int past_color = current_color;
  //   while(!read_QTI()) {
  //     robot_movement = FWD;
  //     if () {
  //       turn_L_degree(90);
  //       past_color = current_color;
  //     }
  //   }
  //   Serial.println(read_QTI());
  //   print_color();
  // }
}