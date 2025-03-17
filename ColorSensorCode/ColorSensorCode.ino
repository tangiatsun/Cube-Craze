int s0 = 3, s1 = 4, s2 = 5, s3 = 6;
int out = 2;
int flag = 0;
byte counter = 0;
byte countR = 0, countG = 0, countB = 0;
void setup() {
  Serial.begin(115200);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}
void TCS() {
  flag = 0;
  digitalWrite(s1, HIGH);
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
int i = 0;
ISR(TIMER2_OVF_vect) {  //the timer 2, 10ms interrupt overflow again. Internal overflow interrupt executive function {
  TCNT2 = 100;
  flag++;
  if (flag == 1) {
    countR = counter;
    Serial.print("red=");
    Serial.println(countR, DEC);
    digitalWrite(s2, HIGH);
    digitalWrite(s3, HIGH);
  } else if (flag == 2) {
    countG = counter;
    Serial.print("green=");
    Serial.println(countG, DEC);
    digitalWrite(s2, LOW);
    digitalWrite(s3, HIGH);
  } else if (flag == 3) {
    countB = counter;
    Serial.print("blue=");
    Serial.println(countB, DEC);
    Serial.println("\n");
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);

  } else if (flag == 4) {
    flag = 0;
  }
  counter = 0;
}
void loop() {
  TCS();
  while (1)
    ;
}


// /***************************************************
// Observe and report an object's visible color spectrum via a TCS3200 light-to-frequency converter

//  Materials Schedule

//   Arduino Uno - 1 each
//    https://store.arduino.cc/usa/arduino-uno-rev3
//   DF Robot Gravity I/O Shield - 1 each
//    https://www.dfrobot.com/product-1009.html
//   DF Robot Color Sensor - 1 each
//    https://www.dfrobot.com/product-540.html
//   A-B Cable - 1 each
//    https://www.staples.com/usb+ab+cable/directory_usb+ab+cable
//   Wire Bundles - 7 female/female wires
//    https://www.adafruit.com/product/266

//  Assembly

//    1.Connect A-B cable to arduino and laptop and upload sketch
//    2.Connect aurduino and shield
//    3.Connect color sensor to I/O shield digital pins 4, 5, 6, 7, 8 and VCC and GND with (7) female/female wires

//       (D)  Digital  D4    Green   S0
//       (D)  Digital  D5    Black   S1
//       (D)  Digital  D6    Red     S2
//       (D)  Digital  D7    White   S3
//       (D)  Digital  D8    Yellow  OUT
//       (+)  Voltage  VCC   Red     VCC
//       (-)  Ground   GND   Black   GND

//    4.Open serial monitor to view sensor readings at 9600 baud

//  Sensor assumptions (Texas Advanced Optoelectronic Solutions TCS3200)

//     Max output frequency = 600 kHz (1 kHz = 0.001 MHz, 600 kHz = 0.6 MHz)
//     Max data acquisition rate = 1 datapoint/microsecond (1,000 microseconds = 1 millisecond)

//     8 bit Timer on Atmel AVR - FreqCount Library, for measuring frequencies - Paul Stoffregen <paul@pjrc.com>

//         *        1ms       2ms       4ms       8ms
//       2 MHz     8x250     32x125    64x125    128x125
//       1 MHz     8x125     8x250     32x125    64x125

//     Sensor power settings

//       Clear:   S2 High  S3 Low
//       Red:     S2 Low   S3 Low
//       Blue:    S2 Low   S3 High
//       Green:   S2 High  S3 High

//     RGB Model - How to Mechatronics

//       Clear:  Red   ?  Green   ?  Blue   ?  Clear ?
//       Red:    Red  25  Green  72  Blue 255  Clear 0  Hex 1946FF https://convertingcolors.com/rgb-color-25_70_255.html#about
//       Green:  Red  30  Green  90  Blue 255  Clear 0  Hex 1E5AFF https://convertingcolors.com/rgb-color-30_90_255.html#about
//       Blue:   Red  25  Green  70  Blue 255  Clear 0  Hex 1946FF https://convertingcolors.com/rgb-color-25_70_255.html#about

//     RGB Model - DF Robot

//       Clear:  Red 118  Green 106  Blue 111  Clear 0  Hex 766A6F  https://convertingcolors.com/rgb-color-118_106_111.html#about
//       Red:    Red  92  Green  33  Blue  35  Clear 0  Hex 5C2123  https://convertingcolors.com/rgb-color-92_33_35.html#about
//       Green:  Red 112  Green  78  Blue  57  Clear 0  Hex 704E70  https://convertingcolors.com/rgb-color-112_78_112.html
//       Blue:   Red  38  Green  33  Blue  45  Clear 0  Hex 26212D  https://convertingcolors.com/rgb-color-38_33_45.html#about

//     RGB Model - surfnm

//       Clear:  Red 255  Green 255  Blue 255  Clear 0  Hex FFFFFF  https://convertingcolors.com/rgb-color-255_255_255.html
//       Red:    Red 255  Green   0  Blue   0  Clear 0  Hex FF0000  https://convertingcolors.com/rgb-color-255_0_0.html
//       Green:  Red   0  Green 255  Blue   0  Clear 0  Hex 00FF00  https://convertingcolors.com/rgb-color-0_255_0.html
//       Blue:   Red   0  Green   0  Blue 255  Clear 0  Hex 0000FF  https://convertingcolors.com/rgb-color-0_0_255.html

//  Engineering

//     Created by Dejan Nedelkovski
//      https://howtomechatronics.com/tutorials/arduino/arduino-color-sensing-tutorial-tcs230-tcs3200-color-sensor/
//     Created by DF Robot
//      https://www.dfrobot.com/wiki/index.php/TCS3200_Color_Sensor_(SKU:SEN0101)
//     Team Arduino Forum
//      Color Sensing with TCS3200, Need Help with Code & Understanding
//      https://forum.arduino.cc/index.php?topic=549632.0
//      Problem with Arduino Uno Color sensor TC3200
//      https://forum.arduino.cc/index.php?topic=469940.0
//     Modified by surfnm

//  *************************************************/

// //=== Global constants and variables ======

// // TCS3200 light-to-frequency converter wiring
// #define S0 3
// #define S1 4
// #define S2 5
// #define S3 6
// #define sensorOut 2,

// int frequency = 0;  // int (16 bit = 2 byte value = range of -32,768 to 32,767)

// // === Setup code, runs once =========================

// void setup() {

//   pinMode(S0, OUTPUT);
//   pinMode(S1, OUTPUT);
//   pinMode(S2, OUTPUT);
//   pinMode(S3, OUTPUT);
//   pinMode(sensorOut, INPUT);

//   // Setting frequency-scaling to 20%
//   digitalWrite(S0,HIGH);
//   digitalWrite(S1,LOW);

//   Serial.begin(9600);  // use the serial port to print out the resultant data
// }

// // === Main code, runs/loops repeatedly ==============

// void loop() {

//   // Setting Red filtered photodiodes to be read
//   digitalWrite(S2,LOW);
//   digitalWrite(S3,LOW);
//   // Reading the output frequency
//   frequency = pulseIn(sensorOut, LOW);
//   // Remap the value of the Frequency to the RGB Model
//   // Syntax - map(value, fromLow, fromHigh, toLow, toHigh)
//   // Frequency Model fromLow = 0, Frequency Model fromHigh = 600
//   // RGB Model toLow = 0, RGB Model toHigh = 255
//   frequency = map(frequency, 0,600,0,255);
//   // Printing the value on the serial monitor
//   Serial.print("R= ");//printing name
//   Serial.print(frequency);//printing RED color frequency
//   Serial.print("  ");
//   delay(100);

//   // Setting Green filtered photodiodes to be read
//   digitalWrite(S2,HIGH);
//   digitalWrite(S3,HIGH);
//   // Reading the output frequency
//   frequency = pulseIn(sensorOut, LOW);
//   // Remap the value of the Frequency to the RGB Model
//   // Syntax - map(value, fromLow, fromHigh, toLow, toHigh)
//   // Frequency Model fromLow = 0, Frequency Model fromHigh = 600
//   // RGB Model toLow = 0, RGB Model toHigh = 255
//   frequency = map(frequency, 0,600,0,255);
//   // Printing the value on the serial monitor
//   Serial.print("G= ");//printing name
//   Serial.print(frequency);//printing GREEN color frequency
//   Serial.print("  ");
//   delay(100);

//   // Setting Blue filtered photodiodes to be read
//   digitalWrite(S2,LOW);
//   digitalWrite(S3,HIGH);
//   // Reading the output frequency
//   frequency = pulseIn(sensorOut, LOW);
//   // Remap the value of the Frequency to the RGB Model
//   // Syntax - map(value, fromLow, fromHigh, toLow, toHigh)
//   // Frequency Model fromLow = 0, Frequency Model fromHigh = 600
//   // RGB Model toLow = 0, RGB Model toHigh = 255
//   frequency = map(frequency, 0,600,0,255);
//   // Printing the value on the serial monitor
//   Serial.print("B= ");//printing name
//   Serial.print(frequency);//printing BLUE color frequency
//   Serial.println("  ");
//   delay(100);
// }