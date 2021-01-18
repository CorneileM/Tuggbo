// This Arduino sketch reads the voltage across a NTC (negative-temperature coefficient) thermistor, converts it to a temperature reading
// and switches a MOSFET connected to a heater cartridge on and of through a PWM signal to keep the temperature close to a set point which is controlled by a rotary encoder.
// Both the set point temperature and actual temperature are displayed on a LCD screen

//**CODE SOURCES**//
// Thermistor reading code, including calibration coefficients for the thermistor were taken from a blog post by Joshua Hrisko: https://makersportal.com/blog/2019/1/15/arduino-thermistor-theory-calibration-and-experiment
// Rotary encoder interrupt code for Arduino Nano every adapted from forum discussion: https://forum.arduino.cc/index.php?topic=694732.0 

//**PIN OUTS**//

  /*ARDUINO -> LCD
   *A4 -> SDA
   *A5 -> SCL
   *5V -> LCD_5V
   *GND -> LCD_GND (make sure LCD ground is separate from the rest of the grounds)
   */
  
  /*ARDUINO -> ROTARY ENCODER
   *D2 -> A
   *D3 -> B
   *5V -> VCC
   *GND -> GND
   */
  
  /*ARDUINO -> THERMISTOR
   * A0 -> THERMISTOR LEAD 1 (one of the Thermistor leads, doesn't matter which one)
   * GND -> THERMISTOR LEAD 2 (one of the Thermistor leads, doesn't matter which one)
   * 3.3V -> Resistor (220K ohm) -> THERMISTOR LEAD 1 
   * There's also a 10uF capacitor connected between GND and 3.3V of the Arduino to reduce noise in the thermistor signal
   */
  
  /*ARDUINO -> MOSFET
   * 
   */

//**LIBRARIES**//
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <PID_v1.h>
#include <thermistor.h>

//**PIN AND VARIABLE DECLARATIONS**//

  //*LCD*//
    LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);  // set the LCD address to 0x27 for a 16 chars and 2 line display

  //*ROTARY ENCODER*//
    static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
    static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
    volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
    volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
    volatile int encoderPos = 30; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
    volatile int oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
    volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

  //*THERMISTOR*//
    thermistor therm1(A0,80);  // Analog Pin which is connected to the 3950 thermistor 100K connected to a 4k7 Pull up and 10Uf Capacitor. 80 represents the configuration in the library for this thermistor
    float T_conv;
    float THERMconvRead;

  //*PID TEMPERATURE CONTROL*//
    //define PID Variables
    double Setpoint, Input, Output;
  
    //Specify the links and initial tuning parameters
    PID heaterPID(&Input, &Output, &Setpoint,9.1,0.3,1.8,P_ON_M, DIRECT); //PID coefficients were taken from http://electronoobs.com/eng_arduino_tut24_2.php as a starting poinoinoi
                                                                      //P_ON_M specifies that Proportional on Measurement be used (see: http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/)
                                                                      //We also specify DIRECT, since the hotter the cartridge gets, the lower the output needs to be
                                                                      
  //*MOSFET heater switch*//
    const int MOSFET = 6; //MOSFET PID (PWM) output goes through pin 6 -- this needs to be a PWN pin. On the Nano Every that's D3, D5, D6, D9, D10

  //*TIMING VARIABLES*//
    //We'll use millis as a timer to loop through multiple samplings of the thermistor, and a samplecounter to set and keep track of the number of samples taken before averaging the reads
    unsigned long previousMillis = 0; //set to zero to begin
    const int intervalMillis = 100; //sets the sampling interval that we want -- let's set the interval to 0.2 seconds for now
    unsigned int sampleCount = 0; //placeholder to count the number of sample readings taken from the Thermistor
    const byte sampleNum = 10; //sets the number of samples we want to take before averaging-- let's set this to 10 for now, which gives us an average reading over 2 seconds
    unsigned int THERMreadTotal = 0; //placeholder to add each new reading to. This will be divided by sampleNum once sampleNum is reached to get an average

void setup() {

  //setup pins and interrupts for rotary encoder
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pinA), PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(pinB), PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  //initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  //Define Input and Setpoint and turn the PID on
  Input = T_conv;
  Setpoint = encoderPos;
  heaterPID.SetMode(AUTOMATIC);

  //set analog reference to read AREF pin
  analogReference(EXTERNAL);

  //setup LCD  
  lcd.begin(16,2); // initialize the lcd 
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0,0);  
  lcd.print("T:      Tset:"); // print prepeositions of temperature and Tset on LCD 
  lcd.setCursor(0,1);  
  lcd.print("Output:   "); // print prepeositions of temperature and Tset on LCD 

  // Display default Tset
  lcd.setCursor(13,0);
  lcd.print(encoderPos);   
    
}

//**ROTARY ENCODER INTERRUPT CODE**//

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  // reading =PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  // if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  if ( ( PORTA.IN & PIN0_bm ) && ( PORTF.IN & PIN5_bm ) && aFlag )  { // if D2 && D3 check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  // else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  else if (PORTA.IN & PIN0_bm) bFlag = 1; // if D2, signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  // reading =PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  // if(reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  if ( ( PORTA.IN & PIN0_bm ) && ( PORTF.IN & PIN5_bm ) && bFlag )  { // if D2 && D3 check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  // else if (reading == B00000100) aFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  else if (PORTF.IN & PIN5_bm) aFlag = 1; // if D3, signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


void loop() {

//*READ ROTARY ENCODER (Tset)*//
  //Read rotary encoder position changes for Tset
  if(oldEncPos != encoderPos) {
    Serial.println(encoderPos);
    oldEncPos = encoderPos;
   }

  //Display set temp
  lcd.setCursor(13,0);
  lcd.print("   ");
  lcd.setCursor(13,0);
  lcd.print(encoderPos);
  Serial.print("Tset: ");
  Serial.println(encoderPos); //for debugging purposes
   
//*READ THERMISTOR*//

  //Loop to take the average of sampleNum separate readings from Mitutoyo taken at intervalMillis
  if(millis() - previousMillis >= intervalMillis) {
    
    previousMillis = millis();
    sampleCount++;

    //Read the analog value from the termistor pin
    double THERMconvRead = therm1.analog2temp(); // read temperature

    //add the reading to the total to be averaged later
    THERMreadTotal = THERMreadTotal + THERMconvRead;
  }
  
  if(sampleCount == sampleNum){
    
      T_conv = THERMreadTotal/sampleNum; //tak the average of all the termistor reads

      // reset counters for the next loop
      sampleCount = 0;
      THERMreadTotal = 0;

    //*DISPLAY TEMP*//
      //Display actual temp
      lcd.setCursor(2,0);
      lcd.print("      ");
      lcd.setCursor(2,0);
      lcd.print(T_conv);
      Serial.print("T: "); //for debugging purposes
      Serial.println(T_conv); //for debugging purposes
      
  }    

    //*PID temp control*//
      Input = T_conv;
      Setpoint = encoderPos;
      heaterPID.Compute();
      analogWrite(MOSFET, Output);

      //Display Output
      lcd.setCursor(9,1);
      lcd.print("   ");
      lcd.setCursor(9,1);
      lcd.print(Output);
      Serial.print("Output: "); //for debugging purposes
      Serial.println(Output); //for debugging purposes
  
  delay(200);
}
