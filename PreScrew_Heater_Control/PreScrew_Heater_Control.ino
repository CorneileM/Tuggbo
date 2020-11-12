// This Arduino sketch reads the voltage across a NTC (negative-temperature coefficient) thermistor, converts it to a temperature reading
// and switches a MOSFET connected to a heater cartidge on and of through a PWM signal to keep the temperature close to a set point which is controlled by a rotary encoder.
// Both the set point temperature and actual temperature are displayed on a LCD screen

//**CODE SOURCES**//
// Thermistor reading code, including calibration coefficients for the thermistor were taken from a blog post by Joshua Hrisko: https://makersportal.com/blog/2019/1/15/arduino-thermistor-theory-calibration-and-experiment

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

//**LIBRARIES**//
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

//**PIN AND VARIABLE DECLARATIONS**//

  //*LCD*//
  LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);  // set the LCD address to 0x27 for a 16 chars and 2 line display

  //*ROTARY ENCODER*//
  static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
  static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
  volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
  volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
  volatile byte encoderPos = 30; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
  volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
  volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

  //*THERMISTOR*//
  #define therm_pin A0
  float T_approx;
  float V_0 = 3.3; // voltage reference
  float R_1 = 220000.0; // first resistance value for voltage divider
  
  //fit coefficients
  float a = 283786.2;
  float b = 0.06593;
  float c = 49886.0;
  
  int avg_size = 10; // averaging size

void setup() {

  //setup pins and interrupts for rotary encoder
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  
  //initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(therm_pin,INPUT);

  //set analog reference to read AREF pin
  analogReference(EXTERNAL);

  //setup LCD
  lcd.begin(16,2); // initialize the lcd 
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear ();
  lcd.setCursor(0,0);  
  lcd.print("T:       Tset:"); // print prepeositions of temperature and Tset on LCD 

  // Display default Tset
  lcd.setCursor(14,0);
  lcd.print(encoderPos);   
    
}

//**ROTARY ENCODER INTERRUPT CODE**//

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
      encoderPos ++; //increment the encoder's position count
      bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


void loop() {

//*READ ROTARY ENCODER (Tset)*//
  //Read rotary encoder position changes for Tset
  if(oldEncPos != encoderPos) {
    Serial.println(encoderPos);
    oldEncPos = encoderPos;
   }
   
//*READ THERMISTOR*//
  //loop over several values to lower noise
  float T_sum = 0.0;
  for (int ii;ii<avg_size;ii++){
    //read the input on analog pin 0:
    int sensorValue = analogRead(therm_pin);
    //Convert the analog reading (which goes from 0 - 1023) to voltage reference (3.3V or 5V or other):
    float voltage = (sensorValue/1023.0)*V_0;

    //this is where the thermistor conversion happens based on parameters from fit
    T_sum+=(-1.0/b)*(log(((R_1*voltage)/(a*(V_0-voltage)))-(c/a)));
  }
  
  //averaging values from loop
  T_approx = T_sum/float(avg_size);

  //readout for Celsius and Fahrenheit
  Serial.print("Temperature: ");
  Serial.print(T_approx);
  Serial.print(" (");
  Serial.print((T_approx*(9.0/5.0))+32.0);
  Serial.println(" F)");

//*DISPLAY CURRENT TEMP*//
  lcd.setCursor(2,0);
  lcd.print(t);
  
  //Display set temp
  lcd.setCursor(14,0);
  lcd.print(encoderPos);

  
  delay(500);
}
