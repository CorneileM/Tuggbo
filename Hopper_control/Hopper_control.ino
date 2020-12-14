//ADD DESCRIPTION

//**PIN OUTS**//

  /*ARDUINO -> LCD
   *A4 -> SDA
   *A5 -> SCL
   *5V -> LCD_5V
   *GND -> LCD_GND (make sure LCD ground is separate from the rest of the grounds)
   */

//**LIBRARIES**//
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MovingAverage.h>


//**PIN AND VARIABLE DECLARATIONS**//

  //*LCD*//
    LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);  // set the LCD address to 0x27 for a 16 chars and 2 line display

  //*IR PELLET SENSOR*//
    int sensorvalueBOTT;
    int sensorvalueTOP;
    int sensorthreshold = 800; // this value indicates the limit reading between dark and light,
    // it has to be tested as it may change acording to the 
    // distance the leds are placed.
    // to see what number is good, check the sensorvalue variable value
    // as printed out in the serial monitor

    
  //*MOVING AVERAGE VARIABLES*//
    //Sets up the movingAverage function with 5 readings taken over 1 seconds per moving average window with a starting value of 1000
    //This function will constantly calcualte a moving average for smoothed Mitutoyo readings -- the moving average window also serves to moderate Tuggbo's control response to make it less jerky 
    MovingAverage<unsigned> TOPread(5, 1000);
    MovingAverage<unsigned> BOTTread(5, 1000);

    // Generates the first input for the MovingAverage function
    unsigned TOPread_x = 1000;
    unsigned BOTTread_x = 1000;

    int TOPreadAve;
    int BOTTreadAve; 
    
  //*MOSFET MOTOR CONTROLLER*//
    const int MOSFET = 6; //MOSFET PWM output goes through pin 6 -- this needs to be a PWN pin. On the Nano Every that's D3, D5, D6, D9, D10
    int pwmSpeed = 255; // This sets the speed of the spooler -- we want this to be faster than the pulling speed of the tuggbo, but the spool diameter is much larger than Tuggbo's wheel, so let's try 30 for now
    long prevMillis = 0; // Timing variables used to allow intermittent motor activation and measurements to occur in parallel
    long interval = 5000; // Timing variables used to allow intermittent motor activation and measurements to occur in parallel
    
void setup() {
  
  Serial.begin(9600); //Start serial communication

  //*MOSFET MOTOR CONTROLLER*//
  //MOSFET pin is set to output so that we can send PWM signals to control the motor
  pinMode(MOSFET, OUTPUT);

  //setup LCD  
  lcd.begin(16,2); // initialize the lcd 
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0,0);  
  lcd.print("TOP:      "); // print prepeositions for sensorvalue statements
  lcd.setCursor(0,1);  
  lcd.print("BOTTOM:   "); // print prepeositions for sensorvalue statements 
  
}

void loop() {

  //*IR PELLET SENSOR*//
    //TOP SENSOR
    TOPread_x = analogRead(1); // read from pin 0 to get bottom sensor value
    TOPread.push(TOPread_x); //push next value to moving average
    TOPreadAve = TOPread.get();
    if(TOPreadAve < sensorthreshold) { //if IR light received (i.e., no pellets)
      Serial.print("TOP: ");
      Serial.print(TOPreadAve);
      Serial.println(" E");
      lcd.setCursor(7,0);
      lcd.print("      ");
      lcd.setCursor(8,0);
      lcd.print(TOPreadAve);
      lcd.setCursor(15,0);
      lcd.print("E");
    } else {
      Serial.print("TOP: ");
      Serial.print(TOPreadAve);
      Serial.println(" F");
      lcd.setCursor(7,0);
      lcd.print("      ");
      lcd.setCursor(8,0);
      lcd.print(TOPreadAve);
      lcd.setCursor(15,0);
      lcd.print("F");
    }
    
    //BOTTOM SENSOR
    BOTTread_x = analogRead(0); // read from pin 0 to get bottom sensor value
    BOTTread.push(BOTTread_x); //push next value to moving average
    BOTTreadAve = BOTTread.get();
    if(BOTTreadAve < sensorthreshold) { //if IR light received (i.e., no pellets)
      Serial.print("BOTTOM: ");
      Serial.print(BOTTreadAve);
      Serial.println(" E");
      lcd.setCursor(7,1);
      lcd.print("      ");
      lcd.setCursor(8,1);
      lcd.print(BOTTreadAve);
      lcd.setCursor(15,1);
      lcd.print("E");
    } else {
      Serial.print("BOTTOM: ");
      Serial.print(BOTTreadAve);
      Serial.println(" F");
      lcd.setCursor(7,1);
      lcd.print("      ");
      lcd.setCursor(8,1);
      lcd.print(BOTTreadAve);
      lcd.setCursor(15,1);
      lcd.print("F");
    }

  //*MOSFET MOTOR CONTROLLER RESPONSE TO SENSOR VALUES*
    //below we give instructions to the motor to react to hopper fill levels as detected by the IR pellet sensors
    //the motor turns a pellet dispenser to add more pellets to the hopper when the sensors detect that the hopper is empty

    unsigned long currentMillis = millis(); // set currentMillis to current run time

    if(currentMillis - prevMillis > interval){ // if time passed since the last motor activation is greater than the timing interval, do hopper check and motor activation (or not) again
    
      //TOP AND BOTTOM EMPTY RESPONSE 
      if(BOTTreadAve < sensorthreshold && TOPreadAve < sensorthreshold) {
          //This means the hopper is almost completely empty, and more aggressive filling is required
          analogWrite(MOSFET, pwmSpeed);
          delay(250); //half rotation of the dispenser
          analogWrite(MOSFET, 0); //stops the engine until the next loop
      }
  
      //TOP EMPTY RESPONSE 
      if(BOTTreadAve > sensorthreshold && TOPreadAve < sensorthreshold) {
          //This means the hopper is half empty, requiring a small top-up
          analogWrite(MOSFET, pwmSpeed);
          delay(250); //half rotation of the dispenser
          analogWrite(MOSFET, 0); //stops the engine until the next loop
      }
  
      //FULL RESPONSE 
      if(BOTTreadAve > sensorthreshold && TOPreadAve > sensorthreshold) {
          //This means the hopper is full, requiring no action from the dispenser
          analogWrite(MOSFET, 0); //stops the engine until the next loop
      }  

      prevMillis = currentMillis;
    }    
 
delay(200); // We take IR readings every 0.2 seconds
/*delay(500); //for debugging and tuning the sensor threshold value*/

}
