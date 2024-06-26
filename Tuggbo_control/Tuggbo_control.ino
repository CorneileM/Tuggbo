//This sketch governs the behaviour of a filament pulling device designed to control the diameter of filament.
//The diameter of the filament is controlled by the speed of a DC motor which pulls filament from an extruder (Filabot EX2): faster = thinner filament; slower = thicker filament.
//The speed of the DC motor is adjusted based on the measured filament diameter which is read from a Mitutoyo digital plunge-dial indicator, to obtain a stable filament diameter.

//This sketch incoporates code from SSpence for reading data from the Mitutoyo plangue dial: https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ 

  
//**LIBRARIES**//
#include <PID_v1.h>
#include <MovingAverage.h>

//**PIN AND VARIABLE DECLARATIONS**//

  //*MITUTOYO*//
    const int req = 5; //Mitutoyo REQ line goes to pin 5 through q1 (arduino high pulls request line low)
    const int dat = 2; //Mitutoyo Data line goes to pin 2
    const int clk = 3; //Mitutoyo Clock line goes to pin 3


   //Mitutoyo data variables//
    int i = 0;
    int j = 0;
    int k = 0;
    float MITread_x;
    float MITreadPush;
    float MITreadAve;
    float MITreadAveDiff;
    const float MIToffset = 30; //Because of a slight groove in the Mitutoyo's filament guide bearing, there is a 0.3 mm offset -- we multiply the offset by 100 to correspond with the raw data from the Mitutoyo
                              //The offset is currently subtracted manually and directly from setpoint and measured values, but will be implemented automatically at a later stage to give more intuitive outputs
    const float FilamentDiam = 1.65; //Sets the filament diameter goal we want. Units are mm/100. At the moment we're aiming for 1.35 mm filament
                                  //this seems to be the best diameter to achieve sufficient cooling with the current setup.

    float TargetHIGH = FilamentDiam+0.16;
    float TargetLOW = FilamentDiam-0.16;
    
    byte mydata[14];
    String value_str;
    long value_int;
    float value;
  
  //*MOSFET MOTOR CONTROLLER*//
    const int MOSFET = 6; //MOSFET PID (PWM) output goes through pin 6 -- this needs to be a PWN pin. On the Nano Every that's D3, D5, D6, D9, D10
    int pwmStart = 70; //variable determining motor speed
                       //stable at 55 

  //*PID FILAMENT DIAMETER CONTROL*//
    //define PID Variables
    double Setpoint, Input, Output;
    float Kp = 2; //The proportional gain (Kp) determines the ratio of output response to the error signal. In general, increasing the proportional gain will increase the speed of the control system response
                  //I'm setting this to 0.3, since preliminary tests showed that Tuggbo was reacting too quickly, even at Kp = 2
                  //0.5
                  
    float Ki = 0.75; //The integral component sums the error term over time. The result is that even a small error term will cause the integral component to increase slowly.
                    //The integral response will continually increase over time unless the error is zero, so the effect is to drive the Steady-State error to zero.
                    //0.15
                    
    float Kd = 0.25; //The derivative component causes the output to decrease if the process variable is increasing rapidly (in our case, this is reversed).
                  //The derivative response is proportional to the rate of change of the process variable.
                  //Increasing the derivative time (Td) parameter will cause the control system to react more strongly to changes in the error term and will increase the speed of the overall control system response.
                  //Most practical control systems use very small derivative time (Td), because the Derivative Response is highly sensitive to noise in the process variable signal.
                  //0.2
    
    //Specify the links and initial tuning parameters
    PID tuggboPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, REVERSE); //PID coefficients were taken from http://electronoobs.com/eng_arduino_tut24_2.php as a starting poinoinoi
                                                                      //I used to have the PID run on P_ON_M mode (proportional on measurement), but this seemed to make the PID very "scared" of going too close to the setpoint
                                                                      //Overshoot is not a big deal in this application, so I've switched this back to the standard proportional on error mode (P_ON_E)
                                                                      //We specify REVERSE, since the thicker the filament gets, the higher the output needs to be
  
  //*MOVING AVERAGE VARIABLES*//
    //Sets up the movingAverage function with 50 readings (1 per millisecond) per moving average window with a starting value of 0
    //This function will constantly calcualte a moving average for smoothed Mitutoyo readings -- the moving average window also serves to moderate Tuggbo's control response to make it less jerky 
    MovingAverage<float> MITread(5, 0);

    // Generates the first input for the MovingAverage function
    //float MITread_x = FilamentDiam*100;

    
void setup() {
  
  Serial.begin(9600); //Start serial communication

  //*MITUTOYO*//
    pinMode(req, OUTPUT); //Set the req (5) pin as an output so that data requests can be triggered by setting the pin LOW
    pinMode(clk, INPUT_PULLUP); //To obtain data we will read from the clk and dat pins, so they need to be set to INPUT with internal PULLUP resistors to stop noisy readings
    pinMode(dat, INPUT_PULLUP); //To obtain data we will read from the clk and dat pins, so they need to be set to INPUT with internal PULLUP resistors to stop noisy readings
    digitalWrite(req, HIGH); // set initial state of req (5) pin to HIGH so that data can be requested from Mitutoyo by setting the pin LOW

  //*MOSFET MOTOR CONTROLLER*//
    //MOSFET pin is set to output so that we can send PWM signals to control the motor
    pinMode(MOSFET, OUTPUT); 

    //Define Input and Setpoint and turn the PID on
    Input = MITreadAve*100;
    Setpoint = FilamentDiam*100;
    tuggboPID.SetOutputLimits(pwmStart, 250); //since the motor only starts working at 43 PWM, we need to set the PWM min to 43. 70 is plenty fast, so I'm capping the max here.
    tuggboPID.SetMode(AUTOMATIC);

    //Starts the motor in forward direction at the motor starting speed
    analogWrite(MOSFET, pwmStart);
  
}

void loop() {

 //*MITUTOYO*//
   //get readings from the SPC cable (see https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ for details)
  
    digitalWrite(req, LOW); // request data from Mitutoyo

    for( i = 0; i < 13; i++ ) {
      k = 0;
      for (j = 0; j < 4; j++) {
        while( digitalRead(clk) == LOW) {
          } // hold until clock is high
          while( digitalRead(clk) == HIGH) {
            } // hold until clock is low
            bitWrite(k, j, (digitalRead(dat) & 0x1));
        }
        mydata[i] = k;
      }
    
    // assemble measurement from bytes    
    
    char buf[7];
    for(int lp=0;lp<6;lp++)
    buf[lp]=mydata[lp+5]+'0';
    buf[6]=0;
    MITread_x = atol(buf); //assembled measurement, no decimal place added
   
    digitalWrite(req, HIGH); //reset req (5) pin to HIGH to stop data request
    
  //*MOVING AVERAGE CALCULATION*//
    // Pushes the MITUTOYO reading as an input to the moving average object
    /*if(MITread_x > 180){ // outliers sometimes pop up and cause shit... this eliminates false highs
      MITread_x = 180;
      
    }if(MITread_x < 60){ // outliers sometimes pop up and cause shit... this eliminates false lows
      MITread_x = 60;
    }*/

    MITreadPush = (MITread_x + MIToffset)/100;
    if(MITread.size() > 4){
      MITreadAveDiff = abs(MITreadPush - MITread.get());
      if(MITreadAveDiff > 0.25){
         delay(10);
         //get readings from the SPC cable (see https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ for details)
  
            digitalWrite(req, LOW); // request data from Mitutoyo
        
            for( i = 0; i < 13; i++ ) {
              k = 0;
              for (j = 0; j < 4; j++) {
                while( digitalRead(clk) == LOW) {
                  } // hold until clock is high
                  while( digitalRead(clk) == HIGH) {
                    } // hold until clock is low
                    bitWrite(k, j, (digitalRead(dat) & 0x1));
                }
                mydata[i] = k;
              }
            
            // assemble measurement from bytes    
            
            char buf[7];
            for(int lp=0;lp<6;lp++)
            buf[lp]=mydata[lp+5]+'0';
            buf[6]=0;
            MITread_x = atol(buf); //assembled measurement, no decimal place added
           
            digitalWrite(req, HIGH); //reset req (5) pin to HIGH to stop data request

            MITreadPush = (MITread_x + MIToffset)/100;
            MITread.push(MITreadPush);
            
      } else {
        MITread.push(MITreadPush);
      }
    } else {
      MITread.push(MITreadPush);
    }

    // Prints each value stored in the moving average for debugging
    for (uint8_t i = 0; i < MITread.size(); i++) {
      Serial.print(MITread[i]);
      Serial.print(" ");
    }  

    MITreadAve = MITread.get();
    //Prints the moving average
    Serial.print(" = | ");
    Serial.print(MITread.get());
    Serial.print(" mm |");
    
    //*PID FILAMENT DIAMETER CONTROL*//
    //if there is no filament is present, we want the motor to continue turning at its minimum speed (0.2 mm is a safe margin for no filament being present)
    //if filament has been fed in (i.e., the Mitutoyo reads higher than 0.2) start PID motor control  
    if(MITreadAve < 1.00) { 
       Serial.print("   THIN/NO FILAMENT: "); //for debugging purposes
       Serial.println(pwmStart); //for debugging purpose
       
       analogWrite(MOSFET, pwmStart); 
    } else {
      Input = MITreadAve*100;
      Setpoint = FilamentDiam*100;
      tuggboPID.Compute();
      analogWrite(MOSFET, Output);
      
      if(MITreadAve > TargetLOW && MITreadAve < TargetHIGH) { 
        Serial.print("   TARGET: "); //for debugging purposes
        Serial.println(Output); //for debugging purposes+
        }
        
      if(MITreadAve > TargetHIGH) {       
        Serial.print("   THICK: "); //for debugging purposes
        Serial.println(Output); //for debugging purposes
        }

      if(MITreadAve < TargetLOW) {
        Serial.print("   THIN: "); //for debugging purposes
        Serial.println(Output); //for debugging purposes
      }
    }
    
delay(200);

}
