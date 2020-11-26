//This sketch governs the behaviour of a filament pulling device designed to control the diameter of filament.
//The diameter of the filament is controlled by the speed of a DC motor which pulls filament from an extruder (Filabot EX2): faster = thinner filament; slower = thicker filament.
//The speed of the DC motor is adjusted based on the measured filament diameter which is read from a Mitutoyo digital plunge-dial indicator, to obtain a stable filament diameter.

//This sketch incoporates code from SSpence for reading data from the Mitutoyo plangue dial: https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ 

//**LIBRARIES**//
#include <PID_v1.h>

//**PIN AND VARIABLE DECLARATIONS**//

  //*MITUTOYO*//
    const int req = 5; //Mitutoyo REQ line goes to pin 5 through q1 (arduino high pulls request line low)
    const int dat = 2; //Mitutoyo Data line goes to pin 2
    const int clk = 3; //Mitutoyo Clock line goes to pin 3

   //Mitutoyo data variables//
    int i = 0;
    int j = 0;
    int k = 0;
    int MITread;
    int MITreadAve;
    int MITreadAveDiff;
    const int FilamentDiam = 145; //Sets the filament diameter goal we want. Units are mm/100. Technically, we want 1.75m, but there's a slight groove in our Mitutoyo's pulley, which leads to a 0.3mm offset.
    byte mydata[14];
    String value_str;
    long value_int;
    float value;
  
  //*MOSFET MOTOR CONTROLLER*//
    const int MOSFET = 6; //MOSFET PID (PWM) output goes through pin 6 -- this needs to be a PWN pin. On the Nano Every that's D3, D5, D6, D9, D10
    int pwmStart = 50; //variable determining motor speed, starts at 40

  //*PID FILAMENT DIAMETER CONTROL*//
    //define PID Variables
    double Setpoint, Input, Output;
    float Kp = 0.05; //The proportional gain (Kp) determines the ratio of output response to the error signal. In general, increasing the proportional gain will increase the speed of the control system response
                  //I'm setting this to 0.25, since preliminary tests showed that Tuggbo was reacting too quickly, even at Kp = 2
                  
    float Ki = 0.01; //The integral component sums the error term over time. The result is that even a small error term will cause the integral component to increase slowly.
                    //The integral response will continually increase over time unless the error is zero, so the effect is to drive the Steady-State error to zero.

    float Kd = 0.001; //The derivative component causes the output to decrease if the process variable is increasing rapidly (in our case, this is reversed).
                  //The derivative response is proportional to the rate of change of the process variable.
                  //Increasing the derivative time (Td) parameter will cause the control system to react more strongly to changes in the error term and will increase the speed of the overall control system response.
                  //Most practical control systems use very small derivative time (Td), because the Derivative Response is highly sensitive to noise in the process variable signal.
    
    //Specify the links and initial tuning parameters
    PID tuggboPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, REVERSE); //PID coefficients were taken from http://electronoobs.com/eng_arduino_tut24_2.php as a starting poinoinoi
                                                                      //I used to have the PID run on P_ON_M mode (proportional on measurement), but this seemed to make the PID very "scared" of going too close to the setpoint
                                                                      //Overshoot is not a big deal in this application, so I've switched this back to the standard proportional on error mode (P_ON_E)
                                                                      //We specify REVERSE, since the thicker the filament gets, the higher the output needs to be
  
  //*TIMING VARIABLES*//
    //We'll use millis as a timer to loop through multiple samplings of the Mitutoyo readings, and a samplecounter to set and keep track of the number of samples taken before averaging the reads
    unsigned long previousMillis = 0; //set to zero to begin
    const int intervalMillis = 25; //sets the sampling interval that we want -- let's set the interval to 0.1 seconds for now
    unsigned int sampleCount = 0; //placeholder to count the number of sample readings taken from the Mitutoyo
    const byte sampleNum = 1; //sets the number of samples we want to take before averaging-- let's set this to 6 for now, which gives us an average reading over 0.6 seconds
    unsigned int MITreadTotal = 0; //placeholder to add each new reading to. This will be divided by sampleNum once sampleNum is reached to get an average

    
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
  Input = MITreadAve;
  Setpoint = FilamentDiam;
  tuggboPID.SetOutputLimits(50, 150); //since the motor only starts working at 50 PWM, we need to set the PWM min to 50 (the max remains at the PWM max of 255. Also at 255, the motor seems too fast, so I'm capping it at 150
  tuggboPID.SetMode(AUTOMATIC);

  //Starts the motor in forward direction at the motor starting speed
  analogWrite(MOSFET, pwmStart);
  
}

void loop() {

 //*MITUTOYO*//
 //get readings from the SPC cable (see https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ for details)

  //Loop to take the average of sampleNum separate readings from Mitutoyo taken at intervalMillis
  
  if(millis() - previousMillis >= intervalMillis) {
    
    previousMillis = millis();
    sampleCount++;
       
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
    MITread = atol(buf); //assembled measurement, no decimal place added

    //Serial.print("r ");
    //Serial.println(MITread);
   
    digitalWrite(req, HIGH); //reset req (5) pin to HIGH to stop data request
    
    MITreadTotal = MITreadTotal + MITread;
  }

  if(sampleCount == sampleNum){
    
      MITreadAve = MITreadTotal/sampleNum;

      Serial.print("ave: ");
      Serial.println(MITreadAve);

      // reset counters for the next loop
      sampleCount = 0;
      MITreadTotal = 0;
      
  }

  //*PID FILAMENT DIAMETER CONTROL*//
  //if there is no filament is present, we want the motor to continue turning at its minimum speed (0.2 mm is a safe margin for no filament being present)
  //if filament has been fed in (i.e., the Mitutoyo reads higher than 0.2) start PID motor control  
  if(MITreadAve <0.2) { 
      // Send PWM signal (motor minimum speed) to MOSFET
      analogWrite(MOSFET, pwmStart);
      Serial.print("NO FILAMENT -- motor speed: "); //for debugging purposes
      Serial.println(pwmStart); //for debugging purposes

  } else { 
      Input = MITreadAve;
      Setpoint = FilamentDiam;
      tuggboPID.Compute();
      analogWrite(MOSFET, Output);
      Serial.print("motor speed: "); //for debugging purposes
      Serial.println(Output); //for debugging purposes
  }

delay(50);

}
