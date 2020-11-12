//This sketch governs the behaviour of a filament pulling device designed to control the diameter of filament.
//The diameter of the filament is controlled by the speed of a DC motor which pulls filament from an extruder (Filabot EX2): faster = thinner filament; slower = thicker filament.
//The speed of the DC motor is adjusted based on the measured filament diameter which is read from a Mitutoyo digital plunge-dial indicator, to obtain a stable filament diameter.

//This sketch combines and significantly modifies original sketches by SSpence: https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ & Dejan Nedelkovski: https://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/


//***PIN AND VARIABLE DECLARATIONS***

//Mitutoyo
  const int req = 5; //Mitutoyo REQ line goes to pin 5 through q1 (arduino high pulls request line low)
  const int dat = 2; //Mitutoyo Data line goes to pin 2
  const int clk = 3; //Mitutoyo Clock line goes to pin 3

//Transistor-switch motor controller
  const int int1 = 6; //L298N mini H-bridge motor controller input 1 goes to pin 6 -- these need to be PWN pins. On the Nano Every that's D3, D5, D6, D9, D10
  int pwmOutput = 50; //variable determining motor speed, starts at 40

//Mitutoyo data variables
  int i = 0;
  int j = 0;
  int k = 0;
  int MITread;
  int MITreadAve;
  int MITreadAveDiff;
  const int FilamentDiam = 145; //Sets the filament diameter goal we want. Units are mm/100
  byte mydata[14];
  String value_str;
  long value_int;
  float value;

  //We'll use millis as a timer to loop through multiple samplings of the Mitutoyo readings, and a samplecounter to set and keep track of the number of samples taken before averaging the reads
  unsigned long previousMillis = 0; //set to zero to begin
  const int intervalMillis = 250; //sets the sampling interval that we want -- let's set the interval to 0.5 seconds for now
  unsigned int sampleCount = 0; //placeholder to count the number of sample readings taken from the Mitutoyo
  const byte sampleNum = 6; //sets the number of samples we want to take before averaging-- let's set this to 6 for now, which gives us an average reading over 3 seconds
  unsigned int MITreadTotal = 0; //placeholder to add each new reading to. This will be divided by sampleNum once sampleNum is reached to get an average

void setup() {
  Serial.begin(9600); //Start serial communication

 //Mitutoyo
  pinMode(req, OUTPUT); //Set the req (5) pin as an output so that data requests can be triggered by setting the pin LOW
  pinMode(clk, INPUT_PULLUP); //To obtain data we will read from the clk and dat pins, so they need to be set to INPUT with internal PULLUP resistors to stop noisy readings
  pinMode(dat, INPUT_PULLUP); //To obtain data we will read from the clk and dat pins, so they need to be set to INPUT with internal PULLUP resistors to stop noisy readings
  digitalWrite(req, HIGH); // set initial state of req (5) pin to HIGH so that data can be requested from Mitutoyo by setting the pin LOW

 //L298N mini H-bridge motor controller
  //motor controller pins are set to outputs so that we can send PWM signals to control the motor
  pinMode(int1, OUTPUT); 
  
  //Starts the motor in forward direction at the motor starting speed
  analogWrite(int1, pwmOutput);
}

void loop() {

 //Mitutoyo -- getting readings from the SPC cable see https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ for details

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

    Serial.print("r ");
    Serial.println(MITread);
   
    digitalWrite(req, HIGH); //reset req (5) pin to high to stop data request
    
    MITreadTotal = MITreadTotal + MITread;
  }

  if(sampleCount == sampleNum){
    
      MITreadAve = MITreadTotal/sampleNum;

      Serial.print("ave: ");
      Serial.println(MITreadAve);

      // reset for the next group
      sampleCount = 0;
      MITreadTotal = 0;

      
      //L298N mini H-bridge motor controller
      //the motor speed is only adjusted once all readings have been taken and averaged
      
      MITreadAveDiff = FilamentDiam - MITreadAve; //Difference between MITreadAve and the goal FilamentDiam. We add 7 to compensate for the dip in the U-groove bearing that the plunge tip can't get into.
    
      if(MITreadAve > 10 && MITreadAve < 600){

        pwmOutput = pwmOutput - MITreadAveDiff/10;
        
        if(pwmOutput > 254){
          pwmOutput = 254;
          } 
        else if(pwmOutput < 50){
          pwmOutput = 50;
          }
          
      } else {
         pwmOutput = 50;
        } 

      Serial.print("PWM ");
      Serial.println(pwmOutput);
      
  }
  
  // Send PWM signal to L298N int1 pin while sending low to the int2 pin sets the direction to forward, and the speed to whatever the PWM is
  analogWrite(int1, pwmOutput);

  delay(100);
}
