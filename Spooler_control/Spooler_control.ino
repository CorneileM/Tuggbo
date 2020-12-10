//This sketch governs the behaviour of a filament pulling device designed to control the diameter of filament.
//The diameter of the filament is controlled by the speed of a DC motor which pulls filament from an extruder (Filabot EX2): faster = thinner filament; slower = thicker filament.
//The speed of the DC motor is adjusted based on the measured filament diameter which is read from a Mitutoyo digital plunge-dial indicator, to obtain a stable filament diameter.

//This sketch incoporates code from SSpence for reading data from the Mitutoyo plangue dial: https://www.instructables.com/id/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/ 

//**LIBRARIES**//

//**PIN AND VARIABLE DECLARATIONS**//
  
  //*MOSFET MOTOR CONTROLLER*//
    const int MOSFET = 6; //MOSFET PWM output goes through pin 6 -- this needs to be a PWN pin. On the Nano Every that's D3, D5, D6, D9, D10
    int pwmSpeed = 45; // This sets the speed of the spooler -- we want this to be faster than the pulling speed of the tuggbo, but the spool diameter is much larger than Tuggbo's wheel, so let's try 30 for now
    
void setup() {
  
  Serial.begin(9600); //Start serial communication

  //*MOSFET MOTOR CONTROLLER*//
  //MOSFET pin is set to output so that we can send PWM signals to control the motor
  pinMode(MOSFET, OUTPUT); 

  //Starts the motor in forward direction at the motor starting speed
  analogWrite(MOSFET, pwmSpeed);
  
}

void loop() {

  //Starts the motor in forward direction at the motor starting speed
  analogWrite(MOSFET, pwmSpeed);
 
delay(50);

  //Starts the motor in forward direction at the motor starting speed
  analogWrite(MOSFET, 0);
 
delay(50);

}
