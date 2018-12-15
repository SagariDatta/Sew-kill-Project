/*
   Sew-kill Project

   This sketch draws from the
   Water Conductivity Sketch by Emily Gertz at https://github.com/ejgertz/EMWA/tree/master/chapter-5
   &
   the Example Sketch 06: Photo Resistor by Sparkfun Electronics
   &
   the DFRobot Example Sketch: https://www.dfrobot.com/wiki/index.php/PH_meter(SKU:_SEN0161)
   &
   the Turbidity Example Sketch by Roland Pelayo https://www.teachmemicro.com/arduino-turbidity-sensor/

   Written by Sagari Datta, Shuchang Dong and John Michael LaSalle, 2018
*/


//conductivity sensor variables
const float ArduinoVoltage = 5.00; //CHANGE THIS FOR 3.3v Arduinos
const float ArduinoResolution = ArduinoVoltage / 1024; //Arduino will read the current, which ranges from 0-5 Volts on a scale of 0-1024
const float resistorValue = 10000.0; //resistance of the 10K Resistor in the voltage divider
float returnVoltage = 0.0; //variable for the voltage of the reading from the conductivity sensor
float resistance = 0.0; //variable for the resistance of the water
double Siemens = 0.0; //variable

//pH sensor variables
float offset = 0.12; //setting an offset for deviation compensate
static float phValue; //variable used in pH computation
static float phVoltage; //variable used in pH computation

//Turbidity sensor variables
float turbidityVoltage; 
float ntu;
int sensorvalue;

//Input Pins
int conductPin = A0; // pin from voltage divider
int turbidityPin = A2; // reading from tubridity sensor
float phPin = A3; // reading from PH sensor

//Output Pins
int greenLED = 6; //green LED pin
int redLED = 5; //red LED pin
int blueLED =3; //blue LED pin

// Input reading variables
int conductReading; //variable for conductivity reading on a scale from 0-1023
int turbidityReading; //variable for conductivity reading on a scale from 0-1023
float phReading; //variable for PH reading on scale from 0-14

//Intermediate variables
int conductance = 1; //1 if water is in good conductivity range and 0 if it is in bad ranges
int ph = 1; //1 if water is in good conductivity range and 0 if it is in bad range
int turbidity = 1; //1 if water is in good conductivity range and 0 if it is in bad ranges

// Output variables
int combinedQuality = 0; //variable for combining final 0,1 


void setup()
{
  Serial.begin(9600); //set baud rate for the serial monitor for debugging
  //Note: Analog pins automatically set to input  
  // Set output pin modes
  pinMode(redLED, OUTPUT); // set the red LED pin mode
  pinMode(greenLED, OUTPUT); // set the Green LED pin mode
  pinMode(blueLED, OUTPUT); // set the Blue LED pin mode
  pinMode(dcPin, OUTPUT); // set the DC motor pin mode
}


void loop()
{  
  //conductance reading and conversion to 0 or 1   
  float Siemens = conductivitySiemens(analogRead(conductPin)); //testing if water is the the healthy fisheries conductivity range or not
  if (Siemens > 150 || Siemens < 500) {
    conductance = 1;                  //specific conductance in bad range
  } else {
    conductance = 0;
  }

  //ph Reading and conversion to 0 or 1 
  phReading = phValueComputation();    //final phReading after using the ph function to compute the phValue
  if (phReading <= 4) {
    ph = 1;                            //ph in bad range if phValue <= 4
  } else {
    ph = 0;                            //ph in good range if phValue < 4
  }
  
  //Turbidity reading and conversion to 0 or 1
  turbidityReading = turbidityValue(); //final turbidity reading after using the tuurbidity functions to compute ntu
  if (turbidityReading > 150) {
    turbidity = 1;                     //turbidity in bad range when ntu > 150
  } else {
    turbidity = 0;                     //turbidity in good range when ntu <= 150
  }

  //combining sensor readings and conversion to 0 or 1 
  if ((conductance + ph + turbidity) >= 2) { // check if at least 2 indicators are in the bad range
    combinedQuality = 1; // water quality is bad
  } else {
    combinedQuality = 0; // water quality is good
  }

  //calling motorResponse function
  motorResponse(combinedQuality);

  delay(1000); // wait 1 second before taking a reading again

}


//Functions
//Function to display color and sensor reading values based on water quality readings
void motorResponse(int Quality) {
  if (Quality == 1) {
    TurnRed();
    Serial.println("water bad");
    Serial.println("conductance");
    Serial.println(Siemens);
    Serial.println("ph");
    Serial.println(phReading);
    Serial.println("turbidity");
    Serial.println(turbidityReading);
  }
  else {
    TurnBlue();
    Serial.println("water good");
    Serial.println("conductance");
    Serial.println(Siemens);
    Serial.println("ph");
    Serial.println(phReading);
    Serial.println("turbidity");
    Serial.println(turbidityReading);
  }
}

//Function to switch to red color
void TurnRed(){ 
  //Red color
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
}

//Function to switch to blue color
void TurnBlue() {
  //Blue color
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, LOW);
}

// function for conductivity reading
float conductivitySiemens(int conductReading) {
  returnVoltage = conductReading * ArduinoResolution; // convert the raw conductivity reading to volts
  resistance = ((5.00 * resistorValue) / returnVoltage) - resistorValue; // convert voltage to ohms
  Siemens = 1.0 / (resistance / 1000000); // convert ohms to microSiemens
  return Siemens;
}

// PH reading function
float phValueComputation(){
  //Calculating phValue
  phVoltage = analogRead(phPin) * 5.0 / 1024;
  phValue = 3.5 * phVoltage + offset;
  return phValue;
}

// Turbidity reading function
float turbidityValue() {
  for (int i = 0; i < 800; i++){                       //voltage from sensor quite noisy so take 800 samples and avg those samples
    int sensorvalue = analogRead(turbidityPin);
    turbidityVoltage += sensorvalue * (5.0 / 1024.0);  //convert analog reading（0~1023）to voltage (0 ~ 5V)
  }
  turbidityVoltage = turbidityVoltage / 800 + 1.2;     //average the reading
  turbidityVoltage = round_to_dp(turbidityVoltage, 1); //round the turbidity voltage number to 1 decimal place       

  if (turbidityVoltage < 2.5) {                        //equation only works between 2.5V ~ 4.2V,
    ntu = 3000;                                        //so set any readings below 2.5V with 3000NTU
  } 
  else{
    ntu = (-1120.4 * square(turbidityVoltage)) + (5742.3 * turbidityVoltage) - 4353.8; //calculate the current NTU
  }
  if(ntu < 0){
    ntu = 0;
  }
  return ntu;
}

//rounding function used in turbidity function
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
