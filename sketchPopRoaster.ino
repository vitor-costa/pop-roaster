
#include <PID_v1.h>                // include the PID library
#include "Max6675.h"

#define CELSIUS //RoastLogger default is Celsius - To output data in Fahrenheit comment out this one line 
#define DP 1  // No. decimal places for serial output

#define maxLength 30                  // maximum length for strings used

#define FAN_PIN 9 // pin used on mosfet gate that will control fan

const int arduino = 0;                // either overridden if power pot is set below 97% to power pot control
const int computer = 1;              

// if using only one thermocouple, PID data source is thermocouple 1
// if using two thermocouples, PID data source is thermocouple 2
// default to use one thermocouple
const bool useSecondThermocouple = false;

/****************************************************************************
 * 
 * Arduino pin assignments
 * 
 * The following are the pin assignments used by my setup.
 * 
 * You only need to change the following 5 constants to suit the pin
 * assignments used by your setup.  
 * 
 ****************************************************************************/

// set pin numbers:
const int pwmPin =  11;         // pin for pulse width modulation of heater

// thermocouple reading Max 6675 pins
Max6675 therm2(6, 7, 8);
Max6675 therm1(3, 4, 5);

/****************************************************************************
 *  After setting the above pin assignments you can use the remainder of this
 *   sketch as is or change it, if you wish, to add additional functionality
 * 
 ****************************************************************************/


// time constants
const int timePeriod = 200;           // total time period of PWM milliseconds see note on setupPWM before changing
const int tcTimePeriod = 250;         // 250 ms loop to read thermocouples

// thermocouple settings
float calibrate1 = 0.0; // Temperature compensation for T1
float calibrate2 = 0.0; // Temperature compensation for T2

// PID variables - initial values just guesses, actual values set by computer
double pidSetpoint, pidInput, pidOutput;
double pidP = 20.0;
double pidI = 45.0;
double pidD = 10.0;

//Specify the links and initial tuning parameters (last three are P I and D values)
PID myPID(&pidInput, &pidOutput, &pidSetpoint, pidP, pidI, pidD, DIRECT);


// set global variables

//temporary values for temperature to be read
float temp1 = 0.0;                   // temporary temperature variable
float temp2 = 0.0;                   // temporary temperature variable 
float t1 = 0.0;                      // Last average temperature on thermocouple 1 - average of four readings
float t2 = 0.0;                      // Last average temperature on thermocouple 2 - average of four readings
float tCumulative1 = 0.0;            // cumulative total of temperatures read before averaged
float tCumulative2 = 0.0;            // cumulative total of temperatures read before averaged
int noGoodReadings1 = 0;             // counter of no. of good readings for average calculation 
int noGoodReadings2 = 0;             // counter of no. of good readings for average calculation

int inByte = 0;                       // incoming serial byte
String inString = String(maxLength);  // input String


// loop control variables
unsigned long lastTCTimerLoop;        // for timing the thermocouple loop
int tcLoopCount = 0;                  // counter to run serial output once every 4 loops of 250 ms t/c loop

// PWM variables
int  timeOn;                          // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
unsigned long lastTimePeriod;         // millis since last turned on pwm
int power = 100;                      //use as %, 100 is always on, 0 always off default 100

int fan = 0;

int controlBy = arduino;              // default is arduino control. PC sends "pccontrol" to gain control or
                                      // swapped back to Arduino control if PC sends "arduinocontrol"

void setup()
{
  
  // start serial port at 115200 baud:
  Serial.begin(115200);
  // use establish contact if you want to wait until 'A' sent to Arduino before start - not used in this version
  // establishContact();  // send a byte to establish contact until receiver responds 
  setupPWM();
 
  //Set up pin VCC1, VCC2, GND2 and GNDRelé
  pinMode(2, OUTPUT); digitalWrite(2, HIGH);
  pinMode(12, OUTPUT); digitalWrite(12, LOW); // SSR rele -

  //Set up fan pin
  pinMode(FAN_PIN, OUTPUT); analogWrite(FAN_PIN, 0);

  //set up the PID
  pidInput = 0;                        // for testing start with temperature of 0
  pidSetpoint = 222;                   // default if not connected to computer
  myPID.SetOutputLimits(0,100);        // set output range 0 - 100 %
  myPID.SetSampleTime(2000);           // set 2 second sample time for pID so equal to PWM cycle time
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  Serial.println("Reset");            // flag that Arduino has reset used for debugging 
}

/****************************************************************************
 * Set up power pwm control for heater.  Hottop uses a triac that switches
 * only on zero crossing so normal pwm will not work.
 * Minimum time slice is a half cycle or 10 millisecs in UK.  
 * Loop in this prog may need up to 10 ms to complete.
 * I will use 20 millisecs as time slice with 100 power levels that
 * gives 2000 milliseconds total time period.
 * The Hottop heater is very slow responding so 2 sec period is not a problem.
 ****************************************************************************/
void setupPWM() {
  // set the digital pin as output:
  pinMode(pwmPin, OUTPUT);

  //setup PWM
  lastTimePeriod = millis();
  digitalWrite(pwmPin, LOW);//set PWM pin off to start
}

// this not currently used
void establishContact() {
  //  while (Serial.available() <= 0) {
  //    Serial.print('A', BYTE);   // send a capital A
  //    delay(300);
  //  }
  Serial.println("Opened but no contact yet - send A to start");
  int contact = 0;
  while (contact == 0){
    if (Serial.available() > 0) {
      inByte = Serial.read();
      Serial.println(inByte);
      if (inByte == 'A'){ 
        contact = 1;
        Serial.println("Contact established starting to send data");
      }    
    }
  }  
}

/****************************************************************************
 * Toggles the heater on/off based on the current power level.  Power level
 * may be determined by arduino or computer.
 ****************************************************************************/
void doPWM()
{
  timeOn = timePeriod * power / 100; //recalc the millisecs on to get this power level, user may have changed
 
 if (millis() - lastTimePeriod > timePeriod) lastTimePeriod = millis();
 if (millis() - lastTimePeriod < timeOn){
      digitalWrite(pwmPin, HIGH); // turn on
  } else {
      digitalWrite(pwmPin, LOW); // turn off
 }
 
}

/****************************************************************************
 * Called to set power level. Now always safe as hardware only turns heater on
 * if BOTH the Arduino AND the Hottop control board call for heat.
 ****************************************************************************/
void setPowerLevel(int p)
{
   if (p > -1 && p < 101) power = p;
}

/****************************************************************************
 * Called to set fan power level.
 ****************************************************************************/
void setFanLevel(int p)
{
   if (p > -1 && p < 101) fan = p;
}

void doFanAjustment()
{
   if (fan == 0) {
      analogWrite(FAN_PIN, 0);
   } else
   if (fan > 0 && fan <= 100) {
        double fanStep = 1.5;
        analogWrite(FAN_PIN, 105 + (fanStep * fan));
   }
}

/****************************************************************************
 * Called when an input string is received from computer
 * designed for key=value pairs or simple text commands. 
 * Performs commands and splits key and value 
 * and if key is defined sets value otherwise ignores
 ****************************************************************************/
void doInputCommand()
{
  float v = -1;
  inString.toLowerCase();
  int indx = inString.indexOf('=');

  if (indx < 0){  //this is a message not a key value pair

    if (inString.equals("pccontrol")) {
      controlBy = computer;      
    } 
    else if (inString.equals("arduinocontrol")) {
      controlBy = arduino;        
    }

  } 
  else {  //this is a key value pair for decoding
    String key = inString.substring(0, indx);
    String value = inString.substring(indx+1, inString.length());

    //parse string value and return float v
    char buf[value.length()+1];
    value.toCharArray(buf,value.length()+1);
    v = atof (buf); 
 
    //only set value if we have a valid positive number - atof will return 0.0 if invalid
    if (v >= 0)
    {
      if (key.equals("power")  && controlBy == computer && v < 101){  
        
        setPowerLevel((long) v);//convert v to integer for power 
                
      } else
      if (useSecondThermocouple && key.equals("sett2")){ // set PID setpoint to T2 value
        pidSetpoint = v;
        
      } else
      if (!useSecondThermocouple && key.equals("sett1")) { // set PID setpoint to T1 value
        pidSetpoint = v;
      } else
      if (key.equals("pidp")) { 
        pidP = v;
      } else
      if (key.equals("pidi")) { 
        pidI = v;
      } else
      if (key.equals("pidd")) { 
        pidD = v;
      } else
      if (key.equals("fan")) {
        setFanLevel((long) v); //convert v to integer for fan
      }
    }
  }
}

/****************************************************************************
 * check if serial input is waiting if so add it to inString.  
 * Instructions are terminated with \n \r or 'z' 
 * If this is the end of input line then call doInputCommand to act on it.
 ****************************************************************************/
void getSerialInput()
{
  //check if data is coming in if so deal with it
  if (Serial.available() > 0) {

    // read the incoming data as a char:
    char inChar = Serial.read();
    // if it's a newline or return or z, print the string:
    if ((inChar == '\n') || (inChar == '\r') || (inChar == 'z')) {

      //do whatever is commanded by the input string
      if (inString.length() > 0) doInputCommand();
      inString = "";        //reset for next line of input
    } 
    else {
      // if we are not at the end of the string, append the incoming character
      if (inString.length() < maxLength) {
                inString += inChar; 

      }
      else {
        // empty the string and set it equal to the incoming char:
      //  inString = inChar;
           inString = "";
           inString += inChar;
      }
    }
  }
}

/****************************************************************************
 * Send data to computer once every second.  Data such as temperatures,
 * PID setting etc.
 * This allows current settings to be checked by the controlling program
 * and changed if, and only if, necessary.
 * This is quicker that resending data from the controller each second
 * and the Arduino having to read and interpret the results.
 ****************************************************************************/
void doSerialOutput()
{
  //send data to logger
   
  float tt1;
  float tt2;
  
  #ifdef CELSIUS
    tt1 = t1;
    tt2 = t2;
  #else
    tt1 = (t1 * 9 / 5) + 32;
    tt2 = (t2 * 9 / 5) + 32;
  #endif   
   
  Serial.print("t1=");
  Serial.println(tt1,DP);

  if (useSecondThermocouple) {
    Serial.print("t2=");
    Serial.println(tt2,DP);
  }

  Serial.print("power%=");
  Serial.println(power);

  Serial.print("fan%=");
  Serial.println(fan);
  

  // only need to send these if Arduino controlling by PID
  if (controlBy == arduino)
  {
    if (useSecondThermocouple) {
      Serial.print("targett2=");
      Serial.println(pidSetpoint,DP);
    } else {
      Serial.print("targett1=");
      Serial.println(pidSetpoint,DP);
    }

    Serial.print("pidP=");
    Serial.println(pidP,DP);

    Serial.print("pidI=");
    Serial.println(pidI,DP);

    Serial.print("pidD=");
    Serial.println(pidD,DP);
  }
}

/****************************************************************************
 * Read temperatures from Max6675 chips Sets t1 and t2, -1 if an error
 * occurred.  Max6675 needs 240 ms between readings or will return last
 * value again. I am reading it once per second.
 ****************************************************************************/
void getTemperatures()
{
 
 temp1 = therm1.getCelsius();
 temp2 = therm2.getCelsius();
  
 if (temp1 > 0.0) 
 {
    tCumulative1 = tCumulative1 + temp1;
     noGoodReadings1 ++;
 }
 if (temp2 > 0.0) 
 {
    tCumulative2 = tCumulative2 + temp2;
    noGoodReadings2 ++;
 }
}

/****************************************************************************
* Purpose to adjust PID settings and compute new output value setting power
* to pidOutput on exit
****************************************************************************/
void updatePID(){

  if (useSecondThermocouple) {
    pidInput = t2;
  } else {
    pidInput = t1;
  }

  myPID.SetTunings(pidP,pidI,pidD);
 
  myPID.Compute();

  setPowerLevel(pidOutput); // range set in setup to 0 - 100%
}

/****************************************************************************
 * Called by main loop once every 250 ms
 * Used to read each thermocouple once every 250 ms
 *
 * Once per second averages temperature results, updates PID and outputs data
 * to serial port.
 ****************************************************************************/
void do250msLoop()
{

    getTemperatures();
           
    if (tcLoopCount > 3)  // once every four loops (1 second) calculate average temp, update PID and do serial output
    {
      
      tcLoopCount = 0;
      
      if (noGoodReadings1 > 0)  t1 = tCumulative1 / noGoodReadings1; else t1 = -1.0;
      if (noGoodReadings2 > 0)  t2 = tCumulative2 / noGoodReadings2; else t2 = -1.0;
      noGoodReadings1 = 0;
      noGoodReadings2 = 0;
      tCumulative1 = 0.0;
      tCumulative2 = 0.0;
      
      // only use PID if in Arduino control.  If Computer control power is set by computer
        if (controlBy == arduino){
          updatePID(); // set to execute once per 2 seconds in setup, just returns otherwise
        }
        
        doSerialOutput(); // once per second
    }
    tcLoopCount++;

}


/****************************************************************************
 * Main loop must not use delay!  PWM heater control relies on loop running
 * at least every 40 ms.  If it takes longer then heater will be on slightly
 * longer than planned. Not a big problem if 1% becomes 1.2%! But keep loop fast.
 * Currently loop takes about 4-5 ms to run so no problem.
 ****************************************************************************/
void loop(){

  getSerialInput();// check if any serial data waiting

  // loop to run once every 250 ms to read TC's update PID etc.
  if (millis() - lastTCTimerLoop >= 250)
  {
    lastTCTimerLoop = millis();
    do250msLoop();  
  }

  doPWM();        // Toggle heater on/off based on power setting
  doFanAjustment(); // Set fan level according to value received from serial

}
