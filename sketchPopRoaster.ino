#include "Max6675.h"

#define DP 1  // No. decimal places for serial output

#define maxLength 30  // maximum length for strings used

#define maxTemperature 220  // maximum temperature for hardware protection


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

const int fanPin = 9;  // pin used on mosfet gate that will control fan
const int pwmPin = 11;  // pin for pulse width modulation of heater (SSR VCC)
const int ssrGnd = 12;  // pin used for SSR GND
Max6675 therm(3, 4, 5);  // thermocouple reading Max 6675 pins

/****************************************************************************
 *  After setting the above pin assignments you can use the remainder of this
 *   sketch as is or change it, if you wish, to add additional functionality
 * 
 ****************************************************************************/

// time constants
const int timePeriod = 200;           // total time period of PWM milliseconds see note on setupPWM before changing
const int tcTimePeriod = 200;         // 200 ms loop to read thermocouples

//temporary values for temperature to be read
float temp = 0.0;                   // temporary temperature variable
float t1 = 0.0;                      // Last average temperature on thermocouple 1 - average of four readings
float tCumulative = 0.0;            // cumulative total of temperatures read before averaged
int noGoodReadings = 0;             // counter of no. of good readings for average calculation

int inByte = 0;                       // incoming serial byte
String inString = String(maxLength);  // input String


// loop control variables
unsigned long lastTCTimerLoop;        // for timing the thermocouple loop
int tcLoopCount = 0;                  // counter to run serial output once every 1 sec of t/c iterations

// PWM variables
int  timeOn;                          // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
unsigned long lastTimePeriod;         // millis since last turned on pwm
int power = 100;                      //use as %, 100 is always on, 0 always off default 100
int lastPowerInput = 100;

int fan = 0;

/****************************************************************************
 * General setup
 ****************************************************************************/
void setup() {
  // start serial port at 115200 baud:
  Serial.begin(115200);

  setupPWM();

  //Set up fan pin
  pinMode(fanPin, OUTPUT); analogWrite(fanPin, 0);
  
  Serial.println("Reset");  // flag that Arduino has reset used for debugging 
}

/****************************************************************************
 * Heater setup
 ****************************************************************************/
void setupPWM() {
  //Set up pin for GND_SSR
  pinMode(ssrGnd, OUTPUT); digitalWrite(ssrGnd, LOW);

  // set the digital pin as output (VCC_SSR)
  pinMode(pwmPin, OUTPUT);

  // setup PWM
  lastTimePeriod = millis();
  digitalWrite(pwmPin, LOW);//set PWM pin off to start
}

/****************************************************************************
 * Toggles the heater on/off based on the current power level.
 ****************************************************************************/
void doPWM() {
  timeOn = timePeriod * power / 100;  //recalc the millisecs on to get this power level, user may have changed
 
 if (millis() - lastTimePeriod > timePeriod) lastTimePeriod = millis();

 if (millis() - lastTimePeriod < timeOn) {
      digitalWrite(pwmPin, HIGH);  // turn on
  } else {
      digitalWrite(pwmPin, LOW);  // turn off
 }
 
}

/****************************************************************************
 * Protected function to set heater power
 * Also applies limited power feature if enabled
 ****************************************************************************/
void setPowerLevel(int p) {
    // SSR change state delay compensation
    // This delay may take up to 20ms, so if the power is too low or too high just round it.
    if (p < 5) {
      p = 0;
    }
    if (p > 95) {
      p = 100;
    }
    // Hardware protection
    // protection against fan failure - turn off heater if fan is off
    if (fan == 0) {
      p = 0;
    }
    // protection against high temperatures - turn off heater if temperature reaches limit
    if (t1 > maxTemperature) {
      p = 0;
    }

    if (p > -1 && p < 101) power = p;
}

/****************************************************************************
 * Protected function to set fan power
 ****************************************************************************/
void setFanLevel(int p) {
   if (p > -1 && p < 101) fan = p;
}

/****************************************************************************
 * Set fan power to reasonable values for bean rotation.
 * Set fan off if zero power and work on a higher setting otherwise
 ****************************************************************************/
void doFanAjustment() {
   if (fan == 0) {
      analogWrite(fanPin, 0);
   } else
   if (fan > 0 && fan <= 100) {
        double fanStep = 1.5;
        analogWrite(fanPin, 105 + (fanStep * fan));
   }
}

/****************************************************************************
 * Called when an input string is received from computer
 * designed for key=value pairs or simple text commands. 
 * Performs commands and splits key and value 
 * and if key is defined sets value otherwise ignores
 ****************************************************************************/
void doInputCommand() {
  float v = -1;
  inString.toLowerCase();
  int indx = inString.indexOf('=');

  if (indx >= 0) {  //this is a key value pair for decoding
    String key = inString.substring(0, indx);
    String value = inString.substring(indx+1, inString.length());

    //parse string value and return float v
    char buf[value.length()+1];
    value.toCharArray(buf,value.length()+1);
    v = atof (buf); 
 
    //only set value if we have a valid positive number - atof will return 0.0 if invalid
    if (v >= 0) {
      if (key.equals("power")) {

        lastPowerInput = (long) v;  
        
        setPowerLevel((long) v);  //convert v to integer for power 
                
      } else
      if (key.equals("fan")) {
        setFanLevel((long) v);  //convert v to integer for fan
      }
    }
  }
}

/****************************************************************************
 * check if serial input is waiting if so add it to inString.  
 * Instructions are terminated with \n \r or 'z' 
 * If this is the end of input line then call doInputCommand to act on it.
 ****************************************************************************/
void getSerialInput() {
  // check if data is coming in if so deal with it
  if (Serial.available() > 0) {

    // read the incoming data as a char:
    char inChar = Serial.read();
    // if it's a newline or return or z, print the string:
    if ((inChar == '\n') || (inChar == '\r') || (inChar == 'z')) {

      //do whatever is commanded by the input string
      if (inString.length() > 0) doInputCommand();
      inString = "";        //reset for next line of input
    } else {
      // if we are not at the end of the string, append the incoming character
      if (inString.length() < maxLength) {
        inString += inChar;
      } else {
        // empty the string and set it equal to the incoming char
        inString = "";
        inString += inChar;
      }
    }
  }
}

/****************************************************************************
 * Sends data to Roastlogger
 ****************************************************************************/
void doSerialOutput() {
  //send data to logger

  Serial.print("t1=");
  Serial.println(t1,DP);

  Serial.print("power%=");
  Serial.println(power);

  Serial.print("fan%=");
  Serial.println(fan);
}

/****************************************************************************
 * Read temperatures from Max6675 chips Sets t1, -1 if an error
 * occurred.  Max6675 needs 240 ms between readings or will return last
 * value again. I am reading it once per second.
 ****************************************************************************/
void getTemperatures() {
 
 temp = therm.getCelsius();
  
 if (temp > 0.0) {
    tCumulative = tCumulative + temp;
     noGoodReadings ++;
 }
}

/****************************************************************************
 * Used to read thermocouple
 *
 * Once per second averages temperature results and outputs data
 * to serial port.
 ****************************************************************************/
void doThermocoupleLoop() {

    getTemperatures();
           
    if (tcLoopCount > 3) {
      // once every four loops (1 second) calculate average temp and do serial output
      
      tcLoopCount = 0;
      
      if (noGoodReadings > 0)  t1 = tCumulative / noGoodReadings; else t1 = -1.0;
      noGoodReadings = 0;
      tCumulative = 0.0;
      
      doSerialOutput();  // once per second

      setPowerLevel(lastPowerInput);  // update power level at least once per second
    }
    tcLoopCount++;

}


/****************************************************************************
 * Main loop must not use delay!  PWM heater control relies on loop running
 * at least every 40 ms.  If it takes longer then heater will be on slightly
 * longer than planned. Not a big problem if 1% becomes 1.2%! But keep loop fast.
 * Currently loop takes about 4-5 ms to run so no problem.
 ****************************************************************************/
void loop() {

  getSerialInput();  // check if any serial data waiting

  if (millis() - lastTCTimerLoop >= tcTimePeriod) {
    lastTCTimerLoop = millis();
    doThermocoupleLoop();  
  }

  doPWM();  // Toggle heater on/off based on power setting
  doFanAjustment();  // Set fan level according to value received from serial
}
