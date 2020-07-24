/* Libraries */
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Adafruit_seesaw.h"

/* Constants */
// There are two type of multi-LEDs. common cathode and common anode.
// We keep this define flag if the multi-LED uses a common anode. Otherwise, comment the line below out!
#define COMMON_ANODE  
#define DHTPIN 2
#define DHTTYPE DHT22
#define TEMPPIN A0 // For TMP36
DHT DHT(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino

#define REDPIN1 5
#define GREENPIN1 6
#define BLUEPIN1 7

#define REDPIN2 8
#define GREENPIN2 9
#define BLUEPIN2 10

#define REDPIN3 11
#define GREENPIN3 12
#define BLUEPIN3 13

#define FANPIN1 45 // This is really connected to the NPN Transistor
#define FANPIN2 47
#define PUMPPIN 27 

#define buttonPin1 18 // The number of the pushbutton pin for fan. There is an interupt on them
#define buttonPin2 19 // The pin for pump. There is an interrupt 

//USER INPUT HUMIDITY AND TEMPERATURE
double hum_low = 50;
double hum_high = 70;
double temp_low = 18.3;
double temp_high = 29.4;


/* Function Prototypes */
double readTemperature(void);
void setColor(int red, int green, int blue, int LEDNum);
void setHumidityLED(double humVal, double lowVal, double highVal);
void setTempLED(double tempVal, double lowVal, double highVal);
void checkFan(double humVal,double tempval, double lowhum, double highhum, double lowtemp, double hightemp);

/* Global Variables */
double humidity = 0;
double tempDHT = 0; // Temperature from DHT22
double tempThermo = 0; // Temperature from TMP36
double tempSystem = 0; // Average of DHT22 and TMP26 sensors 
Adafruit_seesaw ss;
float tempSS = 0;
uint16_t capread = 0; // Capacitive reading from soil sensor  
int fanState = 0; // If  == 1 then fan is on. 
int pumpState = 0;
int overrideFan = 0;
int overridePump = 0; 

/* setup() */
void setup() 
{
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(FANPIN1, OUTPUT);
  pinMode(FANPIN2, OUTPUT);
  pinMode(PUMPPIN, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize serial output
  DHT.begin(); // Initialize DHT sensor

  if (!ss.begin(0x36)) 
  {
    Serial.println("ERROR! seesaw not found");
  } 
  else 
  {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }

  digitalWrite(FANPIN1, LOW);
  digitalWrite(FANPIN2, LOW);
  digitalWrite(PUMPPIN, LOW);
  attachInterrupt(digitalPinToInterrupt(buttonPin1), interruptFan, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), interruptPump, RISING);
  delay(200);
}

/* loop() */
void loop() 
{
  // put your main code here, to run repeatedly:
  Serial.print("Humidity: ");
  humidity = DHT.readHumidity();
  setHumidityLED(humidity, hum_low, hum_high); /* TODO: Change these values */
  Serial.print(humidity);
  Serial.println(" %, ");

  Serial.print("Temp from humidity sensor: ");
  tempDHT = DHT.readTemperature();
  Serial.print(tempDHT);
  Serial.println(" Celsius");

  // Serial.print("Temp from temperature sensor: ");
  // tempThermo = readTemperature();
  // Serial.print(tempThermo);
  // Serial.println(" Celsius");

  tempSS = ss.getTemp();
  capread = ss.touchRead(0);
  Serial.print("Temperature from soil senor: "); Serial.print(tempSS); Serial.println("ºC");
  Serial.print("Capacitive: "); Serial.println(capread);

  /* Calculate temperature of entire system */
  tempSystem = (tempDHT  + tempSS) /  2;
  
  /* If there is an error reading the sensors */
  if(isnan(tempDHT) || isnan(tempSS))
  {
    Serial.println("Reading Failed");
    delay(1000);
    // return(1);
  }
  setTempLED(tempSystem, (double) temp_low, (double) temp_high); // TODO: Change these values. This is in celsius
  if(!overrideFan) /* If ISR happens then this code doesn't run anymore */
  {
    checkFan(humidity, tempSystem, (double) hum_low, (double) hum_high, (double) temp_low, (double) temp_high); /* TODO: Change these values */
  }

  if(!overridePump)
  {
    checkPump(humidity, tempSystem, (double) hum_low, (double) hum_high, (double) temp_low, (double) temp_high);
  }

  if(pumpState)
  {
    digitalWrite(PUMPPIN, HIGH);
    Serial.println("Pump is on");
  }
  else if(!pumpState)
  {
    digitalWrite(PUMPPIN, LOW);
    Serial.println("Pump is off");
  } 

  if(fanState)
  {
    digitalWrite(FANPIN1, HIGH);
    digitalWrite(FANPIN2, HIGH);
    Serial.println("Fan is on");
  }
  else if(!fanState)
  {
    digitalWrite(FANPIN1, LOW);
    digitalWrite(FANPIN2, LOW);
    Serial.println("Fan is off");
  } 

  delay(2000); // Delay 2 sec. This is the time it takes for the DHT22 to refresh 
}

/* Functions */
double readTemperature(void)
{
  float voltage = getVoltage(TEMPPIN);
  
  // Now we'll convert the voltage to degrees Celsius.
  // This formula comes from the temperature sensor datasheet:

  float degreesC = (voltage - 0.5) * 100.0;
  return(degreesC);
}

void setHumidityLED(double humVal, double lowVal, double highVal)
{
  if(humVal < lowVal || capread < 300) // Turn LED Blue
    setColor(0, 0, 255, 1); 
  else if(lowVal <= humVal && humVal <= highVal) // Turn LED Green
    setColor(0, 255, 0, 1);
  else
    setColor(255, 0, 0, 1);  // Turn LED Red
}

void setTempLED(double tempVal, double lowVal, double highVal)
{
  if(tempVal < lowVal) // Turn LED Blue
    setColor(0, 0, 255, 2); 
  else if((lowVal <= tempVal && tempVal <= highVal) ||(300 <= capread && capread <= 700)) // Turn LED Green
    setColor(0, 255, 0, 2);
  else
    setColor(255, 0, 0, 2);  // Turn LED Red
}

void setColor(int red, int green, int blue, int LEDNum)
{
  int redPin = 0;
  int greenPin = 0;
  int bluePin = 0;
  /* Designate which LED to light up */
  if(LEDNum == 1)
  {
    redPin = REDPIN1;
    greenPin = GREENPIN1;
    bluePin = BLUEPIN1;
  }
  else if(LEDNum == 2)
  {
    redPin = REDPIN2;
    greenPin = GREENPIN2;
    bluePin = BLUEPIN2; 
  }
  else if(LEDNum == 3)
  {
    redPin = REDPIN3;
    greenPin = GREENPIN3;
    bluePin = BLUEPIN3;
  }
  else // This should never happen
  {
    Serial.println("Error. Invalid argument for LEDNum. LEDNum must be from 1 to 3");
    int redPin = -1;
    int greenPin = -1;
    int bluePin = -1;
  }


  #ifdef COMMON_ANODE // common anode LEDs are weird 
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  /* Actually change the colors of the LEDs with PWM*/
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

void checkFan(double humval, double tempval, double lowhum, double highhum, double lowtemp, double hightemp)
{
  if ((humval > highhum) || (tempval > hightemp))
  {
    Serial.println("Fan is now ON");
    fanState = 1;
  }
  else
  {
    Serial.println("Fan is now OFF");
    fanState = 0;
  }
}

void checkPump(double humval, double tempval, double lowhum, double highhum, double lowtemp, double hightemp)
{
  if ((humval < lowhum) || capread < 300)
  {
    Serial.println("Pump is now ON");
    pumpState = 1;
  }
  else
  {
    Serial.println("Pump is now OFF");
    pumpState = 0;
  }
}

void interruptFan()
{
  /* This interrupt should be quick. Don't add too much */
  overrideFan = 1;
  Serial.println("Fan button is pushed");
  fanState = !fanState;
  delay(300);
}

void interruptPump()
{
  /* Interrupt for the pump */ 
  overridePump = 1;
  Serial.println("Pump button is pushed");
  pumpState = !pumpState;
  delay(300);
}

float getVoltage(int pin)
{
  // This function has one input parameter, the analog pin number
  // to read. You might notice that this function does not have
  // "void" in front of it; this is because it returns a floating-
  // point value, which is the true voltage on that pin (0 to 5V).
  
  // You can write your own functions that take in parameters
  // and return values. Here's how:
  
    // To take in parameters, put their type and name in the
    // parenthesis after the function name (see above). You can
    // have multiple parameters, separated with commas.
    
    // To return a value, put the type BEFORE the function name
    // (see "float", above), and use a return() statement in your code
    // to actually return the value (see below).
  
    // If you don't need to get any parameters, you can just put
    // "()" after the function name.
  
    // If you don't need to return a value, just write "void" before
    // the function name.

  // Here's the return statement for this function. We're doing
  // all the math we need to do within this statement:
  
  return (analogRead(pin) * 0.004882814);
  
  // This equation converts the 0 to 1023 value that analogRead()
  // returns, into a 0.0 to 5.0 value that is the true voltage
  // being read at that pin.
}
