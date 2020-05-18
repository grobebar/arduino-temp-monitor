#include <Wire.h>
#include <rgb_lcd.h>
#include <math.h>

#include <OneWire.h>
#include <DallasTemperature.h>

//LCD display
rgb_lcd lcd;
const int lcdLineLength = 16;
const int lcdLines = 2;
const int lcdBufferLength = lcdLineLength+1;
char lcdBuffer[lcdBufferLength];
int lightValue = 255;
int lightValueThreshold = 16;

//Analog inputs
const int analogMin = 0;
const int analogMax = 1023; //max value of the analog read

// Grove - Temperature Sensor V1.2 - https://wiki.seeedstudio.com/Grove-Temperature_Sensor_V1.2/
const int pinTempSensor = A0;     // Grove - Temperature Sensor connect to A0

// Grove - Rotary Angle Sensor - https://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/
const int pinRotaryAngleSensor = A1;

// Grove - Light Sensor - https://wiki.seeedstudio.com/Grove-Light_Sensor/
const int pinLightSensor = A2;



//Digital inputs
const int pinButton = 2;     // the number of the pushbutton pin
const int pinTouch = 3;     // the number of the touch pin
const int pinSwitch =  4;      // the number of the LED pin
const int pinWaterTemp = 8;     // the number of the pushbutton pin
OneWire oneWire(pinWaterTemp); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs) 
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress tempDeviceAddress;
DeviceAddress* myArray = 0;


unsigned long timeStart = 0;
unsigned long timeElapsed = 0;
bool heating = false;

//temperature values 
const int tempPrecision = 10; //instead of using floats I'm using int multiplied by this number to get additional decimal places
const int tempDeltaMin=1;
const int tempDeltaMax=20;
const int tempDeltaStep=1;
int tempDelta=5;
const int tempSetMin=200;
const int tempSetMax=350;
int tempSet=0;
int tempWater = 0;
int tempAmbient = 0;
int digitalTempDelayInMillis = 0;
int digitalTempResolution = 12; // for digital temperature sensor
unsigned long digitalTempLastRequest = 0;


//from: https://maxpromer.github.io/LCD-Character-Creator/
//byte charDelta[] = {0x00, 0x04, 0x04, 0x0A, 0x0A, 0x11, 0x1F, 0x00};

#if defined(ARDUINO_ARCH_AVR)
#define debug  Serial
#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
#define debug  SerialUSB
#else
#define debug  Serial
#endif


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(lcdLineLength, lcdLines);
  lcd.setColor(BLUE);
  lcd.print("Welcome!");


  // Start up the sensors library for DS18B20 temperature sensor
  sensors.begin();
  int digitalTempSensors = sensors.getDeviceCount();
  
  myArray = new DeviceAddress [digitalTempSensors];
  Serial.println(sizeof(myArray) / sizeof(int));
  snprintf(lcdBuffer, lcdBufferLength, "Found %d DS18...", digitalTempSensors);
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer);

  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, digitalTempResolution);
  //digitalTempDelayInMillis = 750 / (1 << (12 - digitalTempResolution)); 
  digitalTempDelayInMillis = sensors.millisToWaitForConversion(digitalTempResolution); 
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  digitalTempLastRequest = millis(); 
  //more code from: https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
  // and here: https://create.arduino.cc/projecthub/iotboys/how-to-use-ds18b20-water-proof-temperature-sensor-2adecc
  
  //initialize Ambient temp pin as input
  pinMode(pinTempSensor, INPUT);
  // initialize the rotary pin as input:
  pinMode(pinRotaryAngleSensor, INPUT);
  // initialize the rotary pin as input:
  pinMode(pinLightSensor, INPUT);
  // initialize the pushbutton pin as an input:
  pinMode(pinButton, INPUT);
  // initialize the Touch pin as an input:
  pinMode(pinTouch, INPUT);
  // initialize the Switch pin as an output:
  pinMode(pinSwitch, OUTPUT);
  
  delay(1000);
}

void loop() {
  Serial.print("millis:");
  Serial.println(millis());
  //check button state to see if it's pressed and then reset the timer
  if (digitalRead(pinButton) == HIGH) {
    timeStart=seconds();
  }
  timeElapsed=seconds()-timeStart;

  //check if touch button is pressed and if so increase the temperature delta value
  if (digitalRead(pinTouch) == HIGH){
    tempDelta+=tempDeltaStep;
    if (tempDelta>tempDeltaMax)
      tempDelta=tempDeltaMin;
  }

  //get value from the Ambient temperature sensor
  tempAmbient = groveSensorToCelsius(analogRead(pinTempSensor));

  //get value of rotary knob to adjust Set temperature
  int sensor_value = analogRead(pinRotaryAngleSensor);
  tempSet = map(sensor_value, analogMax, analogMin, tempSetMin, tempSetMax);


  //tempWater = 260;
  if (millis() - digitalTempLastRequest >= digitalTempDelayInMillis){
    tempWater = sensors.getTempC(tempDeviceAddress) * tempPrecision;
    sensors.requestTemperatures(); 
    digitalTempLastRequest = millis(); 
  }
  
  //set the swtitch to correct state and adjust LCD backlight
  if (tempWater<tempSet){
    //turn on heating
    heating = true;
    digitalWrite(pinSwitch, HIGH);
    
  }
  else if(tempWater>(tempSet+tempDelta)){
    //turn off heating
    heating = false;
    digitalWrite(pinSwitch, LOW);
  }

  //read the ambientlight value and change the light value if difference larger than threshold
  int lightValueNew = map(analogRead(pinLightSensor), 0, 800, 1 , 255);
  if (abs(lightValue - lightValueNew) > lightValueThreshold){
    lightValue = lightValueNew;
  }
  if (heating){
    lcd.setRGB(lightValue, 0, 0); 
  }
  else{
    lcd.setRGB(0, lightValue, 0);
  }
  

// write on the LCD
// "________________"
// "t 00:00:00  D0.1"
// "00.0 W00.0 A00.0"

  
  
  snprintf(lcdBuffer, lcdBufferLength, "t%3d:%02d:%02d  D%d.%d", (int)(timeElapsed/3600),(int)((timeElapsed%3600)/60),(int)(timeElapsed%60), tempDelta/10, tempDelta%10);
  lcd.setCursor(0, 0);
  lcd.print(lcdBuffer);
  snprintf(lcdBuffer, lcdBufferLength, "%02d.%d W%02d.%d A%02d.%d", tempSet/10, tempSet%10, tempWater/10, tempWater%10, tempAmbient/10, tempAmbient%10);
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer);

  delay(100);
}

unsigned long seconds() {
  return millis()/1000;
}

int groveSensorToCelsius(int value) {
  const int tempSensorB = 4275;                   // B value of the thermistor
  const long tempSensorR0 = 100000;               // R0 = 100k
  const float tempSensorT0 = 298.15;              // T0 Temperature for R0
  const int temp0 = 273.15*tempPrecision;         // 0C in Kelvin for conversion
  const float BbyT0 = tempSensorB/tempSensorT0;
  return (int)(tempSensorB/(log((float)analogMax/value-1)+BbyT0)*tempPrecision)-temp0;
}
