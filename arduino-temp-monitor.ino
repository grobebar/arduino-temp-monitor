#include <DallasTemperature.h>

#include <OneWire.h>

#include <Wire.h>
#include <rgb_lcd.h>
#include <math.h>

//LCD display
rgb_lcd lcd;
const int lcdLineLength = 16;
const int lcdLines = 2;
const int lcdBufferLength = lcdLineLength+1;
char lcdBuffer[lcdBufferLength];

//Analog inputs
const int pinTempSensor = A0;     // Grove - Temperature Sensor connect to A0
const int tempSensorB = 4275;               // B value of the thermistor
const long tempSensorR0 = 100000;            // R0 = 100k

/*macro definitions of Rotary angle sensor and LED pin*/
const int pinRotaryAngleSensor = A1;
const int rotarySensorAngleMin = 0;
const int rotarySensorAngleMax = 300; //full value of the rotary angle is 300 degrees
const int voltageADC_ref = 5; //reference voltage of ADC is 5v.If the Vcc switch on the seeeduino board switches to 3V3, the ADC_REF should be 3.3
const int voltageGrove_VCC = 5; //VCC of the grove interface is normally 5v


const int pinLightSensor = A2;
int lightValue = 255;
int lightValueThreshold = 16;

//Digital inputs
const int pinButton = 2;     // the number of the pushbutton pin
const int pinTouch = 3;     // the number of the touch pin
const int pinSwitch =  4;      // the number of the LED pin
const int pinWaterTemp = 8;     // the number of the pushbutton pin
OneWire oneWire(pinWaterTemp); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs) 
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 

unsigned long tStart;
unsigned long timeElapsed = 0;

//temperature values populate with some example data until I get the sensors
const int tempPrecision = 10;
const int tempDeltaMin=1;
const int tempDeltaMax=20;
const int tempDeltaStep=1;
int tempDelta=5;
const int tempSetMin=200;
const int tempSetMax=350;
int tempSet=0;
int tempWater = 0;
int tempAmbient = 0;


//from: https://maxpromer.github.io/LCD-Character-Creator/
byte charDelta[] = {0x00, 0x04, 0x04, 0x0A, 0x0A, 0x11, 0x1F, 0x00};

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
  //sensors.begin();
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
  
  tStart=millis()/1000;
  Serial.print("tStart:");
  Serial.println(tStart);
  delay(1000);
}

void loop() {
  Serial.print("millis:");
  Serial.println(millis());
  //check button state to see if it's pressed and then reset the timer
  if (digitalRead(pinButton) == HIGH) {
    tStart=millis()/1000;
  }
  timeElapsed=millis()/1000-tStart;

  //check if touch button is pressed and if so increase the temperature delta value
  if (digitalRead(pinTouch) == HIGH){
    tempDelta+=tempDeltaStep;
    if (tempDelta>tempDeltaMax)
      tempDelta=tempDeltaMin;
  }

  //check the Ambient temperature sensor
  int a = analogRead(pinTempSensor);
  float r = 1023.0/a-1.0;
  // convert to temperature via datasheet
  float temperature = 1.0/(log(r)/tempSensorB+1/298.15)-273.15;
  tempAmbient = temperature*tempPrecision;

  //check the value of rotary knob to adjust Set temperature
  float voltage;
  int sensor_value = analogRead(pinRotaryAngleSensor);
  voltage = (float)sensor_value*voltageADC_ref/1023;
  int degrees = (voltage*rotarySensorAngleMax)/voltageGrove_VCC;
  tempSet = map(degrees, rotarySensorAngleMax, rotarySensorAngleMin, tempSetMin, tempSetMax);


  tempWater = 250;

  
  //read the ambientlight value and change the light value if difference larger than threshold
  int value = analogRead(pinLightSensor);
  value = map(value, 0, 800, 1 , 255);
  if (abs(lightValue - value) > lightValueThreshold){
    lightValue = value;
  }
  
  //set the swtitch to correct state and adjust LCD backlight
  if (tempWater<tempSet){
    digitalWrite(pinSwitch, HIGH);
    lcd.setRGB(lightValue, 0x00, 0x00); 
  }
  else if(tempWater>(tempSet+tempDelta)){
    digitalWrite(pinSwitch, LOW);
    lcd.setRGB(0x00, lightValue, 0x00);
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
