/**
 *  Ventilation controller
 * Author: Borja García
 * GPL
 * For Arduino Pro Mini (https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/ProMini8MHzv1.pdf)
 */

#include <TM1637Display.h> // https://github.com/avishorp/TM1637
#include <Adafruit_Sensor.h> // https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h> // https://github.com/adafruit/DHT-sensor-library
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

// Pins
const int txd_pin = 0;
const int rxd_pin = 1;
const int upBtn_pin = 2; // Because its also INT0
const int downBtn_pin = 3; // Because its also INT1
const int displayClk_pin = 4;
const int displayDio_pin = 5;
const int dht22_pin = 7;
const int fan_pin = 9; // Because is PWM

// Globals
//  PID
volatile double setpoint = 23.0;
double input, output;
double Kp=2, Ki=5, Kd=1;
//  Reverse since the plant tend to rise the temperature, and the action of the PID tends to drop it
PID controller(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

//  DHT
DHT dht(dht22_pin, DHT22);
uint32_t delayms = 2000; // Delay for DHT readings

//  Display
TM1637Display display(displayClk_pin, displayDio_pin);

//  Input buttons
unsigned long debouncingus = 150000;
volatile unsigned long lastus; // No worries about rollovers http://www.utopiamechanicus.com/article/handling-arduino-microsecond-overflow/


#define DEBOUNCE_OPEN  if((unsigned long)(micros() - lastus) >= debouncingus) {
#define DEBOUNCE_CLOSE lastus = micros();}
void upISR() {
  DEBOUNCE_OPEN 
  setpoint += 1.0;
  Serial.println("up");
  show(input, setpoint);
  DEBOUNCE_CLOSE
}
void downISR() {
  DEBOUNCE_OPEN 
  setpoint -= 1.0;
  Serial.println("dwn");
  show(input, setpoint);
  DEBOUNCE_CLOSE
}

void setup() {
  // Serial setup
  Serial.begin(9600); 
  // Input pins setup
  lastus = micros();
  pinMode(upBtn_pin, INPUT);
  pinMode(downBtn_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(upBtn_pin), upISR, RISING);
  attachInterrupt(digitalPinToInterrupt(downBtn_pin), downISR, RISING);
  // Display setup
  display.setBrightness(2);
  // DHT22 setup
  dht.begin();
  // Fan setup
  pinMode(fan_pin, OUTPUT);
  // PID setup
  controller.SetMode(AUTOMATIC);
  controller.SetOutputLimits(0, 255);
  controller.SetSampleTime(delayms);
}

void show(float input, float setpoint) {
  int inputint = int(input);
  int setpointint = int(setpoint);
  display.showNumberDecEx(setpointint*100+inputint, 0b01000000);
}

void loop() {
  // Wait between measurements
  delay(delayms);

  // Obtain the current temperature
  input = dht.readTemperature();
  if (isnan(input)) {
    Serial.println("Failed to read from DHT sensor!");
    display.showNumberDecEx(0, 0b01000000);
  }
  else {
    controller.Compute();
    
    analogWrite(fan_pin, output);

    show(input, setpoint);
    
  }
  Serial.print("Input:");Serial.print(input);Serial.print(", Setpoint:");Serial.print(setpoint);Serial.print(", Output:");Serial.println(output);
}


