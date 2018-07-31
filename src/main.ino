/**
 *  Ventilation controller
 * Author: Borja García
 * GPL
 * For Arduino Pro Mini (https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/ProMini8MHzv1.pdf)
 */

#include <TM1637Display.h> // https://github.com/avishorp/TM1637
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
volatile float setpoint = 20.0;
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
unsigned long debouncingus = 15000;
volatile unsigned long lastus; // No worries about rollovers http://www.utopiamechanicus.com/article/handling-arduino-microsecond-overflow/


#define DEBOUNCE_OPEN  if((unsigned long)(micros() - lastus) >= debouncingus) {
#define DEBOUNCE_CLOSE lastus = micros();}
void upISR() {
  DEBOUNCE_OPEN 
  setpoint += 1.0;
  show(input, setpoint);
  DEBOUNCE_CLOSE
}
void downISR() {
  DEBOUNCE_OPEN 
  setpoint -= 1.0;
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
  {
    dht.begin();
   // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println("---------------STARTUP--------------");
    Serial.println("Temperature");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
    Serial.println("------------------------------------");
    // Set delay between sensor readings based on sensor details.
    delayms = sensor.min_delay / 1000 + 500;
  }
  // Fan setup
  pinMode(fan_pin, OUTPUT);
  // PID setup
  controller.SetMode(AUTOMATIC);
  controller.SetOutputLimits(0, 255);
  controller.SetSampleTime(delayms) 
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
  }
  else {
    controller.Compute();
    
    analogWrite(fan_pin, output);

    show(input, setpoint);
    Serial.println("Input:%f, Setpoint:%f, Output:%f", input, setpoint, output);
  }
}


