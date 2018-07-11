/**
 *  Ventilation controller
 * Author: Borja García
 * GPL
 * For Arduino Pro Mini (https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/ProMini8MHzv1.pdf)
 */

#include <TM1637Display.h> // https://github.com/avishorp/TM1637
#include <DHT.h> // https://github.com/adafruit/DHT-sensor-library

// Pins
const int txd = 0;
const int rxd = 1;
const int upBtn = 2; // Because its also INT0
const int downBtn = 3; // Because its also INT1
const int displayClk = 4;
const int displayDio = 5;
const int dht22 = 7;
const int fan = 9; // Because is PWM

// Globals
DHT dht(dht22, DHT22);
uint32_t delayms = 2000; // Delay for DHT readings
TM1637Display display(displayClk,displayDio);
uint8_t fan_power = 0x00;

volatile float desiredt = 20.0;
volatile int desiredtDisplay = 0;
unsigned long debouncingus = 15000;
volatile unsigned long lastus; // No worries about rollovers http://www.utopiamechanicus.com/article/handling-arduino-microsecond-overflow/


#define DEBOUNCE_OPEN if((unsigned long)(micros() - lastus) >= debouncingus) {
#define DEBOUNCE_CLOSE     lastus = micros();desiredtDisplay=2;}
void upISR() {
  DEBOUNCE_OPEN 
  desiredt += 1.0;
  DEBOUNCE_CLOSE
}
void downISR() {
  DEBOUNCE_OPEN 
  desiredt -= 1.0;
  DEBOUNCE_CLOSE
}

void setup() {
  // Serial setup
  Serial.begin(9600); 
  // Input pins setup
  lastus = micros();
  pinMode(upBtn, INPUT);
  pinMode(downBtn, INPUT);
  attachInterrupt(digitalPinToInterrupt(upBtn), upISR, RISING);
  attachInterrupt(digitalPinToInterrupt(downBtn), downISR, RISING);
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
  pinMode(fan, OUTPUT);
}

void loop() {
  // Wait between measurements
  delay(delayms);

  // Obtain the current temperature
  float t = dht.readTemperature();
  if (isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  else {
    // Regulate
    float err = t-desiredt;
    if (err<0.0) {
      fan_power=(uint8_t)0*256/100;
    }
    if (0.0<=err && err<1.0) {
      fan_power=(uint8_t)25*256/100;
    }
    if (1.0<=err && err<2.0) {
      fan_power=(uint8_t)50*256/100;
    }
    if (2.0<=err && err<3.0) {
      fan_power=(uint8_t)75*256/100;
    }
    if (3.0<=err) {
      fan_power=(uint8_t)100*256/100;
    }
    analogWrite(fan, fan_power);

    // Output the desired or current temperature
    if (desiredtDisplay > 0) {
      display.setBrightness(7);
      display.showNumberDecEx(desiredt, 0b10100000, false, 4, 1);
      display.setSegments(0b00000010, 1, 0);
      display.setSegments(0b00000010, 1, 3);
      desiredtDisplay--;
    }
    else {
      display.setBrightness(2);
      display.showNumberDec(t);
    }
  }
}


