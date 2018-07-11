/**
 * GPL
 * For Arduino Pro Mini (https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/ProMini8MHzv1.pdf)
 */

#include <TM1637.h>
#include <DHT.h>

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
float currentCelsius = 20.0;

uint8_t fan_power = 0x00;

TM1637 display(displayClk,displayDio);
int8_t display_data = {0,0,0,0};

volatile float desiredCelsius = 20.0;
unsigned long debouncingus = 15000;
volatile unsigned long lastus; // No worries about rollovers http://www.utopiamechanicus.com/article/handling-arduino-microsecond-overflow/


#define DEBOUNCE_OPEN if((unsigned long)(micros() - lastus) >= debouncingus) {
#define DEBOUNCE_CLOSE     lastus = micros();}
void upISR() {
  DEBOUNCE_OPEN 
  desiredCelsius += 1.0;
  DEBOUNCE_CLOSE
}
void downISR() {
  DEBOUNCE_OPEN 
  desiredCelsius -= 1.0;
  DEBOUNCE_CLOSE
}

void adaptDisplayData(t, &display_data) {
  
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
  display.set();
  display.init();
  // DHT22 setup
  {
    dht.begin();
   // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.println("Temperature");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
    Serial.println("------------------------------------");
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.println("Humidity");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
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

  // Regulate
  float err = t-desiredCelsius;
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

  // Output the desired or current temperature
  if (desiredCelsiusDisplay > 0) {
    adaptDisplayData(desiredCelsius, &display_data);
    desiredCelsiusDisplay--;
  }
  else {
    adaptDisplayData(t, &display_data);
  }
  display.display(display_data);

  // Finally do what you must to minimize the error
  analogWrite(fan, fan_power);
}


