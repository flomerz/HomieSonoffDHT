#include <Homie.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define PIN_RELAY   12
#define PIN_LED     13
#define PIN_BUTTON  0
#define DHT_PIN     14

#define DHT_TYPE DHT22
#define DHT_MIN_DELAY_FACTOR 2
#define DEEP_SLEEP_TIME 60 * 1000 * 1000 // 1min
#define SENSOR_MESSURES 5

bool isInitializing = true;

DHT_Unified dht(DHT_PIN, DHT_TYPE);

unsigned long buttonDownTime = 0;
byte lastButtonState = 1;
byte buttonPressHandled = 0;

unsigned short readDHTMillis = 0;
unsigned long lastReadDHTMillis = 0;

const char PROPERTY[] = "state";
const char ON[] = "ON";
const char OFF[] = "OFF";

HomieNode switchNode("switch", "switch");
HomieNode sensorNode("sensor", "sensor");

unsigned char currentSensorMessures = 0;
double temperatureSum = 0;
double humiditySum = 0;

void setState(bool on) {
  const char* state = on ? ON : OFF;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  switchNode.setProperty(PROPERTY).send(state);
  Homie.getLogger() << "Switch is " << state << endl;
}

bool switchStateHandler(HomieRange range, String value) {
  if (value != ON && value != OFF) {
    return false;
  }

  setState(value == ON);
  return true;
}

void buttonPressed() {
  const bool on = digitalRead(PIN_RELAY) == LOW;
  setState(on);
}

void checkButtonPressed() {
  byte buttonState = digitalRead(PIN_BUTTON);
  if ( buttonState != lastButtonState ) {
    if (buttonState == LOW) {
      buttonDownTime     = millis();
      buttonPressHandled = 0;
    }
    else {
      unsigned long dt = millis() - buttonDownTime;
      if ( dt >= 90 && dt <= 900 && buttonPressHandled == 0 ) {
        buttonPressed();
        buttonPressHandled = 1;
      }
    }
    lastButtonState = buttonState;
  }
}

void readDHT() {
  sensors_event_t temperatureEvent;
  sensors_event_t humidityEvent;

  dht.temperature().getEvent(&temperatureEvent);
  dht.humidity().getEvent(&humidityEvent);

  if (isnan(temperatureEvent.temperature) || isnan(humidityEvent.relative_humidity)) {
    Homie.getLogger() << "Error reading sensors!" << endl;
    return;
  }

  currentSensorMessures++;
  temperatureSum += temperatureEvent.temperature;
  humiditySum += humidityEvent.relative_humidity;

  Homie.getLogger() << "Temperature: " << temperatureEvent.temperature << " °C" << endl;
  Homie.getLogger() << "Humidity: " << humidityEvent.relative_humidity << " %" << endl;
}

void resetSensorMessures() {
  currentSensorMessures = 0;
  temperatureSum = 0;
  humiditySum = 0;
}

void publishSensor() {
  sensorNode.setProperty("temperature").send(String(temperatureSum / currentSensorMessures).c_str());
  sensorNode.setProperty("humidity").send(String(humiditySum / currentSensorMessures).c_str());
}

void checckReadSensors() {
  if (millis() - lastReadDHTMillis > readDHTMillis) {
    lastReadDHTMillis = millis();

    readDHT();

    if (currentSensorMessures >= SENSOR_MESSURES) {
      publishSensor();
      resetSensorMessures();
    }
  }
}

void loopHandler() {
  checkButtonPressed();
  checckReadSensors();
}

void onHomieEvent(const HomieEvent& event) {
  switch(event.type) {
    case HomieEventType::MQTT_CONNECTED:
      if (isInitializing) {
        isInitializing = false;
        setState(ON);
      }
      break;
  }
}

void initSensors() {
  // Initialize device.
  dht.begin();
  Homie.getLogger() << ("DHT Unified Sensor");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Homie.getLogger() << "------------------------------------" << endl;
  Homie.getLogger() << "Temperature" << endl;
  Homie.getLogger() << "Sensor:       " << sensor.name << endl;
  Homie.getLogger() << "Driver Ver:   " << sensor.version << endl;
  Homie.getLogger() << "Unique ID:    " << sensor.sensor_id << endl;
  Homie.getLogger() << "Max Value:    " << sensor.max_value << " °C" << endl;
  Homie.getLogger() << "Min Value:    " << sensor.min_value << " °C" << endl;
  Homie.getLogger() << "Resolution:   " << sensor.resolution << " °C" << endl;
  Homie.getLogger() << "------------------------------------" << endl;
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Homie.getLogger() << "------------------------------------" << endl;
  Homie.getLogger() << "Humidity" << endl;
  Homie.getLogger() << "Sensor:       " << sensor.name << endl;
  Homie.getLogger() << "Driver Ver:   " << sensor.version << endl;
  Homie.getLogger() << "Unique ID:    " << sensor.sensor_id << endl;
  Homie.getLogger() << "Max Value:    " << sensor.max_value << " %" << endl;
  Homie.getLogger() << "Min Value:    " << sensor.min_value << " %" << endl;
  Homie.getLogger() << "Resolution:   " << sensor.resolution << " %" << endl;
  Homie.getLogger() << "------------------------------------" << endl;
  // Set delay between sensor readings based on sensor details.
  Homie.getLogger() << "Min Delay: " << sensor.min_delay << endl;
  readDHTMillis = sensor.min_delay * DHT_MIN_DELAY_FACTOR / 1000;

  lastReadDHTMillis = millis();
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;

  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);

  digitalWrite(PIN_RELAY, HIGH);

  initSensors();

  Homie_setFirmware("sonoff-dht", "1.0.0");
  Homie.setLedPin(PIN_LED, LOW).setResetTrigger(PIN_BUTTON, LOW, 10000);

  switchNode.advertise(PROPERTY).settable(switchStateHandler);

  Homie.setLoopFunction(loopHandler);
  Homie.onEvent(onHomieEvent);

  Homie.setup();
}

void loop() {
  Homie.loop();
}
