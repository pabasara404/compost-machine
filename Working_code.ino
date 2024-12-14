#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <ThingSpeak.h>
#include <WiFi.h>

#define MQ135_PIN1 26 //mq135
#define MQ135_PIN2 25 //mq135
#define MQ135_PIN3 33 //mq135
#define MOISTURE_PIN1 34 
#define MOISTURE_PIN2 35 
#define MOISTURE_PIN3 32 
#define ONE_WIRE_BUS1 15
#define ONE_WIRE_BUS2 2
#define ONE_WIRE_BUS3 4
#define MOTOR_PIN 12 // Set LED pin to D12
#define SDA 21 
#define SCL 22

// Wi-Fi credentials
const char* ssid = "PotatoChipishipss";
const char* password = "paba@1234";

// ThingSpeak credentials for writing
unsigned long myChannelNumber = 4;
const char* myWriteAPIKey = "TZX5D3TY2PK6YAIA";

// ThingSpeak credentials for reading from the Control Inputs channel
unsigned long controlChannelID = 2696543;
const char* myReadAPIKey = "SN138ZNN5TFZ0L8U";

const int dry = 595; // value for dry sensor
const int wet = 239;
float tempCelsius; 
unsigned long previousMillis = 0; // Variable to store the last time the sensor was read
const long interval = 5000; // Interval for sensor readings

LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(ONE_WIRE_BUS);	// Setup a oneWire instance to communicate with any OneWire device
DallasTemperature sensors(&oneWire);  // Pass oneWire reference to DallasTemperature library
DeviceAddress sensorAddress = {0x28, 0x40, 0x64, 0x94, 0x97, 0x1, 0x3, 0xFE};

WiFiClient client;
int currentLEDState = LOW;  // Store current LED state

void setup() {
  Serial.begin(9600);
  sensors.begin();
  sensors.setResolution(sensorAddress, 12);
  lcd.init();         // initializing the LCD
  lcd.backlight();    // Enable or Turn On the backlight 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  ThingSpeak.begin(client);

  pinMode(MOTOR_PIN, OUTPUT);  // Set the LED pin as output
  digitalWrite(MOTOR_PIN, LOW); // Start with the LED turned off
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // Read control inputs from ThingSpeak
    int controlValue = ThingSpeak.readIntField(controlChannelID, 4, myReadAPIKey);
  
    if (controlValue == -9999) {
      Serial.println("Failed to read data from ThingSpeak");
    } else {
      Serial.print("Control Input Value: ");
      Serial.println(controlValue);

      // Update LED state only if the control value has changed
      if (controlValue != currentLEDState) {
        if (controlValue == 1) {
          digitalWrite(MOTOR_PIN, HIGH);  // Turn the LED ON
          Serial.println("LED ON");
          currentLEDState = HIGH;
        } else if (controlValue == 0) {
          digitalWrite(MOTOR_PIN, LOW);  // Turn the LED OFF
          Serial.println("LED OFF");
          currentLEDState = LOW;
        }
      }

  if (currentMillis - previousMillis >= interval) {
    
    //Gas Sensor
    int adc_MQ = analogRead(MQ135_PIN); //Analog reading from MQ135
    float voltage = adc_MQ * (5.0 / 1023.0);

    lcd.setCursor(0, 1);
    lcd.print("PPM:");
    lcd.print(adc_MQ);
    Serial.print("PPM:");
    Serial.println(adc_MQ);

    //Moisture Sensor
    int moistureSensorValue = analogRead(MOISTURE_PIN);
    int percentageHumidity = map(moistureSensorValue, wet, dry, 0, 100);

    lcd.setCursor(8, 1);
    lcd.print("MOS:");
    lcd.print(percentageHumidity);
    lcd.print("%");
    Serial.print("MOS:");
    Serial.println(percentageHumidity);

    //Temperature Sensor
    sensors.requestTemperatures();
    sensors.requestTemperaturesByAddress(sensorAddress);
    float temperatureC = sensors.getTempC(sensorAddress);    
    
    lcd.setCursor(0, 0);      // start to print at the first row
    lcd.print("TEMP:");
    Serial.print("TEMP:");
    lcd.print(temperatureC);    // print the temperature in Celsius
    Serial.println(temperatureC);    // print the temperature in Celsius
    lcd.print((char)223);      // print ° character
    lcd.print("C");

    // Upload data to ThingSpeak
    ThingSpeak.setField(1, adc_MQ);
    ThingSpeak.setField(2, percentageHumidity);
    ThingSpeak.setField(3, temperatureC);

    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200) {
      Serial.println("Data uploaded to ThingSpeak successfully");
    } else {
      Serial.println("Failed to upload data to ThingSpeak");
    }

    
    }

    previousMillis = currentMillis;
  }
}
