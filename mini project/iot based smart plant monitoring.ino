#include <LiquidCrystal_I2C.h>

/* Connections
Soil.  A0
PIR.   GPIO 5
SDA.   GPIO 21
SCL.   GPIO 22
Temp.  GPIO 4
Buzzer GPIO 15
*/
#define BLYNK_TEMPLATE_ID "TMPL6miCUOrRd"
#define BLYNK_TEMPLATE_NAME "Smart Agriculture"

#include <Wire.h>  // Required for I2C communication
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

// Define SDA and SCL pins for I2C communication
#define SDA_PIN 21  // Define GPIO 21 as SDA
#define SCL_PIN 22  // Define GPIO 22 as SCL

char auth[] = "RrgnAZ5yavxw7g_xJYCK3x56JRcgr8DN";  // Enter your Blynk Auth token
char ssid[] = "iPhone";  // Enter your WIFI SSID
char pass[] = "sherabzangmowoah";  // Enter your WIFI Password

DHT dht(4, DHT11); // Use GPIO 4 for DHT11 sensor
BlynkTimer timer;

// Define component pins for ESP32
#define soil 2    // A0 Soil Moisture Sensor
#define PIR 5       // GPIO 5 PIR Motion Sensor
#define BUZZER 15   // GPIO 15 for Buzzer

LiquidCrystal_I2C lcd(0x27, 16, 2);
WidgetLED LED(V5);  // Declare WidgetLED once globally

void PIRsensor();

void setup() {
  Serial.begin(115200);  // Start serial communication for debugging
  
  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C with GPIO 21 and GPIO 22
  pinMode(PIR, INPUT);
  pinMode(BUZZER, OUTPUT); // Set buzzer pin as output

  dht.begin();  // Initialize DHT sensor

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();

  // Connect to Wi-Fi and display debug messages
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, pass);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // Print the assigned IP address once connected
  Serial.println();
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up the Blynk connection
  Blynk.begin(auth, ssid, pass);

  // Set up the timer for periodic sensor reading
  timer.setInterval(100L, soilMoistureSensor);
  timer.setInterval(100L, DHT11sensor);
  timer.setInterval(500L, PIRsensor);
}

// Get the DHT11 sensor values
void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Print the DHT11 sensor values to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" °C, Humidity: ");
  Serial.print(h);
  Serial.println(" %");

  // Check if temperature exceeds 29°C
  if (t > 29) {
    digitalWrite(BUZZER, HIGH); // Activate buzzer
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pump On!");
  } else {
    digitalWrite(BUZZER, LOW);  // Deactivate buzzer

    // Display temperature and humidity on the LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(t);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(h);
    lcd.print(" %");
  }

  // Send data to Blynk
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
}

// Get the soil moisture values
void soilMoistureSensor() {
  int value = analogRead(soil);
  value = map(value, 0, 1024, 0, 100);
  value = (value - 100) * -1;

  // Print the soil moisture value to the Serial Monitor
  Serial.print("Soil Moisture: ");
  Serial.print(value);
  Serial.println(" %");

  Blynk.virtualWrite(V3, value);
}

// Get the PIR sensor values and update LED status
bool lastMotionState = LOW; // Variable to track last motion state

void PIRsensor() {
  bool motionDetected = digitalRead(PIR);

  // Update only if motion state has changed
  if (motionDetected != lastMotionState) {
    if (motionDetected) {
      Blynk.logEvent("pirmotion", "WARNING! Motion Detected!"); // Log motion event
      LED.on();

      // Print motion detected status to the Serial Monitor
      Serial.println("Motion Detected");
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MOTION DETECTED");
    } else {
      LED.off();

      // Print no motion detected status to the Serial Monitor
      Serial.println("No Motion Detected");
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("NO MOTION DETECTED");
    }
    lastMotionState = motionDetected; // Update the last motion state
  } else {
    Serial.println("PIR sensor state unchanged");  // Corrected Serial output here
  }
}

void loop() {
  // Run Blynk and timer functions
  Blynk.run();  // Run the Blynk library
  timer.run();  // Run the Blynk timer
}
