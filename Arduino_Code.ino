#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <DHT.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <QueueArray.h>

// Pin definitions
#define MQ2_PIN A0
#define MQ7_PIN A1
#define DHTPIN 8
#define DHTTYPE DHT22
#define ULTRASONIC_TRIG 10
#define ULTRASONIC_ECHO 9
#define HEART_RATE_PIN A2
#define DFPlayer_RX_PIN 6
#define DFPlayer_TX_PIN 5

// Define alert IDs
#define ALERT_GAS 1
#define ALERT_HEART_RATE 2
#define ALERT_TEMPERATURE 3
#define ALERT_OBSTACLE 4

// GPS (SoftwareSerial)
TinyGPSPlus gps;
SoftwareSerial gpsSerial(3, 2); // RX, TX for Neo 6M
SoftwareSerial mySoftwareSerial(6, 5); // RX, TX for DFPlayer Mini
SoftwareSerial gsmSerial(18, 19); // RX, TX for GSM module

DFRobotDFPlayerMini myDFPlayer; // Create an instance of the DFPlayer Mini

// DHT22 Setup
DHT dht(DHTPIN, DHTTYPE);

// Queue to store alert events
QueueArray<int> alertQueue(1000);

// DFPlayer Mini state
unsigned long lastPlaybackTime = 0;
const unsigned long playbackDelay = 1000; // Minimum gap between playback

// Data buffer for ESP32
String dataBuffer = "";

// Function prototypes
void addAlert(int alertType);
void processAlerts();
void handleGasAlert();
void handleHeartRateAlert();
void handleTemperatureAlert();
void handleObstacleAlert();
bool isDFPlayerBusy();

// Event handler mapping
typedef void (*AlertHandler)();
AlertHandler alertHandlers[] = {
    nullptr,           // No alert for index 0
    handleGasAlert,    // ALERT_GAS
    handleHeartRateAlert, // ALERT_HEART_RATE
    handleTemperatureAlert, // ALERT_TEMPERATURE
    handleObstacleAlert,  // ALERT_OBSTACLE
};

void setup() {
    Serial.begin(9600);       // Main Serial (Monitor)
    Serial2.begin(9600);      // Start Serial communication with ESP32 
    gpsSerial.begin(9600);      // GPS communication
    mySoftwareSerial.begin(9600); // Communication with DFPlayer Mini
    gsmSerial.begin(9600);       // GSM communication
    dht.begin();
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);

    Serial.println("Arduino Mega is ready.");

    // Initialize DFPlayer Mini
    if (myDFPlayer.begin(mySoftwareSerial)) {
        Serial.println("DFPlayer Mini ready.");
        myDFPlayer.volume(30); // Set volume (0-30)
        myDFPlayer.stop();    // Ensure DFPlayer is stopped on initialization
    }

    // Initialize GSM Module
    gsmSerial.println("AT");   // Test GSM module
    delay(1000);
    gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
    delay(1000);
    gsmSerial.println("AT+CSCS=\"GSM\""); // Set character set
    delay(1000);
    Serial.println("GSM module ready.");
}

void loop() {
    // Sensor readings
    int gas1 = analogRead(MQ2_PIN);
    int gas2 = analogRead(MQ7_PIN);
    float temperature = dht.readTemperature();
    float humidity=dht.readHumidity();
    int distance = readUltrasonic();
    int heartRate = analogRead(HEART_RATE_PIN);

    // Build the data string with actual sensor values
    String data = String(gas1) + "," + String(gas2) + "," + String(temperature) + "," + String(humidity) + "," + String(distance) + "," + String(heartRate) + "," + readGPS() + "\n";
    Serial2.print(data);    // Send data to ESP32

    // Print sensor readings for debugging
    Serial.println("Gas1: " + String(gas1));
    Serial.println("Gas2: " + String(gas2));
    Serial.println("Temperature: " + String(temperature));
    Serial.println("Distance: " + String(distance));
    Serial.println("HeartRate: " + String(heartRate));

    // Trigger alerts based on conditions
    addAlertIfNeeded(gas1 > 400 || gas2 > 400, ALERT_GAS);
    addAlertIfNeeded(heartRate > 1200, ALERT_HEART_RATE);
    addAlertIfNeeded(temperature > 30, ALERT_TEMPERATURE); // Adjust threshold as needed
    addAlertIfNeeded(distance > 0 && distance < 300, ALERT_OBSTACLE);

    // Process alerts sequentially
    processAlerts();

    delay(100); // Stability delay
}

// Function to conditionally add an alert to the queue
void addAlertIfNeeded(bool condition, int alertType) {
    if (condition) {
        addAlert(alertType);
    }
}

// Function to add an alert to the queue
void addAlert(int alertType) {
    if (!alertQueue.isFull()) {
        alertQueue.enqueue(alertType);
        Serial.println("Alert added: " + String(alertType));
    } else {
        Serial.println("Alert queue is full.");
    }
}

// Function to process alerts
void processAlerts() {
    if (!alertQueue.isEmpty() && (millis() - lastPlaybackTime >= playbackDelay)) {
        int alertType = alertQueue.dequeue();
        Serial.println("Processing alert: " + String(alertType));

        if (alertHandlers[alertType]) {
            alertHandlers[alertType]();
            lastPlaybackTime = millis(); // Update the playback time
        }
    }
}

// Alert handlers
void handleGasAlert() {
    Serial.println("Handling Gas Alert!");
    myDFPlayer.play(2);
    sendSMS("Dangerous gas levels detected!");
}

void handleHeartRateAlert() {
    Serial.println("Handling Heart Rate Alert!");
    myDFPlayer.play(4);
    sendSMS("High Heart Rate Detected!");
}

void handleTemperatureAlert() {
    Serial.println("Handling Temperature Alert!");
    myDFPlayer.play(1);
    sendSMS("High Temperature Detected!");
}

void handleObstacleAlert() {
    Serial.println("Handling Obstacle Alert!");
    myDFPlayer.play(3);
    sendSMS("Obstacle Detected Close!");
}

// Function to read distance from ultrasonic sensor
int readUltrasonic() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH);
    return duration * 0.034 / 2; // Convert to cm
}

// Function to read GPS data (not used in current implementation)
String readGPS() {
    String googleMapsLink = "";
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);
        Serial.print(c);  // Debugging: Print raw GPS data to Serial Monitor
    }
    if (gps.location.isValid()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        Serial.print("Latitude: "); 
        Serial.println(latitude, 6);
        Serial.print("Longitude: "); 
        Serial.println(longitude, 6);
    } else {
        Serial.println("Waiting for GPS signal...");
        googleMapsLink = "No GPS Signal";
    }
    return googleMapsLink;
}


// Function to send SMS using GSM module
void sendSMS(String message) {
    gsmSerial.println("AT+CMGS=\"+9449462809\""); // Replace with recipient's phone number
    delay(1000);
    gsmSerial.println(message); // Send the message content
    delay(1000);
    gsmSerial.write(26); // ASCII code for CTRL+Z to send the SMS
    delay(1000);
    Serial.println("SMS Sent: " + message);
}


