#include "TalkieUtils.h"
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include "HCSR04.hpp"
#include "I2Cdev.h"
#include "MPU6050.h"

//UART library
#include <SoftwareSerial.h>

//MPU6050
//SCL   22
//SDA   21

//HC-SR04
//TRIG  4
//ECHO  0

//SIM800L
//RX    17
//TX    16

//GPS
//RX    18
//TX    19

//button
//S     26

//Speaker
//S     25
#define button_pin 26
#define ECHO_IN_PIN 2
#define TRIGGER_OUT_PIN 4
#define tiltAngle -35  // angle to switch detection range from 1m to 1.8m

#define gps_tx 18 //
#define gps_rx 19 

#define sim_tx 17 //rx on gsm
#define sim_rx 16 //tx on gsm

// instantiate libraries/modules
Talkie Voice;
MPU6050 accelgyro;
SoftwareSerial gps_serial(gps_rx, gps_tx);
SoftwareSerial sim_serial(sim_rx, sim_tx);

// gyro variables
int16_t ax, ay, az;  // acceleration axis x, y, and z
double detectionRange = 0;
float duration;

int press_counter;
bool press_enable;
long int time_interval;
String dateUTC, timeUTC;
float lat, lng;

void updateSerial() {
    delay(500);
    while (Serial.available()) sim_serial.write(Serial.read());//Forward what Serial received to Software Serial Port
    while(sim_serial.available()) Serial.write(sim_serial.read());//Forward what Software Serial received to Serial Port
}

void checkButtonState() {
    if(!digitalRead(button_pin) && !press_enable) { //active LOW
        time_interval = millis();
        press_enable = 1;
        press_counter++;
        Serial.println(press_counter);
    }
    else if(digitalRead(button_pin) && press_enable) {
        press_enable = 0;
    }

    if(press_counter == 4) {                //4 presses = Send emergency SMS
        //getGPSCoordinate();
        sendSMS();
        press_counter = 0;
    }

    if((millis() - time_interval) > 5000) { //5 secs before the press counter resets
        press_counter = 0;
    }
}

void setup() {

    Serial.begin(9600);  // initialize serial communication to computer
    sim_serial.begin(9600); 
    gps_serial.begin(9600);
    
    // initialize components
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);  // set pins for HCSR04 ultrasonic sensor
    Wire.begin();
    accelgyro.initialize();  // initialize gyroscope
    
    pinMode(button_pin, INPUT_PULLUP);
    //pinMode(LED_BUILTIN, OUTPUT);  // set LED_BUILTIN mode as output (unnecessary, debug purposes only)

    press_counter = 0;
    press_enable = 0;
    time_interval = 0;
}

unsigned long lastGPSTime = 0;
unsigned long lastSensorTime = 0;
const unsigned long gpsUpdateInterval = 1000; // Update GPS every 600 ms
const unsigned long sensorUpdateInterval = 500; // Update sensors every 500 ms

void sendSMS() { //*    
    delay(100);
    sim_serial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    sim_serial.println("AT+CMGS=\"+639664132090\"");
    //+639664132090
    //+639151635499
    updateSerial();
    sim_serial.print(lat, 6);
    sim_serial.print(",");
    sim_serial.print(lng, 6);
    sim_serial.print(char(26));
    updateSerial();

    delay(3500);
    Serial.println("sending coordinates...");

    sim_serial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    sim_serial.println("AT+CMGS=\"+639664132090\"");
    updateSerial();
    sim_serial.print("(");
    sim_serial.print(dateUTC);
    sim_serial.print(", ");
    sim_serial.print(timeUTC);
    sim_serial.print(") ");
    sim_serial.print("Emergency Feature activated\n\nPlease copy the coordinates & paste it on the Google Maps search bar to view current location of user");
    sim_serial.print(char(26));
    updateSerial();
    delay(100);
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastGPSTime >= gpsUpdateInterval) {
        lastGPSTime = currentTime;
        // Read GPS data
        getDateLoc(timeUTC, dateUTC, lat, lng);
    }

    // Update sensors at intervals
    if (currentTime - lastSensorTime >= sensorUpdateInterval) {
        lastSensorTime = currentTime;
        updateSensors(); // Function to handle distance and MPU6050 updates
    }
    while (Voice.isTalking()) {
        return; // Exit loop until Talkie finishes
    }

    // Check button press to trigger SMS
    checkButtonState();

    // Handle distance announcements
    int tCentimeter = getUSDistanceAsCentimeterWithCentimeterTimeout(300);
    handleDistanceMeasurement(tCentimeter);  // Trigger Talkie to announce
    // if (tCentimeter < 300 && tCentimeter != 0) {
    // } 
}

void updateSensors() {
    accelgyro.getAcceleration(&ax, &ay, &az);  // get acceleration data
    float angleX = atan2(ax, az) * 180 / PI;  // Convert X-axis and Z-axis values to angle

    // Determine detection range based on tilt
    detectionRange = (angleX >= tiltAngle) ? 1 : 1.8; //the angle for debugging
    Serial.print("X angle: ");
    Serial.print(angleX);
    Serial.println(" deg\t");
}

bool isVoiceSpeaking = false;

void handleDistanceMeasurement(int tCentimeter) {
    if (tCentimeter < 300 && tCentimeter != 0) {
        float tDistanceMeter = tCentimeter / 100.0;  // convert cm to m 

        if (tDistanceMeter <= detectionRange) {
            // If Talkie is already speaking, do not interrupt
            if (!isVoiceSpeaking) {
                isVoiceSpeaking = true;
                
                // Make Talkie speak the distance
                sayQFloat(&Voice, tDistanceMeter, 2, true, true);  // voice talk tDistanceMeter
                Voice.sayQ(sp2_METER);  // voice talk "meter"
                
                // Wait for Talkie to finish speaking
                while (Voice.isTalking()) {
                    // Do nothing while Talkie is speaking to prevent interruptions
                }
                
                // Pause for 3 seconds after Talkie finishes speaking "meter"
                delay(3000);
                isVoiceSpeaking = false; // Reset the speaking flag after the pause
            }
        }

        // Debug purposes
        Serial.print("Object ahead @ ");
        Serial.print(tDistanceMeter);
        Serial.print("m\t");
        Serial.print("detection range: ");
        Serial.print(detectionRange);
        Serial.println("m");
    } else {
        // Handle the case where no valid distance was detected
        Serial.println("No object detected or out of range.");
    }
}

void getDateLoc(String &timeUTC, String &dateUTC, float &lat, float &lng)
{
    if(gps_serial.available())
    {
        String nmeaSentence = gps_serial.readStringUntil('\n');

        if(nmeaSentence.startsWith("$GPRMC"))
        {
            if(parseNMEADateTime(nmeaSentence, timeUTC, dateUTC))
            {
                Serial.print(dateUTC);
                Serial.print(", ");
                Serial.print(timeUTC);
                Serial.print(" | ");
            }
            else
            {
                Serial.println("Invalid $GPRMC sentence or no date/time found.");
            }
        }

        if(nmeaSentence.startsWith("$GPGGA")) {
            if (parseNMEALatLon(nmeaSentence, lat, lng)) {
                Serial.print("Lat: ");
                Serial.print(lat, 6); // Print up to 6 decimal places
                Serial.print(", Lng: ");
                Serial.println(lng, 6);
            } else {
                Serial.println("Invalid NMEA sentence or no coordinates found.");
            }
        }
    }
}

bool parseNMEALatLon(String nmeaSentence, float &latitude, float &longitude) {
    int commaIndex = 0;

    // Parse Latitude
    for (int i = 0; i < 2; i++) commaIndex = nmeaSentence.indexOf(',', commaIndex + 1); // Find the latitude field
    String rawLat = nmeaSentence.substring(commaIndex + 1, nmeaSentence.indexOf(',', commaIndex + 1));
    commaIndex = nmeaSentence.indexOf(',', commaIndex + 1); // Latitude Direction
    String latDir = nmeaSentence.substring(commaIndex + 1, commaIndex + 2);

    // Parse Longitude
    commaIndex = nmeaSentence.indexOf(',', commaIndex + 1);
    String rawLon = nmeaSentence.substring(commaIndex + 1, nmeaSentence.indexOf(',', commaIndex + 1));
    commaIndex = nmeaSentence.indexOf(',', commaIndex + 1); // Longitude Direction
    String lonDir = nmeaSentence.substring(commaIndex + 1, commaIndex + 2);

    if (rawLat.length() > 0 && rawLon.length() > 0) {
        // Convert Latitude
        float latDegrees = rawLat.substring(0, 2).toFloat();
        float latMinutes = rawLat.substring(2).toFloat();
        latitude = latDegrees + (latMinutes / 60.0);
        if (latDir == "S") latitude = -latitude;

        // Convert Longitude
        float lonDegrees = rawLon.substring(0, 3).toFloat();
        float lonMinutes = rawLon.substring(3).toFloat();
        longitude = lonDegrees + (lonMinutes / 60.0);
        if (lonDir == "W") longitude = -longitude;

        return true; // Success
    }

    return false; // Invalid or incomplete data
}

bool parseNMEADateTime(String nmeaSentence, String &timeUTC, String &dateUTC) {
    int commaIndex = 0;

    // Extract Time
    commaIndex = nmeaSentence.indexOf(','); // First comma after "$GPRMC"
    String rawTime = nmeaSentence.substring(commaIndex + 1, nmeaSentence.indexOf(',', commaIndex + 1));
    
    // Extract Date
    for (int i = 0; i < 8; i++) commaIndex = nmeaSentence.indexOf(',', commaIndex + 1); // Find the date field
    String rawDate = nmeaSentence.substring(commaIndex + 1, nmeaSentence.indexOf(',', commaIndex + 1));

    if (rawTime.length() >= 6 && rawDate.length() == 6) {
        // Format Time: hhmmss.ss → hh:mm:ss
        String hours = rawTime.substring(0, 2);
        String minutes = rawTime.substring(2, 4);
        String seconds = rawTime.substring(4, 6);
        timeUTC = hours + ":" + minutes + ":" + seconds;

        // Format Date: ddmmyy → dd-mm-yy
        String day = rawDate.substring(0, 2);
        String month = rawDate.substring(2, 4);
        String year = rawDate.substring(4, 6);
        dateUTC = day + "-" + month + "-20" + year;

        return true; // Success
    }

    return false; // Invalid or incomplete data
}
