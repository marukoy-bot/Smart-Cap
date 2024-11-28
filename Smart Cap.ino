#include <Arduino.h>

// declare libraries
#include "TalkieUtils.h"
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include "HCSR04.hpp"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <TinyGPS++.h>

//UART library
#include <SoftwareSerial.h>

#define button_pin 6
#define ECHO_IN_PIN 4
#define TRIGGER_OUT_PIN 5
#define tiltAngle -35  // angle to switch detection range from 1m to 1.8m

#define gps_tx 7 //UART0
#define gps_rx 8

#define sim_tx 9 //UART1
#define sim_rx 10

#define baudrate 57600

// instantiate libraries/modules
Talkie Voice;
TinyGPSPlus gps;
MPU6050 accelgyro;
SoftwareSerial gps_serial(gps_tx, gps_rx);
SoftwareSerial sim_serial(sim_tx, sim_rx);

// gyro variables
int16_t ax, ay, az;  // acceleration axis x, y, and z
double detectionRange = 0;
float duration;

int press_counter;
bool press_enable;
long int time_interval;
double lat, lng;

void updateSerial() {
    delay(500);
    while (Serial.available()) {
        sim_serial.write(Serial.read());//Forward what Serial received to Software Serial Port
    }
    while(sim_serial.available()) {
        Serial.write(sim_serial.read());//Forward what Software Serial received to Serial Port
    }
}

void checkButtonState() {
    if(!digitalRead(button_pin) && !press_enable) {
        time_interval = millis();
        press_enable = 1;
        press_counter++;
        Serial.println(press_counter);
    }
    else if(digitalRead(button_pin)) {
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
    sim_serial.begin(baudrate); 
    gps_serial.begin(9600);
    
    // initialize components
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);  // set pins for HCSR04 ultrasonic sensor
    Wire.begin();
    accelgyro.initialize();  // initialize gyroscope
    
    pinMode(button_pin, INPUT);
    //pinMode(LED_BUILTIN, OUTPUT);  // set LED_BUILTIN mode as output (unnecessary, debug purposes only)

    press_counter = 0;
    press_enable = 0;
    time_interval = 0;
}

unsigned long lastGPSTime = 0;
unsigned long lastSensorTime = 0;
const unsigned long gpsUpdateInterval = 600; // Update GPS every 1 second
const unsigned long sensorUpdateInterval = 500; // Update sensors every 500 ms

void sendSMS() { //*    
    delay(100);
    // while (gps_serial.available()>0) 
    //     gps.encode(gps_serial.read());
    sim_serial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    sim_serial.println("AT+CMGS=\"+639664132090\"");
    updateSerial();
    sim_serial.print("(");
    sim_serial.print(getDate(gps.date.day(), gps.date.month(), gps.date.year()));
    sim_serial.print(", ");
    int currentHour = (int)gps.time.hour() + 8;
    sim_serial.print(currentHour);
    sim_serial.print(":");
    sim_serial.print(gps.time.minute());
    sim_serial.print(") ");
    //sim_serial.print("Emergency Feature activated at location: https://www.google.com/maps?q="); //can't send SMS with URL
    sim_serial.print("Emergency Feature activated\n\nPlease copy the coordinates below & paste it on the Google Maps search bar to view current location of user");
    sim_serial.print(char(26));
    updateSerial();

    delay(3800);
    Serial.println("sending coordinates...");

    sim_serial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    sim_serial.println("AT+CMGS=\"+639664132090\"");
    updateSerial();
    sim_serial.print(gps.location.lat(), 6);
    sim_serial.print(",");
    sim_serial.print(gps.location.lng(), 6);
    sim_serial.print(char(26));
    updateSerial();
    delay(100);
}

String getDate(int day, int month, int year)
{
    String date = (String)day + "-";
    switch(month)
    {
        case 1: date += "Jan-"; break;
        case 2: date += "Feb-"; break;
        case 3: date += "Mar-"; break;
        case 4: date += "Apr-"; break;
        case 5: date += "May-"; break;
        case 6: date += "Jun-"; break;
        case 7: date += "Jul-"; break;
        case 8: date += "Aug-"; break;
        case 9: date += "Sept-"; break;
        case 10: date += "Oct-"; break;
        case 11: date += "Nov-"; break;
        case 12: date += "Dec-"; break;
    }
    date += year;
    return date;
}



void loop() {
    unsigned long currentTime = millis();
    // Update GPS at intervals
    if (currentTime - lastGPSTime >= gpsUpdateInterval) {
        lastGPSTime = currentTime;
        // Read GPS data
        while (gps_serial.available() > 0) 
            gps.encode(gps_serial.read());

        Serial.print(gps.location.lng());
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
    detectionRange = (angleX >= tiltAngle) ? 1 : 1.8;

    // Print the angle for debugging
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
