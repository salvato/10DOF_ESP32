#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <PID_v1.h>
#include <AsyncUDP.h>

#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"
#include "MotorController_BST7960.h"

// Pin used for I2C:
// D21 SDA
// D22 SCK

#define I2CFrequency     400000
#define ACC_ADDR         ADXL345_ADDR_ALT_LOW  // SDO connected to GND
#define ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW  // AD0 connected to GND
#define HMC5883L_ADDR    HMC5883L_Address      // FIXED

// Pin used by the Motor Controller
#define PWM1UP_PIN  12
#define PWM1LOW_PIN 13
#define PWM2UP_PIN  27
#define PWM2LOW_PIN 14

#define LED_BUILTIN 2 // ESP32 On Board Led Pin


uint16_t ahrsSamplingFrequency = 400; // Hz
uint16_t pidUpdateFrequency    = 100; // Hz
uint16_t remoteUpdateFrequency =  10; // Hz

volatile bool bUpdateRemote  = false;
volatile bool bPidUpdate     = false;
volatile bool bUpdateSensor  = false;

volatile bool bWiFiConnected = false;
volatile bool bOverrun       = false;


const char* ssid     = "Vodafone-A71002830";
const char* password = "..Lilly..";
char        sMessage[256];


// IP address to send UDP data to
const char*    udpAddress = "192.168.1.255";
const uint16_t udpPort    = 37755;
AsyncUDP       udp;


// Timers
hw_timer_t* ahrsSampler  = NULL;
hw_timer_t* pidTimer     = NULL;
hw_timer_t* sendingTimer = NULL;


// Sensors
ADXL345    Acc;
ITG3200    Gyro;
HMC5883L   Magn;
Madgwick   Madgwick;


// MotorController_BST7960
float motorSpeedFactorLeft  = 0.6;
float motorSpeedFactorRight = 0.6;

MotorController_BST7960 MotorController(PWM1UP_PIN, PWM1LOW_PIN,
                                        PWM2UP_PIN, PWM2LOW_PIN,
                                        motorSpeedFactorLeft, motorSpeedFactorRight);
                                              
// PID: Define Variables we'll be connecting to
double input, output;
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;
int controllerDirection = 0;
double setpoint = 0.0;
PID Pid(&input, &output, &setpoint, Kp, Ki, Kd, controllerDirection);
uint32_t t, t0;


float accValues[3];
float gyroValues[3];
float magValues[3];
float q0, q1, q2, q3;


void
ErrorHandler() {
    while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(30);
        digitalWrite(LED_BUILTIN, LOW);
        delay(30);
    }
}

 
void
IRAM_ATTR onAHRSTimer() {
    if(bUpdateSensor) {
        bOverrun = true;
    }
    bUpdateSensor = true;
}

 
void
IRAM_ATTR onSendingTimer() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    bUpdateRemote = true;
}


void
IRAM_ATTR onPidTimer() {
    bPidUpdate = true;
}

  
// Start the Sensor's Sampling Timer
void
startSamplingTimer(int usPeriod) {
    // Use 1st timer (out of 4 counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual).
    ahrsSampler = timerBegin(0, 80, true);
    
    // Attach onAHRSTimer() ISR function to our timer.
    timerAttachInterrupt(ahrsSampler, &onAHRSTimer, true);

    // Set alarm to call onAHRSTimer function every usPeriod (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(ahrsSampler, usPeriod, true);

    // Start the AHRS timer
    timerAlarmEnable(ahrsSampler);
}


// Start the Sensor's Sampling Timer
void
startSendingTimer(int usPeriod) {
    // Use 2st timer (out of 4 counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual).
    sendingTimer = timerBegin(1, 80, true);
    
    // Attach onSendingTimer() ISR function to our timer.
    timerAttachInterrupt(sendingTimer, &onSendingTimer, true);

    // Set alarm to call onSendingTimer function every usPeriod (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(sendingTimer, usPeriod, true);

    // Start the AHRS timer
    timerAlarmEnable(sendingTimer);
}


// Start the Sensor's Sampling Timer
void
startPidTimer(int usPeriod) {
    // Use 3rd timer (out of 4 counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual).
    pidTimer = timerBegin(2, 80, true);
    
    // Attach onSendingTimer() ISR function to our timer.
    timerAttachInterrupt(pidTimer, &onPidTimer, true);

    // Set alarm to call onSendingTimer function every usPeriod (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(pidTimer, usPeriod, true);

    timerAlarmEnable(pidTimer);
    t0 = millis();
}


// WiFi Event Handler
void
WiFiEvent(WiFiEvent_t event) {
    switch(event)
    {
        case SYSTEM_EVENT_STA_GOT_IP:
            // When connected set 
            Serial.print("WiFi connected! IP address: ");
            Serial.println(WiFi.localIP());
            Serial.println("Now Streaming data...");
            udp.listen(udpPort);
            sprintf(sMessage, "r#");
            udp.broadcast(sMessage);

            bWiFiConnected = true;
            break;
            
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("WiFi lost connection");
            bWiFiConnected = false;
            break;
            
        default:
            //Serial.print("WiFi Event N. ");
            //Serial.print(event);
            //Serial.println(" Received");
            break;
    }
}


bool
isStationary() {
    uint16_t nSamples = 60;
    double accVal[60];
    double avg[3] = {0};
    for(uint16_t i=0; i<nSamples; i++) {
        while(!Acc.getInterruptSource(7)) {} // Accelerator Data Ready
        Acc.get_Gxyz(accValues);

        avg[0] += accValues[0];
        avg[1] += accValues[1];
        avg[2] += accValues[2];
        accVal[i] = sqrt(accValues[0]*accValues[0] +
                         accValues[1]*accValues[1] +
                         accValues[2]*accValues[2]);
    }
    avg[0] /= double(nSamples);
    avg[1] /= double(nSamples);
    avg[2] /= double(nSamples);
    double average = sqrt((avg[0]*avg[0] + avg[1]*avg[1] + avg[2]*avg[2]));

    double sigma = 0;
    for(uint16_t i=0; i<nSamples; i++) {
        sigma += (accVal[i]-average)*(accVal[i]-average);
    }
    sigma = sqrt(sigma)/double(nSamples);
    Serial.print("Acc Average= ");
    Serial.println(average);
    Serial.print("Acc Sigma= ");
    Serial.println(sigma);
    // Noise Level (from datasheet = 1.1 LSB (3.9mg/LSB) )
    return sigma < 3.0*0.0039;
}


void
Sensors_Init() {
    bool bResult;

    // Accelerator Init
    bResult = Acc.init(ADXL345_ADDR_ALT_LOW);
    if(!bResult) {
        Serial.println("Error in Acc.init()");
        ErrorHandler();
    }
    Acc.setRangeSetting(2); // +/- 2g. Possible values are: 2g, 4g, 8g, 16g

    // Gyroscope Init
    bResult = Gyro.init(ITG3200_ADDR_AD0_LOW);
    if(!bResult) {
        Serial.println("Error in Gyro.init()");
        ErrorHandler();
    }
    delay(100);
    if(isStationary()) {
        Serial.println("Sensor is Stationary: Calibrating the Gyro");
        Gyro.zeroCalibrate(600); // calibrate the ITG3200
    }
    else { // Use Precalculated Values
        Gyro.setOffsets(-3.0, -11.0, -3.0);
    }
    Serial.print("Gyro offsets: Gx= ");
    Serial.print(Gyro.offsets[0]);
    Serial.print(" Gy= ");
    Serial.print(Gyro.offsets[1]);
    Serial.print(" Gz= ");
    Serial.println(Gyro.offsets[2]);
    
    // Magnetometer Init
    bResult = Magn.init(HMC5883L_Address);
    if(!bResult) {
        Serial.println("Error in Magn.init()");
        ErrorHandler();
    }
    delay(100);
    int16_t error = Magn.SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    if(error != 0) {
        Serial.println("Error in Magn.SetScale()");
        ErrorHandler();
    }
    delay(100);
    error = Magn.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    if(error != 0) {
        Serial.println("Error in Magn.SetMeasurementMode()");
        ErrorHandler();
    }
}



void
setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    
    Wire.begin();
    Wire.setClock(I2CFrequency);
    
    // Due to communication speed limitations, the maximum output
    // data rate of ADXL345, when using 400 kHz I2C, is 800 Hz.
    
    pinMode(LED_BUILTIN, OUTPUT);      // set the LED pin mode
    
    Madgwick.begin(ahrsSamplingFrequency);

    Pid.SetMode(AUTOMATIC);
    Pid.SetSampleTime(1000/pidUpdateFrequency);// in ms

    Serial.println("Initializing all Sensors...");
    Sensors_Init();
    Serial.println("ALl Sensors are Initialized");

    // Get the first Sensor's data
    while(!Acc.getInterruptSource(7)) {}
    Acc.get_Gxyz(accValues);
    while(!Gyro.isRawDataReadyOn()) {}
    Gyro.readGyro(gyroValues);
    while(!Magn.isDataReady()) {}
    Magn.ReadScaledAxis(magValues);

    Serial.print("Calculating Sensors initial attitude...");
    // Initial estimate of the attitude (presuming a static sensor !)
    for(uint32_t i=0; i<24*ahrsSamplingFrequency; i++) {
        Madgwick.update(gyroValues[0], gyroValues[1], gyroValues[2],
                        accValues[0],  accValues[1],  accValues[2],
                        magValues[0],  magValues[1],  magValues[2]);
    }
    Serial.println("Done");
    
    //input  = Madgwick.getPitch();
    //input  = Madgwick.getYaw();
    input  = Madgwick.getRoll();
    Pid.Compute();

    // Register WiFi Event Handler
    WiFi.onEvent(WiFiEvent);

    Serial.println();
    Serial.print("Waiting to be connected to: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE); // No Power Save on WiFi !!!
}


uint32_t d;

void
loop() {
    if(bWiFiConnected) {
        startSamplingTimer(1000000/ahrsSamplingFrequency);
        startPidTimer(1000000/pidUpdateFrequency);
        startSendingTimer(1000000/remoteUpdateFrequency);

        while(bWiFiConnected) {
          
            if(bOverrun) {
                Serial.println("Overrun");
                bOverrun = false;
            }
            
            if(bUpdateSensor) {
                Acc.get_Gxyz(accValues);
                Gyro.readGyro(gyroValues);
                Magn.ReadScaledAxis(magValues);
                Madgwick.update(gyroValues[0], gyroValues[1], gyroValues[2],
                                accValues[0],  accValues[1],  accValues[2],
                                magValues[0],  magValues[1],  magValues[2]);

                if(bPidUpdate) {
                    //input  = Madgwick.getPitch();
                    //input  = Madgwick.getYaw();
                    input  = Madgwick.getRoll();
                    Pid.Compute();
                    MotorController.move(output, 0);
                    bPidUpdate = false;
                }

                if(bUpdateRemote) {
                    Madgwick.getRotation(&q0, &q1, &q2, &q3);
                    t = millis()-t0;
                    sprintf(sMessage, "q %g %g %g %g#p %g %g %g#", q0, q1, q2, q3, t/1000.0, input, output);
                    udp.broadcast(sMessage);
                    bUpdateRemote= false;
                } // if(bUpdateRemote)
                
                bUpdateSensor = false;
                
            } // if(bUpdateSensor)
            
        } // while(bWiFiConnected)
        
        timerEnd(sendingTimer);
        timerEnd(pidTimer);
        timerEnd(ahrsSampler);
        sendingTimer = NULL;
        pidTimer     = NULL;
        ahrsSampler  = NULL;
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Disconnected.");
    }

}
