#include "MotorController_BST7960.h"
#include "Arduino.h"

#include <algorithm> // min() & max()
#include "string.h"  // for memset()

using namespace std; // min() & max()


#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13


template
<typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


//=================================================================================
//    43A High Power BTS7960 DC Motor Driver Module. http://handsontec.com
//=================================================================================
//     Pin No    Function                      Description
//---------------------------------------------------------------------------------
//        1         RPWM        Forward Level or PWM signal, Active High
//        2         LPWM        Reverse Level or PWM signal, Active High
//        3         R_EN        Forward Drive Enable Input, Active High
//        4         L_EN        Reverse Drive Enable Input, Active High
//        5         R_IS        Forward Drive, Side current alarm output (Not used)
//        6         L_IS        Reverse Drive, Side current alarm output (Not used)
//        7         Vcc         +5V Power Supply Input to the Logic
//        8         Gnd         Ground: Connect to microcontroller GND
//=================================================================================


MotorController_BST7960::MotorController_BST7960(uint32_t pwm1Up, uint32_t pwm1Low,
                                                 uint32_t pwm2Up, uint32_t pwm2Low,
                                                 double _motor1Const, double _motor2Const)
{
    pwm1UpChannel  = 0;
    pwm1LowChannel = 1;

    pwm2UpChannel  = 2;
    pwm2LowChannel = 3;

    motor1Const = _motor1Const;
    motor2Const = _motor2Const;

    // WARNING !!! a too low frequency will interfere with the IMU readings !!!
    PWMfrequency = 8000; // in Hz

// Motor 1
    ledcSetup(pwm1UpChannel, PWMfrequency, LEDC_TIMER_13_BIT);
    ledcAttachPin(pwm1Up, pwm1UpChannel);

    ledcSetup(pwm1LowChannel, PWMfrequency, LEDC_TIMER_13_BIT);
    ledcAttachPin(pwm1Low, pwm1LowChannel);
    

    currentSpeed = 0;

    //Start (non-zero dutycycle) or stop (0) PWM pulses on the GPIO.
    ledcAnalogWrite(pwm1UpChannel,  currentSpeed);
    ledcAnalogWrite(pwm1LowChannel, currentSpeed);

// Motor 2
    ledcSetup(pwm2UpChannel, PWMfrequency, LEDC_TIMER_13_BIT);
    ledcAttachPin(pwm2Up, pwm2UpChannel);

    ledcSetup(pwm2LowChannel, PWMfrequency, LEDC_TIMER_13_BIT);
    ledcAttachPin(pwm2Low, pwm2LowChannel);


    //Start (non-zero dutycycle) or stop (0) PWM pulses on the GPIO.
    ledcAnalogWrite(pwm2UpChannel,  currentSpeed);
    ledcAnalogWrite(pwm2LowChannel, currentSpeed);
}


MotorController_BST7960::~MotorController_BST7960() {
}


// Arduino like analogWrite: value has to be between 0 and valueMax
void
MotorController_BST7960::ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax) {
    // calculate duty, 8191 from 2 ^ 13 - 1
    uint32_t duty = (8191 / valueMax) * min(value, valueMax);
    // write duty to LEDC
    ledcWrite(channel, duty);
}

int32_t
MotorController_BST7960::map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void
MotorController_BST7960::move(int leftSpeed, int rightSpeed, int minAbsSpeed) {
    int absSpeed = min(max(abs(rightSpeed), minAbsSpeed), 255);
    int realRightSpeed = sgn(rightSpeed)*map(absSpeed, 0, 255, minAbsSpeed, 255);

    absSpeed = min(max(abs(leftSpeed), minAbsSpeed), 255);
    int realLeftSpeed = sgn(leftSpeed)*map(abs(absSpeed), 0, 255, minAbsSpeed, 255);

// Motor 1
    if(realRightSpeed > 0) {
        ledcAnalogWrite(pwm1UpChannel,  abs(realRightSpeed) * motor1Const);
        ledcAnalogWrite(pwm1LowChannel, 0);
    }
    else {
        ledcAnalogWrite(pwm1UpChannel, 0);
        ledcAnalogWrite(pwm1LowChannel,  abs(realRightSpeed) * motor1Const);
    }

// Motor 2
    if(realLeftSpeed > 0) {
        ledcAnalogWrite(pwm2UpChannel,  abs(realLeftSpeed) * motor1Const);
        ledcAnalogWrite(pwm2LowChannel, 0);
    }
    else {
        ledcAnalogWrite(pwm2UpChannel, 0);
        ledcAnalogWrite(pwm2LowChannel,  abs(realLeftSpeed) * motor1Const);
    }

}


void
MotorController_BST7960::move(int speed, int minAbsSpeed) {
    int direction = sgn(speed);
    speed = min(max(abs(speed), minAbsSpeed), 255);

    if(direction*speed == currentSpeed) return;
    
    int realSpeed = max(minAbsSpeed, speed);

    if(direction > 0) {
        ledcAnalogWrite(pwm1UpChannel, abs(realSpeed) * motor1Const);
        ledcAnalogWrite(pwm1LowChannel, 0);

        ledcAnalogWrite(pwm2UpChannel, abs(realSpeed) * motor2Const);
        ledcAnalogWrite(pwm2LowChannel, 0);
    }
    else {
        ledcAnalogWrite(pwm1LowChannel, abs(realSpeed) * motor1Const);
        ledcAnalogWrite(pwm1UpChannel, 0);

        ledcAnalogWrite(pwm2LowChannel, abs(realSpeed) * motor2Const);
        ledcAnalogWrite(pwm2UpChannel, 0);
    }

    currentSpeed = direction * realSpeed;
}


void
MotorController_BST7960::move(int speed) {
    if (speed == currentSpeed) return;
    speed = min(max(abs(speed), 0), 255);

    ledcAnalogWrite(pwm1UpChannel, speed * motor1Const);
    ledcAnalogWrite(pwm1LowChannel, speed * motor1Const);

    ledcAnalogWrite(pwm2UpChannel, speed * motor2Const);
    ledcAnalogWrite(pwm2LowChannel, speed * motor2Const);

    currentSpeed = speed;
}


void
MotorController_BST7960::turnLeft(int speed, bool kick) {
    (void)speed;
// ToDo:
    if (kick) {
        sleep(100);
    }
}


void
MotorController_BST7960::turnRight(int speed, bool kick) {
    (void)speed;
// ToDo:
    if (kick) {
        sleep(100);
    }
}


void
MotorController_BST7960::stopMoving() {
    currentSpeed = 0;

    ledcAnalogWrite(pwm1UpChannel, currentSpeed);
    ledcAnalogWrite(pwm1LowChannel, currentSpeed);

    ledcAnalogWrite(pwm2UpChannel, currentSpeed);
    ledcAnalogWrite(pwm2LowChannel, currentSpeed);
}
