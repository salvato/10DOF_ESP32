#pragma once
#include <inttypes.h>


class MotorController_BST7960
{
public:
    MotorController_BST7960(uint32_t pwm1Up, uint32_t pwm1Low,
                            uint32_t pwm2Up, uint32_t pwm2Low,
                            double _motorAConst, double _motorBConst);
    ~MotorController_BST7960();
    void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255);
    void move(int leftSpeed, int rightSpeed, int minAbsSpeed);
    void move(int speed);
    void move(int speed, int minAbsSpeed);
    void turnLeft(int speed, bool kick);
    void turnRight(int speed, bool kick);
    void stopMoving();

protected:
    int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

protected:
    int32_t  gpioHostHandle;
    uint32_t pwm1UpChannel, pwm1LowChannel;
    uint32_t pwm2UpChannel, pwm2LowChannel;
    uint32_t PWMfrequency;
    uint32_t dutyCycle;
    int32_t  currentSpeed;
    double   motor1Const;
    double   motor2Const;
};
