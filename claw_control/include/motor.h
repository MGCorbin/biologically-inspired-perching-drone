#ifndef INCLUDE_MOTOR
#define INCLUDE_MOTOR

#include <Arduino.h>

enum class MotorDirection
{
    FORWARD, REVERSE, STOP
};

class Motor
{
public:
    explicit Motor(int pinA, int pinB, int adcPin);

    void setDirection(MotorDirection dir);
    MotorDirection getDirection() const { return m_MotorDirection; }
    void handle();

private:
    void monitorCurrent();
    void motorForward();
    void motorReverse();
    void motorStop();
    void detectOvercurrent();

private:
    MotorDirection m_MotorDirection;
    int m_MotorA, m_MotorB;
    int m_ADCpin;
    bool m_Overcurrent;
    uint16_t m_FilteredCurrent;
};

#endif // INCLUDE_MOTOR
