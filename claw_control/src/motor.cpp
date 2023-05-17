
#include "motor.h"

/* helper function - doesn't belong to class */
uint16_t adc_to_current(uint16_t adc_count);

Motor::Motor(int pinA, int pinB, int adcPin)
    : m_MotorDirection(MotorDirection::STOP), m_MotorA(pinA),
    m_MotorB(pinB), m_ADCpin(adcPin), m_Overcurrent(false), m_FilteredCurrent(0)
{
    /* empty */
}

void Motor::setDirection(MotorDirection dir)
{
    static MotorDirection oldDirection;

    if(!m_Overcurrent)
    {
        m_MotorDirection = dir;
        return;
    }

    /*
    We have an overcurrent
    Only allow the direction to be set if its opposite to the previous direction
    This stops continuous overccurrents in the same direction
    */

    if(dir == MotorDirection::FORWARD && oldDirection == MotorDirection::REVERSE)
    {
        /* if we WERE reversing, only allow forward */
        m_MotorDirection = dir;
        m_Overcurrent = false;  // clear the overcurrent flag
        Serial.println("Overcurrent cleared. New direction: FORWARD");
    }
    else if(dir == MotorDirection::REVERSE && oldDirection == MotorDirection::FORWARD)
    {
        /* if we WERE going forward, only allow reverse */
        m_MotorDirection = dir;
        m_Overcurrent = false;  // clear the overcurrent flag
        Serial.println("Overcurrent cleared. New direction: REVERSE");
    }

    if(dir == MotorDirection::FORWARD || dir == MotorDirection::REVERSE)
    {
        /* old direction should only be forward or reverse not STOP */
        oldDirection = dir;
    }

}

void Motor::handle()
{
    monitorCurrent();
    switch(m_MotorDirection)
    {
        case MotorDirection::FORWARD:   motorForward(); break;
        case MotorDirection::REVERSE:   motorReverse(); break;
        case MotorDirection::STOP:      motorStop();    break;
    }
}

/* private functions */

void Motor::motorForward()
{
    digitalWrite(m_MotorA, LOW);
    digitalWrite(m_MotorB, LOW);
    delayMicroseconds(10);      // delay to avoid fet fight
    digitalWrite(m_MotorA, HIGH);
}

void Motor::motorReverse()
{
    digitalWrite(m_MotorA, LOW);
    digitalWrite(m_MotorB, LOW);
    delayMicroseconds(10);      // delay to avoid fet fight
    digitalWrite(m_MotorB, HIGH);
}

void Motor::motorStop()
{
    digitalWrite(m_MotorA, LOW);
    digitalWrite(m_MotorB, LOW);
}

void Motor::monitorCurrent()
{
    const uint16_t SAMPLE_SIZE = 10;
    static uint32_t I_buffer = 0;
    static uint16_t sample_count = 0;


    I_buffer += adc_to_current(analogRead(m_ADCpin));
    sample_count ++;

    if(sample_count == SAMPLE_SIZE)
    {
        sample_count = 0;
        m_FilteredCurrent = I_buffer / SAMPLE_SIZE + 1;
        // Serial.println(m_FilteredCurrent);   // debug only
        I_buffer = 0;
    }
    detectOvercurrent();

}

void Motor::detectOvercurrent()
{
    static uint16_t overcurrent_count = 0;
    const uint16_t OVERCURRENT_TIME = 500;   // number of ms we need overcurrent for before setting flag
    const uint16_t MOTOR_OVERCURRENT = 250; // value in mA

    if(m_FilteredCurrent > MOTOR_OVERCURRENT)
    {
        overcurrent_count ++;

        if(overcurrent_count > OVERCURRENT_TIME)
        {
            setDirection(MotorDirection::STOP);
            m_Overcurrent = true;
            Serial.println("Motor overcurrent detected");
        }
    }
    else
    {
        overcurrent_count = 0;
    }
}

uint16_t adc_to_current(uint16_t adc_count)
{
    /* returns motor current in mA */
#ifdef ARDUINO_ARCH_AVR
    #pragma message("Scaling ADC for Arduino")
    int temp_v =  (adc_count / 1023.0f) * 5000; // calculate the voltage in mV (5V, 10bit system)
#elif ARDUINO_RASPBERRY_PI_PICO
    #pragma message("Scaling ADC for Pico")
    int temp_v =  (adc_count / 1023.0f) * 3300; // calculate the voltage in mV (3.3V, 12bit system)
#else
    #error "Board architecture not supported"
#endif
    /* calculate the current
    * divide by 5 as using X5.6 diff amp,
    * I=V/R - I Sense resistor is 0.5 ohm */
    uint16_t temp_i = (temp_v / 5.6f) / 0.5f;
    return temp_i;
}