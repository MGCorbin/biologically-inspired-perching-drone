#include <Arduino.h>
#include <Servo.h>
#include "Ultrasonic.h"
#include "motor.h"

// #define TEST_DC_MOTOR       0
#define FC_SWITCH           10
#define ULTRASONIC_PIN      11
#define MOTOR_A             2
#define MOTOR_B             3
#define MOTOR_ADC           A0

Servo servo;
Ultrasonic ultrasonic(ULTRASONIC_PIN);
Motor motor(MOTOR_A, MOTOR_B, MOTOR_ADC);

typedef enum
{
    MANUAL,
    ULTRASONIC
} mode_t;
mode_t mode;

void handle_autoclose(int dist, const int threshold);
int get_dist();

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Startup...");
    servo.attach(9);
    pinMode(FC_SWITCH, INPUT);
    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    pinMode(A0, INPUT);

    delay(100);
    if (digitalRead(7))
    {
        mode = ULTRASONIC;
        Serial.print("ULTRASONIC MODE");
    }
    else
    {
        mode = MANUAL;
        Serial.print("MANUAL MODE");
    }
    delay(1000);
}

void loop()
{
    // long pulsewidth = pulseIn(10, HIGH, 1000);

#ifdef TEST_DC_MOTOR
    static uint16_t hundredMsCount = 0, count = 0;
    static unsigned long oldMillis = 0;
    static uint8_t motor_state = 0;

    if (millis() != oldMillis)
    {
        oldMillis = millis();

        motor.handle();
        motor.setDirection(MotorDirection::FORWARD);
    }


#else

    static uint16_t count = 0;
    static unsigned long oldmilis = 0;
    const uint16_t TIMEOUT = 300;   // number of ms PWM input from FC needs to be high before we say its inactive
    if (millis() != oldmilis)
    {
        oldmilis = millis();
        motor.handle();

        switch (mode)
        {
        case MANUAL:        // manual mode: switch active = close, inactive = open
            if (!digitalRead(FC_SWITCH))
            {
                // Serial.println("CLOSING");
                // servo.write(180);
                motor.setDirection(MotorDirection::FORWARD);
                count = 0;
            }
            else if (count < TIMEOUT)
            {
                count++;
            }
            else
            {
                // Serial.println("Opening - MANUAL");
                // servo.write(90);
                motor.setDirection(MotorDirection::REVERSE);
            }
            break;

        case ULTRASONIC:    // ultrasonic mode: swich active = enable ultrasonic sensor, inactive = open
            if (!digitalRead(FC_SWITCH))
            {
                int distance = get_dist();
                /* ULTRASONIC ENABLED */
                Serial.println("Ultra enabled");
                count = 0;
                Serial.println(count);
                handle_autoclose(distance, 20);
            }
            else if (count < TIMEOUT && digitalRead(FC_SWITCH))
            {
                count++;
            }
            else
            {
                Serial.println("Opening - ULTRA");
                Serial.println(count);
                servo.write(0);
            }
            break;
        }
    }
#endif
}

int get_dist() // call every ms
{
    static int distTimer;
    int RangeInCenti;

    distTimer++;
    if (distTimer >= 100)
    {
        distTimer = 0;
        RangeInCenti = ultrasonic.MeasureInCentimeters();
        Serial.print("Ultrasonic Sensing: ");
        Serial.println(RangeInCenti);
    }

    return RangeInCenti;
}

void handle_autoclose(int dist, const int threshold)
{
    static int autoCloseTimer = 0;
    const int CLOSE_TIME = 100;

    if (dist < threshold)
    {
        autoCloseTimer++;
        if (autoCloseTimer >= CLOSE_TIME)
        {
            /* CLOSE */
            servo.write(180);
        }
    }
    else
    {
        autoCloseTimer = 0;
        servo.write(90); // open if we have closed too early
    }
}