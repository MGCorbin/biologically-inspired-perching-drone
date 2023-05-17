#include <Arduino.h>
#include <Servo.h>
#include "Ultrasonic.h"
#include "motor.h"

// #define TEST_DC_MOTOR       0
#define DEBUG_PRINT

#ifdef ARDUINO_ARCH_AVR
    /* Arduino pin defs */
    #pragma message("Compiling for Arduino")
    #define FC_SWITCH           10
    #define ULTRASONIC_PIN      11
    #define MOTOR_A             2
    #define MOTOR_B             3
    #define MOTOR_ADC           A0
    #define MODE_SELECT         7
#elif ARDUINO_RASPBERRY_PI_PICO
    /* PICO pin defs */
    #pragma message("Compiling for Pico")
    #define FC_SWITCH           20
    #define ULTRASONIC_PIN      1
    #define MOTOR_A             27
    #define MOTOR_B             26
    #define MOTOR_ADC           A2      //28
    #define MODE_SELECT         21
#endif

Servo servo;
Ultrasonic ultrasonic(ULTRASONIC_PIN);
Motor motor(MOTOR_A, MOTOR_B, MOTOR_ADC);

typedef enum
{
    MANUAL,
    ULTRASONIC
} ctrl_mode_t;
ctrl_mode_t mode;

/* local functions */
void handle_autoclose(int dist, const int threshold);
uint16_t get_dist();
void debug_print();

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Startup...");
    servo.attach(9);
    pinMode(FC_SWITCH, INPUT_PULLUP);
    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(MODE_SELECT,INPUT_PULLUP);

    delay(100);
    if (digitalRead(MODE_SELECT))
    {
        mode = ULTRASONIC;
        Serial.println("ULTRASONIC MODE");
    }
    else
    {
        mode = MANUAL;
        Serial.println("MANUAL MODE");
    }
    delay(1000);
}

void loop()
{
    static uint16_t count = 0;
    static unsigned long oldmilis = 0;
    const uint16_t TIMEOUT = 300;   // number of ms PWM input from FC needs to be high before we say its inactive

    if (millis() != oldmilis)
    {
        /* run every ms */
        oldmilis = millis();
        motor.handle();
        //debug_print();

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
            Serial.print("Manual\n");
            break;

        case ULTRASONIC:    // ultrasonic mode: swich active = enable ultrasonic sensor, inactive = open
            if (!digitalRead(FC_SWITCH))
            {
                int distance = get_dist();
                /* ULTRASONIC ENABLED */
                Serial.println("Ultra enabled");
                count = 0;
                handle_autoclose(distance, 20);
            }
            else if (count < TIMEOUT && digitalRead(FC_SWITCH))
            {
                count++;
            }
            else
            {
                Serial.println("Opening - ULTRA");
                motor.setDirection(MotorDirection::REVERSE);
            }
            Serial.print("ULTRA!\n");
            break;
        }
    }
}

uint16_t get_dist() // call every ms
{
    static int distTimer;
    static uint16_t RangeInCenti;

    distTimer++;
    if (distTimer >= 100)
    {
        distTimer = 0;
        RangeInCenti = ultrasonic.MeasureInCentimeters();
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
            motor.setDirection(MotorDirection::FORWARD);
        }
    }
    else
    {
        autoCloseTimer = 0;
        motor.setDirection(MotorDirection::REVERSE);
    }
}

void debug_print()
{
#ifdef DEBUG_PRINT
    static uint16_t count = 0;
    const uint16_t PRINT_TIME = 500;
    count ++;
    if(count >= PRINT_TIME)
    {
        /* add whatever you want to debug print here*/
        count = 0;
        Serial.println(motor.getMotorCurrent());
    }
#endif
}


// // LED_BUILTIN in connected to pin 25 of the RP2040 chip.
// // It controls the on board LED, at the top-left corner.
// #include <Arduino.h>
// void setup() {
//   pinMode(LED_BUILTIN, OUTPUT);
// }

// void loop() {
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(500);
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(500);
// }