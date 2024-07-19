#pragma once

#include <Arduino.h>
#include <event_timer.h>

template <uint8_t encA, uint8_t encB> class Encoder
{
protected:
    // Keeps track of encoder changes
    volatile static int16_t prevCount;
    volatile static int16_t encCount;

    static void ProcessEncoderTickA(void) 
    {
        if(digitalRead(encA) == digitalRead(encB)) encCount++;
        else encCount--;
    }

    static void ProcessEncoderTickB(void) 
    {
        if(digitalRead(encA) == digitalRead(encB)) encCount--;
        else encCount++;
    }

public:
    /**
     * calcEncoderDelta() takes a 'snapshot of the encoders and 
     * stores the change since the last call in speed, which has units of "encoder ticks/motor interval" 
     * 
     * We also use the function for zeroing the delta (for example, when switching control modes),
     * so interrupts are cleared when accessing encCount.
     */
    int16_t CalcEncoderDelta(void)
    {
        noInterrupts();
        int16_t currCount = encCount;
        interrupts();

        int16_t speed = currCount - prevCount;
        prevCount = currCount;

        return speed;
    }

    /**
     * Returns the current encoder count.
     * */
    int16_t getCount(void)
    {
        cli();
        int16_t tempCount = count;
        sei();
        return tempCount;
    }


    void InitializeEncoder(void)
    {    
        // Set the pins as pulled-up inputs.
        pinMode(encA, INPUT_PULLUP);
        pinMode(encB, INPUT_PULLUP);
        
        // Attach the interrupt to the A pin
        if(digitalPinToInterrupt(encA) != NOT_AN_INTERRUPT) 
        {
            attachInterrupt(digitalPinToInterrupt(encA), ProcessEncoderTickA, CHANGE); // interrupt
        }

        // Attach the interrupt to the B pin
        if(digitalPinToInterrupt(encB) != NOT_AN_INTERRUPT) 
        {
            attachInterrupt(digitalPinToInterrupt(encB), ProcessEncoderTickB, CHANGE); // interrupt
        }
    }
};

class MotorBase
{
protected: 
    // Used to control the motors in different ways
    enum CTRL_MODE : uint8_t {CTRL_DIRECT, CTRL_SPEED};
    volatile CTRL_MODE ctrlMode = CTRL_DIRECT;

    // Used to manage PID coefficients;
    float Kp = 5;
    float Ki = 0.5;
    float Kd = 0;

    /**
     * Normally, we'd use a hardware timer, but the STM32 is new to me, 
     * so just using software timer for now.
     */
    EventTimer speedTimer = EventTimer(20);

    int16_t maxEffort = 255;

    // Used to keep track of the target speed, also in counts / interval.
    float targetSpeed = 0;

    /**
     * This is the speed of the motor, in "encoder counts / encoder interval".
     * The encoder interval is set by the motorTimer above.
     */ 
    volatile int16_t speed = 0;

    // For tracking the error integral
    int32_t sumError = 0;

    /**
     * 
     */
    virtual void SetEffort(int16_t effort) = 0;

    /** Used to set the motor effort directly. 
     * 
     * Calling this function will switch the control mode, if needed. 
     */
    void SetMotorEffortDirect(int16_t effort)
    {
        ctrlMode = CTRL_DIRECT;
        SetEffort(effort); // curious if this works...
    }

public:
    void SetPIDCoeffs(float p, float i, float d) {Kp = p; Ki = i; Kd = d; sumError = 0;}
};

template <uint8_t encA, uint8_t encB, uint8_t PWM, uint8_t DIR> class EncodedMotor
    : public MotorBase
{
public:
    Encoder<encA, encB> encoder;

    /**
     * Sets the target speed in "encoder ticks/16 ms interval"
     * */
    void SetTargetSpeed(float target)
    {
        targetSpeed = target;

        if(ctrlMode != CTRL_SPEED)
        {
            // Reset the error integral if we are switching from another mode
            // Otherwise, the robot may jump due to residual integral
            sumError = 0;

            // Also set prevCount to encCount so to avoid speed jumps when switching mode
            encoder.CalcEncoderDelta();
        }

        ctrlMode = CTRL_SPEED;
    }

public:
    void InitializeMotor(void)
    {
        digitalWrite(DIR, LOW);
        analogWrite(PWM, 0);

        encoder.InitializeEncoder();
    }

protected:
    void SetEffort(int16_t effort)
    {
        bool reverse = 0;

        if (effort < 0)
        {
            effort = -effort; // Make speed a positive quantity.
            reverse = 1;      // Reverse the direction.
        }
        if (effort > maxEffort)
        {
            effort = maxEffort;
        }

        digitalWrite(DIR, reverse);
        analogWrite(PWM, effort);
    }

public:
    /**
     * This is where you'll put the guts of your motor code
    */
    void ControlMotorSpeed(void)
    {
        if(speedTimer.checkExpired(true)) // true tells it to restart automatically
        {
            speed = encoder.CalcEncoderDelta();
            if(ctrlMode == CTRL_SPEED)
            {
                // Calculate the error in speed
                int16_t error = targetSpeed - speed;
                sumError += error;

                // Calculate the effort from the PID gains
                int16_t effort = Kp * error + Ki * sumError;

                // Set the effort for the motor
                SetEffort(effort);
            }
        }
    }
};

template <uint8_t encXOR, uint8_t encB> volatile int16_t Encoder<encXOR, encB>::prevCount = 0;
template <uint8_t encXOR, uint8_t encB> volatile int16_t Encoder<encXOR, encB>::encCount = 0;
