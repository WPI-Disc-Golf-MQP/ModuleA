#pragma once

#include <Arduino.h>

template <uint8_t encA, uint8_t encB> class Romi32U4Encoder
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

    void InitializeEncoder(void)
    {    
        Serial.println("InitializeEncoder()");

        // Set the pins as pulled-up inputs.
        pinMode(encA, INPUT_PULLUP);
        pinMode(encB, INPUT_PULLUP);
        
        // Attach the interrupt to the A pin
        if(digitalPinToInterrupt(encA) != NOT_AN_INTERRUPT) 
        {
            Serial.print("Attaching interrupt: ");
            Serial.print(encA);
            Serial.print('\n');

            attachInterrupt(digitalPinToInterrupt(encA), ProcessEncoderTickA, CHANGE); // interrupt
        }
        else {while(1) {Serial.println("Not an interrupt pin!");}}

        // Attach the interrupt to the B pin
        if(digitalPinToInterrupt(encB) != NOT_AN_INTERRUPT) 
        {
            Serial.print("Attaching interrupt: ");
            Serial.print(encB);
            Serial.print('\n');

            attachInterrupt(digitalPinToInterrupt(encB), ProcessEncoderTickB, CHANGE); // interrupt
        }
        else {while(1) {Serial.println("Not an interrupt pin!");}}

        Serial.println("/InitializeEncoder()");
    }
};
