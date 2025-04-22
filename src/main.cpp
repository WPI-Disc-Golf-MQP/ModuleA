#define NODE_NAME String("module_a")
#define STATUS_FREQ 1500 // ms

#include <Arduino.h>

#define Serial SerialUSB

#undef min
#undef max

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>


// ---------- ---------- INTAKE MODULE ---------- ----------
MODULE *intake_module;

int BEAM_BREAK_PIN = A3;

int CONVEYOR_SPEED_PIN = A0;
int CONVEYOR_INVERT_PIN = 5;
const byte CONVEYOR_ENCODER_A_PIN = 3;
const byte CONVEYOR_ENCODER_B_PIN = 2;
volatile long conveyorEncoderCount = 0;

void CONVEYOR_ENCODER_ISR_A()
{
    if (digitalRead(CONVEYOR_ENCODER_A_PIN) == digitalRead(CONVEYOR_ENCODER_B_PIN))
        conveyorEncoderCount++;
    else
        conveyorEncoderCount--;
}

void CONVEYOR_ENCODER_ISR_B()
{
    if (digitalRead(CONVEYOR_ENCODER_A_PIN) == digitalRead(CONVEYOR_ENCODER_B_PIN))
        conveyorEncoderCount--;
    else
        conveyorEncoderCount++;
}

void reset_conveyor_encoder_count()
{
    noInterrupts();
    conveyorEncoderCount = 0;
    interrupts();
}

long get_conveyor_encoder_count() 
{
    long tempCount = 0;
    noInterrupts();
    tempCount = conveyorEncoderCount;
    interrupts();
    return tempCount;
}

float get_conveyor_position()
{
    // 24 ticks per motor shaft rotation
    // and the motor gear ratio is 189:1
    // there is a 2 inch diameter on the motor shaft
    return 2 * 3.14159 * get_conveyor_encoder_count() / (24 * 189 * 1.0);
}

int TEETH_SPEED_PIN = 11;
int TEETH_INVERT_PIN = 4;
const byte TEETH_ENCODER_A_PIN = A2;
const byte TEETH_ENCODER_B_PIN = 7;
volatile long teethEncoderCount = 0;
long currentTeethPosition = 0;
long previousTeethPosition = 0;
byte TEETH_ENCODER_A_Last;

void TEETH_ENCODER_ISR_A()
{
    if (digitalRead(TEETH_ENCODER_A_PIN) == digitalRead(TEETH_ENCODER_B_PIN))
        teethEncoderCount++;
    else
        teethEncoderCount--;
}

void TEETH_ENCODER_ISR_B()
{
    if (digitalRead(TEETH_ENCODER_A_PIN) == digitalRead(TEETH_ENCODER_B_PIN))
        teethEncoderCount--;
    else
        teethEncoderCount++;
}

void reset_teeth_encoder_count()
{
    noInterrupts();
    teethEncoderCount = 0;
    interrupts();
}

long get_teeth_encoder_count()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = teethEncoderCount;
    interrupts();
    return tempCount;
}

float get_teeth_position()
{
    // 64 ticks per motor shaft rotation
    // and the motor gear ratio is 43.7:1
    // and the external gear ratio is 5:1
    // there is a 4 inch diameter on the motor shaft
    return 4 * 3.14159 * get_teeth_encoder_count() / (64 * 43.7 * 5);
}

int INTAKE_SPEED_PIN = 9;
int INTAKE_INVERT_PIN = 6;

enum INTAKE_STATE
{
    INTAKE_IDLE = 0,
    INTAKE_SEND = 1,    // sending a disc out of the intake onto the conveyor
    INTAKE_RECEIVE = 2, // getting a disc from the top conveyor into the intake
};
INTAKE_STATE intake_state = INTAKE_STATE::INTAKE_IDLE;

unsigned long moved_to_INTAKE_RELEASE_time = millis();

// ---------- ---------- START & STOP MOTOR FUNCTIONS ---------- ----------

void start_conveyor_motor(int speed = 230)
{
    digitalWrite(CONVEYOR_INVERT_PIN, HIGH);
    analogWrite(CONVEYOR_SPEED_PIN, speed); // start
    Serial.println("Conveyor motor started");

}

void stop_conveyor_motor()
{
    analogWrite(CONVEYOR_SPEED_PIN, 0); // stop
    Serial.println("Conveyor motor stopped");
    loginfo("Conveyor motor stopped");
}

void start_teeth_motor(int speed = 80) {
    loginfo("start_teeth_motor()");
    digitalWrite(TEETH_INVERT_PIN, HIGH);
    analogWrite(TEETH_SPEED_PIN, speed);
    Serial.println("Teeth motor started");
}

void stop_teeth_motor() {
    analogWrite(TEETH_SPEED_PIN, 0);
    Serial.println("Teeth motor stopped");
}

void handle_teeth_conveyor_coordination()
{
    if (intake_state == INTAKE_STATE::INTAKE_RECEIVE) 
    {
        float conveyorPosition = get_conveyor_position();
        float teethPosition = get_teeth_position();

        if (teethPosition > conveyorPosition) 
        {
            Serial.println("teeth motor > conveyor motor");
            start_conveyor_motor();
            stop_teeth_motor();
        }
        else 
        {
            Serial.println("teeth motor < conveyor motor");
            stop_conveyor_motor();
            start_teeth_motor();
        }
    }
}

void start_intake_motor(int speed = 230)
{
    digitalWrite(INTAKE_INVERT_PIN, LOW);
    analogWrite(INTAKE_SPEED_PIN, speed); // start
    Serial.println("intake motor started");
}

void stop_intake_motor()
{
    analogWrite(INTAKE_SPEED_PIN, 0); // stop
    Serial.println("intake motor stopped");
}


// ---------- ---------- ROS INTAKE FUNCTIONS ---------- ----------

void handle_intake_start()
{
    loginfo("start_intake");
    start_intake_motor();
    moved_to_INTAKE_RELEASE_time = millis();
    intake_state = INTAKE_STATE::INTAKE_SEND;
}

void handle_stop_intake()
{
    stop_conveyor_motor();
    stop_teeth_motor();
    stop_intake_motor();
    intake_state = INTAKE_STATE::INTAKE_IDLE;
}

bool verify_intake_complete()
{
    return intake_state == INTAKE_STATE::INTAKE_IDLE;
}



// ---------- ---------- INTAKE TIMER CHECK & HANDLE ---------- ----------

bool check_intake_timer()
{
    return moved_to_INTAKE_RELEASE_time + 2000 < millis();
}

void handle_intake_timer()
{
    if (intake_state == INTAKE_STATE::INTAKE_SEND)
    {
        stop_intake_motor();
        reset_conveyor_encoder_count();
        reset_teeth_encoder_count();
        start_conveyor_motor();
        start_teeth_motor();
        intake_state = INTAKE_STATE::INTAKE_RECEIVE;
    }
}

void calibrate_intake()
{
    // starts INTAKE_RECEIVE state so that one disc moves off the conveyor and is ready for intake
    // should be called before starting the intake
    loginfo("calibrating intake");
    intake_state = INTAKE_STATE::INTAKE_SEND;
    handle_intake_timer();
}

// ---------- ---------- BEAM BREAK CHECK & HANDLE ---------- ----------

bool beam_break_val_prev = 0;
bool check_beam_break()
{
    bool beam_break_val = digitalRead(BEAM_BREAK_PIN); // read beam break pin
    if (beam_break_val != beam_break_val_prev)
        loginfo("Intake beam break changed state to: " + String(beam_break_val)); // logging function
    bool beam_broken = beam_break_val == 0 && beam_break_val_prev == 1;
    beam_break_val_prev = beam_break_val; // set previous value to current value
    return beam_broken;
}

void handle_beam_break()
{
    if (intake_state == INTAKE_STATE::INTAKE_RECEIVE)
    {
        stop_conveyor_motor();
        stop_teeth_motor();
        intake_state = INTAKE_STATE::INTAKE_IDLE;
        intake_module->publish_status(MODULE_STATUS::COMPLETE);
    }
}



// ---------- ---------- SETUP ---------- ----------

void setup()
{
    init_std_node();
    loginfo("setup() Start");
    Serial.begin(57600);
    intake_module = init_module("intake",
                                handle_intake_start,
                                verify_intake_complete,
                                handle_stop_intake,
                                calibrate_intake 
                                /* TODO: add calibration routine if needed */);

    pinMode(LED_BUILTIN, OUTPUT);

    // intake pins
    pinMode(BEAM_BREAK_PIN, INPUT_PULLUP);

    pinMode(CONVEYOR_SPEED_PIN, OUTPUT);
    pinMode(CONVEYOR_INVERT_PIN, OUTPUT);
    pinMode(CONVEYOR_ENCODER_A_PIN, INPUT);
    pinMode(CONVEYOR_ENCODER_B_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(CONVEYOR_ENCODER_A_PIN), CONVEYOR_ENCODER_ISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CONVEYOR_ENCODER_B_PIN), CONVEYOR_ENCODER_ISR_B, CHANGE);

    pinMode(TEETH_SPEED_PIN, OUTPUT);
    pinMode(TEETH_INVERT_PIN, OUTPUT);
    pinMode(TEETH_ENCODER_A_PIN, INPUT);
    pinMode(TEETH_ENCODER_B_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TEETH_ENCODER_A_PIN), TEETH_ENCODER_ISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(TEETH_ENCODER_B_PIN), TEETH_ENCODER_ISR_B, CHANGE);

    pinMode(INTAKE_SPEED_PIN, OUTPUT);
    pinMode(INTAKE_INVERT_PIN, OUTPUT);

    loginfo("setup() Complete");
}

// ---------- ---------- LOOP ---------- ----------

void loop()
{
    periodic_status();
    nh.spinOnce();
    handle_teeth_conveyor_coordination();
    if (check_intake_timer())
        handle_intake_timer();
    if (check_beam_break())
        handle_beam_break();
    
    intake_module->publish_state((int)intake_state);
}

