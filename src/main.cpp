#define NODE_NAME String("module_a")
#define STATUS_FREQ 1500 // ms

#include <Arduino.h>
#undef min
#undef max
#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

// ---------- ---------- INTAKE MODULE ---------- ----------
MODULE *intake_module;

int BEAM_BREAK_PIN = 2;
int INTAKE_SPEED_PIN = 13;
int INTAKE_INVERT_PIN = 6;
int UPPER_SPEED_PIN = 11;
int UPPER_INVERT_PIN = 4;
int TEETH_SPEED_PIN = -1;
int TEETH_INVERT_PIN = -1;

enum INTAKE_STATE
{
    INTAKE_IDLE = 0,
    INTAKE_SEND = 1,    // sending a disc out of the intake onto the conveyor
    INTAKE_RECIEVE = 2, // getting a disc from the top conveyor into the intake
};
INTAKE_STATE intake_state = INTAKE_STATE::INTAKE_IDLE;

unsigned long moved_to_INTAKE_RELEASE_time = millis();

// ---------- ---------- START & STOP MOTOR FUNCTIONS ---------- ----------

void start_top_motor(int speed = 230)
{
    digitalWrite(UPPER_INVERT_PIN, LOW);
    analogWrite(UPPER_SPEED_PIN, speed); // start
}

void stop_top_motor()
{
    analogWrite(UPPER_SPEED_PIN, 0); // stop
}

void start_teeth_motor()
{
    // TODO: make teeth motor start
}

void stop_teeth_motor()
{
    // TODO: make teeth motor stop
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
    stop_top_motor();
    stop_teeth_motor();
    stop_intake_motor();
    intake_state = INTAKE_STATE::INTAKE_IDLE;
}

bool verify_intake_complete()
{
    return intake_state == INTAKE_STATE::INTAKE_IDLE;
}

void calibrate_intake()
{
    loginfo("calibrate_intake; not implemented"); // TODO: Implement calibration
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
        start_top_motor();
        start_teeth_motor();
        stop_intake_motor();
        intake_state = INTAKE_STATE::INTAKE_RECIEVE;
    }
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
    if (intake_state == INTAKE_STATE::INTAKE_RECIEVE)
    {
        stop_top_motor();
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
    intake_module = init_module("intake",
                                handle_intake_start,
                                verify_intake_complete,
                                handle_stop_intake,
                                calibrate_intake /* TODO: add calibration routine if needed */);

    pinMode(LED_BUILTIN, OUTPUT);

    // intake pins
    pinMode(BEAM_BREAK_PIN, INPUT_PULLUP);
    pinMode(INTAKE_SPEED_PIN, OUTPUT);
    pinMode(INTAKE_INVERT_PIN, OUTPUT);

    loginfo("setup() Complete");
}

// ---------- ---------- LOOP ---------- ----------

void loop()
{
    periodic_status();
    nh.spinOnce();
    if (check_intake_timer())
        handle_intake_timer();
    if (check_beam_break())
        handle_beam_break();
    intake_module->publish_state((int)intake_state);
}
