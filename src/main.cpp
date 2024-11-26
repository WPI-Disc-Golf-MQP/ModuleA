#define NODE_NAME String("module_a")
#define STATUS_FREQ 1500 // ms

#include <Arduino.h>

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

// ----- INTAKE -----
MODULE *intake_module;

int BEAM_BREAK_PIN = 2;
int INTAKE_SPEED_PIN = 12;
int INTAKE_INVERT_PIN = 6;
int UPPER_SPEED_PIN = 11;
int UPPER_INVERT_PIN = 4;

enum INTAKE_STATE
{
  INTAKE_IDLE = 0,
  INTAKE_SEND = 1,    // sending a disc out of the intake onto the conveyor
  INTAKE_RECIEVE = 2, // getting a disc from the top conveyor into the intake
};
INTAKE_STATE intake_state = INTAKE_STATE::INTAKE_IDLE;

bool is_disc_present = false;
long moved_to_INTAKE_RELEASE_time = millis();
bool deposited_disc = false; // flag if this instance of calling the start function has yet deposited a disc

void intake_motor_move_forward(int speed = 230)
{
  digitalWrite(INTAKE_INVERT_PIN, LOW);
  analogWrite(INTAKE_SPEED_PIN, speed); // start
}

void intake_motor_stop()
{
  analogWrite(INTAKE_SPEED_PIN, 0); // stop
}

void top_motor_move_forward(int speed = 230)
{
  digitalWrite(UPPER_INVERT_PIN, LOW);
  analogWrite(UPPER_SPEED_PIN, speed); // start
}

void top_motor_stop()
{
  analogWrite(UPPER_SPEED_PIN, 0); // stop
}

void stop_intake()
{
  top_motor_stop();
  intake_motor_stop();
}

bool val = 0;

bool beam_broken()
{

  // logging function
  if (digitalRead(BEAM_BREAK_PIN) != val)
  {
    loginfo("Intake beam break changed state to: " + String(digitalRead(BEAM_BREAK_PIN)));
    val = digitalRead(BEAM_BREAK_PIN);
  }
  // --

  return (digitalRead(BEAM_BREAK_PIN) == 0);
}

bool verify_intake_complete()
{
  return intake_state == INTAKE_STATE::INTAKE_IDLE;
}

void start_intake()
{
  loginfo("start_intake");
  // if (is_disc_present) {
  if (true)
  { // for now want to only deal with if the disc is present in the intake
    intake_state = INTAKE_STATE::INTAKE_SEND;
    moved_to_INTAKE_RELEASE_time = millis();
    intake_motor_move_forward();
  }
  else
  { // there is no disc, we need one
    intake_state = INTAKE_STATE::INTAKE_RECIEVE;
    top_motor_move_forward();
  }
}

void calibrate_intake()
{
  loginfo("calibrate_intake; not implemented"); // TODO: Implement calibration
}

void check_intake()
{
  switch (intake_state)
  {
  case INTAKE_STATE::INTAKE_IDLE:
    intake_motor_stop();
    top_motor_stop();
    break;
  case INTAKE_STATE::INTAKE_SEND:
    if (moved_to_INTAKE_RELEASE_time + 2000 < millis())
    {

      is_disc_present = false;
      // deposited_disc = true;
      intake_state = INTAKE_STATE::INTAKE_RECIEVE;
      // TODO: add a call to the pi to get how many discs are in the top conveyor. For now, it is just always assuming there is a disc in the top conveyor

      intake_motor_stop();
      top_motor_move_forward(); // should also make sure that it does not go under?
    }
    break;
  case INTAKE_STATE::INTAKE_RECIEVE:
    if (beam_broken())
    { // therefore disc has come in, and gone past the first green wheel
      is_disc_present = true;
      intake_state = INTAKE_STATE::INTAKE_IDLE;
      intake_module->publish_status(MODULE_STATUS::COMPLETE);

      // TODO: determine if it is ok for it to go from idle -> recieve -> idle. This code may not be necessary
      // this if statement ensures that a disc has been deposited before going back to the idle state. Basically ensuring that it doesn't go from idle -> recieve -> idle
      // if (deposited_disc) {
      //   intake_state = INTAKE_STATE::INTAKE_IDLE;
      // } else {
      //   intake_state = INTAKE_STATE::INTAKE_SEND;
      // };
    };
    break;
  default:
    logwarn("intake state invalid");
    break;
  }
  intake_module->publish_state((int)intake_state);
  beam_broken();
}

// // ----- SETUP LOOP -----

void setup()
{
  init_std_node();
  loginfo("setup() Start");
  intake_module = init_module("intake",
                              start_intake,
                              verify_intake_complete,
                              stop_intake,
                              calibrate_intake /* TODO: add calibration routine if needed */);

  pinMode(LED_BUILTIN, OUTPUT);

  // intake pins
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP);
  pinMode(INTAKE_SPEED_PIN, OUTPUT);
  pinMode(INTAKE_INVERT_PIN, OUTPUT);

  loginfo("setup() Complete");
}

// ---- testing ----
// bool doit = true;
// void loop() {
//   check_turntable();
//   if (doit) {
//     doit = false;
//     loginfo("debugging test reset");
//     delay(5000);
//     start_turntable();
//   }
// }

// void loop() {
//   // delay(30);
//   check_turntable();
//   run_yaxis_motor = true;
//   Serial.println(run_yaxis_motor);
// }

// ----- ros -----
void loop()
{
  periodic_status();
  nh.spinOnce();
  check_intake();
}
