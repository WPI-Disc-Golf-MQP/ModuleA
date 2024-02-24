#define NODE_NAME String("module_a")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <Arduino.h>


// ----- INTAKE -----  
MODULE* intake_module;

int BEAM_BREAK_PIN = D2;
int INTAKE_SPEED_PIN = A0;
int INTAKE_INVERT_PIN = D13;
int UPPER_SPEED_PIN = A1;
int UPPER_INVERT_PIN = D6;

enum INTAKE_STATE {
  INTAKE_IDLE = 0,
  INTAKE_SEND = 1, // sending a disc out of the intake onto the conveyor 
  INTAKE_RECIEVE = 2, // getting a disc from the top conveyor into the intake 
  };
INTAKE_STATE intake_state = INTAKE_STATE::INTAKE_IDLE;

bool is_disc_present = false;
long moved_to_INTAKE_RELEASE_time = millis();
bool deposited_disc = false; // flag if this instance of calling the start function has yet deposited a disc

void intake_motor_move_forward(int speed = 230) {
  digitalWrite(INTAKE_INVERT_PIN, LOW);
  analogWrite(INTAKE_SPEED_PIN, speed); // start
}

void intake_motor_stop() {
  analogWrite(INTAKE_SPEED_PIN, 0); // stop
}

void top_motor_move_forward(int speed = 230) { 
  digitalWrite(UPPER_INVERT_PIN, LOW);
  analogWrite(UPPER_SPEED_PIN, speed); // start
}

void top_motor_stop() { 
  analogWrite(UPPER_SPEED_PIN, 0); // stop
}

void stop_intake() {
  top_motor_stop();
  intake_motor_stop();
}

bool beam_broken() { 
  return (digitalRead(BEAM_BREAK_PIN) == 0);
}

bool verify_intake_complete() {
  return intake_state == INTAKE_STATE::INTAKE_IDLE;
}

void start_intake() {
  loginfo("start_intake");
  if (is_disc_present) {
    intake_state = INTAKE_STATE::INTAKE_SEND;
    moved_to_INTAKE_RELEASE_time = millis();
    intake_motor_move_forward();

  } else { // there is no disc, we need one
    intake_state = INTAKE_STATE::INTAKE_RECIEVE;
    top_motor_move_forward();
  }
}

void calibrate_intake() {
  loginfo("calibrate_intake; not implemented"); //TODO: Implement calibration
}

void check_intake() {
  switch (intake_state) {
    case INTAKE_STATE::INTAKE_IDLE: 
      intake_motor_stop();
      top_motor_stop();
      break;
    case INTAKE_STATE::INTAKE_SEND: 
      if (moved_to_INTAKE_RELEASE_time+1000 < millis()) {

        is_disc_present = false;
        // deposited_disc = true; 
        intake_state = INTAKE_STATE::INTAKE_RECIEVE;
        // TODO: add a call to the pi to get how many discs are in the top conveyor. For now, it is just always assuming there is a disc in the top conveyor 

        intake_motor_stop();
        top_motor_move_forward(); // should also make sure that it does not go under? 
      }
      break;
    case INTAKE_STATE::INTAKE_RECIEVE: 
      if (!beam_broken()){ // therefore disc has come in, and gone past the first green wheel
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
  intake_module->publish_state((int) intake_state);
}

// ----- CAMERA TURNTABLE ----- 

MODULE* turntable_module;
int dir_pin = D11;
int step_pin = D12;
int sleep_pin = D6; 
int UPPER_LIMIT_SWITCH_PIN = D3; // FINISH THIS FUNCTION 
int LOWER_LIMIT_SWITCH_PIN = D4; // FINISH THIS FUNCTION 

enum TURNTABLE_STATE {
  TURNTABLE_IDLE = 0,
  TURNTABLE_RAISING = 1,
  TURNTABLE_SPINNING = 2,
  TURNTABLE_LOWERING = 3
};
TURNTABLE_STATE turntable_state = TURNTABLE_STATE::TURNTABLE_IDLE;


bool upper_limit_switched() { // FINISH THIS FUNCTION after wiring
  return (digitalRead(UPPER_LIMIT_SWITCH_PIN) == 1);
}

bool lower_limit_switched() { // FINISH THIS FUNCTION after wiring
  return (digitalRead(LOWER_LIMIT_SWITCH_PIN) == 1);
}

bool verify_turntable_complete() {
  return turntable_state == TURNTABLE_STATE::TURNTABLE_IDLE;
}

bool run_yaxis_motor = false;
long yaxis_motor_last_step = millis();
bool yaxis_motor_last_digital_write = false;

bool run_spin_motor = false; 
long spin_motor_last_step = millis();
bool spin_motor_last_digital_write = false;


void start_turntable() {
  turntable_state = TURNTABLE_STATE::TURNTABLE_RAISING;
  run_yaxis_motor = true; 
  yaxis_motor_last_step = millis();
}

void stop_turntable() {
  run_yaxis_motor = false; 
  run_spin_motor = false; 
  turntable_state = TURNTABLE_STATE::TURNTABLE_IDLE;
}

void calibrate_turntable() {
  loginfo("calibrate_turntable; not implemented"); //TODO: Implement calibration
}

void check_turntable() {

  // drive the motor if the flag has been set to run it // TODO implimet the sleep pin as well 
  if ((yaxis_motor_last_step+2 < millis()) && run_yaxis_motor == true) {
    loginfo("triggered correctly");

    // digitalWrite(step_pin, !yaxis_motor_last_digital_write);
    digitalWrite(step_pin, HIGH);
    delay(2);
    digitalWrite(step_pin, LOW);
    delay(2); 

    yaxis_motor_last_digital_write = !yaxis_motor_last_digital_write; 
    yaxis_motor_last_step = millis(); 
  }

  if ((spin_motor_last_step+2 < millis()) && run_spin_motor == true) {
    digitalWrite(step_pin, !spin_motor_last_digital_write);
    spin_motor_last_digital_write = !spin_motor_last_digital_write; 
    spin_motor_last_step = millis();
  }


  switch (turntable_state) {
    case TURNTABLE_STATE::TURNTABLE_IDLE:
      
      break;
    case TURNTABLE_STATE::TURNTABLE_RAISING:

      if (upper_limit_switched() == true) {
        run_yaxis_motor = false; 
        turntable_state = TURNTABLE_STATE::TURNTABLE_SPINNING; 
        run_spin_motor = true; 
        spin_motor_last_step = millis();
      }
      
      break;
    case TURNTABLE_STATE::TURNTABLE_SPINNING:
    //TALK TO THE PI TO TAKE PICTURES
      break;
    case TURNTABLE_STATE::TURNTABLE_LOWERING:
      if (lower_limit_switched()) {
        run_yaxis_motor = false; 
        run_spin_motor = false; 
        turntable_state = TURNTABLE_STATE::TURNTABLE_IDLE; 
        turntable_module->publish_status(MODULE_STATUS::COMPLETE);
      }
      
      break;
  }
  turntable_module->publish_state((int) turntable_state);
};


// void foo() { // blink for testing
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(10);
//     digitalWrite(LED_BUILTIN, LOW);
// }

// // ----- SETUP LOOP -----

void setup() {
  init_std_node();
  loginfo("setup() Start");
  intake_module = init_module("intake",
    start_intake,
    verify_intake_complete,
    stop_intake, 
    calibrate_intake/* TODO: add calibration routine if needed */);
    
  turntable_module = init_module("turntable",
    start_turntable,
    verify_turntable_complete,
    stop_turntable, 
    calibrate_turntable/* TODO: add calibration routine if needed */);

  pinMode(LED_BUILTIN, OUTPUT);

  // intake pins 
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(INTAKE_SPEED_PIN,OUTPUT) ;
  pinMode(INTAKE_INVERT_PIN, OUTPUT) ;

  // camera pins 
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(sleep_pin, OUTPUT);

  digitalWrite(step_pin, LOW);
  digitalWrite(dir_pin, LOW);
  digitalWrite(sleep_pin, HIGH);


  loginfo("setup() Complete");
}

// ---- testing ---- 
// void loop() {
//   check_intake();
//   if (verify_motion_complete()) {
//     loginfo("debugging test reset");
//     delay(5000);
//     start_intake();
//   }
// }



// void loop() { 
//   Serial.println("Starting"); 
//   // delay(30); 
//   check_turntable(); 
//   run_yaxis_motor = true; 
//   Serial.println(run_yaxis_motor);
// } 

// ----- ros -----
void loop() {
  periodic_status();
  nh.spinOnce();
  check_intake();
  check_turntable();
}

