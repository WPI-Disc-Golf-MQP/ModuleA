#define NODE_NAME String("module_a")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <Arduino.h>

#include <button.h>

#include <event_timer.h>
EventTimer intakeTimer;

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
  INTAKE_RECEIVE = 2, // getting a disc from the top conveyor into the intake 
  };
INTAKE_STATE intake_state = INTAKE_STATE::INTAKE_IDLE;

bool is_disc_present = false;
long moved_to_INTAKE_RELEASE_time = millis();
bool deposited_disc = false; // flag if this instance of calling the start function has yet deposited a disc

void intake_motor_start(int speed = 230) 
{
  digitalWrite(INTAKE_INVERT_PIN, LOW);
  analogWrite(INTAKE_SPEED_PIN, speed); // start
}

void intake_motor_stop() {
  analogWrite(INTAKE_SPEED_PIN, 0); // stop
}

void top_motor_start(int speed = 230) 
{ 
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


bool val = 0;

/**
 * The beam break sensor has the same behaviour as a button.
 *    HIGH -> unbroken ("unpushed")
 *    LOW  -> broken   ("pushed")
 */
Button intakeBeamBreakSensor(BEAM_BREAK_PIN);

bool checkBeamBreak() 
{
  return intakeBeamBreakSensor.checkButtonPress();
  // logging function 
  // if (digitalRead(BEAM_BREAK_PIN) != val) {
  //   loginfo("Intake beam break changed state to: "+String(digitalRead(BEAM_BREAK_PIN)));
  //   val = digitalRead(BEAM_BREAK_PIN);
  // }
  // // -- 

  // return (digitalRead(BEAM_BREAK_PIN) == 0);
}

bool verify_intake_complete() {
  return intake_state == INTAKE_STATE::INTAKE_IDLE;
}

void start_intake() {
  loginfo("start_intake");
  // if (is_disc_present) {
  if (true) { // for now want to only deal with if the disc is present in the intake
    intakeTimer.start(2000);
    //moved_to_INTAKE_RELEASE_time = millis();
    intake_motor_start();
    intake_state = INTAKE_STATE::INTAKE_SEND;
  } 
  else // there is no disc, we need one
  { 
    intake_state = INTAKE_STATE::INTAKE_RECEIVE;
    top_motor_start();
  }
}

void calibrate_intake() {
  loginfo("calibrate_intake; not implemented"); //TODO: Implement calibration
}

/**
 * OBSOLETE. Replaced with checker/handler structure in loop()
 */
void check_intake() {
  switch (intake_state) {
    case INTAKE_STATE::INTAKE_IDLE: 
      intake_motor_stop();
      top_motor_stop();
      break;
    case INTAKE_STATE::INTAKE_SEND: 
      if (moved_to_INTAKE_RELEASE_time+2000 < millis()) {

        is_disc_present = false;
        // deposited_disc = true; 
        intake_state = INTAKE_STATE::INTAKE_RECEIVE;
        // TODO: add a call to the pi to get how many discs are in the top conveyor. For now, it is just always assuming there is a disc in the top conveyor 

        intake_motor_stop();
        top_motor_start(); // should also make sure that it does not go under? 
      }
      break;
    case INTAKE_STATE::INTAKE_RECEIVE: 
      if (checkBeamBreak()){ // therefore disc has come in, and gone past the first green wheel
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
  checkBeamBreak();
}

void handleBeamBreak(void)
{
  if(intake_state == INTAKE_STATE::INTAKE_RECEIVE)
  {
    is_disc_present = true; //why?
    top_motor_stop();
    //intake_motor_stop();  // not needed here
    
    intake_module->publish_status(MODULE_STATUS::COMPLETE);
    intake_state = INTAKE_STATE::INTAKE_IDLE;
  } 
}

void handleIntakeTimer(void)
{
  if(intake_state == INTAKE_STATE::INTAKE_SEND)
  {
    intake_motor_stop();

    is_disc_present = false;
    // TODO: add a call to the pi to get how many discs are in the top conveyor. For now, it is just always assuming there is a disc in the top conveyor 

    top_motor_start(); 
    intake_state = INTAKE_STATE::INTAKE_RECEIVE;
  }
}




// ----- CAMERA TURNTABLE ----- 

MODULE* turntable_module;

int yaxis_motor_dir_pin = D11;
int yaxis_motor_step_pin = D12;
int yaxis_motor_enable_pin = D10; 
int UPPER_LIMIT_SWITCH_PIN = D3; 
int LOWER_LIMIT_SWITCH_PIN = D4; 

enum TURNTABLE_STATE {
  TURNTABLE_IDLE = 0,
  TURNTABLE_RAISING = 1,
  TURNTABLE_SPINNING = 2,
  TURNTABLE_LOWERING = 3
};
TURNTABLE_STATE turntable_state = TURNTABLE_STATE::TURNTABLE_IDLE;


bool upper_limit_switched() {  
  return (digitalRead(UPPER_LIMIT_SWITCH_PIN) == 1);
}

bool lower_limit_switched() { 
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

long when_spinning_started = millis();

void start_turntable() {
  loginfo("Hi! Starting turntable!");
  turntable_state = TURNTABLE_STATE::TURNTABLE_RAISING;
  digitalWrite(yaxis_motor_dir_pin, LOW);
  run_yaxis_motor = true; 
  // yaxis_motor_last_step = millis(); // I don't think you need this 
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

  // turn on or off the enable pins
  if (run_yaxis_motor == true) {
    digitalWrite(yaxis_motor_enable_pin, HIGH);
  } else {
    digitalWrite(yaxis_motor_enable_pin, LOW);
  }

  if (run_spin_motor == true) {
    // TODO: // digitalWrite() // digital write the stepper motor enable high 
  } else {
    // TODO: // digial write stepper motor enable low 
  }

  // drive the motor if the flag has been set to run it // TODO implimet the sleep pin as well 
  if ((yaxis_motor_last_step+0.5 < millis()) && run_yaxis_motor == true) {
    // digitalWrite(step_pin, HIGH);
    // delay(2);
    // digitalWrite(step_pin, LOW);
    // delay(2); 

    loginfo("Spinning yaxis motor");

    digitalWrite(yaxis_motor_step_pin, !yaxis_motor_last_digital_write);
    yaxis_motor_last_digital_write = !yaxis_motor_last_digital_write; 
    yaxis_motor_last_step = millis(); 
  } 

  if ((spin_motor_last_step+2 < millis()) && run_spin_motor == true) {
    // TODO: add the spin stepper motor
  }


  switch (turntable_state) {
    case TURNTABLE_STATE::TURNTABLE_IDLE:
      
      break;
    case TURNTABLE_STATE::TURNTABLE_RAISING:

      if (upper_limit_switched() == true) {
        run_yaxis_motor = false; 
        turntable_state = TURNTABLE_STATE::TURNTABLE_SPINNING; 
        when_spinning_started = millis();
        run_spin_motor = true; 
        // spin_motor_last_step = millis(); // I dont think I need this line? 
      }
      
      break;
    case TURNTABLE_STATE::TURNTABLE_SPINNING:
    //TALK TO THE PI TO TAKE PICTURES
    // TODO: do not dead recon this section, actually get confirmation from the pi that picture taking is complete 
      
      if (when_spinning_started+2000 < millis()) {
        turntable_state = TURNTABLE_STATE::TURNTABLE_LOWERING; 
        digitalWrite(yaxis_motor_dir_pin, HIGH);
        run_yaxis_motor = true; 
      }

      break;
    case TURNTABLE_STATE::TURNTABLE_LOWERING:
      if (lower_limit_switched() == true) {
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
  intakeBeamBreakSensor.init();
  //pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(INTAKE_SPEED_PIN,OUTPUT) ;
  pinMode(INTAKE_INVERT_PIN, OUTPUT) ;

  // camera pins 
  pinMode(yaxis_motor_step_pin, OUTPUT);
  pinMode(yaxis_motor_dir_pin, OUTPUT);
  pinMode(yaxis_motor_enable_pin, OUTPUT);

  digitalWrite(yaxis_motor_step_pin, LOW);
  digitalWrite(yaxis_motor_dir_pin, LOW);
  digitalWrite(yaxis_motor_enable_pin, LOW);


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
void loop() {
  periodic_status();
  nh.spinOnce();

  if(checkBeamBreak()) handleBeamBreak();
  if(intakeTimer.checkExpired()) handleIntakeTimer();

  //check_intake();
  check_turntable();
}

