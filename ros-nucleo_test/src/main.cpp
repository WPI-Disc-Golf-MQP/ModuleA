#define NODE_NAME String("module_a")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <Arduino.h>


// ----- INTAKE -----  

int BEAM_BREAK_PIN = D2;
int SPEED_PIN = A0;
int INVERT_PIN = D13;


int step_pin = D1;

// intake feedback
std_msgs::Bool intake_state_msg;
String _intake_state_topic(NODE_NAME + "_feedback__intake_state");
ros::Publisher intake_state_pub(_intake_state_topic.c_str(), &intake_state_msg);

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
  digitalWrite(INVERT_PIN, LOW);
  analogWrite(SPEED_PIN, speed); // start
}

void intake_motor_stop() {
  analogWrite(SPEED_PIN, 0); // stop
}

void top_motor_move_forward(int speed = 230) { // FINISH THIS FUNCTION after wiring
  // digitalWrite(INVERT_PIN, LOW);
  // analogWrite(SPEED_PIN, speed); // start
}

void top_motor_stop() { // FINISH THIS FUNCTION after wiring
  // analogWrite(SPEED_PIN, 0); // stop
}

void stop() {
  top_motor_stop();
  intake_motor_stop();
}

bool beam_broken() { 
  return (digitalRead(BEAM_BREAK_PIN) == 0);
}

bool verify_motion_complete() {
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


void check_intake() {
  switch (intake_state) {
    case INTAKE_STATE::INTAKE_IDLE: 
      intake_motor_stop();
      top_motor_stop();
      break;
    case INTAKE_STATE::INTAKE_SEND: 
      if (moved_to_INTAKE_RELEASE_time+1000 < millis()) {

        is_disc_present = false;
        deposited_disc = true; 
        intake_state = INTAKE_STATE::INTAKE_RECIEVE;
        intake_motor_stop();
        top_motor_move_forward(); // should also make sure that it does not go under? 
      }
      break;
    case INTAKE_STATE::INTAKE_RECIEVE: 
      if (!beam_broken()){ // therefore disc has come in, and gone past the first green wheel
        is_disc_present = true;
        
        // this if statement ensures that a disc has been deposited before going back to the idle state. Basically ensuring that it doesn't go from idle -> recieve -> idle 
        if (deposited_disc) {
          intake_state = INTAKE_STATE::INTAKE_IDLE;
        } else {
          intake_state = INTAKE_STATE::INTAKE_SEND;
        };
        
      };
      break;
  }


  // publish state to pi 
  if (intake_state_msg.data != (int) intake_state) {
      intake_state_msg.data = intake_state;
      loginfo("publishing intake state");
      intake_state_pub.publish(&intake_state_msg);
  }
}

// ----- CAMERA TURNTABLE ----- 
std_msgs::Bool turntable_state_msg;
String _turntable_state_topic(NODE_NAME + "_feedback__intake_state");
ros::Publisher turntable_state_pub(_turntable_state_topic.c_str(), &turntable_state_msg);

enum TURNTABLE_STATE {
  TURNTABLE_IDLE = 0,
  TURNTABLE_RAISING = 1,
  TURNTABLE_SPINNING = 2,
  TURNTABLE_LOWERING = 3
};
TURNTABLE_STATE turntable_state = TURNTABLE_STATE::TURNTABLE_IDLE;


bool upper_limit_switched() { // FINISH THIS FUNCTION after wiring
  return (digitalRead(BEAM_BREAK_PIN) == 0);
}

bool lower_limit_switched() { // FINISH THIS FUNCTION after wiring
  return (digitalRead(BEAM_BREAK_PIN) == 0);
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

bool beam_broken() { 
  return (digitalRead(BEAM_BREAK_PIN) == 0);
}


void check_turntable() {

  // drive the motor if the flag has been set to run it // TODO implimet the sleep pin as well 
  if (yaxis_motor_last_step+2 < millis()) {
    digitalWrite(step_pin, !yaxis_motor_last_digital_write);
    yaxis_motor_last_digital_write = !yaxis_motor_last_digital_write; 
    yaxis_motor_last_step = millis();
  }

  if (spin_motor_last_step+2 < millis()) {
    digitalWrite(step_pin, !spin_motor_last_digital_write);
    spin_motor_last_digital_write = !spin_motor_last_digital_write; 
    spin_motor_last_step = millis();
  }


  switch (turntable_state) {
    case TURNTABLE_STATE::TURNTABLE_IDLE:
      
      break;
    case TURNTABLE_STATE::TURNTABLE_RAISING:

      if (beam_broken()) {
        run_yaxis_motor = false; 
        turntable_state = TURNTABLE_STATE::TURNTABLE_SPINNING; 
        run_spin_motor = true; 
        spin_motor_last_step = millis();
      }
      
      break;
    case TURNTABLE_STATE::TURNTABLE_SPINNING:
       // not sure how to leave this state, when it goes a certain number of steps? or when pi says we got all the pictures... 
      break;
    case TURNTABLE_STATE::TURNTABLE_LOWERING:
      if (beam_broken()) {
        run_yaxis_motor = false; 
        run_spin_motor = false; 
        turntable_state = TURNTABLE_STATE::TURNTABLE_IDLE; 
      }
      
      break;
  }

  // publish state to pi 
  if (turntable_state_msg.data != (int) turntable_state) {
      turntable_state_msg.data = turntable_state;
      loginfo("publishing turntable state");
      turntable_state_pub.publish(&turntable_state_msg);
  }
};



// ----- SETUP LOOP -----

void setup() {
  init_std_node();
  nh.advertise(intake_state_pub);
  set_request_callbacks(
    [] () {},
    [] () {return true;},
    start_intake,
    verify_motion_complete,
    stop);

  // intake pins 
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(SPEED_PIN,OUTPUT) ;
  pinMode(INVERT_PIN, OUTPUT) ;

  // camera pins 
  pinMode(step_pin, OUTPUT);


  loginfo("setup() Complete");
}

// // ---- testing ---- 
// void loop() {
//   check_intake();
//   if (verify_motion_complete()) {
//     loginfo("debugging test reset");
//     delay(5000);
//     start_intake();
//   }

//   move_forward(128);
// }

// ---- ros -----
void loop() {
  periodic_status();
  nh.spinOnce();
  delay(10);
  check_intake();
  check_turntable();
}
