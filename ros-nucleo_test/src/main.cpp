#define NODE_NAME String("module_a")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <Arduino.h>


// ----- INTAKE -----  

int BEAM_BREAK_PIN = D2;
int SPEED_PIN = A0;
int INVERT_PIN = D13;

// intake feedback
std_msgs::Bool feedback_msg;
String _feedback_topic(NODE_NAME + "_feedback__intake");
ros::Publisher feedback_pub(_feedback_topic.c_str(), &feedback_msg);

enum INTAKE_STATE {
  INTAKE_IDLE = 0,
  INTAKE_SENDING = 1,
  INTAKE_WAITING_FOR_DISC = 2,
  INTAKE_RECIEVING = 3,

  };

INTAKE_STATE intake_state = INTAKE_STATE::INTAKE_IDLE;

bool is_disc_present = false;
long moved_to_INTAKE_SENDING_time = millis();

void move_forward(int speed = 230) {
  digitalWrite(INVERT_PIN, LOW);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("move forward");
}

void stop() {
  analogWrite(SPEED_PIN, 0); // stop
  if (intake_state != INTAKE_STATE::INTAKE_IDLE) {
    loginfo("stop");
    intake_state = INTAKE_STATE::INTAKE_IDLE;
  }
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
    intake_state = INTAKE_STATE::INTAKE_SENDING;
    moved_to_INTAKE_SENDING_time = millis();
    move_forward();
  } else {
    intake_state = INTAKE_STATE::INTAKE_WAITING_FOR_DISC;
  }
}


void check_intake() {
  switch (intake_state) {
    case INTAKE_STATE::INTAKE_IDLE:
      stop();
      break;
    case INTAKE_STATE::INTAKE_SENDING:
      if (moved_to_INTAKE_SENDING_time+1000 < millis()) {
        is_disc_present = false;
        stop();
        intake_state = INTAKE_STATE::INTAKE_WAITING_FOR_DISC;
      }
      break;
    case INTAKE_STATE::INTAKE_WAITING_FOR_DISC:
      if (beam_broken()){
        intake_state = INTAKE_STATE::INTAKE_RECIEVING;
        move_forward();
      }
      break;
    case INTAKE_STATE::INTAKE_RECIEVING:
      if (!beam_broken()){ // therefore disc has come in, and gone past the first green wheel
        intake_state = INTAKE_STATE::INTAKE_IDLE;
        is_disc_present = true;
      } 
      break;
  }
}


void setup() {
  init_std_node();
  nh.advertise(feedback_pub);
  set_request_callbacks(
    // std::bind(set_led, true), 
    // get_led, 
    // std::bind(set_led, true), 
    // get_led, 
    // std::bind(set_led, false)
    );

  // intake pins 
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(SPEED_PIN,OUTPUT) ;
  pinMode(INVERT_PIN, OUTPUT) ;

  loginfo("setup() Complete");
}

void loop() {
  periodic_status();
  nh.spinOnce();
  delay(10);
  check_intake();


    // ----- testing ----- 
  if (verify_intake_complete()) {
    Serial.println("farts");
    delay(5000);
    start_intake();
  }

}