#include <Arduino.h>

#define step_pin D12
#define dir_pin D11
#define sleep_pin D6

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(sleep_pin, OUTPUT);

  digitalWrite(dir_pin, LOW);
  digitalWrite(sleep_pin, HIGH);
}

void go_forward_x_rotation(double x, bool forward_back) {
  digitalWrite(sleep_pin, HIGH);
  digitalWrite(dir_pin, forward_back);
  // put your main code here, to run repeatedly:
  int i =0;

  while (true) {
    digitalWrite(step_pin, HIGH);
    delay(2);
    digitalWrite(step_pin, LOW);
    delay(2);  

    if(i++ > x*200)
    {
      // digitalWrite(dir_pin, HIGH);
      
      // digitalWrite(dir_pin, !digitalRead(dir_pin));
      // digitalWrite(sleep_pin, !digitalRead(sleep_pin));
      digitalWrite(sleep_pin, LOW);
      i = 0;
      break;
    }
  }
}


void loop() {
  go_forward_x_rotation(1, false);
  delay(5000);
}








// #define NODE_NAME String("module_a")
// #define STATUS_FREQ 1500 // ms

// #include <std_node.cpp>
// #include <std_msgs/Bool.h>
// #include <Arduino.h>


// // ----- INTAKE -----  

// int BEAM_BREAK_PIN = D2;
// int SPEED_PIN = A0;
// int INVERT_PIN = D13;

// // intake feedback
// std_msgs::Bool intake_state_msg;
// String _intake_state_topic(NODE_NAME + "_feedback__intake_state");
// ros::Publisher intake_state_pub(_intake_state_topic.c_str(), &intake_state_msg);

// enum INTAKE_STATE {
//   INTAKE_IDLE = 0,
//   INTAKE_SENDING = 1,
//   INTAKE_WAITING_FOR_DISC = 2,
//   INTAKE_RECIEVING = 3,

//   };

// INTAKE_STATE intake_state = INTAKE_STATE::INTAKE_IDLE;

// bool is_disc_present = false;
// long moved_to_INTAKE_SENDING_time = millis();

// void move_forward(int speed = 230) {
//   digitalWrite(INVERT_PIN, LOW);
//   analogWrite(SPEED_PIN, speed); // start
//   loginfo("move forward");
// }

// void stop() {
//   analogWrite(SPEED_PIN, 0); // stop
//   if (intake_state != INTAKE_STATE::INTAKE_IDLE) {
//     loginfo("stop");
//     intake_state = INTAKE_STATE::INTAKE_IDLE;
//   }
// }

// bool beam_broken() {
//   return (digitalRead(BEAM_BREAK_PIN) == 0);
// }

// bool verify_motion_complete() {
//   return intake_state == INTAKE_STATE::INTAKE_IDLE;
// }

// void start_intake() {
//   loginfo("start_intake");
//   if (is_disc_present) {
//     intake_state = INTAKE_STATE::INTAKE_SENDING;
//     moved_to_INTAKE_SENDING_time = millis();
//     move_forward();
//   } else {
//     intake_state = INTAKE_STATE::INTAKE_WAITING_FOR_DISC;
//   }
// }


// void check_intake() {
//   switch (intake_state) {
//     case INTAKE_STATE::INTAKE_IDLE:
//       stop();
//       break;
//     case INTAKE_STATE::INTAKE_SENDING:
//       if (moved_to_INTAKE_SENDING_time+1000 < millis()) {
//         is_disc_present = false;
//         stop();
//         intake_state = INTAKE_STATE::INTAKE_WAITING_FOR_DISC;
//       }
//       break;
//     case INTAKE_STATE::INTAKE_WAITING_FOR_DISC:
//       if (beam_broken()){
//         intake_state = INTAKE_STATE::INTAKE_RECIEVING;
//         move_forward();
//       }
//       break;
//     case INTAKE_STATE::INTAKE_RECIEVING:
//       if (!beam_broken()){ // therefore disc has come in, and gone past the first green wheel
//         intake_state = INTAKE_STATE::INTAKE_IDLE;
//         is_disc_present = true;
//       } 
//       break;
//   }


//   // publish state to pi 
//   if (intake_state_msg.data != (int) intake_state) {
//       intake_state_msg.data = intake_state;
//       loginfo("publishing conveyor state");
//       intake_state_pub.publish(&intake_state_msg);
//   }
// }

// // ----- CAMERA TURNTABLE ----- 



// // ----- SETUP LOOP -----

// void setup() {
//   init_std_node();
//   nh.advertise(intake_state_pub);
//   set_request_callbacks(
//     [] () {},
//     [] () {return true;},
//     start_intake,
//     verify_motion_complete,
//     stop);

//   // intake pins 
//   pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
//   pinMode(SPEED_PIN,OUTPUT) ;
//   pinMode(INVERT_PIN, OUTPUT) ;

//   loginfo("setup() Complete");
// }

// void loop() {
//   // periodic_status();
//   // nh.spinOnce();
//   // delay(10);
//   // check_intake();

//   // //----- testing ----- 
//   // check_intake();
//   // if (verify_motion_complete()) {
//   //   loginfo("debugging test reset");
//   //   delay(5000);
//   //   start_intake();
//   // }

//   move_forward();

// }
