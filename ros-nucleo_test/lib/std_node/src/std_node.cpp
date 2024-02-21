/*
 * rosserial Standard Node Interface
 */

#include <Arduino.h>
#include <ros.h>
#include "HardwareSerial.h"
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <functional>

#ifndef NODE_NAME
    #define NODE_NAME String("std_node")
#endif
#ifndef STATUS_FREQ
    #define STATUS_FREQ 1500 // ms
#endif

enum MODULE_STATUS {
    INITIALIZING_MODULE = 0,
    IDLE = 1,
    IN_PROGRESS = 2,
    COMPLETE = 3,
    INVALID_REPEAT = 4,
    ERROR_REBOOT = 5};

enum REQUEST {
    INITIALIZING = 0,
    STOP = 1,
    START = 2,
    VERIFY_COMPLETE = 3,
    CALIBRATE = 4,
    REBOOT = 5};



HardwareSerial hserial(PA_15, PA_2); // NUCLEO-F303K8 RX, TX
#define Serial1 hserial // This will overwrite the current Serial1 serial port and will use hserial port.
#define USE_STM32_HW_SERIAL
#define __STM32F3xxxx__

ros::NodeHandle nh;
unsigned long last_status = 0;

String get_node_tag() { return String("[")+NODE_NAME+String("] "); };
void loginfo(String msg) { nh.loginfo((get_node_tag()+msg).c_str()); };
void logwarn(String msg) { nh.logwarn((get_node_tag()+msg).c_str()); };
void logerr(String msg) { nh.logerror((get_node_tag()+msg).c_str()); };

struct MODULE {
    String module_name;
    std_msgs::Int8 request;
    std_msgs::Int8 status;

    std::function<bool(void)> verify_complete_callback;
    std::function<void(void)> start_callback, idle_callback, calibrate_callback;

    ros::Publisher status_pub;
    ros::Subscriber<std_msgs::Int8, MODULE> request_sub;

    MODULE(String _module_name, 
        std::function<void(void)> _start_callback, 
        std::function<bool(void)> _verify_complete_callback, 
        std::function<void(void)> _idle_callback, 
        std::function<void(void)> _calibrate_callback) : 
        status_pub((_module_name + "_status").c_str(), &status), 
        request_sub((_module_name + "_request").c_str(), &MODULE::process_request_callback, this) {
        module_name = _module_name;
        verify_complete_callback = _verify_complete_callback;
        start_callback = _start_callback;
        idle_callback = _idle_callback;
        calibrate_callback = _calibrate_callback;
    }

    void init() {
        nh.advertise(status_pub);
        nh.subscribe(request_sub);
    }

    void process_request_callback(const std_msgs::Int8& msg) {
        bool verify_result;
        std_msgs::Int8 onetime_status;
        switch (msg.data) {
            case REQUEST::START:
                status.data = MODULE_STATUS::IN_PROGRESS; 
                status_pub.publish(&status);
                start_callback(); 
                break;
            case REQUEST::VERIFY_COMPLETE:
                verify_result = verify_complete_callback();
                onetime_status.data = verify_result ? MODULE_STATUS::COMPLETE : MODULE_STATUS::IN_PROGRESS;
                status_pub.publish(&onetime_status);
                break;
            case REQUEST::STOP:
                status.data = MODULE_STATUS::IDLE; 
                status_pub.publish(&status);
                idle_callback();
                break;
            case REQUEST::CALIBRATE:
                status.data = MODULE_STATUS::IN_PROGRESS;
                status_pub.publish(&status);
                calibrate_callback();
                break;
            default: 
                return;
        }
        status_pub.publish(&status);
        loginfo("Request Received, "+String(msg.data));
    }
};

std::vector<MODULE> modules;

void init_std_node() {
    nh.initNode();
    nh.setSpinTimeout(100);
    last_status = millis();

    // std::for_each(modules.begin(), modules.end(), [](MODULE &module) {
    //     module.init();
    // });
}

void init_module(String _module_name, 
        std::function<void(void)> _start_callback, 
        std::function<bool(void)> _verify_complete_callback, 
        std::function<void(void)> _idle_callback, 
        std::function<void(void)> _calibrate_callback) { 
    MODULE module = MODULE(_module_name, _start_callback, _verify_complete_callback, _idle_callback, _calibrate_callback);
    modules.emplace_back(module);
    module.init();
};
            
void publish_status() {
    std::for_each(modules.begin(), modules.end(), [](MODULE &module) {
        module.status_pub.publish(&module.status);
    });
    last_status = millis();
}

void periodic_status() {
    if (millis() - last_status > STATUS_FREQ) {
        publish_status();
    }
}

