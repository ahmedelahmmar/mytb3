#ifndef __PRJ_H__
#define __PRJ_H__

#include <ros.h>
#include <cmath>
#include <cstdint>
#include <Arduino.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>


#define BUILTIN_LED 2

// Left Motor pins
#define LEFT_MOT_ENA 19  // PWM pin
#define LEFT_MOT_IN1 18
#define LEFT_MOT_IN2 5 // switched with in3
#define LEFT_MOT_ENCA 34  // Motor 1 Encoder A Yellow
#define LEFT_MOT_ENCB 35  // Motor 1 Encoder B Green

// Right Motor pins
#define RIGHT_MOT_ENA 4  // PWM pin
#define RIGHT_MOT_IN1 16
#define RIGHT_MOT_IN2 17
#define RIGHT_MOT_ENCA 32  // Motor 2 Encoder A Yellow
#define RIGHT_MOT_ENCB 33  // Motor 2 Encoder B Green

#define IMU_UPDATE_INTERVAL_MS                  20
#define MOTOR_UPDATE_INTERVAL_MS                IMU_UPDATE_INTERVAL_MS
#define MOTOR_MAX_RPM                           120
#define MOTOR_MIN_RPM                           30
#define ENCODER_PPR                             1419
#define WHEEL_RADIUS_METER                      0.04
#define WHEEL_BASE                              0.17
#define TB_MAX_LINEAR_VELOCITY                  0.5     // m/s @ max rpm = 120 rpm
#define TB_MAX_ANGULAR_VELOCITY                 5.9     // rad/s @ wheel base = 17 cm (2 vmax / wheel base)

#define FORWARD     0
#define REVERSE     1

#endif /* __PRJ_H__ */