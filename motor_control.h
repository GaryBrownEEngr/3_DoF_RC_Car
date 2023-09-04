#pragma once

#include <Adafruit_PWMServoDriver.h>

// -800 to 800
typedef struct wheel_speeds_s
{
  int16_t left_front;
  int16_t left_rear;
  int16_t right_front;
  int16_t right_rear;
} wheel_speeds_t;





wheel_speeds_t compute_motor_speeds(int16_t pitch, int16_t roll, int16_t yaw) {
  int16_t command_sum = abs(pitch) + abs(roll) + abs(yaw);
  if(command_sum > 800){
    float factor = 800.0f / command_sum;
    pitch = (int16_t)(pitch * factor);
    roll = (int16_t)(roll * factor);
    yaw = (int16_t)(yaw * factor);
  }
  
  wheel_speeds_t ret;
  ret.left_front  = pitch + roll - yaw;
  ret.left_rear   = pitch - roll - yaw;
  ret.right_front = pitch - roll + yaw;
  ret.right_rear  = pitch + roll + yaw;
  
  return ret;
}


void set_motor_speeds(Adafruit_PWMServoDriver pwm, wheel_speeds_t in) {
  if(in.left_front >= 0){
    pwm.setPWM(13, 0, map(in.left_front, 0, 800, 0, 4095));
    pwm.setPWM(12, 0, 0);
  }else{
    pwm.setPWM(13, 0, 0);
    pwm.setPWM(12, 0, map(-in.left_front, 0, 800, 0, 4095));
  }

  if(in.left_rear >= 0){
    pwm.setPWM(15, 0, map(in.left_rear, 0, 800, 0, 4095));
    pwm.setPWM(14, 0, 0);
  }else{
    pwm.setPWM(15, 0, 0);
    pwm.setPWM(14, 0, map(-in.left_rear, 0, 800, 0, 4095));
  }

  if(in.right_front >= 0){
    pwm.setPWM(10, 0, map(in.right_front, 0, 800, 0, 4095));
    pwm.setPWM(11, 0, 0);
  }else{
    pwm.setPWM(10, 0, 0);
    pwm.setPWM(11, 0, map(-in.right_front, 0, 800, 0, 4095));
  }

  if(in.right_rear >= 0){
    pwm.setPWM(8, 0, map(in.right_rear, 0, 800, 0, 4095));
    pwm.setPWM(9, 0, 0);
  }else{
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, map(-in.right_rear, 0, 800, 0, 4095));
  }
}

