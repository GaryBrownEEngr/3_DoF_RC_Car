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
  if(abs(pitch) < 15) {
    pitch = 0;
  }
  if(abs(roll) < 15) {
    roll = 0;
  }
  if(abs(yaw) < 15) {
    yaw = 0;
  }


  // int16_t command_sum = abs(pitch) + abs(roll) + abs(yaw);
  // if(command_sum > 800){
  //   float factor = 800.0f / command_sum;
  //   pitch = (int16_t)(pitch * factor);
  //   roll = (int16_t)(roll * factor);
  //   yaw = (int16_t)(yaw * factor);
  // }
  
  wheel_speeds_t ret;
  ret.left_front  = pitch + roll - yaw;
  ret.left_rear   = pitch - roll - yaw;
  ret.right_front = pitch - roll + yaw;
  ret.right_rear  = pitch + roll + yaw;
  

  ret.left_front = constrain(ret.left_front, -800, 800);
  ret.left_rear = constrain(ret.left_rear, -800, 800);
  ret.right_front = constrain(ret.right_front, -800, 800);
  ret.right_rear = constrain(ret.right_rear, -800, 800);
  return ret;
}


int16_t compute_motor_voltage(int16_t speed_cmd) {
  float x = speed_cmd;
  float x_2 = x * x;
  float x_3 = x_2 * x;

  #define A (234.654246254354)
  #define B (1.05560076208825)
  #define C (-0.00204336398965036)
  #define D (8.44467792778767E-06)

  float y = A + B*x + C*x_2 + D*x_3;
  int16_t ret = (int16_t)y;
  return ret;
}


void set_motor_speeds(Adafruit_PWMServoDriver pwm, wheel_speeds_t in) {
  if(abs(in.left_front) < 15){
    pwm.setPWM(13, 0, 0);
    pwm.setPWM(12, 0, 0);
  }else if(in.left_front >= 0){
    pwm.setPWM(13, 0, compute_motor_voltage(in.left_front));
    pwm.setPWM(12, 0, 0);
  }else{
    pwm.setPWM(13, 0, 0);
    pwm.setPWM(12, 0, compute_motor_voltage(-in.left_front));
  }

  if(abs(in.left_rear) < 15){
    pwm.setPWM(15, 0, 0);
    pwm.setPWM(14, 0, 0);
  }else if(in.left_rear >= 0){
    pwm.setPWM(15, 0, compute_motor_voltage(in.left_rear));
    pwm.setPWM(14, 0, 0);
  }else{
    pwm.setPWM(15, 0, 0);
    pwm.setPWM(14, 0, compute_motor_voltage(-in.left_rear));
  }

  if(abs(in.right_front) < 15){
    pwm.setPWM(10, 0, 0);
    pwm.setPWM(11, 0, 0);
  }else if(in.right_front >= 0){
    pwm.setPWM(10, 0, compute_motor_voltage(in.right_front));
    pwm.setPWM(11, 0, 0);
  }else{
    pwm.setPWM(10, 0, 0);
    pwm.setPWM(11, 0, compute_motor_voltage(-in.right_front));
  }

  if(abs(in.right_rear) < 15){
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, 0);
  }else if(in.right_rear >= 0){
    pwm.setPWM(8, 0, compute_motor_voltage(in.right_rear));
    pwm.setPWM(9, 0, 0);
  }else{
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, compute_motor_voltage(-in.right_rear));
  }
}

