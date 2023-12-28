#pragma once

#include <Adafruit_PWMServoDriver.h>

// -992 to 992
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
  

  ret.left_front = constrain(ret.left_front, -992, 992);
  ret.left_rear = constrain(ret.left_rear, -992, 992);
  ret.right_front = constrain(ret.right_front, -992, 992);
  ret.right_rear = constrain(ret.right_rear, -992, 992);
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




void front_left(int speed) {
  if(speed >= 0) {
    digitalWrite(LeftMotorDirPin1,0);
    digitalWrite(LeftMotorDirPin2,1); 
    analogWrite(speedPinL,speed);
  }else{
    digitalWrite(LeftMotorDirPin1,1);
    digitalWrite(LeftMotorDirPin2,0);
    analogWrite(speedPinL,-speed);
    
  }

}

void front_right(int speed){
  if(speed >= 0) {
    digitalWrite(RightMotorDirPin1,0);
    digitalWrite(RightMotorDirPin2,1);
    analogWrite(speedPinR,speed);
  }else{
    digitalWrite(RightMotorDirPin1,1);
    digitalWrite(RightMotorDirPin2,0);
    analogWrite(speedPinR,-speed);
    
  }

}

void back_left(int speed){
  if(speed >= 0) {
    digitalWrite(LeftMotorDirPin1B,0);
    digitalWrite(LeftMotorDirPin2B,1);
    analogWrite(speedPinLB,speed);
  }else{
    digitalWrite(LeftMotorDirPin1B,1);
    digitalWrite(LeftMotorDirPin2B,0);
    analogWrite(speedPinLB,-speed);
  }
}

void back_right(int speed){
  if(speed >= 0) {
    digitalWrite(RightMotorDirPin1B,0);
    digitalWrite(RightMotorDirPin2B,1);
    analogWrite(speedPinRB,speed);
  }else{
    digitalWrite(RightMotorDirPin1B,1);
    digitalWrite(RightMotorDirPin2B,0);
    analogWrite(speedPinRB,-speed);
  }
}


void set_motor_speeds(wheel_speeds_t in) {
  if(in.left_front >= 0){
    front_left(map(in.left_front, 0, 992, 0, 255));
  }else{
    front_left(map(in.left_front, 0, -992, 0, -255));
  }

  if(in.right_front >= 0){
    front_right(map(in.right_front, 0, 992, 0, 255));
  }else{
    front_right(map(in.right_front, 0, -992, 0, -255));
  }

  if(in.left_rear >= 0){
    back_left(map(in.left_rear, 0, 992, 0, 255));
  }else{
    back_left(map(in.left_rear, 0, -992, 0, -255));
  }

  if(in.right_rear >= 0){
    back_right(map(in.right_rear, 0, 992, 0, 255));
  }else{
    back_right(map(in.right_rear, 0, -992, 0, -255));
  }

}




