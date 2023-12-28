// Requires the adafruit PWM Servo Driver Library to be installed.
// https://www.arduino.cc/reference/en/



#define SPEED 170    
#define TURN_SPEED 170 

#define speedPinR 9   //  Front Wheel PWM pin connect Model-Y M_B ENA 
#define RightMotorDirPin1  22    //Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1)
#define RightMotorDirPin2  24   //Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)                                 
#define LeftMotorDirPin1  26    //Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define LeftMotorDirPin2  28   //Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 10   //  Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 11   //  Rear Wheel PWM pin connect Left Model-Y M_A ENA 
#define RightMotorDirPin1B  5    //Rear Right Motor direction pin 1 to Model-Y M_A IN1 ( K1)
#define RightMotorDirPin2B 6    //Rear Right Motor direction pin 2 to Model-Y M_A IN2 ( K1) 
#define LeftMotorDirPin1B 7    //Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
#define LeftMotorDirPin2B 8  //Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 12    //  Rear Wheel PWM pin connect Model-Y M_A ENB





#include "crc.h"
#include "crsf.h"
#include "motor_control.h"





void setup() {
  gen_elrs_crc8();


  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }



  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);

  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);


  channel_commands.ch0 = 992;
  channel_commands.ch1 = 992;
  channel_commands.ch2 = 992;
  channel_commands.ch3 = 992;

}

/*
void test_packet_finder(){
  //uint8_t example_data[] = {0xEE, 0x04, 0x28, 0x00, 0xEA, 0x54};
  //uint8_t example_data[] = {0xEE, 0x06, 0x2D, 0xEE, 0xEF, 0x11, 0x01, 0xA5};
  //uint8_t example_data[] = {0xee, 0x18, 0x16, 0xe0, 0x3, 0x1f, 0xf8, 0xc0, 0x7, 0x3e, 0xf0, 0x81, 0xf, 0x7c, 0xe0, 0x3, 0x1f, 0xf8, 0xc0, 0x7, 0x3e, 0xf0, 0x81, 0xf, 0x7c, 0xad};
  uint8_t example_data[] = {0x00, 0x1, 0x2, 0x3, 0xEE, 0x06, 0x2D, 0xEE, 0xEF, 0x11, 0x01, 0xA5, 0xff, 0x00, 0xff, 0xee, 0x18, 0x16, 0xe0, 0x3, 0x1f, 0xf8, 0xc0, 0x7, 0x3e, 0xf0, 0x81, 0xf, 0x7c, 0xe0, 0x3, 0x1f, 0xf8, 0xc0, 0x7, 0x3e, 0xf0, 0x81, 0xf, 0x7c, 0xad};


  crsf_packet_finder_t packet_finder;
  memset(&packet_finder, 0, sizeof packet_finder);


  for(uint16_t i; i<sizeof example_data; i++) {
    search_for_packet(&packet_finder, example_data[i]);
  }
}
*/


void loop() {
  // wheel_speeds_t motor_cmd;
  // motor_cmd.left_front=0;
  // motor_cmd.left_rear=0;
  // motor_cmd.right_front=0;
  // motor_cmd.right_rear=800;
  // set_motor_speeds(pwm, motor_cmd);


  wheel_speeds_t motor_cmd = compute_motor_speeds(0, 0, 0);
  set_motor_speeds(motor_cmd);

  crsf_packet_finder_t packet_finder;
  memset(&packet_finder, 0, sizeof packet_finder);


  //Serial1.print("Start reading RX for packets:\n\n");


  uint32_t last_motor_update_time = millis();
  while(1) {
    // if we get a valid byte,
    while (Serial1.available() > 0) {
      uint8_t in = Serial1.read();
      //Serial1.println(in, 16);
      search_for_packet(&packet_finder, in);
    }
    uint32_t new_time = millis();

    if( new_time > last_motor_update_time + 10){
      last_motor_update_time = new_time;

      // Serial1.print(channel_commands.ch0);
      // Serial1.print(", ");
      // Serial1.print(channel_commands.ch1);
      // Serial1.print(", ");
      // Serial1.print(channel_commands.ch2);
      // Serial1.print(", ");
      // Serial1.println(channel_commands.ch3);

      int16_t pitch = (int16_t)channel_commands.ch1 - 992;
      int16_t roll = (int16_t)channel_commands.ch0 - 992;
      int16_t yaw = -((int16_t)channel_commands.ch3 - 992);

      // Serial1.print(pitch);
      // Serial1.print(", ");
      // Serial1.print(roll);
      // Serial1.print(", ");
      // Serial1.println(yaw);

      motor_cmd = compute_motor_speeds(pitch, roll, yaw);

      // Serial1.print(motor_cmd.left_front);
      // Serial1.print(", ");
      // Serial1.print(motor_cmd.left_rear);
      // Serial1.print(", ");
      // Serial1.print(motor_cmd.right_front);
      // Serial1.print(", ");
      // Serial1.println(motor_cmd.right_rear);

      set_motor_speeds(motor_cmd);
    }
    
  }
}

