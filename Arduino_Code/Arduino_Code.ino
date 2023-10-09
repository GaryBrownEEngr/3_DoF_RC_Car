// Requires the adafruit PWM Servo Driver Library to be installed.
// https://www.arduino.cc/reference/en/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#include "crc.h"
#include "crsf.h"
#include "motor_control.h"

#define Battery_Sensor_Pin (A6)
#define Buzzer_Pin (10)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  gen_elrs_crc8();
  pinMode(Buzzer_Pin, OUTPUT);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(200);

  Serial.begin(115200);
  //Serial1.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }


  channel_commands.ch0 = 990;
  channel_commands.ch1 = 990;
  channel_commands.ch2 = 990;
  channel_commands.ch3 = 990;

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

bool is_battery_low() {
  uint16_t adc = analogRead(Battery_Sensor_Pin);
  float battery_voltage = adc * (5.0f / 1023.0f);

  bool is_low = battery_voltage < 3.3f;
  return is_low;
}


void loop() {
  // wheel_speeds_t motor_cmd;
  // motor_cmd.left_front=0;
  // motor_cmd.left_rear=0;
  // motor_cmd.right_front=0;
  // motor_cmd.right_rear=800;
  // set_motor_speeds(pwm, motor_cmd);


  wheel_speeds_t motor_cmd = compute_motor_speeds(0, 0, 0);
  set_motor_speeds(pwm, motor_cmd);

  crsf_packet_finder_t packet_finder;
  memset(&packet_finder, 0, sizeof packet_finder);


  //Serial.print("Start reading RX for packets:\n\n");


  uint32_t last_motor_update_time = millis();
  uint32_t last_buzzer_time = millis();
  uint8_t buzzer_state = 0;
  while(1) {
    // if we get a valid byte,
    while (Serial.available() > 0) {
      uint8_t in = Serial.read();
      //Serial.println(in, 16);
      search_for_packet(&packet_finder, in);
    }
    uint32_t new_time = millis();

    if(buzzer_state == 0 && abs(motor_cmd.left_front) <= 200 && abs(motor_cmd.left_rear) <= 200 && abs(motor_cmd.right_front <= 200) && abs(motor_cmd.right_rear <= 200)
    && is_battery_low()){
      buzzer_state = 1;
      last_buzzer_time = new_time;
      tone(Buzzer_Pin, 300);
    }
    else if( buzzer_state == 1 && new_time - last_buzzer_time >= 250){
      buzzer_state = 2;
      last_buzzer_time = new_time;
      tone(Buzzer_Pin, 200);
    }
    else if( buzzer_state == 2 && new_time - last_buzzer_time >= 250){
      buzzer_state = 3;
      last_buzzer_time = new_time;
      noTone(Buzzer_Pin);
    }
    else if( buzzer_state == 3 && new_time - last_buzzer_time >= 500){
      buzzer_state = 0;
    }

    if( new_time > last_motor_update_time + 10){
      last_motor_update_time = new_time;

      // Serial.print(channel_commands.ch0);
      // Serial.print(", ");
      // Serial.print(channel_commands.ch1);
      // Serial.print(", ");
      // Serial.print(channel_commands.ch2);
      // Serial.print(", ");
      // Serial.println(channel_commands.ch3);

      int16_t pitch = (int16_t)channel_commands.ch1 - 990;
      int16_t roll = (int16_t)channel_commands.ch0 - 990;
      int16_t yaw = -((int16_t)channel_commands.ch3 - 990);

      // Serial.print(pitch);
      // Serial.print(", ");
      // Serial.print(roll);
      // Serial.print(", ");
      // Serial.println(yaw);

      motor_cmd = compute_motor_speeds(pitch, roll, yaw);

      // Serial.print(motor_cmd.left_front);
      // Serial.print(", ");
      // Serial.print(motor_cmd.left_rear);
      // Serial.print(", ");
      // Serial.print(motor_cmd.right_front);
      // Serial.print(", ");
      // Serial.println(motor_cmd.right_rear);

      set_motor_speeds(pwm, motor_cmd);
    }
    
  }
}

