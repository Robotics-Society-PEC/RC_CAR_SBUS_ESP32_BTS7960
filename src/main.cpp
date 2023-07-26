/*
  Robotics Pec
  robotics@pec.edu.in

                     GNU GENERAL PUBLIC LICENSE
                        Version 3, 29 June 2007

  Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
  Everyone is permitted to copy and distribute verbatim copies
  of this license document, but changing it is not allowed.

  The GNU General Public License is a free, copyleft license for
  software and other kinds of works.

  THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
  APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT
  HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY
  OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
  IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF
  ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
*/

#include <Arduino.h>
#include <sbus.h>

#define AILERON 0
#define ELEVATOR 1
#define THROTTLE 2
#define RUDDER 3
#define ARM 4

#define RPWM_1 22 // pin for RPWM
#define R_EN_1 4  // pin for R_EN

#define LPWM_1 5  // pin for LPWM
#define L_EN_1 13 // pin for L_EN

// pins for motor 2
#define RPWM_2 14 // pin for RPWM
#define R_EN_2 15 // pin for R_EN

#define LPWM_2 21

// pin for LPWM
#define L_EN_2 19 // pin for L_EN

#define CW 1  // don't change
#define CCW 0 // don't change

#define STOP_HARD_MODE 1
#define STOP_SOFT_MODE 0

/////// //////////////////////////////PWM Config
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 8
#define PWM_MIN 0   // corresponds to 0 degress
#define PWM_MAX 255 // corresponds to 180 degrees

#define motor1Channel 0
#define motor2Channel 1
#define motor3Channel 2
#define motor4Channel 3

///////////////////////////////////LOW pass filter config / integrator

// Sbus config

#define SBUS_RX 16
#define SBUS_TX 17

bool stop_mode = STOP_SOFT_MODE;
unsigned long time_since_last_command = 0;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SBUS_RX, SBUS_TX, true);
/* SBUS data */
bfs::SbusData data;

unsigned last_packet_recieved = 0;
#define failsafe_time_in_millis 2000

/////////////////////////////////////////////////User Specified Data /////////////////////////////////////////////
int max_ch[16] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
int mid_ch[16] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
int min_ch[16] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

#define deadzone 20

int motor_left_rpm_1 = 0;
int motor_left_rpm_2 = 0;
int motor_right_rpm_3 = 0;
int motor_right_rpm_4 = 0;
int16_t y_axis_movement = 0;
int16_t x_axis_movement = 0;
int16_t z_axis_rotation = 0;

void calibrate();
void calibrate_alternate();

void setup()
{

  pinMode(R_EN_1, OUTPUT);

  pinMode(L_EN_1, OUTPUT);

  pinMode(R_EN_2, OUTPUT);

  pinMode(L_EN_2, OUTPUT);

  ledcSetup(motor1Channel, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(motor2Channel, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(motor3Channel, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(motor4Channel, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LPWM_1, motor1Channel);
  ledcAttachPin(RPWM_1, motor2Channel);
  ledcAttachPin(LPWM_2, motor3Channel);
  ledcAttachPin(RPWM_2, motor4Channel);

  // Debugging
  Serial.begin(115200);
  while (!Serial)
  {
  }

  // Begin Sbus communication
  sbus_rx.Begin();

  // calibrate();
}

// add failsafe time of 2 sec

void loop()
{
  if (sbus_rx.Read())
  {
    last_packet_recieved = millis();
    data = sbus_rx.data();

    // ARM on channel 5
    if ((data.ch[ARM] < (max_ch[ARM] + min_ch[ARM]) / 2) || data.failsafe)
    {
      motor_left_rpm_1 = 0;
      motor_left_rpm_2 = 0;
      motor_right_rpm_3 = 0;
      motor_right_rpm_4 = 0; // stop all motors if not armed or failsafe
      digitalWrite(R_EN_1, LOW);
      digitalWrite(R_EN_2, LOW);
      digitalWrite(L_EN_1, LOW);
      digitalWrite(L_EN_2, LOW);
      ledcWrite(motor1Channel, motor_left_rpm_1);
      ledcWrite(motor2Channel, motor_left_rpm_2);
      ledcWrite(motor3Channel, motor_right_rpm_3);
      ledcWrite(motor4Channel, motor_right_rpm_4);
    }
    else
    {
      // Serial.println("armed");
      if (!data.lost_frame)
      {

        if (data.ch[ELEVATOR] >= mid_ch[ELEVATOR] + deadzone)
        {
          Serial.println("go forward");
          y_axis_movement = map(data.ch[ELEVATOR], mid_ch[ELEVATOR], max_ch[ELEVATOR], PWM_MIN, PWM_MAX);
          y_axis_movement = constrain(y_axis_movement, PWM_MIN, PWM_MAX);
          // go forward
        }
        else if (data.ch[ELEVATOR] < mid_ch[ELEVATOR] - deadzone)
        {

          y_axis_movement = map(data.ch[ELEVATOR], mid_ch[ELEVATOR], min_ch[ELEVATOR], PWM_MIN, PWM_MAX);
          y_axis_movement = -constrain(y_axis_movement, PWM_MIN, PWM_MAX);
          // go backward
        }
        else
        {
          y_axis_movement = 0;
        }

        if (data.ch[AILERON] >= mid_ch[AILERON] + deadzone)
        {
          Serial.println("go right");
          x_axis_movement = map(data.ch[AILERON], mid_ch[AILERON], max_ch[AILERON], PWM_MIN, PWM_MAX);
          x_axis_movement = constrain(x_axis_movement, PWM_MIN, PWM_MAX);
          // go right
        }
        else if (data.ch[AILERON] < mid_ch[AILERON] - deadzone)
        {

          x_axis_movement = map(data.ch[AILERON], mid_ch[AILERON], min_ch[AILERON], PWM_MIN, PWM_MAX);
          x_axis_movement = -constrain(x_axis_movement, PWM_MIN, PWM_MAX);
          // go left
        }
        else
        {
          x_axis_movement = 0;
        }

        // handel rotation
        if (data.ch[RUDDER] >= mid_ch[RUDDER] + deadzone)
        {
          Serial.println("rotate right");
          z_axis_rotation = map(data.ch[RUDDER], mid_ch[RUDDER], max_ch[RUDDER], PWM_MIN, PWM_MAX);
          z_axis_rotation = constrain(z_axis_rotation, PWM_MIN, PWM_MAX);
          // rotate right
        }
        else if (data.ch[RUDDER] < mid_ch[RUDDER] - deadzone)
        {

          z_axis_rotation = map(data.ch[RUDDER], mid_ch[RUDDER], min_ch[RUDDER], PWM_MIN, PWM_MAX);
          z_axis_rotation = -constrain(z_axis_rotation, PWM_MIN, PWM_MAX);
          // rotate left
        }
        else
        {
          z_axis_rotation = 0;
        }

        if (z_axis_rotation > 0)
        {
          // rotate right
          motor_left_rpm_1 = 0;
          motor_left_rpm_2 = z_axis_rotation;
          motor_right_rpm_3 = z_axis_rotation;
          motor_right_rpm_4 = 0;
        }
        else if (z_axis_rotation < 0)
        {
          // rotate left
          motor_left_rpm_1 = -z_axis_rotation;
          motor_left_rpm_2 = 0;
          motor_right_rpm_3 = 0;
          motor_right_rpm_4 = -z_axis_rotation;
        }
        else if (y_axis_movement == 0 && x_axis_movement == 0)
        {
          // go nowhere
          motor_left_rpm_1 = 0;
          motor_left_rpm_2 = 0;
          motor_right_rpm_3 = 0;
          motor_right_rpm_4 = 0;
        }
        else if (y_axis_movement > 0 && x_axis_movement > 0)
        {
          // go forward right
          motor_left_rpm_1 = 0;
          motor_left_rpm_2 = y_axis_movement;
          motor_right_rpm_3 = 0;
          motor_right_rpm_4 = y_axis_movement - x_axis_movement;
        }
        else if (y_axis_movement > 0 && x_axis_movement == 0)
        {
          // go forward
          motor_left_rpm_1 = 0;
          motor_left_rpm_2 = y_axis_movement;
          motor_right_rpm_3 = 0;
          motor_right_rpm_4 = y_axis_movement;
        }
        else if (y_axis_movement > 0 && x_axis_movement < 0)
        {
          // go forward left
          motor_left_rpm_1 = 0;
          motor_left_rpm_2 = y_axis_movement + x_axis_movement; // right is negative so to decrease you have to add
          motor_right_rpm_3 = 0;
          motor_right_rpm_4 = y_axis_movement;
        }
        else if (y_axis_movement < 0 && x_axis_movement == 0)
        {
          // go backward
          motor_left_rpm_1 = -y_axis_movement;
          motor_left_rpm_2 = 0;
          motor_right_rpm_3 = -y_axis_movement;
          motor_right_rpm_4 = 0;
        }
        else if (y_axis_movement < 0 && x_axis_movement > 0)
        {
          // go backward right
          motor_left_rpm_1 = -y_axis_movement;
          motor_left_rpm_2 = 0;
          motor_right_rpm_3 = -y_axis_movement - x_axis_movement;
          motor_right_rpm_4 = 0;
        }
        else if (y_axis_movement < 0 && x_axis_movement < 0)
        {
          // go backward left
          motor_left_rpm_1 = -y_axis_movement + x_axis_movement;
          motor_left_rpm_2 = 0;
          motor_right_rpm_3 = -y_axis_movement;
          motor_right_rpm_4 = 0;
        }
        else
        {
          motor_left_rpm_1 = 0;
          motor_left_rpm_2 = 0;
          motor_right_rpm_3 = 0;
          motor_right_rpm_4 = 0;
        }

        if (y_axis_movement != 0 || x_axis_movement != 0 || z_axis_rotation != 0 || stop_mode == STOP_HARD_MODE)
        {
          digitalWrite(R_EN_1, HIGH);
          digitalWrite(R_EN_2, HIGH);
          digitalWrite(L_EN_1, HIGH);
          digitalWrite(L_EN_2, HIGH);
        }
        else
        {
          // stop softly output floating
          digitalWrite(R_EN_1, LOW);
          digitalWrite(R_EN_2, LOW);
          digitalWrite(L_EN_1, LOW);
          digitalWrite(L_EN_2, LOW);
        }
        motor_left_rpm_1 = constrain(motor_left_rpm_1, PWM_MIN, PWM_MAX);
        motor_left_rpm_2 = constrain(motor_left_rpm_2, PWM_MIN, PWM_MAX);
        motor_right_rpm_3 = constrain(motor_right_rpm_3, PWM_MIN, PWM_MAX);
        motor_right_rpm_4 = constrain(motor_right_rpm_4, PWM_MIN, PWM_MAX);
        ledcWrite(motor1Channel, motor_left_rpm_1);
        ledcWrite(motor2Channel, motor_left_rpm_2);
        ledcWrite(motor3Channel, motor_right_rpm_3);
        ledcWrite(motor4Channel, motor_right_rpm_4);
      }
    }
    // debugging
    for (int8_t i = 0; i < 5; i++)
    {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }

    Serial.print(data.lost_frame);
    Serial.print("\t");

    Serial.println(data.failsafe);

    if (millis() - last_packet_recieved > failsafe_time_in_millis)
    {

      // failsafe
    }
  }
}
void calibrate_alternate()
{
  // add live calibration for sticks butt center calibration of some other channel
  // calibration on every startup
  // on startup rotate sticks and switches to all of thier end points
  // and then center all sticks within 20 seconds
  Serial.println("LIVE CALIBRATION MODE");
  for (int i = 0; i < 16; i++)
  {
    min_ch[i] = 1000;
    max_ch[i] = 1000;
    mid_ch[i] = 1000;
  }
  unsigned t = millis();
  while (true and millis() - t < 20000)
  {
    if (sbus_rx.Read())
    {
      data = sbus_rx.data();
      Serial.print("Move sticks to all corners   ");
      Serial.println(t + 15000 - millis());

      for (int i = 0; i < 16; i++)
      {
        max_ch[i] = data.ch[i] > max_ch[i] ? data.ch[i] : max_ch[i];
        min_ch[i] = data.ch[i] < min_ch[i] ? data.ch[i] : min_ch[i];
      }
    }
  }

  for (int i = 0; i < 16; i++)
  {
    mid_ch[i] = data.ch[i];
  }
}

void calibrate()
{
  Serial.println("Rotate sticks in all directions");
  int max_ch[16] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  int mid_ch[16] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  int min_ch[16] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

  unsigned t = millis();
  while (true and millis() - t < 15000)
  {
    if (sbus_rx.Read())
    {
      data = sbus_rx.data();
      Serial.print("Move sticks to all corners   ");
      Serial.println(t + 15000 - millis());

      for (int i = 0; i < 16; i++)
      {
        max_ch[i] = data.ch[i] > max_ch[i] ? data.ch[i] : max_ch[i];
        min_ch[i] = data.ch[i] < min_ch[i] ? data.ch[i] : min_ch[i];
      }
    }
  }

  t = millis();
  while (millis() - t < 10000)
  {
    if (sbus_rx.Read())
    {
      data = sbus_rx.data();
      Serial.print("Center all sticks   ");
      Serial.println(t + 10000 - millis());
    }
  }

  for (int i = 0; i < 16; i++)
  {
    mid_ch[i] = data.ch[i];
  }

  Serial.println("Calibration done paste this in code");
  Serial.print("int max_ch[16] = {");
  for (int i = 0; i < 15; i++)
  {
    Serial.print(max_ch[i]);
    Serial.println(",");
  }
  Serial.print(max_ch[15]);
  Serial.println("};");

  Serial.print("int mid_ch[16] = {");
  for (int i = 0; i < 15; i++)
  {
    Serial.print(mid_ch[i]);
    Serial.println(",");
  }
  Serial.print(mid_ch[15]);
  Serial.println("};");

  Serial.print("int min_ch[16] = {");
  for (int i = 0; i < 15; i++)
  {
    Serial.print(min_ch[i]);
    Serial.println(",");
  }
  Serial.print(min_ch[15]);
  Serial.println("};");

  while (true)
  {
  }
}
