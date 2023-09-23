#include <Arduino.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU9150.h"

#include <ContinuousStepper.h>
#include <TimerOne.h>


const int pin_step_x = 2;
const int pin_step_z = 4;
const int pin_dir_x = 5;
const int pin_dir_z = 7;
const int pin_enable = 8;

const float full_scale_accel = 4.0f / 32768.0f;
const float full_scale_gyro = 1000.0f / 32768.0f;
const float step_per_deg = 4.0f / 1.8f;


MPU9150 mpu;
ContinuousStepper stepper_a;
ContinuousStepper stepper_b;

float angle_deg_setpoint = 0.0;
float wheel_speed_deg_setpoint = 0.0;

float wheel_speed_deg = 0.0;

void stepper_callback()
{
    stepper_a.loop();
    stepper_b.loop();
}

void setup() {
  // Setup serial
  Serial.begin(9600);

  // Setup I2C
  Wire.begin();

  // Configure mpu
  mpu.initialize();
  mpu.setFullScaleAccelRange(1); // [0,1,2,3] = [2g,4g,8g,16g]
  mpu.setFullScaleGyroRange(2); // [0,1,2,3] = [250,500,1000,2000] degs/sec


  stepper_a.begin(pin_step_x,pin_dir_x);
  stepper_b.begin(pin_step_z,pin_dir_z);
  stepper_a.setAcceleration(999999.0f);
  stepper_b.setAcceleration(999999.0f);


  Timer1.initialize(50);
  Timer1.attachInterrupt(stepper_callback);

  pinMode(pin_enable,OUTPUT);
  digitalWrite(pin_enable,LOW);

}

float calculate_delta_time(){
  static unsigned long last_time = 0;
  unsigned long current_time = micros();
  unsigned long delta_time = current_time - last_time;
  last_time = current_time;
  return (float)delta_time / 1000000;
}

float complementary_filter(float x, float x_dot, float dt)
{
  static float y = x;
  const float a = 0.03;
  y += a*(x - y) + (1.0-a)*x_dot*dt;
  return y;
}

void read_angle_and_rate_from_mpu(float* angle_deg, float* angle_rate_deg)
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
  // Read imu state
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  //Scale imu states of interest
  float accel_g_y = (float)ay * full_scale_accel;
  float accel_g_z = (float)az * full_scale_accel;
  *angle_rate_deg = (float)gx * full_scale_gyro;
  
  *angle_deg = atan2(accel_g_y,accel_g_z) *180 / PI;
}

float calculate_balance_angle_rate(float angle_error_deg)
{
  //converts the angle error into a angle rate target that should stand the bot back up

  const float C = 123.0f; //Constant found from fall test

  float angle_rate_rad = sqrt(C*(1.0f-cos(radians(angle_error_deg))));
  float angle_rate_deg = degrees(angle_rate_rad);

  if (angle_error_deg > 0.0f)
  {
    angle_rate_deg = -angle_rate_deg;
  }

  return angle_rate_deg; 
}

float sign(float x){
  if (x>0.0f){
    return 1.0f;
  }
  else{
    return -1.0f;
  }
}

void loop() {

  float angle_deg, angle_rate_deg;

  read_angle_and_rate_from_mpu(&angle_deg, &angle_rate_deg);

  float dt = calculate_delta_time();

  float filtered_angle = complementary_filter(angle_deg, angle_rate_deg, dt);

  float wheel_speed_error_deg = wheel_speed_deg_setpoint - wheel_speed_deg;

  float angle_deg_setpoint_offset = 0.01*wheel_speed_error_deg;
  angle_deg_setpoint_offset = constrain(angle_deg_setpoint_offset,-5,5);
  angle_deg_setpoint += 0.001 * wheel_speed_error_deg*dt;

  float angle_deg_target = angle_deg_setpoint + angle_deg_setpoint_offset;

  float angle_error_deg = filtered_angle - angle_deg_target;

  float angle_rate_deg_setpoint = calculate_balance_angle_rate(angle_error_deg);

  float angle_rate_deg_error = angle_rate_deg - angle_rate_deg_setpoint; 

  float wheel_acceleration = 54 * angle_rate_deg_error;

  wheel_acceleration = constrain(wheel_acceleration,-2882,2882);

  wheel_speed_deg += wheel_acceleration*dt;
  const float wheel_speed_deg_limit = 360*5;
  wheel_speed_deg = constrain(wheel_speed_deg,-wheel_speed_deg_limit,wheel_speed_deg_limit);
  
  // angle_deg_setpoint -= 0.1 * sign(wheel_speed);

  bool enable_wheels = abs(filtered_angle) < 45;

  digitalWrite(pin_enable,!enable_wheels); 

  if (!enable_wheels){
    wheel_speed_deg = 0.0;
    angle_deg_setpoint=0.0;
  }


  noInterrupts();
  stepper_a.spin(-wheel_speed_deg*step_per_deg);
  stepper_b.spin(wheel_speed_deg*step_per_deg);
  interrupts();

  Serial.print(angle_deg_setpoint); Serial.print("\t");
  Serial.print(filtered_angle); Serial.print("\t");
  // Serial.print(angle_error_deg); Serial.print("\t");
  Serial.print(angle_rate_deg_setpoint); Serial.print("\t");
  Serial.print(angle_rate_deg); Serial.print("\t");
  // Serial.print(angle_rate_deg_error); Serial.print("\t");
  // Serial.print(wheel_speed); Serial.print("\t");
  Serial.println();
  
  // Serial.print(dt); Serial.print("\t");
  // Serial.print(accel_g_y); Serial.print("\t");
  // Serial.print(accel_g_z); Serial.print("\t");
  // Serial.print(angle_rate_deg); Serial.print("\t");
  // Serial.print(angle_deg); Serial.print("\t");
  // Serial.print(filtered_angle); Serial.print("\t");
  // Serial.println();
  // Serial.print("a/g/m:\t");
  // Serial.print(ax); Serial.print("\t");
  // Serial.print(ay); Serial.print("\t");
  // Serial.print(az); Serial.print("\t");
  // Serial.print(gx); Serial.print("\t");
  // Serial.print(gy); Serial.print("\t");
  // Serial.print(gz); Serial.print("\t");
  // Serial.print(int(mx)*int(mx)); Serial.print("\t");
  // Serial.print(int(my)*int(my)); Serial.print("\t");
  // Serial.print(int(mz)*int(mz)); Serial.print("\t | ");

  // put your main code here, to run repeatedly:
  // delayMicroseconds(500);
  // digitalWrite(pin_step_x,HIGH);
  // digitalWrite(pin_step_z,HIGH);
  // delayMicroseconds(500);
  // digitalWrite(pin_step_x,LOW);
  // digitalWrite(pin_step_z,LOW);
  // delay(10);




  
}




