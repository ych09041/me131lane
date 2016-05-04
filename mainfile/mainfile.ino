/* ME 131 Spring 2016 Term Project
 * Autonomous Lane-Keeping and Obstacle Avoidance
 * Cheng Hao Yuan, Hohyun Song, Tony Abdo
 * 05/09/2016
 */

#include <Servo.h>

// loop period in s and ms, and rate in Hz
#define LOOP_PERIOD       0.05
#define LOOP_RATE         1.0/LOOP_PERIOD
#define LOOP_PERIOD_MS    LOOP_PERIOD*1000.0

// camera constants
#define EXPOSURE_TIME_US  9000
#define PEAK_CUTOFF       0.3
#define MAX_WIDTH_LIMIT   40
#define MIN_WIDTH_LIMIT   2
#define FOV_CENTER        63

// controller gains
#define STEERING_KP     1
#define STEERING_KD     0
#define VELOCITY_KP     0.1
#define VELOCITY_KI     0

// pinout definitions
#define ESC_PIN       10
#define SERVO_PIN     11
#define ENCODER_FL    2
#define ENCODER_FR    3
#define ULTRASONIC_1  A0
#define ULTRASONIC_2  A1
#define ULTRASONIC_3  A2
#define ULTRASONIC_4  A3
#define CAMERA_AOUT   A4
#define CAMERA_SI     9
#define CAMERA_CLK    4
#define LED           13

// actuator clamping limits
#define MAX_STEERING  140
#define MIN_STEERING  40
#define MAX_THROTTLE  130
#define MIN_THROTTLE  90


// global storage
Servo ESC, SERVO;
volatile int encoder_count;
int camera_data[128];
float normalized_data[128];
int cutoff_data[128];
int lane_centers[3];
float ultrasonic_dist[4];
float velocity_ref;
int lane_ref;

unsigned long loop_start_time;


void setup() {
  // begin serial
  Serial.begin(115200);

  // set pinmodes
  pinMode(ENCODER_FL, INPUT_PULLUP);
  pinMode(ENCODER_FR, INPUT_PULLUP);
  pinMode(ESC_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(CAMERA_SI, OUTPUT);
  pinMode(CAMERA_CLK, OUTPUT);  
  

  // attach interrupts and servos
  ESC.attach(ESC_PIN);
  SERVO.attach(SERVO_PIN);
  attachInterrupt(0, encoderISR, CHANGE); // D2, FL
  attachInterrupt(1, encoderISR, CHANGE); // D3, FR


  // initialize velocity control variables
  encoder_count = 0;
  velocity_ref = 0;
  lane_ref = 0; // middle lane by default
  

  // arming ESC and set initial posture
  SERVO.write(90);
  ESC.write(90);
  delay(1000);
  

  // blink LED for indication
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);  

}

void loop() {
  // set time stamp
  loop_start_time = millis();
  
  // take in and process Serial commands ('L', 'M', 'R')
  read_lane_change_serial();

  // read camera
  clear_camera();
  delayMicroseconds(EXPOSURE_TIME_US);
  read_camera();
  
  // process camera
  process_camera();
  Serial.print(lane_centers[0]);
  Serial.print('\t');
  Serial.print(lane_centers[1]);
  Serial.print('\t');
  Serial.print(lane_centers[2]);
  Serial.println();
  

  // read ultrasonics


  // high level steering strategy (lane selection and obstacle avoidance)


  // low level lane keeping PID
  float steering_percent = steering_PID();
  int servo_write = steering_to_servo(steering_percent);
  Serial.println(servo_write);
  set_servo(servo_write);


  // compute and set throttle


  // wait for long enough to fulfill loop period
//  Serial.println(millis()-loop_start_time);
  delay(LOOP_PERIOD_MS-(millis()-loop_start_time));
  
}






/* Author: Cheng Hao Yuan
 * reads lane change command from serial, if available.
 */
void read_lane_change_serial() {
  if (Serial.available() > 0) {
    int byte_read = Serial.read();
    if (byte_read == 76) lane_ref = 0;
    if (byte_read == 77) lane_ref = 1;
    if (byte_read == 82) lane_ref = 2;
    Serial.print("Lane change to: ");
    Serial.println(lane_ref);
  }
}


/* Author: Cheng Hao Yuan
 * float pwm is between 0.0 and 1.0, where 0.0 is coasting, and 1.0 is full throttle forward.
 * converts float pwm to analogWrite(0-255).
 * returns nothing.
 */
void motor_forward(float pwm) {
  if (pwm < MIN_THROTTLE) pwm = MIN_THROTTLE;
  if (pwm > MAX_THROTTLE) pwm = MAX_THROTTLE;
  ESC.write(pwm);
}

/* Author: Cheng Hao Yuan
 * brake to stop, halts the program until reset is pressed. turns off LED13 for indication.
 */
void motor_brake() {
  ESC.write(80);
  digitalWrite(LED, LOW);
  while(1);
}


/* Author: Cheng Hao Yuan
 * ISR for both wheel encoders. Increments encoder_count by 1 when called.
 * returns nothing.
 */
void encoderISR() {
  encoder_count++;
}


/* Author:
 * computes throttle PID based on velocity setpoint (arg) and encoder count (global).
 * velocity is computed inside this function, the velocity global is updated.
 * returns throttle as float in range of 0.0-1.0
 */
float motor_PID(float velocity_setpoint) {
  // FIXME
  
  return 0.0;
}


/* Author: Cheng Hao Yuan
 * converts steering angle percent (0.0-1.0) to servo angle (0-180).
 * this is the servo mapping.
 */
int steering_to_servo(float steering) {
  return (int)((steering + 0.5) * 180.0);
}


/* Author: Cheng Hao Yuan
 * writes the servo angle to the servo
 * returns nothing.
 */
void set_servo(int servo_angle) {
  if (servo_angle < MIN_STEERING) servo_angle = MIN_STEERING;
  if (servo_angle > MAX_STEERING) servo_angle = MAX_STEERING;
  SERVO.write(servo_angle);
}


/* Author: Cheng Hao Yuan
 * dumps old pixel values on the camera.
 */
void clear_camera() {
  digitalWrite(CAMERA_SI, HIGH);
  digitalWrite(CAMERA_CLK, HIGH);
  digitalWrite(CAMERA_SI, LOW);
  digitalWrite(CAMERA_CLK, LOW);
  for(int i = 0; i < 128; i++){
    digitalWrite(CAMERA_CLK, HIGH);
    digitalWrite(CAMERA_CLK, LOW);
  }
}


/* Author: Cheng Hao Yuan
 * read camera into the passed-in array.
 * returns nothing.
 */
void read_camera() {
  digitalWrite(CAMERA_SI, HIGH);
  digitalWrite(CAMERA_CLK, HIGH);
  digitalWrite(CAMERA_SI, LOW);
  digitalWrite(CAMERA_CLK, LOW);
  for(int i = 0; i < 128; i++) {
    camera_data[i] = analogRead(CAMERA_AOUT);
    digitalWrite(CAMERA_CLK, HIGH);
    digitalWrite(CAMERA_CLK, LOW);
  }
}



/* Author: Cheng Hao Yuan
 * returns the index of the max entry in an 128-element array.
 */
int find_max(int *arr) {
  int temp_max = 0;
  int temp_indmax = 0;
  for (int i=0; i<128; i++) {
    if (arr[i]>temp_max) {
      temp_max = arr[i];
      temp_indmax = i;
    }
  }
  return temp_indmax;
}

/* Author: Cheng Hao Yuan
 * returns the index of the min entry in an 128-element array.
 */
int find_min(int *arr) {
  int temp_min = 10000;
  int temp_indmin = 0;
  for (int i=0; i<128; i++) {
    if (arr[i]<temp_min) {
      temp_min = arr[i];
      temp_indmin = i;
    }
  }
  return temp_indmin;
}

/* Author: Cheng Hao Yuan
 * process camera data and produce three lane centers (L,M,R).
 * writes in global arrays, returns nothing.
 */
void process_camera() {
  // normalize data to 0.0-1.0
  int max_reading = camera_data[find_max(camera_data)];
  int min_reading = camera_data[find_min(camera_data)];
  float height_diff = (float)max_reading - (float)min_reading;
  for (int i=0; i<128; i++) {
    normalized_data[i] = (float)(camera_data[i] - min_reading) / height_diff;
  }

  // cutoff data at threshold
  for (int i=0; i<128; i++) {
    if (normalized_data[i] > PEAK_CUTOFF) {
      cutoff_data[i] = 1;
    } else {
      cutoff_data[i] = 0;
    }
  }

  // find three line centers
  bool started1 = false;
  bool ended1 = false;
  int start1 = 0;
  int end1 = 0;
  bool started2 = false;
  bool ended2 = false;
  int start2 = 0;
  int end2 = 0;
  bool started3 = false;
  bool ended3 = false;
  int start3 = 0;
  int end3 = 0;
  for (int i=0; i<128; i++) {
    // first peak
    if (cutoff_data[i] == 1 && !started1) {
      started1 = true;
      start1 = i;
    }
    if (cutoff_data[i] == 0 && started1) {
      end1 = i;
      if (end1 - start1 <= MAX_WIDTH_LIMIT && end1 - start1 >= MIN_WIDTH_LIMIT) {
        lane_centers[0] = (start1+end1)/2;
        ended1 = true;
      }
    }
/*
    // second peak
    if (cutoff_data[i] == 1 && !started2 && ended1) {
      started2 = true;
      start2 = i;
    }
    if (cutoff_data[i] == 0 && started2) {
      end2 = i;
      if (end2 - start2 <= MAX_WIDTH_LIMIT && end2 - start2 >= MIN_WIDTH_LIMIT) {
        lane_centers[1] = (start2+end2)/2;
        ended2 = true;
      }
    }

    // third peak
    if (cutoff_data[i] == 1 && !started3 && ended2) {
      started3 = true;
      start3 = i;
    }
    if (cutoff_data[i] == 0 && started3) {
      end3 = i;
      if (end3 - start3 <= MAX_WIDTH_LIMIT && end3 - start3 >= MIN_WIDTH_LIMIT) {
        lane_centers[2] = (start3+end3)/2;
        ended3 = true;
      }
    }

    */
  }
}


/* Author:
 * reads all ultrasonic sensors, converts to meters, and store in global array ultrasonic_dist.
 * returns nothing.
 */
void read_ultrasonic() {
  // FIXME
  
}


/* Author:
 * takes lane positions and ultrasonic clearance distances and logically decide which lane position to use.
 * sets the velocity_ref and lane_ref (globals).
 * this is the logic for obstacle avoidance and lane selection (and stopping if can't merge).
 * returns nothing.
 */
void path_control() {
  // FIXME
  

}

/* Author: Cheng Hao Yuan
 * uses lane_ref and lane_centers to determine lateral error.
 * produce steering angle (0.0-1.0) to reduce lateral error.
 */
float steering_PID() {
  float e_lat = (FOV_CENTER - lane_centers[lane_ref])/128.0;
  float steering_signal = STEERING_KP * e_lat;
  return steering_signal;
}







