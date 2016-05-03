/* ME 131 Spring 2016 Term Project
 * Autonomous Lane-Keeping and Obstacle Avoidance
 * Cheng Hao Yuan, Hohyun Song, Tony Abdo
 * 05/09/2016
 */

#include <Servo.h>

// loop period in s and ms, and rate in Hz
#define LOOP_PERIOD       0.02
#define LOOP_RATE         1.0/LOOP_PERIOD
#define LOOP_PERIOD_MS    LOOP_PERIOD*1000.0
#define EXPOSURE_TIME_US  6000

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
#define MAX_STEERING  150
#define MIN_STEERING  30
#define MAX_THROTTLE  130
#define MIN_THROTTLE  90


// global storage
Servo ESC, SERVO;
volatile int encoder_count;
int camera_data[128];
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
  

  // initialize camera


  // initialize velocity control variables
  encoder_count = 0;
  velocity_ref = 0;
  lane_ref = 1; // middle lane by default
  

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


  // read camera
  clear_camera();
  delayMicroseconds(EXPOSURE_TIME_US);
  read_camera();
  Serial.println(find_max(camera_data));

  // process camera


  // read ultrasonics


  // high level steering strategy (lane selection and obstacle avoidance)


  // low level lane keeping PID


  // compute and set throttle


  // wait for long enough to fulfill loop period
  Serial.println(millis()-loop_start_time);
//  delay(LOOP_PERIOD_MS-(millis()-loop_start_time));
  
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
  return;
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
  return;
}

/* Author: Cheng Hao Yuan
 * brake to stop, halts the program until reset is pressed. turns off LED13 for indication.
 */
void motor_brake() {
  ESC.write(80);
  digitalWrite(LED, LOW);
  while(1); 
  return;
}


/* Author: Cheng Hao Yuan
 * ISR for both wheel encoders. Increments encoder_count by 1 when called.
 * returns nothing.
 */
void encoderISR() {
  encoder_count++; 
  return;
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


/* Author:
 * converts steering angle percent (0.0-1.0) to servo angle (0-180).
 * this is the servo mapping.
 */
int steering_to_servo(float steering) {
  // FIXME
  
  return 90;
}


/* Author: Cheng Hao Yuan
 * writes the servo angle to the servo
 * returns nothing.
 */
void set_servo(int servo_angle) {
  if (servo_angle < MIN_STEERING) servo_angle = MIN_STEERING;
  if (servo_angle > MAX_STEERING) servo_angle = MAX_STEERING;
  SERVO.write(servo_angle);  
  return;
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
  return;
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
  return;
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


/* Author:
 * process camera data and produce three lane centers (L,M,R).
 * writes in passed-in arrays, returns nothing.
 */
void process_camera(int *camera, int *lanes) {
  // FIXME
  
  return;
}


/* Author:
 * reads all ultrasonic sensors, converts to meters, and store in passed-in array.
 * returns nothing.
 */
void read_ultrasonic(float *dist) {
  // FIXME
  
  return;
}


/* Author:
 * takes lane positions and ultrasonic clearance distances and logically decide which lane position to use.
 * sets the velocity reference and lane reference (globals).
 * this is the logic for obstacle avoidance and lane selection (and stopping if can't merge).
 * returns nothing.
 */
void path_control(int *lanes, float *distances) {
  // FIXME
  
  return;
}

/* Author:
 * uses lane reference and lane positions to determine lateral error.
 * produce steering angle (0.0-1.0) to reduce lateral error.
 */
float steering_PID(int *lanes, int lane_num) {
  // FIXME
  
  return 0.5;
}







