#include <Servo.h>

// loop period in s and ms, and rate in Hz
#define LOOP_PERIOD 0.02
#define LOOP_RATE 1.0/LOOP_PERIOD
#define LOOP_PERIOD_MS LOOP_PERIOD*1000.0

// pinout definitions
#define ESC_PIN       10
#define SERVO_PIN     11
#define ENCODER_FL    2
#define ENCODER_FR    3
#define ULTRASONIC_1  5
#define ULTRASONIC_2  6
#define ULTRASONIC_3  7
#define ULTRASONIC_4  8
#define CAMERA_AOUT   A0
#define CAMERA_SI     A1
#define CAMERA_CLK    A2
#define LED           13

// global storage
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


  // attach interrupts


  // initialize camera


  // initialize velocity control variables


  // arming ESC and set initial posture


  // blink LED for indication
  

}

void loop() {
  // set time stamp
  loop_start_time = millis();
  
  // take in and process Serial commands ('L', 'M', 'R')


  // read camera


  // process camera


  // read ultrasonics


  // high level steering strategy (lane selection and obstacle avoidance)


  // low level lane keeping PID


  // compute and set throttle


  // wait for long enough to fulfill loop period
  Serial.println(millis()-loop_start_time);
  delay(LOOP_PERIOD_MS-(millis()-loop_start_time));
}

/* Author:
 * float pwm is between 0.0 and 1.0, where 0.0 is coasting, and 1.0 is full throttle forward.
 * converts float pwm to analogWrite(0-255).
 * returns nothing.
 */
void motor_forward(float pwm) {
  // FIXME
  
  return;
}

/* Author:
 * float pwm is between 0.0 and 1.0, where 0.0 is coasting, and 1.0 is full brake.
 * should have protection against reverse throttle upon second brake.
 * converts float pwm to analogWrite(0-255).
 * returns nothing.
 */
void motor_brake(float pwm) {
  // FIXME
  
  return;
}


/* Author:
 * ISR for both wheel encoders. Increments encoder_count by 1 when called.
 * returns nothing.
 */
void encoderISR() {
  // FIXME
  
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


/* Author:
 * writes the servo angle to the servo
 * returns nothing.
 */
void set_servo(int servo_angle) {
  // FIXME
  
  return;
}


/* Author:
 * initializes camera.
 */
void initialize_camera() {
  // FIXME
  
  return;
}


/* Author:
 * read camera into the passed-in array.
 * returns nothing.
 */
void read_camera(int *data) {
  // FIXME
  
  return;
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
 * reads all ultrasonic sensors and store in passed-in array.
 * returns nothing.
 */
void read_ultrasonic(int *dist) {
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







