/* ME 131 Spring 2016 Term Project
 * Autonomous Lane-Keeping and Obstacle Avoidance at Constant Velocity
 * Cheng Hao Yuan, Hohyun Song, Tony Abdo
 * 05/09/2016
 */

#include <Servo.h>

// loop period in s and ms, and rate in Hz
#define LOOP_PERIOD       0.025
#define LOOP_RATE         1.0/LOOP_PERIOD
#define LOOP_PERIOD_MS    LOOP_PERIOD*1000.0

// camera constants
#define EXPOSURE_TIME_US  10000
#define PEAK_CUTOFF       0.5
#define MAX_WIDTH_LIMIT   40
#define MIN_WIDTH_LIMIT   1
#define CENTER_LANE       80
#define LEFT_LANE         110
#define RIGHT_LANE        53

// controller gains
#define STEERING_KP     0.5
#define STEERING_KD     0
#define VELOCITY_KP     .12
#define VELOCITY_KI     0.15
#define VELOCITY_KD     0.0

// pinout definitions
#define ESC_PIN       10
#define SERVO_PIN     11
#define ENCODER_FL    2
#define ENCODER_FR    3
#define ULTRASONIC_F  1  // A1
#define ULTRASONIC_L  2  // A2
#define ULTRASONIC_R  3  // A3
#define CAMERA_AOUT   4  // A4
#define CAMERA_SI     9
#define CAMERA_CLK    4
#define LED           13

// actuator clamping limits
#define MAX_STEERING  140
#define MIN_STEERING  40
#define MAX_THROTTLE  130
#define MIN_THROTTLE  90

// ultrasonic obstacle detection thresholds
#define FRONT_SPACE   0.60  // meters
#define SIDE_SPACE    0.30

// Define ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);


// global storage
Servo ESC, SERVO;
volatile int encoder_count1;
volatile int encoder_count2;
int camera_data[128];
int cutoff_height;
float normalized_data[128];
int cutoff_data[128];
int line_centers[2];
int number_of_lines;
int lat_pos;
float ultrasonic_dist[4]; // front, left, right
float velocity_ref;
int lane_ref;
int merge_state;
bool run_motor_PID;

unsigned long loop_start_time;

// Added by Tony
float ITerm = 0.0;
float currVelocity = 0.0;
float lastInput =0.0;
float MOTORINTEGRALMIN = -.6;
float MOTORINTEGRALMAX = .6;
float TICKTOMETERS = .07253/4.0*3.1415;
float currPos = 0.0;
float prevPos = 0.0;




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

  // setup fast analog read
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_16; 

  // attach interrupts and servos
  ESC.attach(ESC_PIN);
  SERVO.attach(SERVO_PIN);
  attachInterrupt(0, encoderISR1, CHANGE); // D2, FL
  attachInterrupt(1, encoderISR2, CHANGE); // D3, FR


  // initialize velocity control variables
  encoder_count1 = 0;
  encoder_count2 = 0;
  velocity_ref = 0.5;
  lane_ref = CENTER_LANE; // middle lane by default
  merge_state = 0;
  run_motor_PID = true;

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
  
  // read camera
  clear_camera();
  delayMicroseconds(EXPOSURE_TIME_US);
  read_camera();
  
  // process camera
  process_camera();
//  Serial.print("line centers: ");
//  Serial.print(line_centers[0]);
//  Serial.print('\t');
//  Serial.print(line_centers[1]);
//  Serial.println();

//  Serial.print("num lines: ");
//  Serial.println(number_of_lines);
//  Serial.print("lane_ref: ");
//  Serial.println(lane_ref);
//  Serial.print("lat_pos: ");
//  Serial.println(lat_pos);
  Serial.print("merge state: ");
  Serial.println(merge_state);
  

  // read serial commands ('L', 'M', 'R')
//  read_serial_command();  
  
//  Serial.print("ultrasonic: ");
//  Serial.print(ultrasonic_dist[0]);
//  Serial.print('\t');
//  Serial.print(ultrasonic_dist[1]);
//  Serial.print('\t');
//  Serial.print(ultrasonic_dist[2]);
//  Serial.println();

  // read ultrasonic sensors
  read_ultrasonics();
  
  // high level steering strategy (lane selection and obstacle avoidance)
  path_control_by_ultrasonics();
  

  // low level lane keeping PID
  float steering_percent = steering_PID();
  int servo_write = steering_to_servo(steering_percent);
//  Serial.println(servo_write);
  set_servo(servo_write);

  
  // velocity control, compute and set throttle, every other loop
  if (true) {
    currPos = (encoder_count1 + encoder_count2)*TICKTOMETERS/2;
    
    currVelocity = (currPos-prevPos)/LOOP_PERIOD;
    prevPos = currPos;
    float motor_output = motor_PID(velocity_ref,currVelocity);
    motor_forward(motor_output);
  }
  run_motor_PID = !run_motor_PID;

//  Serial.print("Velocity");
//  Serial.println(currVelocity);
//  Serial.print("Encoders");
//  Serial.print(encoder_count1);
//  Serial.print('\t');
//  Serial.println(encoder_count2);


  // wait for long enough to fulfill loop period
  Serial.print("Loop usage us: ");
  Serial.println(millis()-loop_start_time);
  delay(LOOP_PERIOD_MS-(millis()-loop_start_time));
  
}


////////////////////////////////////////////////////////////////////////
////////////////////////// Utitlities //////////////////////////////////
////////////////////////////////////////////////////////////////////////



/* Author: Cheng Hao Yuan
 * reads lane change command from serial, if available.
 */
int command = 77;
void read_serial_command() {
  // read serial and update command char, if any
  if (Serial.available() > 0) {
    command = Serial.read();
  }
  
  if (command == 76) { // 'L'
    merge_to_left();
  } else if (command == 77) { // 'M'
    merge_to_middle();
  } else if (command == 82) { // 'R'
    merge_to_right();
  }
}

/* Author: Cheng Hao Yuan
 * helper function that sets state machine to track left lane
 */
void merge_to_left() {
  lane_ref = LEFT_LANE;
  merge_state = 1;
}

/* Author: Cheng Hao Yuan
 * helper function that sets state machine to track/return to center lane
 */
void merge_to_middle() {
  if (number_of_lines == 2) {
    lane_ref = CENTER_LANE;
    merge_state = 0;
  } else if (number_of_lines == 1) {
    if (merge_state == 1) { // currently in left, trying to return middle
      lane_ref = RIGHT_LANE;
    } else if (merge_state == 2) { // currently in right, trying to return middle
      lane_ref = LEFT_LANE;
    }
  }
}


/* Author: Cheng Hao Yuan
 * helper function that sets state machine to track right lane
 */
void merge_to_right() {
  lane_ref = RIGHT_LANE;
  merge_state = 2;

}


/* Author: Tony Abdo
 * scales x originally in range of [in_min, in_max] to [out_min, out_max]
 */
double myMap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Author: Tony Abdo
 * float pwm is between 0.0 and 1.0, where 0.0 is coasting, and 1.0 is full throttle forward.
 * converts float pwm to analogWrite(0-255).
 * returns nothing.
 */
void motor_forward(float pwm) {
  int command = (int)myMap(pwm, 0.0, 1.0,90.0, 180.0 );
  
  if (command < MIN_THROTTLE) command = MIN_THROTTLE;
  if (command > MAX_THROTTLE) command = MAX_THROTTLE;
  Serial.print("motor write: ");
  Serial.println(command);
  ESC.write(command);
}

/* Author: Cheng Hao Yuan
 * brake to stop, halts the program until reset is pressed. turns off LED13 for indication.
 */
void motor_brake() {
  ESC.write(80);
  digitalWrite(LED, LOW);
  while(1);
}


/* Author: Tony Abdo
 * ISR for one wheel encoder. Increments encoder_count by 1 when called.
 * returns nothing.
 */
void encoderISR1() {
  encoder_count1++;
}

/*
 * Author: Tony Abdo
 * ISR for other whell. Ensures that both interrupts are heard
 */
void encoderISR2() {
  encoder_count2++;
//  currVelocity = 1000000.0*TICKTOMETERS/(micros()-velocity_timestamp);
//  velocity_timestamp = micros();
}


/* Author: Tony Abdo
 * computes throttle PID based on velocity setpoint (arg) and encoder count (global).
 * velocity is computed inside this function, the velocity global is updated.
 * returns throttle as float in range of 0.0-1.0
 */
float motor_PID(float velocity_setpoint, float input) {
  /*Compute all the working error variables*/
  float error = velocity_setpoint - input;
  ITerm+= (VELOCITY_KI * error*(LOOP_PERIOD*2)); // x2 because every other loop
//  Serial.print("Iterm");
//  Serial.println(ITerm);
//  Serial.print("Error");
//  Serial.println(error);
  if(ITerm > MOTORINTEGRALMAX) ITerm= MOTORINTEGRALMAX;
  else if(ITerm < MOTORINTEGRALMIN) ITerm= MOTORINTEGRALMIN;
  float dInput = (input - lastInput);

  /*Compute PID Output*/
  float output = (VELOCITY_KP*error + ITerm- VELOCITY_KD*dInput/(LOOP_PERIOD*2)); // x2 because every other loop
  
  if(output > 1.0) output = 1.0;
  else if(output < -1.0) output = -1.0;
  
  /*Remember for next time*/
  lastInput = input;
  
  return output;
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
  PORTB |= B00000010; // SI high
  PORTD |= B00010000; // CLK high
  PORTB ^= B00000010; // SI low
  PORTD ^= B00010000; // CLK low
  for(int i = 0; i < 128; i++){
    PORTD |= B00010000; // CLK high
    PORTD ^= B00010000; // CLK low
  }
}


/* Author: Cheng Hao Yuan
 * read camera into the passed-in array.
 * returns nothing.
 */
void read_camera() {
  PORTB |= B00000010; // SI high
  PORTD |= B00010000; // CLK high
  PORTB ^= B00000010; // SI low
  PORTD ^= B00010000; // CLK low
  for(int i = 0; i < 128; i++){
    camera_data[i] = analogRead(CAMERA_AOUT);
    PORTD |= B00010000; // CLK high
    PORTD ^= B00010000; // CLK low
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
 * process camera data and produce one or two line centers.
 * writes in global arrays, returns nothing.
 */
void process_camera() {
  int max_reading = camera_data[find_max(camera_data)];
  int min_reading = camera_data[find_min(camera_data)];
  cutoff_height = min_reading + (int)((float)PEAK_CUTOFF * (float)(max_reading - min_reading));
  for (int i=0; i<128; i++) {
    cutoff_data[i] = camera_data[i] / cutoff_height;
  }

  // count number of line centers
  int pi = 0, pj = 0;
  number_of_lines = 0;
  for (int i=0; i<128-1; i++) {
    pi = cutoff_data[i];
    pj = cutoff_data[i+1];
    if ((pi == 0 && pj == 1) || (pi == 1 && pj == 0)) {
      number_of_lines++;
    }
  }
  number_of_lines /= 2;

  // find line centers
  bool started1 = false;
  bool ended1 = false;
  int start1 = 0;
  int end1 = 0;
  bool started2 = false;
  bool ended2 = false;
  int start2 = 0;
  int end2 = 0;
  line_centers[0] = 0;
  line_centers[1] = 0;
  for (int i=0; i<128; i++) {
    // first peak
    if (cutoff_data[i] == 1 && !started1) {
      started1 = true;
      start1 = i;
    }
    if (cutoff_data[i] == 0 && started1) {
      end1 = i;
      if (end1 - start1 <= MAX_WIDTH_LIMIT && end1 - start1 >= MIN_WIDTH_LIMIT) {
        line_centers[0] = (start1+end1)/2;
        ended1 = true;
      }
    }
    
    // second peak
    if (cutoff_data[i] == 1 && !started2 && ended1) {
      started2 = true;
      start2 = i;
    }
    if (cutoff_data[i] == 0 && started2) {
      end2 = i;
      if (end2 - start2 <= MAX_WIDTH_LIMIT && end2 - start2 >= MIN_WIDTH_LIMIT) {
        line_centers[1] = (start2+end2)/2;
        ended2 = true;
      }
    }
  }
}


/* Author: Hohyun Song, Cheng Hao Yuan
 * reads all ultrasonic sensors, converts to meters, and store in global array ultrasonic_dist.
 * returns nothing.
 */
void read_ultrasonics() {
  int sumF = 0, sumL = 0, sumR = 0;
  for (unsigned int i = 0; i < 10 ; i++) {
    sumF += analogRead(ULTRASONIC_F) / 2;
    sumL += analogRead(ULTRASONIC_L) / 2;
    sumR += analogRead(ULTRASONIC_R) / 2;
  }
  ultrasonic_dist[0] = (float)sumF / 10.0 * 0.0254;
  ultrasonic_dist[1] = (float)sumL / 10.0 * 0.0254;
  ultrasonic_dist[2] = (float)sumR / 10.0 * 0.0254;
}


/* Author: Hohyun Song, Cheng Hao Yuan
 * takes lane positions and ultrasonic clearance distances and logically decide which lane position to use.
 * sets the velocity_ref and lane_ref (globals).
 * this is the logic for obstacle avoidance and lane selection (and stopping if can't merge).
 * returns nothing.
 */


float start_merge_pos = 0.0;

void path_control_by_ultrasonics() {
  if (ultrasonic_dist[0] < FRONT_SPACE) { // if front is blocked
    if (ultrasonic_dist[1] > SIDE_SPACE) { // if left side is open
//      Serial.println("gon merge left11111111111111111111111111");
      merge_to_left();
    } else if (ultrasonic_dist[2] > SIDE_SPACE) { // if right side is open
//      Serial.println("gon merge right2222222222222222222222222");
      merge_to_right();
    } else { // all blocked, stop
      Serial.println("Emergency stop");
      motor_brake();
    }
    start_merge_pos = currPos;
  }
  Serial.print("currPos:");
  Serial.println(currPos);
  if (currPos - start_merge_pos > FRONT_SPACE) { // attempt merge back
    if ((merge_state == 1 && ultrasonic_dist[2] > SIDE_SPACE) || (merge_state == 2 && ultrasonic_dist[1] > SIDE_SPACE)){
//      Serial.println("gon merge mid00000000000000000000000000000");
      merge_to_middle();
    }
  }  
}



/* Author: Cheng Hao Yuan
 * uses lane_ref and lane_centers to determine lateral error.
 * produce steering angle (0.0-1.0) to reduce lateral error.
 */
float e_lat_prev = 0;
float steering_PID() {
  if (number_of_lines == 2) {
    lat_pos = (line_centers[0] + line_centers[1])/2;
  } else if (number_of_lines == 1) {
    lat_pos = line_centers[0];
  }
  float e_lat = (lane_ref - lat_pos)/128.0;
  float e_lat_d = (e_lat - e_lat_prev) / LOOP_PERIOD;
  float steering_signal = STEERING_KP * e_lat + STEERING_KD * e_lat_d;
  e_lat_prev = e_lat;
  return steering_signal;
}







