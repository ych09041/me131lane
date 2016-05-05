// Arrays to save our results in

int camera_data[128];

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// Setup the serial port and pin 2
void setup() {
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  pinMode(9, OUTPUT);
  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library

  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_16;    // set our own prescaler to 64 

}


void loop() {
  
////    PORTB |= B00000010; // SI high
////    PORTD |= B00010000; // CLK high
////    PORTB ^= B00000010; // SI low
////    PORTD ^= B00010000; // CLK low
//    
  unsigned long start_time_us = micros();
clear_camera();
//  for (unsigned int i=0; i<128; i++) {
//    PORTB |= B00010000; // SI high
//    PORTD |= B00010000; // CLK high
//    PORTB ^= B00010000; // SI low
//    PORTD ^= B00010000; // CLK low
////    digitalWrite(4, HIGH);  CLK is 4
////    digitalWrite(9, HIGH);  SI is 9
////    digitalWrite(4, LOW);
////    digitalWrite(9, LOW);
//    camera_data[i] = analogRead(4);
//  }
  Serial.println(micros()-start_time_us);
////  
////
//  delay(1000);
}




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
