const int anPin = 1;

//variables needed to store values
long anVolt, inches, cm;
int sum = 0; //Create sum variable so it can be averaged
int avgrange = 10; //Quantity of values to average (sample size)

void setup() {
  Serial.begin(115200);
}

float ultrasonic_dist[3];

void loop() {
  read_ultrasonic();
  Serial.print("ultrasonic: ");
  Serial.print(ultrasonic_dist[0]);
  Serial.print('\t');
  Serial.print(ultrasonic_dist[1]);
  Serial.print('\t');
  Serial.print(ultrasonic_dist[2]);
  Serial.println();
  delay(200);


  /*
  //MaxSonar Analog reads are known to be very sensitive. See the Arduino forum for more information.
  //A simple fix is to average out a sample of n readings to get a more consistent reading.
  //Even with averaging I still find it to be less accurate than the PW method.
  //This loop gets 60 reads and averages them
  unsigned long start_time = micros();
  for (int i = 0; i < avgrange ; i++) {
    //Used to read in the analog voltage output that is being sent by the MaxSonar device.
    //Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
    //Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
    anVolt = analogRead(anPin) / 2;
    sum += anVolt;
  }

  inches = sum / avgrange;
  cm = inches * 2.54;
  Serial.println(micros() - start_time);
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  //reset sample total
  sum = 0;
  delay(500);
  */
}




void read_ultrasonic() {
  int sumF=0, sumL=0, sumR=0;
  for (unsigned int i = 0; i < 10 ; i++) {
    sumF += analogRead(1) / 2;
    sumL += analogRead(2) / 2;
    sumR += analogRead(3) / 2;
  }
  ultrasonic_dist[0] = (float)sumF / 10.0 * 0.0254;
  ultrasonic_dist[1] = (float)sumL / 10.0 * 0.0254;
  ultrasonic_dist[2] = (float)sumR / 10.0 * 0.0254;
}
