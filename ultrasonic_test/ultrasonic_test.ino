const int anPin = 1;

//variables needed to store values
long anVolt, inches, cm;
int sum = 0; //Create sum variable so it can be averaged
int avgrange = 10; //Quantity of values to average (sample size)

void setup() {
  Serial.begin(115200);
}



void loop() {
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
}
