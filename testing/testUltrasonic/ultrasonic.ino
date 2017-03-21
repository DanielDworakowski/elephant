long duration1, cm1, inches1, duration2, cm2, inches2;
int trig1 = 3;
int echo1 = 4;
int trig2 = 6;
int echo2 = 7;

void setup() {
  Serial.begin (115200);

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
}
 
void loop()
{
  digitalWrite(trig1, LOW);
  delayMicroseconds(5);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  
  pinMode(echo1, INPUT);
  duration1 = pulseIn(echo1, HIGH);
 
  cm1 = (duration1/2) / 29.1;

  digitalWrite(trig2, LOW);
  delayMicroseconds(5);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  
  pinMode(echo2, INPUT);
  duration2 = pulseIn(echo2, HIGH);
 
  cm2 = (duration2/2) / 29.1;
  // inches = (duration/2) / 74; 
  
  // Serial.print(inches);
  // Serial.print(" ");
  Serial.print(cm1);
  Serial.print(" ");
  Serial.print(cm2);
  Serial.println();
  
  delay(250);
}
