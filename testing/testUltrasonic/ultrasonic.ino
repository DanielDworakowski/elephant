long duration, cm, inches;
int trig = 4;
int echo = 3;
 
void setup() {
  Serial.begin (9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}
 
void loop()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);
 
  cm = (duration/2) / 29.1;
  inches = (duration/2) / 74; 
  
  Serial.print(inches);
  Serial.print(" ");
  Serial.print(cm);
  Serial.println();
  
  delay(250);
}

