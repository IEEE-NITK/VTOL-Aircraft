#define servoPin 7 
int onTime =500;
int pos;
void setup() {
  pinMode(servoPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(onTime); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000-onTime); // 20ms - duration of the pusle
    Serial.println(onTime);
    if(onTime>2450)
      onTime = 500;
    else 
      onTime++; 
}
// 0 deg - 500us, 90deg - 1475us, 180deg - 2450us
