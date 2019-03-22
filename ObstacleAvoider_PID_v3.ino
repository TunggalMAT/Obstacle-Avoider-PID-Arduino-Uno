//------------------------- Variabel ----------------------------
double Kp = 60;
double Ki = 0;           // pu 17.39
double Kd = 0;

unsigned long lastTime;
double lastErr;
double sumErr;
double PID;
double pwm;
int    setPoint = 15;

//----------------------------- Pin ----------------------------
// HY-SRF05
    const int trigPin = 12;
    const int echoPin = 13;

// motor 1
    int enA = 10;
    int in1 = 9;
    int in2 = 8;

// motor 2
    int enB = 5;
    int in3 = 7;
    int in4 = 6;

//-----------------------read sensor----------------------
float readPosition(){
  delay(10);
  long duration, distance;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2)/29.1;

  if (distance > 100){
    distance = 100;
  }
  return distance;
  //Serial.println(distance);
}

//------------------------- Move Motor ------------------------
    void maju(){
      // turn on motor A
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, pwm);
      // turn on motor B
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, pwm);
    }
    
    void mundur(){
      // turn on motor A
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, pwm);
      // turn on motor B
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB, pwm);
    }

    void berhenti(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);  
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }

//---------------------------- Setup ----------------------------
    void setup() {
      Serial.begin(9600); 
      
      // motor
      pinMode(enA, OUTPUT);
      pinMode(enB, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
    
    }

//----------------------------- Loop ---------------------------- 
void loop() {
  
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  
  double error = setPoint - readPosition();
  sumErr = sumErr + (error * timeChange);
  double dErr = (error - lastErr) / timeChange;  

  PID = (Kp * error) + (Ki * sumErr) + (Kd * dErr);

  if (PID < -255){
    pwm = 255;
    maju();
  }
  else if(PID > 255){
    pwm = 255;
    mundur();                 
  }
  else {
    pwm = PID;
    if (pwm >= 0){
      mundur();
    }else{
      pwm = pwm *(-1);
      maju();
    }
  }
  
  //______________________________________________________________
  //Serial.print("Input        :");
  Serial.println(readPosition());
  
  //______________________________________________________________
  
  delay(10);
  lastErr = error;
  lastTime = now;
}
