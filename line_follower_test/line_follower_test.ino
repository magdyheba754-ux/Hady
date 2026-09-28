#include "BluetoothSerial.h"
#define INR1 4
#define INR2 16
#define INL1 17
#define INL2 18
#define ENABLERIGHT 19
#define ENABLELEFT 23
#define SENSOR_LEFT 39
#define SENSOR_RIGHT 35
#define SENSOR_MIDDLE 34
#define SENSOR_FARLEFT 36
#define SENSOR_FAR_RIGHT 32
 BluetoothSerial BT;


int lineThreshold = 500;


int low_speed = 100;     
int high_speed = 150;   

float Kp = 25.0;
float Ki = 0.05;
float Kd = 30.0;


int speed = 200;

bool autonomous = false, manual = true;



const unsigned long STARTUP_GRACE_MS = 2000;
bool bootFlushDone = false;
unsigned long bootStartTime = 0;


#define USE_BT_STATE_PIN false
#define BT_STATE_PIN 8

// PID state
float lastError = 0;
float integral = 0;
int lastDirection = 0; 

unsigned long lastPrintTime = 0;


void forward(int speed);
void left(int speed);
void right(int speed);
void backward(int speed);
void stop();
void forwardleft(int speed);
void forwardright(int speed);
void backwardleft(int speed);
void backwardright(int speed);


void driveMotors(int leftSpeed, int rightSpeed);

void setup() {
  Serial.begin(115200);
  BT.begin("hady");
  pinMode(INR1, OUTPUT);
  pinMode(INR2, OUTPUT);
  pinMode(INL1, OUTPUT);
  pinMode(INL2, OUTPUT);
  pinMode(ENABLERIGHT, OUTPUT);
  pinMode(ENABLELEFT, OUTPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_MIDDLE, INPUT);
  pinMode(SENSOR_FARLEFT, INPUT);
  pinMode(SENSOR_FAR_RIGHT, INPUT);
 analogReadResolution(10);
  #if USE_BT_STATE_PIN
  pinMode(BT_STATE_PIN, INPUT);
  #endif

 
  autonomous = false;
  manual = true;
  stop();

  bootStartTime = millis();
}

void loop() {
  
  if (!bootFlushDone) {
    while (Serial.available() > 0) Serial.read(); // discard whatever came in
    if (millis() - bootStartTime >= STARTUP_GRACE_MS) {
      bootFlushDone = true;
      while (Serial.available() > 0) Serial.read(); // final flush right before trusting input
    }
  }

  
  bool serialTrusted = bootFlushDone;
  #if USE_BT_STATE_PIN
  serialTrusted = serialTrusted && (digitalRead(BT_STATE_PIN) == HIGH);
  if (digitalRead(BT_STATE_PIN) == LOW) {
    while (Serial.available() > 0) Serial.read(); 
  }
  #endif

  if (serialTrusted && BT.available() > 0) {
    char command = BT.read();
     Serial.println(command);
    if (command == 'W') {
      autonomous = true;
      manual = false;
      integral = 0;
      lastError = 0;
      stop();
    } else if (command == 'w') {
      autonomous = false;
      manual = true;
      stop();
    }

    if (manual) {
      if (command == 'F') forward(speed);
      else if (command == 'L') left(speed);
      else if (command == 'R') right(speed);
      else if (command == 'B') backward(speed);
      else if (command == 'S') stop();
      else if (command == 'a') forwardleft(speed);
      else if (command == 'b') forwardright(speed);
      else if (command == 'c') backwardleft(speed);
      else if (command == 'd') backwardright(speed);
      else if (command == '+') {
        speed += 10;
        if (speed > 255) speed = 255;
      } else if (command == '-') {
        speed -= 10;
        if (speed < 0) speed = 0;
      }
    }
  }

  if (autonomous) {
    int farleft  = analogRead(SENSOR_FARLEFT);
    int leftV    = analogRead(SENSOR_LEFT);
    int middle   = analogRead(SENSOR_MIDDLE);
    int rightV   = analogRead(SENSOR_RIGHT);
    int farright = analogRead(SENSOR_FAR_RIGHT);

    bool sFarLeft  = farleft  > lineThreshold;
    bool sLeft     = leftV    > lineThreshold;
    bool sMiddle   = middle   > lineThreshold;
    bool sRight    = rightV   > lineThreshold;
    bool sFarRight = farright > lineThreshold;

    int activeCount = sFarLeft + sLeft + sMiddle + sRight + sFarRight;

    if (millis() - lastPrintTime >= 250) {
      lastPrintTime = millis();
      Serial.print("FL:"); Serial.print(farleft);
      Serial.print(" L:"); Serial.print(leftV);
      Serial.print(" M:"); Serial.print(middle);
      Serial.print(" R:"); Serial.print(rightV);
      Serial.print(" FR:"); Serial.print(farright);
      Serial.print(" | err:"); Serial.println(lastError);
    }

    
    if (activeCount == 5) {
      stop();
      return;
    }

    if (activeCount > 0) {
      
      float weightedSum = (sFarLeft ? -2.0 : 0) + (sLeft ? -1.0 : 0) +
                           (sMiddle ?  0.0 : 0) + (sRight ?  1.0 : 0) +
                           (sFarRight ? 2.0 : 0);
      float error = weightedSum / activeCount;

      if (error != 0) lastDirection = (error > 0) ? 1 : -1;

      
      if (error <= -2.0) {
        driveMotors(-high_speed, high_speed);
        integral = 0;
        lastError = error;
        return;
      }
      if (error >= 2.0) {
        driveMotors(high_speed, -high_speed);
        integral = 0;
        lastError = error;
        return;
      }

      // ---- PID ----
      integral += error;
      integral = constrain(integral, -50, 50); 
      float derivative = error - lastError;
      float correction = Kp * error + Ki * integral + Kd * derivative;
      lastError = error;

      
      int baseSpeed = high_speed - (int)(abs(error) * (high_speed - low_speed) / 2.0);
      baseSpeed = constrain(baseSpeed, low_speed, high_speed);

      int leftSpeed  = constrain(baseSpeed + (int)correction, -255, 255);
      int rightSpeed = constrain(baseSpeed - (int)correction, -255, 255);

      driveMotors(leftSpeed, rightSpeed);

    } else {
      
      integral = 0;
      if (lastDirection == 1) {
        driveMotors(high_speed, -high_speed);
      } else if (lastDirection == -1) {
        driveMotors(-high_speed, high_speed);
      } else {
        stop();
      }
    }
  }
}



void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(INL1, 1);
    digitalWrite(INL2, 0);
  } else {
    digitalWrite(INL1, 0);
    digitalWrite(INL2, 1);
  }
  if (rightSpeed >= 0) {
    digitalWrite(INR1, 1);
    digitalWrite(INR2, 0);
  } else {
    digitalWrite(INR1, 0);
    digitalWrite(INR2, 1);
  }
  analogWrite(ENABLELEFT, constrain(abs(leftSpeed), 0, 255));
  analogWrite(ENABLERIGHT, constrain(abs(rightSpeed), 0, 255));
}

// ---- Manual-mode helpers (unchanged from the original) ----

void forward(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void backward(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void right(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void left(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void stop() {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, 0);
  analogWrite(ENABLELEFT, 0);
}

void forwardright(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed / 4);
  analogWrite(ENABLELEFT, speed);
}

void forwardleft(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed / 4);
}

void backwardright(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed / 4);
  analogWrite(ENABLELEFT, speed);
}

void backwardleft(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed / 4);
}
