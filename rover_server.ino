#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>

const int SLAVE_ADDRESS = 0x04;

// Request Protocol

//
// Function     Flags
// xxxx         xxxx
// 
// 2^4 possible functions and 2^4 possible flags (parameters)
//
// Forward       Speed
// 0001          0000  SLOW
// 0001          0001  MEDIUM
// 0001          0010  FAST
// 0001          0011  FULL
// 0001          0100  STOP
//
// Backward      Speed
// 0010          0000  SLOW
// 0010          0001  MEDIUM
// 0010          0010  FAST
// 0010          0011  FULL
// 0010          0100  STOP
//
// LeftForward   Speed
// 0011          0000  SLOW
// 0011          0001  MEDIUM
// 0011          0010  FAST
// 0011          0011  FULL
// 0011          0100  STOP
//
// LeftBackward  Speed
// 0100          0000  SLOW
// 0100          0001  MEDIUM
// 0100          0010  FAST
// 0100          0011  FULL
// 0100          0100  STOP
//
// RightForward  Speed
// 0101          0000  SLOW
// 0101          0001  MEDIUM
// 0101          0010  FAST
// 0101          0011  FULL
// 0101          0100  STOP
//
// RightBackward Speed
// 0110          0000  SLOW
// 0110          0001  MEDIUM
// 0110          0010  FAST
// 0110          0011  FULL
// 0110          0100  STOP
//
// LeftEncoder   Action
// 0111          0000  Reset
// 0111          0001  Read
//
// RightEncoder  Action
// 1000          0000  Reset
// 1000          0001  Read
//
// Encoders      Action
// 1001          0000  Reset
// 1001          0001  Read

const byte PROTO_FUNC_FORWARD        = 1;
const byte PROTO_FUNC_BACKWARD       = 2;
const byte PROTO_FUNC_LEFT_FORWARD   = 3;
const byte PROTO_FUNC_LEFT_BACKWARD  = 4;
const byte PROTO_FUNC_RIGHT_FORWARD  = 5;
const byte PROTO_FUNC_RIGHT_BACKWARD = 6;
const byte PROTO_FUNC_LEFT_ENCODER   = 7;
const byte PROTO_FUNC_RIGHT_ENCODER  = 8;
const byte PROTO_FUNC_ENCODERS       = 9;

const byte PROTO_FLAG_SLOW   = 0;
const byte PROTO_FLAG_MEDIUM = 1;
const byte PROTO_FLAG_FAST   = 2;
const byte PROTO_FLAG_FULL   = 3;
const byte PROTO_FLAG_STOP   = 4;

const byte PROTO_FLAG_RESET  = 0;
const byte PROTO_FLAG_READ   = 1;

// rover / arduino constants
const int PWM1   = 3;
const int PWM2   = 11;
const int MOTOR1 = 12;
const int MOTOR2 = 13;
const int BRAKE1 = 9;
const int BRAKE2 = 8;
// direction constants
const int FORWARD  = 1;
const int BACKWARD = 2;
// speed constants
const int SLOW   = 65;
const int MEDIUM = 129;
const int FAST   = 193;
const int FULL   = 255;

Encoder leftEncoder(2, 4);
Encoder rightEncoder(18, 19);

byte request;

struct Ticks {
  long left;
  long right;
};

Ticks ticks;

void setDirection(int motor, int motorDirection) {  
  // set the direction on the motor
  if (motorDirection == FORWARD)
    digitalWrite(motor, HIGH);
  else if (motorDirection == BACKWARD)
    digitalWrite(motor, LOW);
}
  
void move(int motor, int speed) {
  if (speed == PROTO_FLAG_STOP) {
    if (motor == MOTOR1) {
      digitalWrite(PWM1, LOW);
      digitalWrite(BRAKE1, HIGH);
    } else if (motor == MOTOR2) {
      digitalWrite(PWM2, LOW);
      digitalWrite(BRAKE2, HIGH);
    }
 
    return;
  }
 
  if (motor == MOTOR1) {
    digitalWrite(PWM1, HIGH);
    digitalWrite(BRAKE1, LOW);
    switch (speed) {
    case PROTO_FLAG_SLOW:
      analogWrite(PWM1, SLOW);
      break;
    case PROTO_FLAG_MEDIUM:
      analogWrite(PWM1, MEDIUM);
      break;
    case PROTO_FLAG_FAST:
      analogWrite(PWM1, FAST);
      break;
    case PROTO_FLAG_FULL: 
      analogWrite(PWM1, FULL);
    break;
    }
  } else if (motor == MOTOR2) {
    digitalWrite(PWM2, HIGH);
    digitalWrite(BRAKE2, LOW);
    switch (speed) {
    case PROTO_FLAG_SLOW:
      analogWrite(PWM2, SLOW);
      break;
    case PROTO_FLAG_MEDIUM:
      analogWrite(PWM2, MEDIUM);
      break;
    case PROTO_FLAG_FAST:
      analogWrite(PWM2, FAST);
      break;
    case PROTO_FLAG_FULL: 
      analogWrite(PWM2, FULL);
      break;
    }
  }
}

void setup() {
  // setup channel A for output
  pinMode(PWM1, OUTPUT);
  pinMode(MOTOR1, OUTPUT); // motor pin
  pinMode(BRAKE1, OUTPUT); // brake pin

  // setup channel B for output
  pinMode(PWM2, OUTPUT);
  pinMode(MOTOR2, OUTPUT); // motor pin
  pinMode(BRAKE2, OUTPUT); // brake pin
 
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
 
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

void loop() {
  ticks.left = leftEncoder.read();
  ticks.right = rightEncoder.read();

  //delay(10);
}

void receiveData(int byteCount) {
  byte speed;
  
  while (Wire.available()) {
    request = Wire.read();
    
    switch ((request >> 4) & B00001111) {
    case PROTO_FUNC_FORWARD:
      setDirection(MOTOR1, FORWARD);
      setDirection(MOTOR2, FORWARD);
      speed = (request & B00001111);
      move(MOTOR1, speed);
      move(MOTOR2, speed);
      break;
    case PROTO_FUNC_BACKWARD:
      setDirection(MOTOR1, BACKWARD);
      setDirection(MOTOR2, BACKWARD);
      speed = (request & B00001111);
      move(MOTOR1, speed);
      move(MOTOR2, speed);
      break;
    case PROTO_FUNC_LEFT_FORWARD:
      setDirection(MOTOR1, FORWARD);
      speed = (request & B00001111);
      move(MOTOR1, speed);
      break;
    case PROTO_FUNC_LEFT_BACKWARD:
      setDirection(MOTOR1, BACKWARD);
      speed = (request & B00001111);
      move(MOTOR1, speed);
      break;
    case PROTO_FUNC_RIGHT_FORWARD:
      setDirection(MOTOR2, FORWARD);
      speed = (request & B00001111);
      move(MOTOR2, speed);
      break;
    case PROTO_FUNC_RIGHT_BACKWARD:
      setDirection(MOTOR2, BACKWARD);
      speed = (request & B00001111);
      move(MOTOR2, speed);
      break;
    case PROTO_FUNC_LEFT_ENCODER:
      switch (request & B00001111) {
      case PROTO_FLAG_RESET:
        leftEncoder.write(0);
        break;
      }
      break;
    case PROTO_FUNC_RIGHT_ENCODER:
      switch (request & B00001111) {
      case PROTO_FLAG_RESET:
        rightEncoder.write(0);
        break;
      }
      break;
    case PROTO_FUNC_ENCODERS:
      switch (request & B00001111) {
      case PROTO_FLAG_RESET:
        leftEncoder.write(0);
        rightEncoder.write(0);
        break;
      }
      break;
    }
  }
}

void sendData() {
  switch ((request >> 4) & B00001111) {
    case PROTO_FUNC_LEFT_ENCODER:
      switch (request & B00001111) {
      case PROTO_FLAG_READ:
        Wire.write((byte*) &ticks.left, sizeof(long));
        break;
      }
      break;
    case PROTO_FUNC_RIGHT_ENCODER:
      switch (request & B00001111) {
      case PROTO_FLAG_READ:
        Wire.write((byte*) &ticks.right, sizeof(long));
        break;
      }
      break;
    case PROTO_FUNC_ENCODERS:
      switch (request & B00001111) {
      case PROTO_FLAG_READ:
        Wire.write((byte*) &ticks, sizeof(Ticks));
        break;
      }
      break;
    }
}

