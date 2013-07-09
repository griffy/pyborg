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

// rover / arduino pin constants
const byte PWM_A   = 3;
const byte PWM_B   = 11;
const byte MOTOR_A = 12;
const byte MOTOR_B = 13;
const byte BRAKE_A = 9;
const byte BRAKE_B = 8;

// speed constants
const byte SLOW   = 85;
const byte MEDIUM = 142;
const byte FAST   = 199;
const byte FULL   = 255;

// direction constants
const byte FORWARD  = 0;
const byte BACKWARD = 1;

// wheel constants
const byte WHEEL_LEFT  = 0;
const byte WHEEL_RIGHT = 1;

const byte PWM_INDEX   = 0;
const byte MOTOR_INDEX = 1;
const byte BRAKE_INDEX = 2;

// create arrays for O(1) lookup
const byte WHEELS[] = {
  { PWM_A, MOTOR_A, BRAKE_A },
  { PWM_B, MOTOR_B, BRAKE_B }
};

const byte DIRECTIONS[] = { HIGH, LOW };

const byte SPEEDS[] = { SLOW, MEDIUM, FAST, FULL };

struct Ticks {
  long left;
  long right;
};

Encoder leftEncoder(2, 4);
Encoder rightEncoder(18, 19);

byte request;

Ticks ticks;

void setupWheels() {
  // prepare the associated wheel pins for output
  for (byte i = 0; i < sizeof(WHEELS) / 3; i++) {
    pinMode(WHEELS[i][PWM_INDEX], OUTPUT);
    pinMode(WHEELS[i][MOTOR_INDEX], OUTPUT);
    pinMode(WHEELS[i][BRAKE_INDEX], OUTPUT);
  }
}

void setDirection(byte wheel, byte direction) {
  digitalWrite(WHEELS[wheel][MOTOR_INDEX], DIRECTIONS[direction]);
}
  
void move(byte wheel, byte speed) {
  switch (speed) {
  case PROTO_FLAG_STOP:
    // engage the brake
    digitalWrite(WHEELS[wheel][BRAKE_INDEX], HIGH);
    break;
  default:
    // disengage the brake
    digitalWrite(WHEELS[wheel][BRAKE_INDEX], LOW);
    // set the speed
    analogWrite(WHEELS[wheel][PWM_INDEX], SPEEDS[speed]);
  }
}

void receiveData(int byteCount) {
  byte speed;
  
  while (Wire.available()) {
    request = Wire.read();
    
    switch ((request >> 4) & B00001111) {
    case PROTO_FUNC_FORWARD:
      setDirection(WHEEL_LEFT, FORWARD);
      setDirection(WHEEL_RIGHT, FORWARD);
      speed = (request & B00001111);
      move(WHEEL_LEFT, speed);
      move(WHEEL_RIGHT, speed);
      break;
    case PROTO_FUNC_BACKWARD:
      setDirection(WHEEL_LEFT, BACKWARD);
      setDirection(WHEEL_RIGHT, BACKWARD);
      speed = (request & B00001111);
      move(WHEEL_LEFT, speed);
      move(WHEEL_RIGHT, speed);
      break;
    case PROTO_FUNC_LEFT_FORWARD:
      setDirection(WHEEL_LEFT, FORWARD);
      speed = (request & B00001111);
      move(WHEEL_LEFT, speed);
      break;
    case PROTO_FUNC_LEFT_BACKWARD:
      setDirection(WHEEL_LEFT, BACKWARD);
      speed = (request & B00001111);
      move(WHEEL_LEFT, speed);
      break;
    case PROTO_FUNC_RIGHT_FORWARD:
      setDirection(WHEEL_RIGHT, FORWARD);
      speed = (request & B00001111);
      move(WHEEL_RIGHT, speed);
      break;
    case PROTO_FUNC_RIGHT_BACKWARD:
      setDirection(WHEEL_RIGHT, BACKWARD);
      speed = (request & B00001111);
      move(WHEEL_RIGHT, speed);
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

void setup() {
  setupWheels();
 
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
 
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

void loop() {
  ticks.left = leftEncoder.read();
  ticks.right = rightEncoder.read();
}
