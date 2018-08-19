//
// Terms of use
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


//
//  SkaterBot
//

#include "MPU.h"
#include "PID.h"

// Pins
#define batteryLowIndicator 13
#define leftPulse           2
#define leftDirection       3
#define rightPulse          4
#define rightDirection      5

// PID Paramaters
#define pidP      15
#define pidI      1.5
#define pidD      2                                                                                                                                                                        
#define pidDB     5
#define pidOPMin  -400
#define pidOPMax  400

// Dynamic Paramaters
float turnSpeed = 30;
float maxSpeed = 150;

// State Engine
int robotState;
#define Starting  0
#define Balancing 1

// Battery Monitoring
int   batteryVoltage;
bool  lowBattery;

// Bluetooth Comms
byte  receivedByte;
int   receivedCounter;

// Loop Timing
unsigned long loopTimer;
unsigned long reportCounter;

// Control Variables
float angle;
float outputLeft;
float outputRight;

int leftMotor;
int throttleLeftMotor;
int throttleCounterLeftMotor;
int throttleLeftMotorMemory;

int rightMotor;
int throttleRightMotor;
int throttleCounterRightMotor;
int throttleRightMotorMemory;

// Class Instances
MPU mpu(0x68);          // MPU-6050 I2C address (0x68 or 0x69)
PID pid;

//******************
// SETUP
//******************

void setup() {

  Serial.begin(9600);           //Start the serial port at 9600 bps

    pinMode(batteryLowIndicator, OUTPUT);
    pinMode(leftPulse, OUTPUT);
    pinMode(leftDirection, OUTPUT);
    pinMode(rightPulse, OUTPUT);
    pinMode(rightDirection, OUTPUT);
  
  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  // Setup PID
  pid.reset();
  pid.setTuningParameters(pidP, pidI, pidD);
  pid.setDeadband(pidDB);
  pid.setOutputLimited(pidOPMin, pidOPMax);

  // Initialise the MPU
  mpu.initialise();

  // get calibration values
  mpu.getGyroCalibrationValues();
 
  robotState = Starting;
  loopTimer = micros() + 4000;
}


//******************
// LOOP
//******************

void loop() {

  if (Serial.available()) {
    receivedByte = Serial.read();
    receivedCounter = 0;
  }

  if (receivedCounter <= 25) {
    receivedCounter++;
  }
  else {
    receivedByte = 0x00;
  }
  
  batteryVoltage = map(analogRead(0),0,1023,0,1250);

  if (batteryVoltage < 1070 && batteryVoltage > 1050) {
    Serial.print("Battery Warning");
  }
  
  if (batteryVoltage < 1050 && batteryVoltage > 800) {
    digitalWrite(batteryLowIndicator, HIGH);
    lowBattery = true;  
    Serial.print("Battery Low");
  }
 
  // State Machine
  switch (robotState) {
    case Starting:
    {
      angle = mpu.getAccelAngle();
      if (angle > -0.5 && angle < 0.5) {
        mpu.setTiltAngle(angle);
        robotState = Balancing;
        
      }
      break;
    }
    case Balancing:
    {
      angle = mpu.getTiltAngle();
      if (angle > 30 || angle < -30 || lowBattery) {
        robotState = Starting;
        pid.reset();
      }
      else {
        // Process PID
        pid.processVariable = angle;
        pid.calculate();
      }
      break;
    }
  }

  // Calculate control signals
  outputLeft = pid.output;
  outputRight = pid.output;

  // Turn Left
  if (receivedByte & 0b00000010) {
    outputLeft += turnSpeed;
    outputRight -= turnSpeed;
  }

  // Turn Right
  if (receivedByte & 0b00000001) {
    outputLeft -= turnSpeed;
    outputRight += turnSpeed;
  }

  // Forward
  if (receivedByte & 0b00000100) {
    if (pid.setpoint > -2.5) {
      pid.setpoint -= 0.05;
    }
    if (pid.output > -maxSpeed) {
      pid.setpoint -= 0.005;
    }
  }
 
  // Reverse
  if (receivedByte & 0b00001000) {
    if (pid.setpoint < 2.5) {
      pid.setpoint += 0.05;
    }
    if (pid.output < maxSpeed) {
      pid.setpoint += 0.005;
    }
  }

  // Not forward or reverse
  if (!(receivedByte & B00001100)) {
    if (pid.setpoint > 0.5) {
      pid.setpoint -= 0.05; 
    }
    else if (pid.setpoint < -0.5) {
      pid.setpoint += 0.05;
    }
    else {
      pid.setpoint = 0;
    }
  }

  if (pid.setpoint == 0) {
    if (pid.output < 0) {
      pid.selfBalanceSetpoint += 0.0015;
    }
    if (pid.output > 0) {
      pid.selfBalanceSetpoint -= 0.0015;
    }
  }

  // Motor Calculations

  // Compensate for the non-linear behaviour of the stepper motors
  if (outputLeft > 0) {
    outputLeft = 405 - (1/(outputLeft + 9)) * 5500;
  }
  else if (outputLeft < 0) {
    outputLeft = -405 - (1/(outputLeft - 9)) * 5500;
  }
  
  if (outputRight > 0) {
    outputRight = 405 - (1/(outputRight + 9)) * 5500;
  }
  else if (outputRight < 0) {
    outputRight = -405 - (1/(outputRight - 9)) * 5500;
  }

  // Calculate the needed pulse time for the left and right stepper motor controllers
  if (outputLeft > 0) {
    leftMotor = 400 - outputLeft;
  }
  else if (outputLeft < 0) {
    leftMotor = -400 - outputLeft;
  }
  else {
    leftMotor = 0;
  }

  if (outputRight > 0) {
    rightMotor = 400 - outputRight;
  }
  else if (outputRight < 0) {
    rightMotor = -400 - outputRight;
  }
  else {
    rightMotor = 0;
  }

  // Copy for interrupt to use
  throttleLeftMotor = leftMotor;
  throttleRightMotor = rightMotor;

  if (reportCounter > 50) {
    reportCounter = 0;
  
//    Serial.print(angle);
//    Serial.print(",");
//    Serial.print(pid.selfBalanceSetpoint);
//    Serial.print(",");
//    Serial.print(pid.output);
//    Serial.print(",");
//    Serial.print(throttleLeftMotor);
//    Serial.print(",");
//    Serial.print(throttleRightMotor);
//    Serial.println("");
  }
 
  // Delay 4 milliseconds
  while(loopTimer > micros());
  loopTimer += 4000;
  reportCounter ++;
}

//******************
// INTERUPT HANDLERS
//******************

ISR(TIMER2_COMPA_vect) {
  
  // Left motor pulse calculations
  throttleCounterLeftMotor ++;                                //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttleCounterLeftMotor > throttleLeftMotorMemory) {   //If the number of loops is larger then the throttle_left_motor_memory variable
    throttleCounterLeftMotor = 0;                             //Reset the throttle_counter_left_motor variable
    throttleLeftMotorMemory = throttleLeftMotor;              //Load the next throttle_left_motor variable
    if (throttleLeftMotorMemory < 0) {                        //If the throttle_left_motor_memory is negative
      PORTD &= 0b11110111;                                    //Set output 3 low to reverse the direction of the stepper controller
      throttleLeftMotorMemory *= -1;                          //Invert the throttle_left_motor_memory variable
    }
    else {
      PORTD |= 0b00001000;                                    // Set output 3 high for a forward direction of the stepper motor
    }
  }
  else if (throttleCounterLeftMotor == 1) {
    PORTD |= 0b00000100;                                      //Set output 2 high to create a pulse for the stepper controller
  }
  else if (throttleCounterLeftMotor == 2) {
    PORTD &= 0b11111011;                                      //Set output 2 low because the pulse only has to last for 20us 
  }

  // Right motor pulse calculations
  throttleCounterRightMotor ++;                               //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if (throttleCounterRightMotor > throttleRightMotorMemory) { //If the number of loops is larger then the throttle_right_motor_memory variable
    throttleCounterRightMotor = 0;                            //Reset the throttle_counter_right_motor variable
    throttleRightMotorMemory = throttleRightMotor;            //Load the next throttle_right_motor variable
    if (throttleRightMotorMemory < 0) {                       //If the throttle_right_motor_memory is negative
      PORTD |= 0b00100000;                                    //Set output 5 low to reverse the direction of the stepper controller
      throttleRightMotorMemory *= -1;                         //Invert the throttle_right_motor_memory variable
    }
    else {
      PORTD &= 0b11011111;                                    //Set output 5 high for a forward direction of the stepper motor
    }
  }
  else if (throttleCounterRightMotor == 1) {
    PORTD |= 0b00010000;                                      //Set output 4 high to create a pulse for the stepper controller
  }
  else if (throttleCounterRightMotor == 2) {
    PORTD &= 0b11101111;                                      //Set output 4 low because the pulse only has to last for 20us
  }
}

