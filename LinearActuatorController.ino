/**
 * Linear Actuator Controller with PID Control
 * 
 * This project implements a multi-actuator control system using Arduino
 * with PID controllers for precise position control of linear actuators.
 * 
 * Features:
 * - Multi-driver support (H-bridge motor drivers)
 * - PID control with feedforward
 * - Position feedback via potentiometers
 * - Real-time telemetry output
 * - Safety limits and error handling
 * 
 * Author: [Your Name]
 * Date: 2024
 * License: MIT
 */

#include <Arduino.h>

// Actuator states
enum ActuatorState {
  LIFT,     // Moving up/extending
  RETRACT,  // Moving down/retracting
  IDLE      // Stationary
};

// System configuration
#define CONTROL_PIN A7
#define MAX_POSITION 73.8    // Maximum actuator position in mm
#define MIN_POSITION 0.0     // Minimum actuator position in mm
#define POSITION_TOLERANCE 0.1  // Position tolerance in mm
#define DEFAULT_SETPOINT 30.0   // Default target position in mm

/**
 * Driver class for H-bridge motor drivers
 * Each driver can control two motors independently
 */
class MotorDriver {
public:
  int in1, in2, ena;  // First motor control pins
  int in3, in4, enb;  // Second motor control pins
  int id;             // Driver identifier
  
  MotorDriver(int in1, int in2, int ena, int in3, int in4, int enb, int id) {
    this->in1 = in1;
    this->in2 = in2;
    this->ena = ena;
    this->in3 = in3;
    this->in4 = in4;
    this->enb = enb;
    this->id = id;
    
    // Configure pins as outputs
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enb, OUTPUT);
    
    Serial.print("Driver ");
    Serial.print(id);
    Serial.println(" initialized");
  }
};

/**
 * Actuator class representing a single linear actuator
 * with position feedback and state management
 */
class LinearActuator {
public:
  int IN1, IN2, ENA;        // Motor control pins
  MotorDriver* driver;       // Associated motor driver
  int potentiometerPin;     // Position feedback pin
  ActuatorState state;      // Current actuator state
  int outputNumber;         // Output number on driver (1 or 2)
  
  LinearActuator(MotorDriver* driver, int potPin, int outNum) {
    if (driver == nullptr) {
      Serial.println("ERROR: Driver cannot be null");
      return;
    }
    
    this->driver = driver;
    this->potentiometerPin = potPin;
    
    // Validate output number
    if (outNum != 1 && outNum != 2) {
      Serial.println("ERROR: Invalid output number. Must be 1 or 2");
      return;
    }
    this->outputNumber = outNum;
    
    // Assign control pins based on output number
    this->IN1 = (outNum == 1) ? driver->in1 : driver->in3;
    this->IN2 = (outNum == 1) ? driver->in2 : driver->in4;
    this->ENA = (outNum == 1) ? driver->ena : driver->enb;
    
    // Determine initial state based on current position
    float currentPosition = readPosition();
    this->state = (currentPosition > 10.0) ? RETRACT : LIFT;
    
    Serial.print("Actuator setup complete - Pin: ");
    Serial.print(potPin);
    Serial.print(", Position: ");
    Serial.print(currentPosition);
    Serial.println(" mm");
  }
  
  /**
   * Read current position from potentiometer
   * @return Position in millimeters
   */
  float readPosition() {
    int sensorValue = analogRead(potentiometerPin);
    return map(sensorValue, 0, 1023, 0, 7380) / 100.0;
  }
};

/**
 * PID Controller with Feedforward
 * Implements proportional, integral, derivative, and feedforward control
 */
class PIDFController {
public:
  double kp, ki, kd, kf;    // Control gains
  double setpoint;          // Target value
  double error;             // Current error
  double prevError;         // Previous error for derivative
  double prevTime;          // Previous time for derivative
  double integralError;     // Accumulated error for integral
  double measuredValue;     // Current measured value
  double period;            // Time period between calculations
  
  PIDFController(double kp, double ki, double kd, double kf) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kf = kf;
    this->prevTime = 0;
    this->integralError = 0;
    this->prevError = 0;
    this->period = 0;
    this->measuredValue = 0;
  }
  
  PIDFController(double kp) {
    this->kp = kp;
    this->ki = 0;
    this->kd = 0;
    this->kf = 0;
    this->prevTime = 0;
    this->integralError = 0;
    this->prevError = 0;
    this->period = 0;
    this->measuredValue = 0;
  }
  
  /**
   * Set the target setpoint
   * @param setpoint Target value
   */
  void setSetpoint(double setpoint) {
    this->setpoint = setpoint;
  }
  
  /**
   * Update PID parameters
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param kf Feedforward gain
   */
  void setPIDF(double kp, double ki, double kd, double kf) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kf = kf;
  }
  
  /**
   * Calculate PID output
   * @param currentValue Current measured value
   * @param targetValue Target value
   * @return Control output
   */
  double calculate(double currentValue, double targetValue) {
    // Clamp target value to valid range
    if (targetValue > MAX_POSITION) targetValue = MAX_POSITION;
    if (targetValue < MIN_POSITION) targetValue = MIN_POSITION;
    
    // Calculate time period
    double currentTime = millis();
    if (this->prevTime == 0.0) {
      this->prevTime = currentTime;
    }
    this->period = currentTime - this->prevTime;
    this->prevTime = currentTime;
    
    // Update values
    this->measuredValue = currentValue;
    this->error = targetValue - currentValue;
    this->integralError += error;
    
    // Calculate PID components
    double proportional = this->kp * this->error;
    double integral = this->ki * this->integralError;
    
    // Calculate derivative with protection against division by zero
    double derivative = 0;
    if (this->period > 0) {
      derivative = this->kd * (this->error - this->prevError) / this->period;
    }
    
    double feedforward = this->kf * targetValue;
    
    // Update for next iteration
    this->prevError = this->error;
    
    return proportional + integral + derivative + feedforward;
  }
};

/**
 * Display telemetry information for an actuator
 */
void displayTelemetry(int driverId, int actuatorId, int sensorValue, 
                     float distance, bool upperLimitReached, bool lowerLimitReached, 
                     ActuatorState state, double power, double target) {
  Serial.print("Driver ");
  Serial.print(driverId);
  Serial.print(" | Actuator ");
  Serial.print(actuatorId);
  Serial.print(" | Sensor: ");
  Serial.print(sensorValue);
  Serial.print(" | Position: ");
  Serial.print(distance, 2);
  Serial.print(" mm | Limits: ");
  Serial.print(upperLimitReached ? "UP " : "   ");
  Serial.print(lowerLimitReached ? "DOWN " : "     ");
  Serial.print("| State: ");
  
  switch (state) {
    case LIFT:
      Serial.print("LIFT   ");
      break;
    case RETRACT:
      Serial.print("RETRACT");
      break;
    case IDLE:
      Serial.print("IDLE   ");
      break;
  }
  
  Serial.print(" | Power: ");
  Serial.print(power, 1);
  Serial.print(" | Target: ");
  Serial.print(target, 1);
  Serial.print(" | Error: ");
  Serial.print(target - distance, 2);
  Serial.println(" mm");
}

/**
 * Update actuator position using PID control
 * @param actuator Pointer to actuator object
 * @param pidf Pointer to PID controller
 * @param setPoint Target position
 */
void updateActuator(LinearActuator* actuator, PIDFController* pidf, double setPoint) {
  // Read current position
  float currentPosition = actuator->readPosition();
  int sensorValue = analogRead(actuator->potentiometerPin);
  
  // Calculate PID output
  double power = pidf->calculate(currentPosition, setPoint);
  power = constrain(power, -255, 255);
  
  // Check limits and target
  bool upperLimitReached = (abs(MAX_POSITION - currentPosition) <= POSITION_TOLERANCE);
  bool lowerLimitReached = (abs(MIN_POSITION - currentPosition) <= POSITION_TOLERANCE);
  bool targetReached = (abs(setPoint - currentPosition) <= POSITION_TOLERANCE);
  
  // Update state based on power output
  if (power > 0) {
    actuator->state = LIFT;
  } else if (power < 0) {
    actuator->state = RETRACT;
  } else {
    actuator->state = IDLE;
  }
  
  // Display telemetry
  displayTelemetry(actuator->driver->id, actuator->outputNumber, sensorValue, 
                  currentPosition, upperLimitReached, lowerLimitReached, 
                  actuator->state, power, setPoint);
  
  // Control motor based on state
  switch (actuator->state) {
    case LIFT:
      digitalWrite(actuator->IN1, LOW);
      digitalWrite(actuator->IN2, HIGH);
      analogWrite(actuator->ENA, abs(power));
      if (targetReached) {
        actuator->state = IDLE;
      }
      break;
      
    case RETRACT:
      digitalWrite(actuator->IN1, HIGH);
      digitalWrite(actuator->IN2, LOW);
      analogWrite(actuator->ENA, abs(power));
      if (targetReached) {
        actuator->state = IDLE;
      }
      break;
      
    case IDLE:
      analogWrite(actuator->ENA, 0);
      break;
  }
}

/**
 * Move actuator to zero position
 * @param actuator Pointer to actuator object
 */
void returnActuatorToZero(LinearActuator* actuator) {
  digitalWrite(actuator->IN1, HIGH);
  digitalWrite(actuator->IN2, LOW);
  analogWrite(actuator->ENA, 255);
}

/**
 * Simple position control without PID
 * @param actuator Pointer to actuator object
 * @param target Target position
 */
void moveToTarget(LinearActuator* actuator, double target) {
  float currentPosition = actuator->readPosition();
  
  if (abs(currentPosition - target) > POSITION_TOLERANCE) {
    if (target > currentPosition) {
      digitalWrite(actuator->IN1, LOW);
      digitalWrite(actuator->IN2, HIGH);
      analogWrite(actuator->ENA, 255);
    } else {
      digitalWrite(actuator->IN1, HIGH);
      digitalWrite(actuator->IN2, LOW);
      analogWrite(actuator->ENA, 255);
    }
  } else {
    analogWrite(actuator->ENA, 0);
  }
}

/**
 * Read control input from potentiometer
 * @return Setpoint value
 */
double readControlInput() {
  int controlValue = analogRead(CONTROL_PIN);
  return map(controlValue, 0, 1023, 0, 7380) / 100.0;
}

// Global objects
MotorDriver* driver1;
MotorDriver* driver2;
MotorDriver* driver3;
LinearActuator* actuator1;
LinearActuator* actuator2;
LinearActuator* actuator3;
LinearActuator* actuator4;
LinearActuator* actuator5;
LinearActuator* actuator6;
PIDFController* pidController;

double currentSetpoint = DEFAULT_SETPOINT;

void setup() {
  Serial.begin(9600);
  Serial.println("=== Linear Actuator Controller ===");
  Serial.println("Initializing system...");
  
  // Initialize motor drivers
  driver1 = new MotorDriver(43, 41, 2, -1, -1, -1, 1);
  driver2 = new MotorDriver(42, 40, 3, 37, 39, 6, 2);
  driver3 = new MotorDriver(38, 36, 4, -1, -1, -1, 3);
  
  // Initialize actuators
  actuator1 = new LinearActuator(driver1, A0, 1);
  actuator2 = new LinearActuator(driver1, A5, 2);
  actuator3 = new LinearActuator(driver2, A1, 1);
  actuator4 = new LinearActuator(driver2, A4, 2);
  actuator5 = new LinearActuator(driver3, A2, 1);
  actuator6 = new LinearActuator(driver3, A3, 2);
  
  // Initialize PID controller
  pidController = new PIDFController(100, 0, 0, 0);
  
  Serial.println("System initialization complete!");
  Serial.print("Default setpoint: ");
  Serial.print(DEFAULT_SETPOINT);
  Serial.println(" mm");
  Serial.println("Starting control loop...\n");
}

void loop() {
  // Read control input (uncomment to enable external control)
  // currentSetpoint = readControlInput();
  
  // Update active actuators
  updateActuator(actuator1, pidController, currentSetpoint);
  updateActuator(actuator3, pidController, currentSetpoint);
  updateActuator(actuator5, pidController, currentSetpoint);
  updateActuator(actuator4, pidController, currentSetpoint);
  
  // Small delay for stability
  delay(10);
}
