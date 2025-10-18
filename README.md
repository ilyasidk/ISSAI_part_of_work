# Linear Actuator Controller with PID Control

![Project Demo](demo.jfif)

## About the Project

This project represents a linear actuator control system using Arduino and PID controller. Development was carried out applying principles of tensegrity (tensile integrity) - creating stable structures through force balance.

## Development Timeline

### Stage 1: Initial Construction
![Initial Construction](constr_1.jfif)
- Creating basic robot structure
- Planning actuator placement
- Defining main system components

### Stage 2: Construction Without Motors
![Construction Without Motors](constr_without_motors.jfif)
- Testing mechanical components
- Checking component compatibility
- Debugging mechanical connections

### Stage 3: Working with Motors
![Working with Motors](working_with_motors.jfif)
- Connecting and testing motors
- Configuring motor drivers
- First actuator movement tests

### Stage 4: Robot Parts Assembly
![Robot Part 1](part_of_robot1.jfif)
![Robot Part 2](part_of_robot2.jfif)
![Robot Part 3](part_of_robot3.jfif)
- Step-by-step assembly of various robot parts
- Integrating actuators into overall structure
- Testing component interactions

### Stage 5: Final Demonstration
![Final Demonstration](76db0df1-c8bc-41dc-8df3-0bb8433a01b3.jfif)

**🎥 Watch the final demonstration video:** [https://www.youtube.com/watch?v=iIaVtzwRE7Q](https://www.youtube.com/watch?v=iIaVtzwRE7Q)

- Fully assembled system
- Testing all functions
- Demonstrating PID controller operation

## Applied Technologies

### Hardware Stack
- **Arduino** - microcontroller for control
- **H-bridge drivers** - motor direction and speed control
- **Linear actuators** - actuating mechanisms
- **Potentiometers** - position feedback sensors
- **Power system** - 12V for actuators, 5V for Arduino

### Software Stack
- **C++** - main programming language
- **Arduino IDE** - development environment
- **PID algorithm** - feedback control algorithm
- **Object-oriented programming** - modular architecture

## System Architecture

### Tensegrity Principles in Code

**Modularity and Force Balance:**
- Each component has clearly defined responsibility
- Interaction between modules is balanced
- System remains stable when individual components change

**Main Classes:**
```cpp
MotorDriver     // Motor driver control
LinearActuator  // Actuator representation with feedback
PIDFController  // PID controller with feedforward
```

### PID Algorithm

Implemented classic PID controller with additional feedforward component:

```
output = Kp × error + Ki × ∫error×dt + Kd × d(error)/dt + Kf × setpoint
```

**Implementation Features:**
- Protection against division by zero in derivative
- Output signal limitation (-255 to +255)
- Automatic target value constraints

## Development Process

### Stage 1: Requirements Analysis
- Determining number of actuators (6 units)
- Choosing positioning range (0-73.8 mm)
- Accuracy requirements (±0.1 mm)

### Stage 2: Architecture Design
- Dividing into logical modules
- Defining interfaces between components
- Planning data flows

### Stage 3: Implementation
- Creating base classes
- Implementing PID algorithm
- Hardware integration

### Stage 4: Testing and Optimization
- PID parameter tuning
- Testing under various loads
- Performance optimization

## Technical Solutions

### Actuator State Management
```cpp
enum ActuatorState {
  LIFT,     // Upward/extension movement
  RETRACT,  // Downward/retraction movement
  IDLE      // Stationary
};
```

### Telemetry System
Implemented detailed real-time monitoring system:
- Current position of each actuator
- Limit states
- Output power
- Positioning error

### System Safety
- Automatic power limitation
- Position limit checking
- Initialization error handling

## Results

### Achieved Characteristics
- **Positioning accuracy**: ±0.1 mm
- **Update frequency**: ~100 Hz
- **Response time**: <10 ms
- **Operation stability**: High

### Development Experience

**What worked well:**
- Modular architecture simplified development and testing
- PID controller provided precise positioning
- Telemetry system enabled effective system debugging

**Challenges and Solutions:**
- **Actuator synchronization** - solved through unified PID controller
- **Error handling** - added input data validation
- **Performance** - optimized sensor reading algorithms

## Application of Tensegrity Principles

In this project, tensegrity principles manifested in:

1. **Structural integrity** - each system component performs its role, supporting overall stability
2. **Force balance** - interaction between modules is balanced, no overload of individual components
3. **Adaptability** - system can easily adapt to changes (adding new actuators)
4. **Efficiency** - minimal resources for maximum functionality

This project became an excellent experience working with modern control technologies and principles of designing stable systems.

---

**Project Author**: TenSegrity Summer Research Program 2024 ISSAI  
**Date**: 2024  
**License**: MIT
