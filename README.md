# 3-DOF-Robotic-Arm
The 3-Degree-of-Freedom (DOF) Autonomous Robotic Arm is a compact and efficient robotic system designed for precision pick-and-place operations. The system integrates MATLAB for trajectory planning, while Arduino IDE is used for real-time control and programming of the servo motors.
Hardware Components
3-DOF Robotic Arm Structure (custom design or pre-built)
Servo Motors (for joint actuation)
Arduino Board (e.g., Arduino Uno or Mega for microcontroller control)
12-bit 16-channel Servo Motor Controller (for better PWM control)
Power Supply Unit
Gripper Mechanism (end-effector for object manipulation)
Software and Algorithms
1. MATLAB for Robotic Modeling and Control
Denavit-Hartenberg (DH) Parameter Table: Defines the arm's kinematic structure.
Forward Kinematics (FK): Determines the position and orientation of the end effector.
Inverse Kinematics (IK): Calculates joint angles for a given end-effector position.
Trajectory Planning: Generates smooth motion paths using cubic or quintic polynomial interpolation.
Visualization and Simulation: Uses Peter Corke’s Robotics Toolbox to simulate movements.
2. Arduino IDE for Real-time Control
Servo Motor Control: Uses PWM signals to control motor positions.
Serial Communication: Receives position commands from MATLAB and executes movements.
Autonomous Motion Execution: Implements pre-defined sequences or reacts to sensor inputs (if applicable).
