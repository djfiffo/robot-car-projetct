# PID Line Following Robot Car with Grid Mapping and Obstacle Handling

## Project Overview

This project is an autonomous robot car designed to follow a line using a PID control algorithm while navigating a grid-based environment. 

The robot is capable of:
- Line following using a 5-sensor array
- Tracking its position using a coordinate-based grid system
- Detecting obstacles using an ultrasonic sensor
- Updating blocked paths dynamically
- Performing object pushing tasks
- Making directional decisions based on target coordinates

The navigation logic is inspired by the concept of Breadth-First Search (BFS). However, instead of implementing a full BFS algorithm, this project uses a rule-based directional priority system that approximates BFS-style decision making in a simplified and efficient way suitable for embedded systems.

---

## Hardware Components

- Arduino (Microcontroller)
- 5 IR Line Sensors
- L298N Motor Driver
- DC Motors
- Ultrasonic Sensor (HC-SR04)
- Buzzer
- Differential Drive Robot Chassis
This allows:
- Smooth curve handling
- Reduced oscillation
- Faster correction response
- Stable reverse line following during push-back sequence

The PID controller dynamically adjusts motor speeds to maintain alignment with the line.

---

## Control System: PID Line Following

The robot uses a 5-sensor weighted error calculation system:

- Sensors are assigned weights: `[-2, -1, 0, 1, 2]`
- The position error is calculated based on the weighted average
- PID formula: output = Kp * error + Ki * integral + Kd * derivative

---

## Grid Mapping System

The robot operates on a 6x6 coordinate grid.

It maintains:
- Current position `(x, y)`
- Current facing direction (N, E, S, W)
- A 2D array marking blocked cells (black boxes)

Each intersection detection updates the robot’s coordinate using directional logic.

This enables the robot to:
- Navigate to specific target coordinates
- Avoid previously detected obstacles
- Recalculate direction at intersections

---

## Navigation Logic (BFS-Inspired Strategy)

Although BFS is not fully implemented, the decision-making system is inspired by BFS principles:

- Move toward target X coordinate first
- Then move toward target Y coordinate
- Avoid blocked cells stored in memory
- Avoid immediate backtracking unless necessary
- If trapped, perform a 180-degree turn

This results in:
- Deterministic path selection
- Reduced memory usage (important for Arduino)
- Fast runtime without recursion or queue structures

It is effectively a lightweight heuristic grid navigation system.

---

## Obstacle Detection

An ultrasonic sensor continuously scans forward distance.

If an object is detected:
- The cell ahead is marked as blocked
- The robot selects a new direction
- A buzzer sound confirms obstacle detection

This allows dynamic environment adaptation.

---

## Orange Box Push Mechanism

The robot includes an object pushing routine:

1. Detect object using ultrasonic sensor
2. Push object forward to next intersection
3. Nudge slightly to center object
4. Reverse using PID-based reverse line following
5. Return to stable grid alignment

This demonstrates:
- Forward and reverse PID control
- Coordinated sensor + motion integration
- Task-based autonomous behavior

---

## Key Features

- 5-Sensor PID Line Following
- Reverse PID Alignment
- Grid Coordinate Tracking
- Dynamic Obstacle Mapping
- BFS-Inspired Direction Selection
- Autonomous Object Pushing
- Embedded-System Optimized Logic

---

## Engineering Highlights

- Uses proportional-derivative balancing for stability
- Implements weighted sensor averaging
- Memory-efficient grid representation
- Deterministic decision system suitable for low-RAM microcontrollers
- Combines control theory + discrete navigation logic

---

## Educational Value

This project demonstrates integration of:

- Control Systems (PID)
- Embedded Programming
- Grid-based Navigation
- Sensor Fusion
- Autonomous Robotics Concepts
- Algorithm-inspired decision systems

---

## Future Improvements

- Full BFS or A* path planning implementation
- Dynamic path re-optimization
- PID auto-tuning
- SLAM-style environment mapping
- Multi-robot coordination

---
