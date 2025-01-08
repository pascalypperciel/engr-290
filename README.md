# ENGR 290 Hovercraft Project

## Overview

This project showcases the design, construction, and programming of an autonomous hovercraft for the ENGR 290 course at Concordia University. The hovercraft navigates through a maze using "Pure C" code—no Arduino libraries were used. The project emphasizes teamwork, problem-solving, and the application of engineering principles in a real-world context.

## Features

- **Pure C Implementation**: The code meets the "Pure C" requirement, compiling in AVR Studio without Arduino libraries, earning a 25% bonus.
- **Autonomous Navigation**: Incorporates a pathfinding algorithm using ultrasonic sensors and an IMU for precise movement and obstacle detection.
- **Iterative Design**: Developed through three design iterations, each improving speed, stability, and accuracy.
- **Real-time Control**: Features a PID controller to maintain balance and ensure accurate turns.
- **Minimalist Design**: Optimized to minimize components while achieving full functionality.

## Project Highlights

- **Competition Goals**:
  - Complete the maze autonomously.
  - Score based on time, distance, and component count.
  - Overcome obstacles (1mm to 3mm) on the track.

- **Components Used**:
  - Arduino Nano with Atmega328p chip
  - Two fans (lift and propulsion)
  - Two ultrasonic sensors (HC-SR04)
  - MPU-6050 IMU
  - HS-311 servo motor
  - Custom skirt and Styrofoam frame

## Challenges and Solutions

1. **Sensor Interference**: Ultrasonic sensors were affected by nearby cell towers. The team added logic to filter out erroneous readings and conducted tests in interference-free environments.
2. **IMU Calibration**: Required a deep understanding of I2C communication, register manipulation, and data interpolation to achieve accurate yaw measurements.
3. **Balancing the Hovercraft**: Dynamic balancing techniques were applied to stabilize the craft.
4. **Pure C Coding**: Refactored code to remove Arduino libraries, inspired by AVR and community-sourced examples.

## Development Roadmap

1. **Material Selection**: Lightweight, strong Styrofoam frame and durable skirt material.
2. **Fabrication**: Constructed the frame, skirt, and mounted components with precision.
3. **Programming**: Iterative development of pathfinding logic and real-time control using AVR Studio.
4. **Testing and Optimization**: Addressed design flaws and fine-tuned parameters for competition readiness.

## Key Takeaways

- Gained practical experience in embedded systems programming and microcontroller interfacing.
- Learned to navigate challenges in sensor integration and real-time system design.
- Developed teamwork and project management skills in a high-pressure environment.

## Acknowledgments
Thanks to Concordia University's ENGR 290 staff and fellow classmates for their support and collaboration throughout the project in H-933.