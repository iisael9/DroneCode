Autonomous Drone Shadowing with AI
Overview

This project focuses on developing a system for autonomous drone detection and tracking using AI-powered technologies. The goal is to enable drones to detect, identify, and dynamically track other drones in real time, addressing the growing need for safe and scalable drone traffic management. This work was conducted during the CREST Summer Program at California State University, San Bernardino.
Key Features

    Object Detection: Implemented TensorFlow Lite for real-time detection of drones in diverse environments.
    Algorithm Development: Created a Python-based algorithm that calculates the position and flight path for drone tracking.
    Control Integration: Used PyMavlink to translate detection data into precise drone flight commands.
    Adaptability: Designed the system to operate autonomously, adjusting to environmental changes dynamically.

Tools and Technologies

    TensorFlow Lite: Lightweight AI model for real-time object detection.
    PyMavlink: Framework for drone communication and control.
    Raspberry Pi 5: Onboard processing unit for running the detection model.
    Dronecode SDK: Supplemental layer for drone communication.
    Python: Core programming language for developing algorithms and integrations.

Project Achievements

    Detection Accuracy: Achieved 90% detection accuracy in controlled environments within the optimal range of 10-20 meters.
    Testing and Training:
        Captured drone images from various angles and lighting conditions to train the TensorFlow Lite model.
        Tested the model's performance in static scenarios, demonstrating reliable detection capabilities.
    Architecture:
        Developed a real-time pipeline from image preprocessing to drone control, integrating AI detection with flight adjustments.
    Simulation: While full flight tests were restricted, the system performed successfully in theoretical simulations and stationary testing.

Challenges and Limitations

    Hardware Constraints: Limited processing power of Raspberry Pi for complex models.
    Environmental Factors: Detection performance impacted by low-light conditions and long distances.
    Flight Testing: Due to regulatory and logistical constraints, in-flight validation has not been completed, though the system is designed to perform in-flight.

Future Work

    Incorporate additional sensors such as LiDAR for improved detection in adverse conditions.
    Enhance the AI model for better long-distance and low-light performance.
    Conduct in-flight testing to validate real-world tracking capabilities.
    Expand system scalability for larger drone traffic management solutions.
