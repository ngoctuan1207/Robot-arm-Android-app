# Android application for controlling 6-DOF robot arm

## Overview
The application communicates with the robot controller via Wi-Fi using the ESP32 chip. In access point mode, the ESP32 creates a dedicated wireless network to which the tablet connects, enabling data exchange through the TCP/IP protocol. Control commands, including joint angles, velocities, and accelerations, are transmitted to the controller, while joint position feedback is received in JSON format.


The application core logic is organized under the `com.example.RobotArm` package. Below is an overview of the key classes and their responsibilities:

<img width="679" height="484" alt="Screenshot 2026-05-22 161112" src="https://github.com/user-attachments/assets/4ab546ea-9e40-4953-b17b-95d76d427fe6" />

| Class / Component | Description |
| :--- | :--- |
| **`MainActivity`** | The core application lifecycle manager. It handles the user interface (UI) and processes direct user interactions. |
| **`ESP32_Server`** | Manages TCP/IP network communication. It establishes and maintains the connection to the ESP32 Wi-Fi access point to transmit control commands. |
| **`Esp32Feedback.kt`** | A data class that defines the structured format of the telemetry and feedback data received from the ESP32. |
| **`Esp32Parser`** | Processes and parses the raw data packets received from the ESP32 into readable formats for the application. |
| **`LinearPlanner`** | Responsible for trajectory planning, generating smooth linear motion paths for the robotic arm. |
| **`RobotFK`** | Handles **Forward Kinematics** calculations to determine the end-effector's position based on joint angles. |
| **`RobotIK`** | Handles **Inverse Kinematics** calculations to determine the required joint angles for a target end-effector position. |
| **`RobotModel.kt`** | Stores the physical parameters and dimensions of the robot, including its **Denavit–Hartenberg (DH) table**. |
| **`SpinnerUtils.kt`** | A utility class providing helper functions to manage UI spinner components effectively. |


