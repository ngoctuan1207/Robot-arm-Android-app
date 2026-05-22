# Android application for controlling 6-DOF robot arm

## Overview
The application communicates with the robot controller via Wi-Fi using the ESP32 chip. In access point mode, the ESP32 creates a dedicated wireless network to which the tablet connects, enabling data exchange through the TCP/IP protocol. Control commands, including joint angles, velocities, and accelerations, are transmitted to the controller, while joint position feedback is received in JSON format.

## Technical Specifications
### Android & Java Environment
* **Minimum Android Version:** Android 9.0 (API Level 28 / Pie)
* **Target Android Version:** Android 15 (API Level 35 / Vanilla Ice Cream)
* **Compile SDK Version:** API Level 35
* **Java/JVM Version:** Java 11 (`JavaVersion.VERSION_11`)

### Required Permissions
To interact with the hardware and manage connections, the application requires the following Android permissions:
* `INTERNET`: Allows the app to open network sockets for TCP/IP communication with the ESP32 server.
* `ACCESS_WIFI_STATE` & `CHANGE_WIFI_STATE`: Enables the app to view and configure Wi-Fi connections (connecting to the ESP32 access point).
* `ACCESS_FINE_LOCATION` & `ACCESS_COARSE_LOCATION`: Required by Android system specifications to scan and discover nearby Wi-Fi networks.
* `READ_EXTERNAL_STORAGE` & `WRITE_EXTERNAL_STORAGE`: Allows the app to read and write data to device storage (e.g., saving configurations or logs).
---
## Key Features
* **TCP/IP Socket Communication:** Handles stable, low-latency, wireless data transmission between the mobile interface and the ESP32 control board.
* **Virtual Joystick Integration:** Embedded standard virtual joystick inputs for intuitive, real-time manual manual maneuvering of the robotic arm.
* **JSON Data Parsing (GSON):** Streamlines data serialization and deserialization, converting continuous telemetry streams from the hardware into readable formats.
* **Game-optimized Activity Handling:** Uses low-latency game activity libraries to ensure responsive UI controls during heavy kinematics calculation and continuous joystick input.

## Software Architecture
The application core logic is organized under the `com.example.RobotArm` package. Below is an overview of the key classes and their responsibilities:
<p align="center">
<img width="679" height="484" alt="Screenshot 2026-05-22 161112" src="https://github.com/user-attachments/assets/e1204820-0f09-451d-b474-45979ca673c2" />
</p>

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

## Getting Started

Follow these instructions to get a local copy of the project up and running, and to understand how to connect it to your robot hardware.

### Prerequisites

Before building the application, ensure you have the following installed and configured:
* **Android Studio:** Ladybug (2024.2.1) or newer recommended.
* **Android SDK:** API Level 35 (Android 15).
* **Hardware:** A 6-DOF Robotic Arm controlled by an ESP32 microcontroller configured in **Access Point (AP) Mode**.

### Installation & Build

1. **Clone the Repository:**

2. **Open the Project:**
   * Launch Android Studio.
   * Select **Open An Existing Project** and navigate to the cloned `RobotArm_app` directory.

3. **Sync and Build:**
   * Allow Android Studio to automatically sync the Gradle files and download the required dependencies (`GSON`, `Virtual Joystick`, etc.).
   * Click the **Make Project** (Hammer icon) to verify there are no compilation errors.

4. **Run the Application:**
   * Enable **USB Debugging** on your physical Android device (Android 9.0+).
   * Connect your device to your computer via USB.
   * Select your device from the target device dropdown in Android Studio and click **Run** (Play icon).
     
## Android Teach Pendant Application
<p align="center">
<img width="2560" height="1568" alt="Android_app" src="https://github.com/user-attachments/assets/fde9c6ec-1336-4a49-b3d6-2c931b80fd42" />
</p>

### How It Works
* **Wireless Communication:** The application establishes a bidirectional connection with the robot controller via **Wi-Fi** of ESP32 access point mode. 
* **Command Transmission:** It packs and transmits motion control commands—including target joint angles, velocities, and accelerations—directly to the controller.
* **Real-time Feedback:** The app continuously receives joint position feedback from the robot in **JSON format**.
* **Live Kinematics Computation:** Utilizing the incoming joint data, the application calculates the Tool Center Point (TCP) position and orientation ($X, Y, Z, EX, EY, EZ$) in real time.

---
### Key App Functions

#### 1. Real-time Monitoring & Configuration
* **Live Telemetry Display:** Displays real-time data for individual joint positions ($J_1 \rightarrow J_6$) and the calculated TCP pose.
* **Coordinate System Selection:** Allows users to easily switch the reference coordinate system (e.g., *Base*) for kinematic calculations.
* **Work Object Management:** Supports selecting different work objects (e.g., *Wobj0*) to dynamically redefine the coordinate origin.

#### 2. Motion Control & Navigation
* **Dual On-Screen Joysticks:** Features two intuitive joysticks for manual 3D jog control.
* **Jog Speed Adjustment:** Includes a slider to dynamically scale the robot's jogging speed from 0% to 100%.
* **Home Position Return:** A dedicated **Home** button quickly commands the robot arm to return to its default calibration position.
* **Multiple Motion Modes:** Supports joint jogging for specific groups (Joints 1–3 and Joints 4–6), as well as simultaneous 6-joint control in either **Angle Mode** or **Position Mode** *(Linear motion and tool reorientation are currently under development)*.

#### 3. Industrial Safety Features
* **Deadman Switch (Enable Button):** As a core safety measure, the robot motion can only be initiated and maintained while the **Enable** button is actively pressed and held. Releasing the button stops the movement immediately.
* **Emergency Stop (E-STOP):** A prominent, easily accessible **E-STOP** button that instantly halts the entire robot system operation in emergency situations.





