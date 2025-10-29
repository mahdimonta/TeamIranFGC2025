# 🇮🇷 Team Iran - First Global 2025 Robot Code

This repository contains the Java source code for a robot participating in the **First Global 2025 Challenge**. The code is built to control a robot featuring a **Holonomic Drive chassis** and is equipped with advanced systems for precise driving and controlling game mechanisms.

---

## 🌟 Key Robot Systems

### Drive Control System (`RobotController.java` & `IMUClass.java`)
The robot's drive code is implemented with high flexibility for movement:

* **Holonomic Drive:** Enables the robot to move forward, backward, strafe (sideways), and rotate either independently or simultaneously using four drive motors.
* **Field-Centric Drive:** Uses the **IMU sensor** (Inertial Measurement Unit) to maintain the robot's movement direction relative to the field, making driving intuitive.
* **Movement Mode Switch:** The driver can toggle between **"Field-Centric Mode"** and **"Normal Mode"** (Robot-Centric).
* **IMU Initialization:** The `IMUClass.java` handles IMU setup and provides the robot's **Yaw** angle (rotation) for field-centric calculations.

### AprilTag Vision System (`AprilTag.java`)
The code uses AprilTag vision for precise targeting and positioning:

* **AprilTag Detection:** Sets up the camera and `AprilTagProcessor` to detect and locate AprilTags on the playing field.
* **PID Camera Tracking:** Includes a function (`trackingAprilTagAuto`) that uses a **P-Control** loop (or similar basic control) to adjust two CRServos (`lArm`, `rArm`) to keep a detected tag centered in the camera's view.
* **Autonomous Drive to Tag (`driveToTag`):** Implements an autonomous function using Proportional-Control (P-Control) based on the tag's pose data (X-error, Range, and Bearing) to automatically drive the robot toward a specific tag.

### Game Mechanism Control (`MainTeleOP.java` & `RobotController.java`)
Mechanism control is handled via Gamepads:

* **Arm Mechanism:** Two continuous rotation servos (`lArm` and `rArm`) are controlled via the analog sticks of Gamepad 2.
* **Intake/Spinner and Accelerator:** Includes controls for a `spinner` motor and two continuous rotation servos (`intackServo` and `accelerator`) for intake and object acceleration/indexing.
* **Shooter and Winch:** Dedicated motor controls for the `shooter` and `winch` mechanisms to execute game-specific scoring or deployment tasks.
* **Elevator:** A motor (`elevator`) is controlled for vertical movement or height adjustment.

---

## 🏗️ File Structure

| File Name | Functionality |
| :--- | :--- |
| **`MainTeleOP.java`** | Main OpMode. Initializes all hardware and systems, and contains the logic for the driver-controlled period (TeleOp). |
| **`RobotController.java`** | Controls the hardware layer: manages the 4 drive motors (Holonomic/Mecanum drive logic), the `spinner`, and the `elevator` motor. |
| **`AprilTag.java`** | Handles the vision system, camera setup, AprilTag detection, and autonomous tracking/driving algorithms. |
| **`IMUClass.java`** | Hardware abstraction class for the IMU sensor, including initialization and reading the robot's yaw angle. |

---

## 🛠️ Prerequisites and Setup

* **Robot SDK:** Compatible Software Development Kit for the competition.
* **Hardware Configuration:** All components (motors, servos, IMU, webcam) must be correctly named and configured in the Robot Controller configuration file to match the names used in the Java code (e.g., `"FR"`, `"imu"`, `"Webcam 1"`, `"shooter"`, etc.).

---
