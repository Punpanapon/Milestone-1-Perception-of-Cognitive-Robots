# Milestone 1: Perception of Cognitive Robots 🤖📷

**Course:** 2147331.i Perception of Cognitive Robots  
**Platform:** Webots (Python Controller)  
**Assignment:** Milestone 1 - Computer Vision & Control  

## 📖 Project Overview
This repository contains the autonomous robot controller developed for Milestone 1. The goal was to build a complete **Computer Vision Pipeline from scratch** (without using external libraries like OpenCV) to enable an e-puck robot to navigate an environment, detect objects, and classify them correctly.

The robot uses a **Finite State Machine (FSM)** to switch between patrolling, inspecting objects, and avoiding obstacles, relying entirely on its own visual processing for decision-making.

---

## 👁️ Computer Vision Pipeline
To comply with the course architecture, the vision system processes raw pixel data through the following stages:

### 1. Preprocessing: Gaussian Blur
* **Technique:** Applied a **3x3 Convolution Kernel**.
* **Purpose:** Smooths high-frequency noise (such as the wood grain texture on the floor) to prevent false positives during edge detection.

### 2. Grayscale Conversion
* **Technique:** Weighted Channel Averaging `(R+G+B) // 3`.
* **Purpose:** Reduces computational complexity by converting 3-channel RGB data into a single intensity channel.

### 3. Edge Detection
* **Technique:** Gradient Magnitude Calculation.
* **Implementation:** Calculates the absolute difference between a pixel and its neighbor `abs(pixel_x - pixel_x+n)`. If the difference exceeds the `EDGE_SENSITIVITY` threshold, it is marked as an edge.

### 4. Classification & Feature Extraction
We implemented a custom **Spatial-Filtered Edge Counter** to distinguish between object types.
* **The Problem:** Textured boxes (with text like "ABCD") create too many "edges," looking like complex objects.
* **The Solution (Spatial Filter):** Implemented a "Cooldown" mechanism. Once an edge is detected, the scanner ignores the next **30 pixels**. This allows the robot to detect the *structure* of the box (Left/Right sides) while ignoring the *texture* (Text on the box).

#### Classification Logic:
| Object Type | Detected Features | Logic |
| :--- | :--- | :--- |
| **Wall / Background** | **0 - 1 Edges** | Smooth surface, distinct only at screen borders. |
| **Static Box** | **2 - 8 Edges** | Simple geometry (Left + Right sides). Text noise is filtered out. |
| **Complex Object (Robot)** | **> 8 Edges** | Intricate geometry triggers multiple edge detections despite filtering. |

---

## 🕹️ Control Architecture (FSM)
The robot operates using a reactive Finite State Machine:

1.  **STATE_WANDER:** Patrols at cruise speed (`MAX_SPEED = 6.28`). Monitors Proximity Sensors (`dist > 60`) to trigger inspection.
2.  **STATE_INSPECT:** Stops movement to stabilize the camera. Runs the full vision pipeline to classify the object.
3.  **STATE_AVOID:** If the object is an obstacle, performs a "Blind Turn" maneuver to clear the path.
4.  **STATE_STAY (Turret Mode):** Manual override (Hold `S` key). The robot stays stationary and outputs continuous classification data to the HUD.

---

## 🚀 How to Run
1.  Install **Webots R2023b** (or newer).
2.  Clone this repository.
3.  Load the world file `worlds/Percog1.wbt`.
4.  Ensure the controller is set to `observer_robot.py`.
5.  Press **Play** in Webots.

### Controls
* **`M` Key:** Toggle Manual/Autonomous Mode.
* **`S` Key (Hold):** Activate Turret Mode (Stationary Debugging).
* **Arrow Keys:** Drive robot in Manual Mode.

---

## 📊 Simulation Results
The robot successfully differentiates between huge smooth walls (ignored) and small textured boxes (avoided), solving the "ABCD texture" problem using the implemented spatial cooldown filter.

