# Autonomous Hemispherical Spatial Mapping & Target Detection
*A Robust ToF Approach for Static Target Localization*

## Project Overview
This project, developed for the **Build-A-Bot 3.0** competition, is an autonomous 2-DoF scanning system designed for precise spatial analysis of closed environments.

The system uses **Time-of-Flight (ToF)** technology to map the upper hemisphere of an enclosure and accurately localize static targets without human intervention.

---

## System Architecture
* **The Master (Laptop):** Handles background subtraction, signal filtering, and 3D visualization.
* **The Slave (Arduino Nano):** Embedded control unit driving the motors and triggering sensor reads.
* **The Sensor:** VL53L0X ToF sensor providing millimeter-accurate distance measurements.
* **The Mechanism:** A 2-DoF Pan-Tilt Servo Turret enabling 0°-180° Azimuth and 0°-180° Elevation.


---

## Core Methodology
### Zig-Zag Raster Scanning
To ensure efficient data acquisition, the system utilizes an optimized "Left → Right → Left" scanning path. The turret performs a "backflip" (180° turn) to scan the rear hemisphere, achieving complete coverage without a 360° motor.

### Background Subtraction Algorithm
Traditional thresholding often fails in cuboidal rooms because the corners are farther than the wall centers.
1. **Calibration:** Creates a baseline depth map of the empty room.
2. **Differential Analysis:** Real-time scans are compared against the baseline to isolate "new" targets.
3. **Result:** Filters out room geometry to highlight targets with high precision.


---

## Signal Processing Pipeline
To maintain data integrity, the system implements several layers of noise reduction:
* The Arduino averages three readings per point to deal with sensor jitter.
* Python-side filtering for spatial consistency.
* Stitches fragmented points together to ensure solid target representations.
* Computes the mathematical 3D center ($x, y, z$) of each detected cluster for precise localization.

---

## 3D Visualization
The software generates a fully interactive 3D Spatial Map:
- **Gray Points:** Reconstructed enclosure context.
- **Red Points:** Detected target clusters.
- **Black Stars:** Mathematical centroids of each object detected.