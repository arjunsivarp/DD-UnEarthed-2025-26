## LEGO SPIKE PRIME – "Daring Divers" Robot Code

### Overview

This Python script controls a LEGO® SPIKE™ Prime robot for the **FIRST LEGO League (FLL) “Daring Divers”** Unearthed season.
It includes a complete set of autonomous routines (“runs”) for different missions, plus utility functions for movement, turning, and PID-based gyro driving.

The script is designed to run on the **SPIKE Prime Hub** using the **SPIKE Prime Python environment** (MicroPython API with `hub`, `motor`, `motor_pair`, etc.).

---

## Features

* **PID-controlled straight driving** using the gyroscope
* **Accurate turning** (left/right pivot turns)
* **Forward and reverse distance control** (using encoder feedback)
* **Stall detection** and adaptive drive timeouts
* **Modular “Run” structure** — each competition run is defined as an async function
* **Button-based execution control** – waits for left button press to start each run
* **Logging system** (DEBUG/INFO/ERROR levels) for troubleshooting

---

## Hardware Requirements

* LEGO SPIKE Prime Hub
* Motors connected as follows:

  * **Port A & E** → Drive motors (paired as `PAIR_1`)
  * **Port D/F** → Attachments (varies per mission)
* Color, motion, and light sensors connected to standard SPIKE Prime ports as referenced.
* Fully charged SPIKE Prime Hub battery.

---

## Software Requirements

* LEGO Education SPIKE App (desktop or web)
* Python runtime for SPIKE (MicroPython environment)
* This file must be named (e.g.) `main.py` and uploaded to the Hub.

---

## Code Structure

### 1. **Configuration and Constants**

Defines settings like wheel circumference, speed, and PID parameters for precise motion.

### 2. **Utility Functions**

* `do_init()` — calibrates gyro and prepares sensors.
* Logging helpers: `log_info()`, `log_debug()`, `log_error()`.
* Helper math and sensor utilities for encoders and yaw readings.

### 3. **Drive and Turn Control**

* `runFwd(distance_cm, speed)` – Move forward with gyro correction.
* `runRev(distance_cm, speed)` – Reverse motion with correction.
* `turnLeft(angle_deg)` / `turnRight(angle_deg)` – Pivot in place.

### 4. **Mission Run Functions**

Each mission or test sequence is an async function:

| Function     | Description                                          |
| ------------ | ---------------------------------------------------- |
| `run1()`     | M11 (Angler Artifacts), M12 (Salvage Operation), M15 |
| `run2()`     | M1 (Surface Brushing)                                |
| `run3()`     | M2 (Map Reveal)                                      |
| `run4()`     | M3 (Mineshaft Explorer), M4 (Careful Recovery), M15  |
| `run5()`     | M5–M8 (Who Lived Here, Forge, Heavy Lifting, Silo)   |
| `run6()`     | M9–M10 (What’s on Sale, Tip the Scales), M15         |
| `run7()`     | M13–M14 (Statue Rebuild, Forum)                      |
| `run94–99()` | Diagnostic tests (driving and turning calibration)   |

### 5. **Main Executor**

* `execute(run_numbers)` handles sequential execution of selected runs.
* Waits for a **left button press** before starting each run.
* Displays run number on the Hub’s LED matrix.
* Tracks timing for each run and prints a summary report.

---

## How to Use

### Step 1: Setup

1. Open the LEGO SPIKE app and connect to your SPIKE Prime Hub.
2. Upload this Python file to the hub (recommended name: `main.py`).
3. Ensure your motors and sensors are connected to the correct ports.

### Step 2: Run the Program

By default, the script automatically runs the **chained sequence of runs 1, 2, 3, 5, 6**:

```python
runloop.run(execute([1, 2, 3, 5, 6]))
```

To run individual missions, uncomment one of these lines near the bottom:

```python
runloop.run(execute([1]))   # Run 1 only  
runloop.run(execute([2]))   # Run 2 only  
runloop.run(execute([99]))  # Full distance test  
```

### Step 3: Start a Run

* When the robot is ready at the base, press the **Left Button** on the Hub to begin the next mission.
* The hub’s power light color indicates state:

  * **Red** – Waiting to start
  * **Green** – Running
  * **Yellow** – Finished

### Step 4: Review Output

After all runs finish, the program prints:

* Individual run times
* Transition times between runs
* Total mission time summary

---

## Debugging and Logging

Adjust logging settings near the top of the file:

```python
DEBUG = True
LOG_LEVEL = 2  # 0 = ERROR, 1 = INFO, 2 = DEBUG (most verbose)
```

Output is shown in the console if connected to the SPIKE app.

---

## Notes and Best Practices

* Always **reset the gyro** (`do_init()`) before starting each run to maintain accuracy.
* Keep wheels clean and consistent for accurate distance measurement.
* Tune `kp`, `kd`, `ki` constants in `runFwd` and `runRev` for your robot’s drive system.
* Avoid coasting by using `BRAKE` mode in `_safe_stop()`.

---

## File Summary

| File         | Description                     |
| ------------ | ------------------------------- |
| `main.py`    | Main control script (this file) |
| `README.txt` | This documentation file         |

---

## Author / Credits

Created for the **FLL "Daring Divers"** Unearthed season.
Developed by: *Daring Divers*
Version: 1.0
Date: October 2025
