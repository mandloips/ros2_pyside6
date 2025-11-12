# Turtlesim GUI — README

## Overview
This project implements a simple desktop GUI that interfaces with a ROS 2 turtlesim node. It demonstrates:
- publishing velocity commands (Start / Stop),
- subscribing to `/turtle1/pose` and showing live state,
- use of a virtual environment to smoothly install pyside6 without clashing in the externatal environment.
- a live plot of a data stream (linear velocity),
- CSV data logging for recorded poses.

---

## Design goals and trade-offs
- **Simplicity & clarity**: The code is split into a ROS interface (`ros_gui.py`), a GUI (`gui.py`), a data-logger (`data_logger.py`) and a small `main.py` that wires everything together.
- **Responsiveness**: ROS spinning is isolated in a separate thread.
- **Minimal external dependencies**: use of PySide6 and `matplotlib` for GUI and plotting because they are stable, cross-platform, and common in robotics education.
- **Ease of testing**: components are loosely coupled so individual parts can be run or mocked separately (e.g., replacing ROS with a simulated publisher).

Trade-offs:
- Using a separate thread for ROS spin is simple to implement and maintainable but requires careful shutdown handling. Alternatives would be embedding the Qt event loop inside rclpy (harder) or using inter-process communication.
- Plotting is done with `matplotlib` and a simple fixed-length buffer (circular window) to keep resource usage predictable. For higher throughput or more complex plots, a dedicated plotting library (e.g., pyqtgraph) would perform better.

---

## Architecture (module responsibilities)
- `main.py`
  - Initializes `rclpy`, creates the `RosGui` node and the PySide6 application, and starts a dedicated thread that runs a non-blocking ROS spin loop. It wires together shutdown synchronization between GUI and ROS.

- `ros_gui.py`
  - ROS 2 node (`RosGui`) that:
    - publishes `geometry_msgs/Twist` messages on `/turtle1/cmd_vel`,
    - subscribes to `turtlesim/Pose` and updates local state values,
    - exposes `start_ros_timer()` and `stop_ros_timer()` which start/stop a ROS timer that periodically publishes randomized velocity commands.
  - Also calls the logging helper to persist each received pose to CSV.

- `gui.py`
  - PySide6-based UI that displays the current state (x, y, theta, linear and angular velocities), has Start/Stop buttons to control the ROS node, and contains a small live plot of linear velocity using a `matplotlib` `FigureCanvas`.
  - The GUI periodically reads state directly from the `RosGui` node object and updates the UI on a `QTimer` callback.

- `data_logger.py`
  - Small, single-purpose module that appends pose records to a CSV file (`pose_data_log.csv`). It uses the standard `csv` library for portability.

---

## Key design decisions

### Threading and event loops
- **Why a separate ROS thread?**
  - Qt and `rclpy` both have event loops. Running both in the same thread is possible but tricky. Spinning ROS in a background thread with `rclpy.spin_once(node, timeout_sec=0.1)` keeps the GUI thread free for Qt's event loop and allows timely ROS callbacks.

### Data logging strategy
- The `data_logger` module handles CSV initialization and append-only writing. Each incoming `Pose` callback calls `log_pose_data(...)`. This keeps logging responsibility decoupled from ROS node logic and makes it easy to swap in a different storage backend later.
- Timestamping uses `datetime.now()` to keep records human-readable and portable across machines.

### Plotting decisions
- The GUI maintains a fixed-length buffer (`n_data = 50`) for the plotted series and shifts data on each update. This bounding prevents unbounded memory growth and keeps redraws fast for small demo data rates.
- `matplotlib` is used because it integrates cleanly with Qt and is suitable for simple plots. For higher update rates or more interactive needs, switching to `pyqtgraph` is recommended.

### Randomized command generation
- The `send_velocity_command` function publishes randomized velocities to demonstrate control and streaming data. For a real application, replace this with user control inputs, a control policy, or recorded trajectories.

### Error handling and shutdown
- The GUI `closeEvent` signals the `shutdown_event` which the ROS spin thread is checking. When set, the spin loop exits.
- The `main.py` intentionally uses `spin_once` and a `try/finally` to ensure the node is destroyed and rclpy is shut down even if the thread is interrupted.

---

## Files in this repository
- `main.py` — app entrypoint and thread wiring
- `ros_gui.py` — ROS node, publisher & subscriber, timer-based command publisher
- `gui.py` — PySide6 GUI, plot canvas and UI controls
- `data_logger.py` — CSV logging helper
- `pose_data_log.csv` — produced at runtime (not tracked in repository)

---
