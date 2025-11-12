# Setup and Dependencies

## Environment overview
This project was designed to run in **two environments simultaneously**:
1. **Virtual Environment (venv)** — where the GUI and Python codebase (`main.py`, `ros_gui.py`, `gui.py`, `data_logger.py`) execute.
2. **External ROS 2 Environment** — where the ROS 2 core and `turtlesim_node` run.

This separation allows the Python GUI to use a custom virtual environment for package management (PySide6, matplotlib, etc.) while still interacting with the ROS 2 graph in the external environment.

---

## Step 1: Create and activate a virtual environment

```bash
# Navigate to the project root
cd path/to/this/project

# Create a virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate
```

> 💡 *All GUI dependencies are installed inside this virtual environment. ROS 2 itself remains in the external system installation.*

---

## Step 2: Install Python dependencies

Create a `requirements.txt` file (included below) or install packages manually:

### requirements.txt
```
PySide6
matplotlib
```

---

## Step 3: Source the ROS 2 environment

Before running the GUI, ensure that the external ROS 2 environment is sourced **in the same terminal session** or a separate one running the ROS 2 nodes.

In one terminal (external ROS 2 environment):
```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

In another terminal (virtual environment):
```bash
# Activate the virtual environment
source venv/bin/activate

# Also source the external ROS setup to ensure ROS Python libraries are visible
source /opt/ros/humble/setup.bash

# Run the GUI
python main.py
```

> ⚙️ **Important:** `rclpy` is not installed inside the virtual environment. Instead, the interpreter in the venv accesses it from the sourced external ROS environment. This ensures compatibility with ROS 2 installation.

---

## Step 4: Verify connection between environments
1. Confirm that `/turtle1/pose` and `/turtle1/cmd_vel` topics are visible in both terminals:
   ```bash
   ros2 topic list
   ```
2. Launch the GUI — the turtle should start moving when you press **Start**.
3. The CSV file (`pose_data_log.csv`) should be generated in the project directory.

---

## Summary
This dual-environment setup ensures:
- **Modularity:** GUI and ROS runtime are loosely coupled.
- **Safety:** No risk of breaking the global ROS installation.
- **Reproducibility:** Anyone can recreate the environment using `venv` and `requirements.txt`.
