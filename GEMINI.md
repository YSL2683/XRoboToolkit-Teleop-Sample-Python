# XRoboToolkit-Teleop-Sample-Python

## Project Overview
`XRoboToolkit-Teleop-Sample-Python` is a comprehensive framework for robot teleoperation using XR (VR/AR) devices. It bridges the gap between XR input (via Pico headsets and controllers) and robotic control in both MuJoCo simulation and physical hardware.

### Key Technologies
- **Python 3.10+**: Primary programming language.
- **MuJoCo**: Physics engine for high-fidelity simulation.
- **Placo**: Quadratic Programming (QP) based Inverse Kinematics (IK) solver for whole-body control.
- **xrobotoolkit-sdk**: Python bindings for interfacing with the XR device service.
- **Tyro**: Library for simplifying CLI argument parsing and configuration.
- **meshcat**: Visualization tool for robot frames and targets.

### Architecture
The project follows a modular, object-oriented design:
- **`xrobotoolkit_teleop.common.BaseTeleopController`**: Abstract base class containing the core teleoperation loop, XR pose processing, and IK task management.
- **`xrobotoolkit_teleop.simulation`**: Contains controllers for MuJoCo and Placo-only visualizations.
- **`xrobotoolkit_teleop.hardware`**: Includes robot-specific controllers (e.g., `DualArmURController`, `ARXR5TeleopController`) and hardware interfaces (RealSense, Dynamixel, UR RTDE).
- **`xrobotoolkit_teleop.utils`**: Helper modules for geometry (quaternions, transforms), MuJoCo synchronization, and path management.

---

## Building and Running

### Environment Setup
The project is optimized for Ubuntu 22.04/24.04. Conda is the recommended environment manager.

1. **Create Environment:**
   ```bash
   bash setup_conda.sh --conda <env_name>
   conda activate <env_name>
   ```

2. **Install Dependencies:**
   ```bash
   bash setup_conda.sh --install
   ```
   *Note: This installs the core package in editable mode and fetches external dependencies like `XRoboToolkit-PC-Service-Pybind`.*

### Running the Application

- **MuJoCo Simulation (Dual UR5e):**
  ```bash
  python scripts/simulation/teleop_dual_ur5e_mujoco.py
  ```

- **Placo Visualization (X7S):**
  ```bash
  python scripts/simulation/teleop_x7s_placo.py
  ```

- **Physical Hardware (Dual UR5e):**
  ```bash
  python scripts/hardware/teleop_dual_ur5e_hardware.py
  ```

### Data Collection
Teleoperation data (joint states, poses, camera streams, XR inputs) can be logged during hardware sessions.
- **Start/Stop Logging:** Press the **'B' button** on the XR controller.
- **Discard Session:** Click the **Right Joystick**.
- **Log Location:** `logs/<robot_name>/teleop_log_YYYYMMDD_HHMMSS_<session_id>.pkl`.

---

## Development Conventions

### Coding Style
- **Formatting:** Use `black` with a line length of **120** characters.
- **CLI Arguments:** Use `tyro` for all script entry points to ensure consistent help messages and type safety.
- **Robot Configuration:** Defined via nested dictionaries (e.g., `manipulator_config`) specifying link names, pose sources, and triggers.

### Testing & Validation
- **Simulation First:** Always validate teleoperation logic in MuJoCo before deploying to hardware.
- **IK Verification:** Use the `--visualize_placo` flag to debug IK solver targets and convergence.
- **Data Validation:** Use `scripts/misc/test_data_log_analysis.py` to check the integrity of collected datasets.

### Extending the Framework
- **New Robot:** Inherit from `BaseTeleopController` and implement the abstract methods: `_robot_setup`, `_update_robot_state`, `_send_command`, and `_get_link_pose`.
- **New Interface:** Add hardware-specific communication logic in `xrobotoolkit_teleop/hardware/interface/`.
