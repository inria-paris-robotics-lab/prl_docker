# PRL Docker Infrastructure (Humble & Jazzy)

This repository provides a centralized, scalable Docker infrastructure for various robotics projects at the PRL lab. It is designed to share a common development environment across different ROS 2 distributions (**Humble** and **Jazzy**) while handling the complexities of hardware access, and user permissions.

## Architecture

The infrastructure uses a **Layered Approach** to optimize build times and maintenance:

1.  **Base Images (`prl:base-[distro]`)**:
    *   Standardized development tools (Vim, Tmux, Terminator, Git, Byobu...).
    *   Heavy mathematical libraries: **Pinocchio** (via RobotPkg), **VisP**.
    *   Universal **"Fix-on-Run" Entrypoint** for dynamic user mapping.
2.  **Robot/Project Images (`prl:[robot]-[distro]`)**:
    *   Inherits from the Base image.
    *   Contains hardware-specific drivers (UR, Franka, Realsense, Orbbec...).
    *   Includes a **Workspace Auto-Setup hook** to clone and build source code on the first run.

The objectif if to allow user to esaly customise the robot setup and add their dependencies just by creating a new Dockerfile and a setup script, without having to worry about the underlying infrastructure or user permissions.
## 🚀 Quick Start

### 1. Prerequisites
*   Docker and Docker Compose installed.

> [!IMPORTANT]
> For GUI support on Linux, ensure you have the necessary permissions set up for X11 forwarding. The `xhost +local:docker` command allows Docker containers to access your display server.

### 2. Launching a Project (e.g., Mantis - Jazzy)
We provide a launch script that automatically synchronizes your local **UID/GID** with the container to prevent file permission issues.

```bash
chmod +x start_docker.bash
./start_docker.bash
```

**What this script does:**
1. Get your local User and Group IDs.
2. Get hardware GIDs (Docker, Render, Input) from your host.
3. Launches the container in the background.
4. The entrypoint of the image executes the workspace setup script if there are in the .entrypoint-setup/ directory.
5. Drops you into an interactive shell as the `ros` user.

## 🛠 Building the Images

If you need to update the infrastructure (e.g., adding a new system-wide tool):

```bash
# 1. Build the Base image (e.g., Jazzy)
docker build -t prl:base-jazzy -f Dockerfile.base .

# 2. Build the Robot-specific image
docker build -t prl:mantis-jazzy --build-arg BASE_IMAGE=prl:base-jazzy -f Dockerfile.mantis .
```

## ⚙️ Technical Key Features

### Dynamic User Mapping (Universal Entrypoint)
Unlike standard Dockerfiles with hardcoded IDs, these images are **generic**. At runtime, the `entrypoint.sh` script:
1. Takes `HOST_UID` and `HOST_GID` environment variables.
2. Modifies the internal `ros` user to match your host account.
3. Patches internal group IDs for `docker`, `render` (GPU), and `input` (Joysticks) to match your specific machine.
4. **Result:** You can edit files, use the GPU, and access USB devices without ever using `sudo`.

### Workspace Auto-Setup Hook
The Robot images utilize an **Entrypoint Hook**. On the first run, if the source folder is empty, the `setup_mantis.sh` script (located in `/.entrypoint-setup/`) will:
*   Clone required repositories.
*   Import dependencies using `vcs import`.
*   Install system dependencies via `rosdep`.
*   Build the workspace using `colcon build --symlink-install`.

### Hardware & GUI Support
The `docker-compose.yml` is pre-configured for:
*   **GPU Acceleration**: Direct access to `/dev/dri` for Gazebo/Rviz.
*   **USB Access**: Full access to `/dev/bus/usb` for Realsense/Orbbec cameras.
*   **X11 Forwarding**: Shared X11 socket for all GUI-based tools.

## 📁 Workspace Structure
Inside the container, the environment is organized as follows:
*   `/home/ros/share/mantis_ws/src`: Your local project folder (bind-mounted, editable in real-time).
*   `/home/ros/ros2_utils_ws`: Compiled system utilities (e.g., OrbbecSDK).
*   `/opt/openrobots`: Pinocchio installation path.

## ➕ Adding a New Project/Robot
1. Create a new Dockerfile (e.g., `Dockerfile.franka`).
2. Use `FROM prl:base-humble` or `base-jazzy` as the starting point.
3. Create a `setup_robot.sh` script and `COPY` it into `/.entrypoint-setup/` in your Dockerfile.
4. **Do not** redefine the `ENTRYPOINT`.

---
