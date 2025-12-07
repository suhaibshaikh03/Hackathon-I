---
sidebar_position: 3
---

# Hardware & Lab Setup Guide

## System Requirements
- **Operating System**: Ubuntu 22.04 LTS (required)
- **GPU**: NVIDIA RTX Series (RTX 3060 or higher recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 500GB SSD minimum
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen 7 recommended)

## Software Dependencies

### Install Docusaurus (as specified in FR-003a)
```bash
npx create-docusaurus@latest my-website classic
```

### ROS 2 Installation (Humble Hawksbill)
```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions
```

### NVIDIA Isaac Dependencies
```bash
# Install NVIDIA drivers
sudo apt install nvidia-driver-535

# Install CUDA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key add /var/cuda-repo-slim-12-0-local/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"
sudo apt update
sudo apt -y install cuda

# Install Isaac Sim prerequisites
sudo apt install python3.10-venv python3-pip
pip3 install --upgrade pip
```

### Gazebo Installation
```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### Additional Tools
```bash
# Install Git and version control
sudo apt install git

# Install Python development tools
sudo apt install python3-dev python3-pip

# Install essential build tools
sudo apt install build-essential cmake
```

## Development Environment Setup

### Workspace Structure
```
~/humanoid_ws/
├── src/                 # Source code
│   ├── ros2_packages/   # ROS 2 packages
│   ├── isaac_packages/  # Isaac packages
│   └── custom_packages/ # Custom packages
├── build/              # Build artifacts
├── install/            # Installation directory
└── log/                # Log files
```

### Environment Configuration
Add to your `~/.bashrc`:
```bash
# ROS 2 Setup
source /opt/ros/humble/setup.bash

# Isaac Sim Setup (update path as needed)
export ISAACSIM_PATH=/path/to/isaac-sim
export PYTHONPATH=$ISAACSIM_PATH/python:$PYTHONPATH

# Workspace Setup
source ~/humanoid_ws/install/setup.bash
```

## Testing Your Setup

### Verify ROS 2 Installation
```bash
# Test ROS 2
ros2 --version
ros2 topic list
```

### Verify CUDA Installation
```bash
nvidia-smi
nvcc --version
```

### Verify Gazebo Installation
```bash
gazebo --version
```

## Troubleshooting

### Common Issues

1. **CUDA Installation Issues**
   - Ensure compatible NVIDIA drivers are installed
   - Check compatibility between driver, CUDA, and GPU

2. **ROS 2 Package Dependencies**
   - Run `rosdep install --from-paths src --ignore-src -r -y` in workspace
   - Verify ROS 2 distribution is correctly sourced

3. **Isaac Sim Dependencies**
   - Check NVIDIA GPU compatibility
   - Ensure proper graphics drivers are installed

### Verification Checklist
- [ ] Ubuntu 22.04 LTS installed
- [ ] NVIDIA RTX GPU detected
- [ ] ROS 2 Humble installed and functional
- [ ] CUDA toolkit installed and working
- [ ] Gazebo simulation environment operational
- [ ] Isaac Sim dependencies satisfied
- [ ] Development workspace properly configured