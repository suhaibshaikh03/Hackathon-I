---
sidebar_position: 1
---

# Appendix A: Installation Guides

## System Requirements
- **Operating System**: Ubuntu 22.04 LTS (required)
- **GPU**: NVIDIA RTX Series (RTX 3060 or higher recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 500GB SSD minimum
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen 7 recommended)

## Prerequisites Installation

### 1. Install Docusaurus (as specified in FR-003a)
```bash
npx create-docusaurus@latest my-website classic
```

### 2. ROS 2 Humble Hawksbill Installation
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

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. NVIDIA Isaac Dependencies
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

### 4. Gazebo Installation
```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### 5. Additional Development Tools
```bash
# Install Git and version control
sudo apt install git

# Install Python development tools
sudo apt install python3-dev python3-pip

# Install essential build tools
sudo apt install build-essential cmake

# Install audio processing tools (for VLA)
sudo apt install portaudio19-dev python3-pyaudio

# Install computer vision dependencies
sudo apt install libopencv-dev python3-opencv
```

## Module-Specific Installations

### Module 1: ROS 2 Setup
```bash
# Install additional ROS 2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-rosbridge-suite
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

### Module 2: Digital Twin Setup
```bash
# Install Unity (download from unity.com)
# Install Unity Robotics packages via Package Manager

# Install Gazebo classic (for comparison)
sudo apt install gazebo-classic
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
```

### Module 3: Isaac Platform Setup
```bash
# Download and install Isaac Sim from NVIDIA Developer website
# Follow official installation guide for Omniverse integration

# Install Isaac Lab
git clone https://github.com/isaac-lab/isaac-lab.git
cd isaac-lab
./isaaclab.sh -i

# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-perception
sudo apt install ros-humble-isaac-ros-navigation
```

### Module 4: VLA Setup
```bash
# Install OpenAI Whisper
pip install openai-whisper
pip install torch torchaudio

# Install transformers for foundation models
pip install transformers
pip install torch torchvision torchaex

# Install speech recognition tools
pip install SpeechRecognition
pip install pyaudio
```

## Environment Configuration

### 1. Workspace Structure
Create the recommended workspace structure:
```bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws
```

### 2. Environment Variables
Add to your `~/.bashrc`:
```bash
# ROS 2 Setup
source /opt/ros/humble/setup.bash

# Isaac Sim Setup (update path as needed)
export ISAACSIM_PATH=/path/to/isaac-sim
export PYTHONPATH=$ISAACSIM_PATH/python:$PYTHONPATH

# Workspace Setup
source ~/humanoid_ws/install/setup.bash

# CUDA Setup
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

### 3. Source Environment
```bash
source ~/.bashrc
```

## Verification Steps

### 1. ROS 2 Verification
```bash
# Test ROS 2 installation
ros2 --version
ros2 topic list

# Test basic functionality
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

### 2. CUDA Verification
```bash
# Check NVIDIA drivers
nvidia-smi

# Check CUDA installation
nvcc --version
```

### 3. Gazebo Verification
```bash
# Launch Gazebo
gazebo --version
gazebo
```

### 4. Isaac Components Verification
```bash
# Check Isaac ROS packages
ros2 pkg list | grep isaac

# Test Isaac ROS nodes
ros2 run isaac_ros_apriltag_interfaces apriltag_node
```

## Troubleshooting Common Issues

### 1. ROS 2 Installation Issues
**Problem**: Package not found during ROS 2 installation
**Solution**:
```bash
sudo apt update
sudo apt upgrade
# Check ROS 2 distribution compatibility
```

### 2. CUDA Installation Issues
**Problem**: CUDA not detected or driver issues
**Solution**:
```bash
# Check driver compatibility
nvidia-smi
# Verify CUDA installation
cat /usr/local/cuda/version.txt
```

### 3. Isaac Sim Installation Issues
**Problem**: Isaac Sim fails to launch
**Solution**:
- Verify NVIDIA GPU compatibility
- Check Omniverse account and permissions
- Ensure proper graphics drivers are installed

### 4. Audio Processing Issues
**Problem**: Voice recognition fails due to audio problems
**Solution**:
```bash
# Test audio input
arecord -D hw:0,0 -f cd -d 5 test.wav
aplay test.wav

# Check audio permissions
sudo usermod -a -G audio $USER
```

## Updating and Maintenance

### 1. Regular Updates
```bash
# Update system packages
sudo apt update && sudo apt upgrade

# Update ROS 2 packages
sudo apt update
sudo apt upgrade ros-humble-*
```

### 2. Isaac Platform Updates
- Check NVIDIA Developer website for Isaac platform updates
- Follow official update procedures for each component
- Test updates in a separate environment first

### 3. Backup and Recovery
```bash
# Create backup of workspace
tar -czf ~/workspace_backup.tar.gz ~/humanoid_ws

# Restore workspace if needed
tar -xzf ~/workspace_backup.tar.gz -C ~/
```

## Performance Optimization

### 1. System Optimization
```bash
# Disable unnecessary services
sudo systemctl disable unattended-upgrades

# Optimize swappiness for systems with sufficient RAM
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
```

### 2. GPU Optimization
```bash
# Check GPU status
nvidia-smi -l 1

# Set GPU to performance mode
sudo nvidia-smi -ac 5000,1590  # Example values, adjust based on GPU
```

This installation guide provides comprehensive setup instructions for the Physical AI & Humanoid Robotics textbook environment. Follow the steps in order, and verify each component before proceeding to the next module.