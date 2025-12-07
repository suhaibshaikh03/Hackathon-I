---
sidebar_position: 1
---

# Appendix A: Installation Guides

## System Requirements
- **Operating System**: Ubuntu 22.04 LTS (required)
- **GPU**: NVIDIA RTX Series (RTX 3060 12GB or higher recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 500GB SSD minimum
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen 7 recommended)
- **Network**: Gigabit Ethernet for multi-machine setups

## ROS 2 Humble Hawksbill Installation

### Prerequisites
```bash
# Set locale
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Install dependencies
sudo apt update && sudo apt install curl gnupg lsb-release
```

### Add ROS 2 GPG Key
```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add ROS 2 Repository
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2 python3-vcstool
```

### Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

### Install Colcon Build Tools
```bash
sudo apt install python3-colcon-common-extensions
```

### Source ROS 2 Environment
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## NVIDIA Isaac Platform Installation

### Install NVIDIA GPU Drivers
```bash
sudo apt install nvidia-driver-535
sudo reboot
```

### Install CUDA Toolkit
```bash
# Download and install CUDA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key add /var/cuda-repo-slim-12-0-local/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"
sudo apt update
sudo apt -y install cuda
```

### Install Isaac Sim
1. Visit NVIDIA Developer website to download Isaac Sim
2. Follow the installation guide at: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html
3. Activate with your NVIDIA Developer account

### Install Isaac Lab
```bash
git clone https://github.com/isaac-sim/isaac-lab.git
cd isaac-lab
./isaac-sim.sh -i  # Interactive installation
```

### Install Isaac ROS
```bash
sudo apt install ros-humble-isaac-ros-*  # Installs all Isaac ROS packages
```

## Additional Development Tools

### Python Development
```bash
sudo apt install python3-dev python3-pip
pip3 install --upgrade pip
```

### Audio Processing Tools
```bash
sudo apt install portaudio19-dev python3-pyaudio
pip3 install pyaudio speechrecognition
```

### Computer Vision Dependencies
```bash
sudo apt install libopencv-dev python3-opencv
pip3 install opencv-python open3d
```

### Build Tools
```bash
sudo apt install build-essential cmake
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator
```

### Git and Version Control
```bash
sudo apt install git git-lfs
git lfs install
```

## Verification Steps

### ROS 2 Verification
```bash
# Test basic ROS 2 functionality
ros2 --version
ros2 topic list

# Test basic publisher/subscriber
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

### CUDA Verification
```bash
nvidia-smi
nvcc --version
```

### Isaac Sim Verification
```bash
# Launch Isaac Sim
isaac-sim.sh
```

### Isaac Lab Verification
```bash
# Test Isaac Lab installation
cd isaac-lab
./isaaclab.sh -c
```

## Common Installation Issues and Solutions

### 1. Permission Issues
**Problem**: Permission denied during installation
**Solution**:
```bash
# Check if you're using sudo when needed
# For pip installations, use --user flag if needed
pip3 install --user package_name
```

### 2. Dependency Conflicts
**Problem**: Package conflicts during installation
**Solution**:
```bash
# Update package lists
sudo apt update
sudo apt upgrade

# Fix broken dependencies
sudo apt --fix-broken install
```

### 3. GPU Driver Issues
**Problem**: GPU not recognized or CUDA not working
**Solution**:
```bash
# Check GPU detection
lspci | grep -i nvidia

# Verify driver installation
nvidia-smi

# Reinstall drivers if needed
sudo apt purge nvidia-*
sudo apt autoremove
sudo apt install nvidia-driver-535
```

### 4. ROS 2 Environment Issues
**Problem**: ROS 2 commands not found after installation
**Solution**:
```bash
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Or add to bashrc permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Workspace Setup

### Create ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Verify Workspace
```bash
# Check if workspace is properly sourced
echo $ROS_PACKAGE_PATH
# Should include your workspace path
```

## Troubleshooting Checklist

- [ ] Ubuntu 22.04 LTS is installed
- [ ] NVIDIA GPU drivers are properly installed and recognized
- [ ] CUDA toolkit is installed and functional
- [ ] ROS 2 Humble is installed and accessible
- [ ] Isaac platform components are installed (if needed)
- [ ] Development tools are installed
- [ ] Workspace is created and properly sourced
- [ ] Basic functionality tests pass