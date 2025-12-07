---
sidebar_position: 3
---

# Appendix C: Full Hardware Tables

## Recommended Hardware Specifications for Humanoid Robotics

### Minimum System Requirements
| Component | Minimum Specification | Recommended Specification | Justification |
|-----------|----------------------|---------------------------|---------------|
| **Operating System** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS | Long-term support, ROS 2 compatibility |
| **GPU** | NVIDIA RTX 3060 12GB | NVIDIA RTX 4080 16GB or RTX 6000 Ada | RTX ray tracing, CUDA acceleration, PhysX support |
| **RAM** | 16GB DDR4 | 32GB+ DDR4/DDR5 | Simulation, training, and perception processing |
| **Storage** | 500GB NVMe SSD | 1TB+ NVMe SSD | Fast I/O for simulation and model loading |
| **Processor** | Intel i5-12400 / AMD Ryzen 5 5600X | Intel i7-13700K / AMD Ryzen 7 7800X | Multi-core for parallel processing |
| **Power Supply** | 750W 80+ Gold | 1000W+ 80+ Platinum | Sufficient power for high-end GPU |
| **Cooling** | Standard CPU cooler | High-performance air or liquid cooling | Thermal management for intensive workloads |
| **Network** | Gigabit Ethernet | 10GbE or Wi-Fi 6 | High-bandwidth communication for distributed systems |

### Humanoid Robot Platforms Comparison
| Platform | DOF | Height | Weight | Price Range | Key Features | Use Case |
|----------|-----|--------|--------|-------------|--------------|----------|
| **Unitree G1** | 32 | 1.1m | 32kg | $100K-200K | RTX-required GPU, high-speed locomotion | Research & Development |
| **Unitree H1** | 23 | 1.3m | 47kg | $160K-200K | Optimized for human-like walking | Research & Development |
| **Figure 02** | 23+ | 1.75m | ~60kg | $200K+ | Advanced manipulation, human interaction | Industrial & Research |
| **Agility Digit** | 20+ | 1.7m | 75kg | $200K+ | Bipedal walking, object manipulation | Industrial & Research |
| **Boston Dynamics Atlas** | 28 | 1.75m | 80kg | $750K+ | Advanced mobility, manipulation | Research (Limited availability) |
| **Tesla Optimus Gen 2** | TBD | 1.73m | 57kg | TBD | Vision-based control, dexterity | Research (Prototype) |

### Sensor Specifications for Humanoid Robotics
| Sensor Type | Model | Purpose | Accuracy | Interface | Price Range | Notes |
|-------------|-------|---------|----------|-----------|-------------|-------|
| **IMU** | VectorNav VN-300 | High-precision navigation | ±0.25° heading | RS-232/Ethernet | $2K-5K | Critical for balance and orientation |
| **Camera** | Intel RealSense D455 | Depth sensing & SLAM | Sub-cm at 1-4m | USB 3.2 | $150-300 | RGB-D capabilities |
| **LiDAR** | Ouster OS0-64 | 3D mapping & navigation | ±2-3cm | Ethernet | $8K-12K | High-resolution 3D scanning |
| **Force/Torque** | ATI Gamma | Manipulation feedback | &lt;1% of full scale | EtherCAT | $3K-8K | Essential for safe interaction |
| **GPS** | Swift Navigation Duro | Outdoor localization | &lt;1m | Serial/Ethernet | $2K-4K | For outdoor navigation |
| **Stereo Camera** | ZED 2i | Depth & tracking | Sub-cm to 20m | USB 3.0 | $400-500 | Extended range depth sensing |
| **Thermal Camera** | FLIR Lepton 3.5 | Heat signature detection | 160×120 resolution | SPI/I2C | $200-400 | Environmental awareness |
| **ToF Camera** | Intel RealSense L515 | Indoor depth sensing | mm accuracy indoors | USB-C | $200-400 | Low-power depth sensing |

### Computing Platforms for Humanoid Control
| Platform | GPU | CPU | RAM | Storage | Power | Use Case |
|----------|-----|-----|-----|---------|-------|----------|
| **Jetson Orin AGX** | RTX-class GPU | 12-core ARM | 32GB LPDDR5 | 64GB eMMC | 60W | Embedded humanoid control |
| **Jetson Orin NX** | RTX-class GPU | 8-core ARM | 8GB LPDDR4x | 16GB eMMC | 25W | Lightweight humanoid control |
| **Jetson Orin Nano** | RTX-class GPU | 6-core ARM | 4GB LPDDR4x | 16GB eMMC | 15W | Basic humanoid functionality |
| **Desktop Workstation** | RTX 4080 16GB | i7-13700K | 32GB DDR5 | 1TB NVMe | 1000W+ | Development & simulation |
| **NUC 12 Pro** | Arc GPU | i7-1260P | 16GB LPDDR4x | 512GB NVMe | 45W | Compact humanoid brain |
| **UP Xtreme i11** | Iris Xe | i7-1165G7 | 16GB LPDDR4x | 256GB NVMe | 28W | Edge computing for humanoid |

### Actuator Specifications
| Actuator Type | Model | Torque | Speed | Weight | Price | Use Case |
|---------------|-------|--------|-------|--------|-------|----------|
| **Dynamixel Pro** | XL430-W250-T | 2.5 N·m | 47 RPM | 76g | $150-200 | Lightweight joints |
| **Dynamixel Pro+** | XM430-W350-R | 3.5 N·m | 44 RPM | 76g | $200-250 | Medium-load joints |
| **Herkitux DRS-0101** | DRS-0101 | 2.0 N·m | 114 RPM | 56g | $80-100 | Budget option |
| **LewanSoul LX-224HV** | LX-224HV | 2.0 N·m | 0.22 sec/60° | 75g | $30-40 | Basic humanoid joints |
| **Trossen Robotics W1-AT01** | W1-AT01 | 18.0 N·m | 180 deg/s | 0.77kg | $1500-2000 | High-torque joints |
| **Robotis Dynamixel Pro L** | L54-30-S500-R | 30.0 N·m | 180 deg/s | 1.2kg | $1200-1500 | Heavy-duty joints |
| **Faulhaber Servo** | Various | Various | Various | Various | $500-5000 | Precision actuators |
| **Maxon EC-i** | Various | Various | Various | Various | $800-3000 | High-performance actuators |

### Network Infrastructure Requirements
| Component | Specification | Purpose | Cost |
|-----------|---------------|---------|------|
| **Managed Switch** | 8-port, 1Gbps, PoE | Robot networking | $200-400 |
| **Wireless AP** | Wi-Fi 6, 1Gbps uplink | Mobile robot connectivity | $300-600 |
| **Industrial Router** | VPN, firewall, QoS | Secure robot communication | $500-1000 |
| **Ethernet Cables** | Cat 6a, shielded | Reliable connections | $5-20/meter |
| **Network Analyzer** | Port mirroring, QoS | Network performance monitoring | $200-500 |

### Power Systems
| Component | Specification | Purpose | Cost |
|-----------|---------------|---------|------|
| **UPS** | 1000VA+, Pure sine | Uninterruptible power for critical systems | $200-500 |
| **Power Distribution** | 12V/24V, multiple outputs | Safe power distribution | $100-300 |
| **Battery Pack** | LiPo 22.2V 10000mAh | Mobile robot power | $150-300 |
| **Charging System** | Smart charger, balancing | Safe battery charging | $50-150 |
| **DC-DC Converter** | 24V to 12V/5V | Multiple voltage rails | $20-50 |

## Performance Benchmarks by Hardware Configuration

### Simulation Performance
| Configuration | Platform | Isaac Sim RTF | Gazebo Performance | Training Speedup | Use Case |
|---------------|----------|---------------|-------------------|------------------|----------|
| **Entry Level** | RTX 3060 12GB | 0.7 RTF | 1.0x real-time | 100x | Development, Testing |
| **Standard** | RTX 4070 12GB | 0.9 RTF | 1.2x real-time | 500x | Research, Development |
| **High-End** | RTX 4080 16GB | 1.0+ RTF | 1.5x real-time | 1000x | Production, Advanced Research |
| **Professional** | RTX 6000 Ada 48GB | 1.2+ RTF | 2.0x real-time | 2000x | Industrial, Advanced Research |

### Real-Time Performance
| Component | Minimum | Target | Achieved by |
|-----------|---------|--------|-------------|
| **Perception Latency** | &lt;100ms | &lt;60ms | Isaac ROS + RTX GPU |
| **Control Loop Rate** | 100Hz | 200Hz+ | Optimized control code + RT kernel |
| **Navigation Response** | &lt;500ms | &lt;200ms | Optimized path planning |
| **Voice Recognition** | &lt;2s | &lt;1s | Whisper + RTX acceleration |
| **LLM Response** | &lt;5s | &lt;2s | Optimized models + GPU |

### Memory and Storage Requirements
| System Component | RAM Usage | Storage Requirement | Notes |
|------------------|-----------|---------------------|-------|
| **Isaac Sim** | 8-16GB | 50GB+ | Depends on scene complexity |
| **Isaac Lab Training** | 16-32GB | 100GB+ | For model storage and datasets |
| **ROS 2 Runtime** | 2-4GB | 10GB | For node operations |
| **Perception Pipeline** | 4-8GB | 50GB | For sensor processing |
| **LLM Integration** | 8-16GB | 20GB | Model loading and inference |
| **Navigation Stack** | 1-2GB | 5GB | Map storage and path planning |

## Safety and Compliance Standards

### Hardware Safety Standards
| Standard | Description | Requirement | Compliance Method |
|----------|-------------|-------------|-------------------|
| **ISO 13482** | Personal care robots safety | Electrical, mechanical, data safety | Certification testing |
| **ISO 12100** | Machinery safety | Risk assessment and safety design | Design review |
| **IEC 60950-1** | IT equipment safety | Electrical safety requirements | Component certification |
| **UL 1012** | Control equipment | Safety for industrial controls | Testing and certification |
| **CSA C22.2 No. 142** | Process control equipment | Industrial safety standards | Compliance testing |

### Robot-Specific Certifications
| Certification | Purpose | Target Platform | Renewal Period |
|---------------|---------|-----------------|----------------|
| **CE Marking** | European conformity | All commercial robots | When design changes |
| **FCC Part 15** | US radio frequency | Wireless robots | Per product variant |
| **RoHS** | Restriction of hazardous substances | All electronic components | Per component batch |
| **IP Rating** | Ingress protection | Outdoor/harsh environment robots | Per design |
| **ATEX** | Explosive atmosphere | Hazardous environment robots | Per application |

## Cost Analysis for Different Implementations

### Research Lab Setup (Basic)
| Component Category | Item | Quantity | Unit Cost | Total Cost |
|-------------------|------|----------|-----------|------------|
| **Development Workstation** | RTX 4070 + i7-13700K | 1 | $2500 | $2500 |
| **Simulation License** | Isaac Sim Pro | 1 | $15000 | $15000 |
| **Robot Platform** | Unitree G1 | 1 | $150000 | $150000 |
| **Sensors** | Complete set (LiDAR, cameras, IMU) | 1 | $10000 | $10000 |
| **Networking** | Managed switch, cables | 1 | $500 | $500 |
| **Safety Equipment** | Emergency stops, barriers | 1 | $2000 | $2000 |
| **Total** | | | | **$179,500** |

### Research Lab Setup (Advanced)
| Component Category | Item | Quantity | Unit Cost | Total Cost |
|-------------------|------|----------|-----------|------------|
| **High-End Workstation** | RTX 6000 Ada + Threadripper | 2 | $6000 | $12000 |
| **Isaac Platform Licenses** | Complete suite | 2 | $30000 | $60000 |
| **Multiple Robot Platforms** | 2x Unitree H1 | 2 | $180000 | $360000 |
| **Advanced Sensors** | Multi-modal sensor suite | 2 | $20000 | $40000 |
| **Networking Infrastructure** | Industrial networking | 1 | $3000 | $3000 |
| **Safety & Compliance** | Complete safety system | 1 | $5000 | $5000 |
| **Total** | | | | **$479,000** |

### Educational Setup (University Course)
| Component Category | Item | Quantity | Unit Cost | Total Cost |
|-------------------|------|----------|-----------|------------|
| **Student Workstations** | 10x RTX 4060 + i5 | 10 | $1500 | $15000 |
| **Robot Platforms** | 5x Unitree G1 | 5 | $150000 | $750000 |
| **Shared Sensors** | Complete lab kit | 1 | $25000 | $25000 |
| **Networking & Infrastructure** | Lab networking setup | 1 | $5000 | $5000 |
| **Software Licenses** | Isaac Sim Edu licenses | 10 | $3000 | $30000 |
| **Safety Equipment** | Lab safety setup | 1 | $10000 | $10000 |
| **Total** | | | | **$835,000** |

## Maintenance and Lifecycle Considerations

### Hardware Maintenance Schedule
| Component | Maintenance Interval | Typical Tasks | Estimated Cost/Year |
|-----------|---------------------|---------------|-------------------|
| **GPU** | 6 months | Dust cleaning, thermal paste | $100-200 |
| **Robot Actuators** | 3 months | Lubrication, calibration | $500-1000 |
| **Sensors** | Monthly | Calibration, cleaning | $200-500 |
| **Workstation** | 3 months | Cleaning, updates | $100-200 |
| **Networking** | Quarterly | Firmware updates, checks | $100-300 |
| **Batteries** | 6 months | Cycle testing, replacement | $200-800 |

### Upgrade Pathways
| Component | Current Cycle | Next Upgrade | Upgrade Benefit |
|-----------|---------------|--------------|-----------------|
| **GPU** | RTX 4080 | RTX 5080/6080 | 2-3x performance improvement |
| **Robot Platform** | Unitree G1 | Next gen humanoid | New capabilities, better performance |
| **Isaac Platform** | Isaac Lab 2.3 | Next major release | New features, better optimization |
| **ROS 2** | Humble | Jazzy/Iron | New features, security updates |
| **Sensors** | Current gen | Next gen with AI | Better accuracy, embedded processing |

### Deprecation and End-of-Life Planning
- **GPU Support**: NVIDIA typically provides 3-5 years of driver support
- **Robot Platforms**: 5-7 years typical lifecycle for research platforms
- **Software Platforms**: ROS 2 LTS releases supported for 5 years
- **Sensors**: 3-4 years typical product lifecycle
- **Safety Standards**: Updates every 2-3 years, compliance tracking needed