# pNet-gem5: Full-System Simulation with High-Performance Networking Enabled by Parallel Network Packet Processing

## Overview
This project is based on the [gem5](https://www.gem5.org/) simulator. This repository provides an enhancement to the full-system simulator, gem5. The modification introduces support for multi-queue Network Interface Cards (NICs), allowing improved packet processing and parallelism within gem5's full-system simulation.

## Features
- **Multi-Queue NIC Support**: Enables gem5 to utilize NICs with multiple RX/TX queues, improving network packet processing efficiency.
- **Enhanced Packet Distribution**: Allows packets to be processed in parallel across multiple queues, reducing bottlenecks.
- **Flexible Configuration**: Supports custom queue mappings and scheduling policies to optimize simulation scenarios.

## Installation & Setup
It is built based on gem5, and installation and execution are almost the same as gem5.
### Prerequisites
- Recommended (and tested) operating system: Ubuntu 20.04
- Required dependencies: [Official Docs](https://www.gem5.org/documentation/general_docs/building) 

### Building the Simulator
```bash
# Clone the repository
git clone https://github.com/caslab-yonsei/pNet-gem5.git
cd pNet-gem5

# Install dependencies
# Please visit https://www.gem5.org/documentation/general_docs/building

# Build gem5 (ARM only)
# For Ubuntu 18.04
scons build/ARM/gem5.opt -j$(nproc)
```

## To be written
### Running gem5
```bash
```
