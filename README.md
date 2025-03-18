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
# For Ubuntu 20.04
cd <pNet-gem5 root>/gem5
scons build/ARM/gem5.opt -j$(nproc)
```

### Building the kernel
The NIC in pNet-gem5 requires a modified e1000 based driver.
We modified Linux to control the NIC of pNet-gem5 and use the MSI controller of pNet-gem5. In order to run pNet-gem5 in a full-system environment, we need to use the kernel we modified.
The provided config has options pre-set for using pNet-gem5.
To use the NIC of pNet-gem5, you need to set the following options.
- CONFIG_PCI_MSI=y
- CONFIG_GENERIC_MSI_IRQ=y
- CONFIG_NEPU1000=y/m (m recommended)
- CONFIG_E1000=n/m (m recommended)
Do not load both e1000 driver and nepu1000 driver. Please load the one that matches the device you are going to use.
```bash
# move to linux path
cd <pNet-gem5 root>/linux
./build.sh
```

## To be written
### Resources


### Running gem5

```bash
```
