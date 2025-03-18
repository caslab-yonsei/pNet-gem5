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

### pNet-gem5 Kernel
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

### Other Resources
You will need the usual resources for a full system simulation: Linux kernel (requires kernel for pNet-gem5), boot binary (included with gem5), DTB (for pNet-gem5), disk image. You 
#### DTB and binaries
can build the required binaries and DTB as follows:
```bash
# Build DTB.
# If If device-tree-compiler(dtc) is not installed, install it first.
# Ubuntu
sudo apt install dtc

# Build DTB
cd <pNet-gem5 root>/gem5/system/arm/dt
make

# Build binaries
cd <pNet-gem5 root>/gem5/system/arm/binaries
make
```
#### Disk Image
The disk image is not terribly important, but to avoid unexpected problems, it is recommended to use an image with a somewhat older version of Ubuntu installed.
You can create the disk image yourself, but you can also download the image from [gem5_guest_binaries](https://www.gem5.org/documentation/general_docs/fullsystem/guest_binaries).
Or you can download the latest images from [gem5-resources](https://resources.gem5.org/).

### Config gem5
Instead of IGbE_e1000, which is the NIC model of gem5, use NepGbE_base.
Here is an example config:
```python
self.realview.ethernet = NepGbE_base(pci_bus=0, pci_dev=0, pci_func=0,
                    num_of_queues=4, num_msi_engine = 4, MSICAPMsgCtrl = 0xFBFB,
                    port_specific=port_specific_, dist_rank=dist_rank)
```

### Running gem5
It's not much different from running dist-gem5. For a detailed example, see the run script we provided.
