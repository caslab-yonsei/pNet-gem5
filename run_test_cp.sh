export M5_PATH=/home/jshin/research/mQ-gem5/mQ-gem5/gem5/system/arm/
#M5_PATH=/home/jshin/research/moodum/binaries/
ls $M5_PATH
gem5_dir=/home/jshin/research/mQ-gem5/mQ-gem5/gem5
#kernel=/home/nepuko/research/gem5_share/binaries/vmlinux.aarch64.20140821
kernel=/home/jshin/research/mQ-gem5/mQ-gem5/linux/vmlinux
disk=/home/jshin/research/moodum/disks/ubuntu.img
exe=$gem5_dir/build/ARM/gem5.opt
#exe=$gem5_dir/build/ARM/gem5.fast
dtb_host=/home/jshin/research/mQ-gem5/mQ-gem5/gem5/system/arm/dt/armv8_gem5_v1_4cpu.dtb
#dtb_host=/home/nepuko/research/gem5_share/binaries/armv8_gem5_v1_1cpu.dtb

BOOT_SCRIPT=/home/jshin/research/mQ-gem5/mQ-gem5/cp_test_scripts/test/single-ckpt.rcS
BOOT_SCRIPT=/home/jshin/research/mQ-gem5/mQ-gem5/cp_test_scripts/test/driverloader.rcS

#debug="--debug-flags=Cache,DMA,DRAM,CacheVerbose,CacheRepl,CachePort,CacheTags,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar"
#debug="--debug-flags=Cache,DMA,DRAM,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar"
#debug="--debug-start=496512600 --debug-flags=Cache,DMA,DRAM,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar,CacheVerbose,CacheRepl,CachePort,O3CPU,Fetch,Commit,LSQ,BaseXBar,MemoryAccess,PacketQueue,SnoopFilter"
#debug="--debug-flags=IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,DMA"
#debug="--debug-flags=Interrupt,GIC,GICV2M,Ethernet,PciHost,PciDevice"
#debug="--debug-flags=GICV2M,NepNicOthers,NepNicIntrMsi,EthernetIntr,EthernetSM,NepNicIntr "
cpu=AtomicSimpleCPU
#cpu=ex5_big

#opts="--ddio --caches --l2cache --num-cpus=1 --dual --cpu-clock=10GHz --sys-clock=2GHz --mem-type=DDR4_2400_4x16"
opts="--num-cpus=4 --cpu-type=$cpu" # --caches"
#opts="--ddio --caches --l2cache --num-cpus=1 --cpu-clock=10GHz --sys-clock=2GHz"

#$exe --outdir $outdir -re $debug $gem5_dir/configs/example/fs.py --disk-image=$disk --kernel=$kernel  --machine-type=VExpress_GEM5_V1 --dtb-filename=$dtb_host --mem-size=256MB $opts --script=$BOOT_SCRIPT 

outdir=rundir/atomicboot

$exe --outdir $outdir -re $debug $gem5_dir/configs/example/fs.py --disk-image=$disk --kernel=$kernel --machine-type=VExpress_GEM5_V1 --dtb-filename=$dtb_host --mem-size=256MB $opts --script=$BOOT_SCRIPT --checkpoint-restore=1 #--caches
