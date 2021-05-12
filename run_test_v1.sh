export M5_PATH=/home/nepuko/research/gem5_share

gem5_dir=/home/nepuko/research/gem5
kernel=/home/nepuko/research/gem5_share/binaries/vmlinux.vexpress_gem5_v1_64
disk=/home/nepuko/research/gem5_share/disks/mcn_aarch64.img
exe=$gem5_dir/build/ARM/gem5.opt
#dtb_host=/home/nepuko/research/gem5_share/binaries/armv8_gem5_v1_1cpu.dtb

BOOT_SCRIPT=/home/nepuko/research/gem5/simple-lspci.rcS

#debug="--debug-flags=Cache,DMA,DRAM,CacheVerbose,CacheRepl,CachePort,CacheTags,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar"
debug="--debug-flags=GICV2M,Ethernet,PciHost,PciDevice"
#debug="--debug-flags=Cache,DMA,DRAM,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar"
#debug="--debug-start=496512600 --debug-flags=Cache,DMA,DRAM,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar,CacheVerbose,CacheRepl,CachePort,O3CPU,Fetch,Commit,LSQ,BaseXBar,MemoryAccess,PacketQueue,SnoopFilter"
#debug="--debug-flags=IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,DMA"
#debug="--debug-flags=Interrupt,GIC,GICV2M,Ethernet,PciHost,PciDevice"
cpu=AtomicSimpleCPU
#cpu=ex5_big

#opts="--ddio --caches --l2cache --num-cpus=1 --dual --cpu-clock=10GHz --sys-clock=2GHz --mem-type=DDR4_2400_4x16"
opts="--num-cpus=1 --cpu-type=$cpu --caches"
#opts="--ddio --caches --l2cache --num-cpus=1 --cpu-clock=10GHz --sys-clock=2GHz"

#$exe --outdir $outdir -re $debug $gem5_dir/configs/example/fs.py --disk-image=$disk --kernel=$kernel  --machine-type=VExpress_GEM5_V1 --dtb-filename=$dtb_host --mem-size=256MB $opts --script=$BOOT_SCRIPT 

outdir=test_out3

$exe --outdir $outdir -re $debug $gem5_dir/configs/example/fs.py --disk-image=$disk --kernel=$kernel --machine-type=VExpress_GEM5_V1 --dtb-filename=$dtb_host --mem-size=4GB $opts --script=$BOOT_SCRIPT
