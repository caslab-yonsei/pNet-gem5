export M5_PATH=/home/nepuko/research/gem5_share

gem5_dir=/home/nepuko/research/gem5-land/gem5-msi-rss-neo/gem5
#kernel=/home/nepuko/research/gem5_share/binaries/vmlinux.aarch64.20140821
kernel=/home/nepuko/research/gem5-land/gem5-msi-rss-neo/linux/vmlinux
#kernel=/home/nepuko/research/gem5-land/new-linux/linux/vmlinux
disk=/home/nepuko/research/gem5_share/disks/aarch64-ubuntu-trusty-headless-nep.img
#disk=/media/nepuko/MOODUM/mcn_aarch64.img
exe=$gem5_dir/build/ARM/gem5.opt
#exe=$gem5_dir/build/ARM/gem5.fast
#dtb_host=/home/nepuko/research/gem5-land/gem5-msi-rss-neo/gem5/system/arm/dt/armv8_gem5_v1_1cpu.dtb
#dtb_host=/home/nepuko/research/gem5-land/mlc-prefetcher-ddio/gem5/system/arm/dt/testsys.dtb
#dtb_host=/home/nepuko/research/gem5_share/binaries/armv8_gem5_v1_2cpu.dtb
#dtb_host=/home/nepuko/research/gem5-land/gem5-msi-rss-neo/test_out_newlinux/test.dtb

BOOT_SCRIPT=/home/nepuko/research/gem5_share/simple_bootscript.rcS

#debug="--debug-flags=Cache,DMA,DRAM,CacheVerbose,CacheRepl,CachePort,CacheTags,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar"
#debug="--debug-flags=Cache,DMA,DRAM,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar"
#debug="--debug-start=496512600 --debug-flags=Cache,DMA,DRAM,IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,CoherentXBar,CacheVerbose,CacheRepl,CachePort,O3CPU,Fetch,Commit,LSQ,BaseXBar,MemoryAccess,PacketQueue,SnoopFilter"
#debug="--debug-flags=IOCtrl,IOCtrlPort,IOCtrlVerbose,Ethernet,DMA"
#debug="--debug-flags=Interrupt,GIC,GICV2M,Ethernet,PciHost,PciDevice"
#debug="--debug-flags=GICV2M,NepMsi,Interrupt,GIC "
debug="--debug-flags=GICV2M,NepMsi,NepNicIntr "
cpu=AtomicSimpleCPU
#cpu=TimingSimpleCPU
#cpu=ex5_big

#opts="--ddio --caches --l2cache --num-cpus=1 --dual --cpu-clock=10GHz --sys-clock=2GHz --mem-type=DDR4_2400_4x16"
opts="--num-cpus=1 --cpu-type=$cpu --caches --l2cache --l3cache"
#cp_opts="-r1 "
cp_opts2="--restore-with-cpu=AtomicSimpleCPU "
#opts="--ddio --caches --l2cache --num-cpus=1 --cpu-clock=10GHz --sys-clock=2GHz"

#$exe --outdir $outdir -re $debug $gem5_dir/configs/example/fs.py --disk-image=$disk --kernel=$kernel  --machine-type=VExpress_GEM5_V1 --dtb-filename=$dtb_host --mem-size=256MB $opts --script=$BOOT_SCRIPT 

outdir=test_out_newlinux
#outdir=test_out_ooo8

$exe --outdir $outdir -re $debug $gem5_dir/configs/example/fs.py --disk-image=$disk --kernel=$kernel --machine-type=VExpress_GEM5_V1 --dtb-filename=$dtb_host --mem-size=256MB --num-nep-rx-q=4 $opts $cp_opts

# insmod /nepu1000/nepu1000.ko
# /sbin/ifconfig eth0 hw ether 00:90:00:00:00:02
# /sbin/ifconfig eth0 192.168.0.3 netmask 255.255.255.0 up
# /sbin/ifconfig -a
# cat /proc/interrupts