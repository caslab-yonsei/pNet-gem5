#! /bin/bash

#
# Copyright (c) 2015 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Gabor Dozsa
#
#
# This is an example script to start a dist gem5 simulations using
# two AArch64 systems. It is also uses the example
# dist gem5 bootscript util/dist/test/simple_bootscript.rcS that will
# run the linux ping command to check if we can see the peer system
# connected via the simulated Ethernet link.
export M5_PATH=/home/caslab/bst/IISWC/mQ-gem5/
GEM5_DIR=/home/caslab/bst/IISWC/mQ-gem5/gem5

IMG=/home/caslab/bst/IISWC/mQ-gem5/ubuntu.img
#VMLINUX=/home/nepuko/research/gem5_share/binaries/vmlinux.vexpress_gem5_v1
VMLINUX=/home/caslab/bst/IISWC/mQ-gem5//linux/vmlinux
DTB=/home/caslab/bst/IISWC/mQ-gem5/gem5/system/arm/dt/armv8_gem5_v1_$1cpu.dtb

FS_CONFIG=$GEM5_DIR/configs/example/fs.py
SW_CONFIG=$GEM5_DIR/configs/dist/sw.py
GEM5_EXE=$GEM5_DIR/build/ARM/gem5.opt


#BOOT_SCRIPT="/media/nepuko/MOODUM/rss-run/simple-test-4core-ckp.rcS"
#BOOT_SCRIPT="/home/ssibal/MOODUM/rss-run/iperf4.rcS"
BOOT_SCRIPT="/home/caslab/bst/IISWC/mQ-gem5/script/memcached_base.rcS"
GEM5_DIST_SH=$GEM5_DIR/util/dist/memcached.sh

#DEBUG_FLAGS="--debug-flags=GICV2M,EthernetIntr,GIC,NepNicOthers " #"--debug-flags=DistEthernet"
#CHKPT_RESTORE= "--dist-sync-start=1000000 " #"-r1"

NNODES=2
#DEBUG_FLAGS="--debug-flags=MMU,GICV2M,NepMsi,NepNicIntr,NepNicOthers,NepNicRxManager,NepNicIntrMsi,PciDevice,MSIXBar,Ethernet,EthernetIntr "
l3size=$(expr $1 "*" 4)
CKPTDIR=$(pwd)/../ckpt/ckpt_$1core_$2_16G 
RUNDIR=$(pwd)/memcached_$1core_base_RSS
echo $RUNDIR
# -r $RUN_SCRIPT   -c $RUN_SCRIPT                                \
$GEM5_DIST_SH -n $NNODES                                                     \
              -x $GEM5_EXE                                                   \
              -c $CKPTDIR                                                    \
              -r $RUNDIR                                                     \
              -s $SW_CONFIG                                                  \
              -f $FS_CONFIG                                                  \
              --m5-args                                                      \
                 $DEBUG_FLAGS                                                \
              --fs-args                                                      \
	      --cpu-type=ex5_big --cpu-clock=4GHz   --caches   --l2cache --l3cache --l3_size=${l3size}MB        \
                  --mem-type=DDR4_2400_4x16 --ethernet-linkdelay=100ns --ethernet-linkspeed=99Gbps --mem-channels=4  \
		  --num-cpus=$1                                               \
                  --machine-type=VExpress_GEM5_V1                            \
                  --disk-image=$IMG                                          \
                  --kernel=$VMLINUX                                          \
                  --dtb-filename=$DTB                                        \
                  --script=$BOOT_SCRIPT                                      \
                  --mem-size=16GB   --num-nep-rx-q=0                                \
              --cf-args                                                      \
                  $CHKPT_RESTORE --dist-sync-start=1000000t --checkpoint-restore=1

