/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Ali Saidi
 */

/* @file
 * Device model for Intel's 8254x line of gigabit ethernet controllers.
 * In particular an 82547 revision 2 (82547GI) MAC because it seems to have the
 * fewest workarounds in the driver. It will probably work with most of the
 * other MACs with slight modifications.
 */

#include "dev/net/nep_GBe.hh"

/*
 * @todo really there are multiple dma engines.. we should implement them.
 */

#include <algorithm>
#include <memory>

// NEPU TEST MSI
#include <cstdlib>

#include "base/inet.hh"
#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/EthernetAll.hh"

#include "debug/PciDevice.hh"

#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/NepGbE.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

// SHIN.
// Use Policy RSS
#include "nep_GBe_Policy_RSS.hh"

// For Debug RSS
#include "debug/NepNicOthers.hh"
#include "debug/NepNicRxManager.hh"
#include "debug/NepNicIntrMsi.hh"
#include "debug/NepNicIntr.hh"

#include "debug/NepTx.hh"

// SHIN.

using namespace iGbReg;
using namespace Net;


NepGbE::NepGbE(const Params &p)
    : EtherDevice(p), etherInt(NULL),
      txFifo(p.tx_fifo_size), inTick(false),
      txTick(false), txFifoTick(false), rxDmaPacket(false),
      pktOffset(0), fetchDelay(p.fetch_delay), wbDelay(p.wb_delay),
      fetchCompDelay(p.fetch_comp_delay), wbCompDelay(p.wb_comp_delay),
      rxWriteDelay(p.rx_write_delay), txReadDelay(p.tx_read_delay),
      rdtrEvent([this]{ rdtrProcess(); }, name()),
      radvEvent([this]{ radvProcess(); }, name()),
      tadvEvent([this]{ tadvProcess(); }, name()),
      tidvEvent([this]{ tidvProcess(); }, name()),
      tickEvent([this]{ tick(); }, name()),
      loadGenEvent([this]{ SimpleLoad(); }, name()),
      interEvent([this]{ delayIntEvent(); }, name()),
      rxDescCache(this, name()+".RxDesc", p.rx_desc_cache_size),
      txDescCache(this, name()+".TxDesc", p.tx_desc_cache_size),
      mqManager(this, p.num_of_queues, p.rx_fifo_size, p.rx_desc_cache_size,
                      p.num_of_queues, p.tx_fifo_size, p.tx_desc_cache_size),       // SHIN
      lastInterrupt(0)
{
    etherInt = new NepGbEInt(name() + ".int", this);
    num_of_queues = p.num_of_queues;
    port_specific = p.port_specific;

    // Initialized internal registers per Intel documentation
    // All registers intialized to 0 by per register constructor
    regs.ctrl.fd(1);
    regs.ctrl.lrst(1);
    regs.ctrl.speed(2);
    regs.ctrl.frcspd(1);
    regs.sts.speed(3); // Say we're 1000Mbps
    regs.sts.fd(1); // full duplex
    regs.sts.lu(1); // link up
    regs.eecd.fwe(1);
    regs.eecd.ee_type(1);
    regs.imr = 0;
    regs.iam = 0;
    regs.rxdctl.gran(1);
    regs.rxdctl.wthresh(1);
    regs.fcrth(1);
    regs.tdwba = 0;
    regs.rlpml = 0;
    regs.sw_fw_sync = 0;

    regs.pba.rxa(0x30);
    regs.pba.txa(0x10);

    regs.num_enabled_queues = p.num_of_queues;

    // Init NEPU Regs
    for(int i = 0; i < 32; i++){
        // RX
        regs.nep_ex_regs[i].rxdctl = 0;
        regs.nep_ex_regs[i].rxdctl.gran(1);
        regs.nep_ex_regs[i].rxdctl.wthresh(1);
        regs.nep_ex_regs[i].rctl = 0;
        regs.nep_ex_regs[i].rdba = 0;
        regs.nep_ex_regs[i].rdlen = 0;
        regs.nep_ex_regs[i].rdh = 0;
        regs.nep_ex_regs[i].rdt = 0;
        regs.nep_ex_regs[i].rdtr = 0;
        regs.nep_ex_regs[i].srrctl = 0;
        regs.nep_ex_regs[i].radv = 0;
        regs.nep_ex_regs[i].rsrpd = 0;
        regs.nep_ex_regs[i].rxcsum = 0;
        regs.nep_ex_regs[i].rlpml = 0;
        regs.nep_ex_regs[i].rfctl = 0;

        // TX
        regs.nep_ex_regs[i].tctl = 0;
        regs.nep_ex_regs[i].tdba = 0;
        regs.nep_ex_regs[i].tdlen = 0;
        regs.nep_ex_regs[i].tdh = 0;
        regs.nep_ex_regs[i].tdt = 0;
        regs.nep_ex_regs[i].txdctl = 0;
        regs.nep_ex_regs[i].tadv = 0;
        regs.nep_ex_regs[i].tipg = 0;

        // MSI
        regs.nep_ex_regs[i].mqicr = 1;

        // SHIN. For Test. It must be modified by host.(driver)
        regs.nep_ex_regs[i].imr = 0xFFFFFFFF;
        regs.nep_ex_regs[i].iam = 0;

        regs.nep_ex_regs[i].itr = 0; //TEMP
    }

    // SHIN. For test. It must be modified by host.(driver)
    regs.imr = 0xFFFFFFFF;
    //msicap.ma=0x2c1c0040;
    

    eeOpBits            = 0;
    eeAddrBits          = 0;
    eeDataBits          = 0;
    eeOpcode            = 0;

    // clear all 64 16 bit words of the eeprom
    memset(&flash, 0, EEPROM_SIZE*2);

    // Set the MAC address
    memcpy(flash, p.hardware_address.bytes(), ETH_ADDR_LEN);
    for (int x = 0; x < ETH_ADDR_LEN/2; x++)
        flash[x] = htobe(flash[x]);

    uint16_t csum = 0;
    for (int x = 0; x < EEPROM_SIZE; x++)
        csum += htobe(flash[x]);


    // Magic happy checksum value
    flash[EEPROM_SIZE-1] = htobe((uint16_t)(EEPROM_CSUM - csum));

    // Store the MAC address as queue ID
    macAddr = p.hardware_address;

    //rxFifo.clear();
    txFifo.clear();


    // Todo get val from options
    int num_q = p.num_of_queues;
    for(int i = 0; i < num_q; i++)
    {
        // int queue = i;
        // uint64_t workload = APP_HASH;
        // uint64_t rps = 1000000;
        // uint64_t rps_for_warmup = 1000000;
        // uint64_t warmuptime_ns = 10000;
        // string name_ = name() + "loadgenerator" + to_string(i);
        // string wcname = "";
        // string kvname = "";
        // string hashname = "";
        // EtherDevice* dev = this;
        // EtherInt *Int = etherInt; 

    //LoadGenerator(int queue, uint64_t workload, uint64_t rps_, uint64_t rps_for_warmup, 
    //string name, string wcname, string kvname,  string hashname, 
    //EtherDevice* dev_, EtherInt* eint_);
        // LoadGenerator* loadgenerator = new LoadGenerator(queue, workload, rps, rps_for_warmup, warmuptime_ns, name_, wcname, kvname, hashname, dev, Int);
        // load_generators.push_back(loadgenerator);
        // dma_engines.push_back(new MultiDmaEngine((MultiDmaEngineParams*)p, this, "dma_engine." + to_string(i)));
        // msi_engines.push_back(new MultiDmaEngine((MultiDmaEngineParams*)p, this, "msi_engine." + to_string(i)));
    }
    //dma_engines.push_back(new DmaEngine(p, this, "ff." + to_string(1)));
    //new MultiDmaEngine((MultiDmaEngineParams*)p, this, "ff." + to_string(1));
    
    // SHIN. For simple test
    //schedule(tickEvent, clockEdge(Cycles(1000)));
    //schedule(loadGenEvent, clockEdge(Cycles(10000)));
    //schedule(loadGenEvent, curTick() + 1*SimClock::Int::s);

    //DPRINTF(NepNicOthers, "Test PCI config val %x\n", p->MSICAPBaseOffset);
}

NepGbE::~NepGbE()
{
    delete etherInt;
}

void
NepGbE::init()
{
    PciDevice::init();
}

Port &
NepGbE::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "interface")
        return *etherInt;
    else if (if_name == "msiport_devside"){
        return *msi_engines_ports[idx];
    }
    // for(int i = 0; i < msi_engines.size(); i++){
    //     if(if_name == "msi" + std::to_string(i))
    //         return (msi_engines[i]->getPort("msi", idx));
    // }
    // for(int i = 0; i < dma_engines.size(); i++){
    //     if(if_name == "dma" + std::to_string(i))
    //         return (msi_engines[i]->getPort("dma", idx));
    // }
    return EtherDevice::getPort(if_name, idx);
}

#define PCI_DEVICE_SPECIFIC_RSS             0xFF    // 192 bytes

Tick
NepGbE::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC_RSS){
        PciDevice::writeConfig(pkt);
        
        // For MSI controll
        bool enable = msicap.mc & 0x1; // mc: 0th bit is msie
        if(enable)
        {
            if(etherInt->set_int_mode(INT_MSI)!=0)
            {panic("NEPU CANNOT SET MSI!\n");}
            else
            {
                DPRINTF(EthernetIntr, "NEPU MSI. SET INTMODE MSI\n");
                
            }
        }
        // To do Control MSI-X
        else
        {
            if(etherInt->set_int_mode(INT_LEGACY)!=0)
            {panic("NEPU CANNOT SET LEGACY INT!\n");}
            else
            {
                DPRINTF(EthernetIntr, "NEPU MSI. SET INTMODE LEGACY\n");
            }
        }
    }
    else{
        /**
         * Tour Capability island!
         * 이 영역은 Capability 영역으로 추정이 됩니다. ARM!
         * 그니까 그거에 맞게 돌아야 합니다. ARM!
         */
        DPRINTF(EthernetIntr, "Tour Capability island!\n");
        panic("Device specific PCI config space not implemented.\n");
    }

    //sendingMSI();

    //
    // Some work may need to be done here based for the pci COMMAND bits.
    //

    return configDelay;
}

// Handy macro for range-testing register access addresses
#define IN_RANGE(val, base, len) (val >= base && val < (base + len))

Tick
NepGbE::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    // SHIN. NEP_NIC Multi Q support
    
    // For example regs.nep_rx_regs[qid].radv
    

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    int qid = addrToQid(daddr);
    //qid = 0;
    daddr = daddr % (uint64_t)REG_END;
    assert(bar == 0);

    // Only 32bit accesses allowed
    assert(pkt->getSize() == 4);

    DPRINTF(Ethernet, "Read device register %#X\n", daddr);

    //
    // Handle read of register here
    //


    switch (daddr) {
      case REG_CTRL:
        pkt->setLE<uint32_t>(regs.ctrl());
        break;
      case REG_STATUS:
        pkt->setLE<uint32_t>(regs.sts());
        break;
      case REG_EECD:
        pkt->setLE<uint32_t>(regs.eecd());
        break;
      case REG_EERD:
        pkt->setLE<uint32_t>(regs.eerd());
        break;
      case REG_CTRL_EXT:
        pkt->setLE<uint32_t>(regs.ctrl_ext());
        break;
      case REG_MDIC:
        pkt->setLE<uint32_t>(regs.mdic());
        break;
      case REG_ICR:
        DPRINTF(Ethernet, "Reading ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                regs.icr(), regs.imr, regs.iam, regs.ctrl_ext.iame());
        pkt->setLE<uint32_t>(regs.icr());
        if (regs.icr.int_assert() || regs.imr == 0) {
            regs.icr = regs.icr() & ~mask(30);
            DPRINTF(Ethernet, "Cleared ICR. ICR=%#x\n", regs.icr());
        }
        if (regs.ctrl_ext.iame() && regs.icr.int_assert())
            regs.imr &= ~regs.iam;
        chkInterrupt();
        break;
      case REG_EICR:
        // This is only useful for MSI, but the driver reads it every time
        // Just don't do anything
        pkt->setLE<uint32_t>(0);
        break;

      // SHIN.
      case REG_MQICR:       // Only MultiQueue
        if(qid > -1)
        {
            DPRINTF(NepNicIntrMsi, "Reading MQICR(%d). MQICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                    qid, regs.nep_ex_regs[qid].mqicr(), regs.nep_ex_regs[qid].imr, 
                    regs.nep_ex_regs[qid].iam, regs.nep_ex_regs[qid].ctrl_ext.iame());
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].mqicr());
            if (regs.nep_ex_regs[qid].mqicr.int_assert() || regs.nep_ex_regs[qid].imr == 0) {
                regs.nep_ex_regs[qid].mqicr = regs.nep_ex_regs[qid].mqicr() & ~mask(30);
                DPRINTF(NepNicIntrMsi, "Cleared MQICR(%d). MQICR=%#x\n", 
                                qid, regs.nep_ex_regs[qid].mqicr());
            }
            if (regs.nep_ex_regs[qid].ctrl_ext.iame() && regs.nep_ex_regs[qid].mqicr.int_assert())
                regs.nep_ex_regs[qid].imr &= ~regs.nep_ex_regs[qid].iam;
            mqManager.chkMultiMSIs();
            mqManager.getCompByIdx(qid).chkMultiMSI();
        }
        break;

      case REG_ITR:
        if (qid > -1)
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].itr());
        else
            pkt->setLE<uint32_t>(regs.itr());
        break;
      case REG_RCTL:
        if(qid > -1){
            DPRINTF(NepNicOthers, "read. REG_RCTL start\n");
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rctl());
            DPRINTF(NepNicOthers, "read. REG_RCTL succ\n");
            //schedule(loadGenEvent, curTick() + 50*SimClock::Int::us);
        }
        else{
            DPRINTF(NepNicOthers, "read. REG_RCTL(Not MQ)\n");
            pkt->setLE<uint32_t>(regs.rctl());
        }
        break;
      case REG_FCTTV:
        pkt->setLE<uint32_t>(regs.fcttv());
        break;
      case REG_TCTL:
        if(qid < 0){
            DPRINTF(NepNicOthers, "read. REG_TCTL(Not MQ)\n");
            pkt->setLE<uint32_t>(regs.tctl());
        } else {
            DPRINTF(NepNicOthers, "read. REG_TCTL(MQ) qid %d\n", qid);
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tctl());
        }
        break;
      case REG_PBA:
        pkt->setLE<uint32_t>(regs.pba());
        break;
      case REG_WUC:
      case REG_WUFC:
      case REG_WUS:
      case REG_LEDCTL:
        pkt->setLE<uint32_t>(0); // We don't care, so just return 0
        break;
      case REG_FCRTL:
        pkt->setLE<uint32_t>(regs.fcrtl());
        break;
      case REG_FCRTH:
        pkt->setLE<uint32_t>(regs.fcrth());
        break;
      case REG_RDBAL:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rdba.rdbal());
        }
        else {
            pkt->setLE<uint32_t>(regs.rdba.rdbal());
        }
        break;
      case REG_RDBAH:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rdba.rdbah());
        }else{
            pkt->setLE<uint32_t>(regs.rdba.rdbah());
        }
        break;
      case REG_RDLEN:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rdlen());
        } else {
            pkt->setLE<uint32_t>(regs.rdlen());
        }
        break;
      case REG_SRRCTL:
        //pkt->setLE<uint32_t>(regs.srrctl());
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].srrctl());
        }else{
            pkt->setLE<uint32_t>(regs.srrctl());
        }
        break;
      case REG_RDH:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rdh());
        } else {
            pkt->setLE<uint32_t>(regs.rdh());
        }
        break;
      case REG_RDT:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rdt());
        } else {
            pkt->setLE<uint32_t>(regs.rdt());
        }
        break;
      case REG_RDTR:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rdtr());
            if (regs.nep_ex_regs[qid].rdtr.fpd()) {
                mqManager.rxDescCache_Wb(qid);//rxDescCache.writeback(0);
                DPRINTF(NepNicIntr,
                        "Posting interrupt because of RDTR.FPD write\n");
                postInterrupt_MSI(IT_RXT, qid);
                regs.nep_ex_regs[qid].rdtr.fpd(0);
            }
        } else {
            pkt->setLE<uint32_t>(regs.rdtr());
            if (regs.rdtr.fpd()) {
                rxDescCache.writeback(0);
                DPRINTF(EthernetIntr,
                        "Posting interrupt because of RDTR.FPD write\n");
                postInterrupt(IT_RXT);
                regs.rdtr.fpd(0);
            }
        }
        break;
      case REG_RXDCTL:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rxdctl());
        } else {
            pkt->setLE<uint32_t>(regs.rxdctl());
        }
        break;
      case REG_RADV:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].radv());
        } else {
            pkt->setLE<uint32_t>(regs.radv());
        }
        break;
      case REG_TDBAL:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tdba.tdbal());
        } else {
            pkt->setLE<uint32_t>(regs.tdba.tdbal());
        }
        break;
      case REG_TDBAH:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tdba.tdbah());
        } else {
            pkt->setLE<uint32_t>(regs.tdba.tdbah());
        }
        break;
      case REG_TDLEN:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tdlen());
        } else {
            pkt->setLE<uint32_t>(regs.tdlen());
        }
        break;
      case REG_TDH:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tdh());
        } else {
            pkt->setLE<uint32_t>(regs.tdh());
        }
        break;
      case REG_TXDCA_CTL:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].txdca_ctl());
        } else {
            pkt->setLE<uint32_t>(regs.txdca_ctl());
        }
        break;
      case REG_TDT:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tdt());
        } else {
            pkt->setLE<uint32_t>(regs.tdt());
        }
        break;
      case REG_TIDV:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tidv());
        } else {
            pkt->setLE<uint32_t>(regs.tidv());
        }
        break;
      case REG_TXDCTL:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].txdctl());
        } else {
            pkt->setLE<uint32_t>(regs.txdctl());
        }
        break;
      case REG_TADV:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tadv());
        } else {
            pkt->setLE<uint32_t>(regs.tadv());
        }
        break;
      case REG_TDWBAL:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tdwba & mask(32));
        } else {
            pkt->setLE<uint32_t>(regs.tdwba & mask(32));
        }
        break;
      case REG_TDWBAH:
        if(qid > -1){
            // TODO
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].tdwba >> 32);
        } else {
            pkt->setLE<uint32_t>(regs.tdwba >> 32);
        }
        break;
      case REG_RXCSUM:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rxcsum());
        } else {
            pkt->setLE<uint32_t>(regs.rxcsum());
        }
        break;
      case REG_RLPML:
        if(qid > -1){
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rlpml);
        } else {
            pkt->setLE<uint32_t>(regs.rlpml);
        }
        break;
      case REG_RFCTL:
        if (qid > -1) {
            pkt->setLE<uint32_t>(regs.nep_ex_regs[qid].rfctl());
        } else {
            pkt->setLE<uint32_t>(regs.rfctl());
        }
        break;
      case REG_MANC:
        if (qid > -1) {
            // TODO?
        } else {
            pkt->setLE<uint32_t>(regs.manc());
        }
        break;
      case REG_SWSM:
        if (qid > -1) {
            // TODO?
        } else {
            pkt->setLE<uint32_t>(regs.swsm());
            regs.swsm.smbi(1);
        }
        break;
      case REG_FWSM:
        if (qid > -1) {
            // TODO?
        } else {
            pkt->setLE<uint32_t>(regs.fwsm());
        }
        break;
      case REG_SWFWSYNC:
        if (qid > -1) {
            // TODO?
        } else {
            pkt->setLE<uint32_t>(regs.sw_fw_sync);
        }
        break;
      case REG_CTRL_MSI_TEST:
        pkt->setLE<uint32_t>(regs.ctrl_msi_test());
        break;
      case REG_LOAD_TEST:
        break;
      case REG_NUM_ENABLED_QUEUES:
        pkt->setLE<uint32_t>(regs.num_enabled_queues);
        break;
      default:
        if (!IN_RANGE(daddr, REG_VFTA, VLAN_FILTER_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_RAL, RCV_ADDRESS_TABLE_SIZE*8) &&
            !IN_RANGE(daddr, REG_MTA, MULTICAST_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_CRCERRS, STATS_REGS_SIZE))
            panic("Read request to unknown register number: %#x\n", daddr);
        else
            pkt->setLE<uint32_t>(0);
    };

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
NepGbE::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;
    

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");
    int qid = addrToQid(daddr);
    daddr = daddr % (uint64_t)REG_END;
    DPRINTF(NepNicOthers, "Write Reg daddr %#llx, Qid %d\n", daddr, qid);
    //qid = 0;
    // Only Memory register BAR is allowed
    assert(bar == 0);

    // Only 32bit accesses allowed
    assert(pkt->getSize() == sizeof(uint32_t));

    DPRINTF(Ethernet, "Wrote device register %#X value %#X\n",
            daddr, pkt->getLE<uint32_t>());

    // for simple test
    // sendingMSI();

    //
    // Handle write of register here
    //
    uint32_t val = pkt->getLE<uint32_t>();

    Regs::RCTL oldrctl;
    Regs::TCTL oldtctl;

    switch (daddr) {
      case REG_CTRL:
        regs.ctrl = val;
        if (regs.ctrl.tfce())//상의필요
            warn("TX Flow control enabled, should implement\n");
        if (regs.ctrl.rfce())
            warn("RX Flow control enabled, should implement\n");
        break;
      case REG_CTRL_EXT:
        regs.ctrl_ext = val;
        break;
      case REG_STATUS:
        regs.sts = val;
        break;
      case REG_EECD:
        int oldClk;
        oldClk = regs.eecd.sk();
        regs.eecd = val;
        // See if this is a eeprom access and emulate accordingly
        if (!oldClk && regs.eecd.sk()) {
            if (eeOpBits < 8) {
                eeOpcode = eeOpcode << 1 | regs.eecd.din();
                eeOpBits++;
            } else if (eeAddrBits < 8 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
                eeAddr = eeAddr << 1 | regs.eecd.din();
                eeAddrBits++;
            } else if (eeDataBits < 16 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
                assert(eeAddr>>1 < EEPROM_SIZE);
                DPRINTF(EthernetEEPROM, "EEPROM bit read: %d word: %#X\n",
                        flash[eeAddr>>1] >> eeDataBits & 0x1,
                        flash[eeAddr>>1]);
                regs.eecd.dout((flash[eeAddr>>1] >> (15-eeDataBits)) & 0x1);
                eeDataBits++;
            } else if (eeDataBits < 8 && eeOpcode == EEPROM_RDSR_OPCODE_SPI) {
                regs.eecd.dout(0);
                eeDataBits++;
            } else
                panic("What's going on with eeprom interface? opcode:"
                      " %#x:%d addr: %#x:%d, data: %d\n", (uint32_t)eeOpcode,
                      (uint32_t)eeOpBits, (uint32_t)eeAddr,
                      (uint32_t)eeAddrBits, (uint32_t)eeDataBits);

            // Reset everything for the next command
            if ((eeDataBits == 16 && eeOpcode == EEPROM_READ_OPCODE_SPI) ||
                (eeDataBits == 8 && eeOpcode == EEPROM_RDSR_OPCODE_SPI)) {
                eeOpBits = 0;
                eeAddrBits = 0;
                eeDataBits = 0;
                eeOpcode = 0;
                eeAddr = 0;
            }

            DPRINTF(EthernetEEPROM, "EEPROM: opcode: %#X:%d addr: %#X:%d\n",
                    (uint32_t)eeOpcode, (uint32_t) eeOpBits,
                    (uint32_t)eeAddr>>1, (uint32_t)eeAddrBits);
            if (eeOpBits == 8 && !(eeOpcode == EEPROM_READ_OPCODE_SPI ||
                                   eeOpcode == EEPROM_RDSR_OPCODE_SPI ))
                panic("Unknown eeprom opcode: %#X:%d\n", (uint32_t)eeOpcode,
                      (uint32_t)eeOpBits);


        }
        // If driver requests eeprom access, immediately give it to it
        regs.eecd.ee_gnt(regs.eecd.ee_req());
        break;
      case REG_EERD:
        regs.eerd = val;
        if (regs.eerd.start()) {
            regs.eerd.done(1);
            assert(regs.eerd.addr() < EEPROM_SIZE);
            regs.eerd.data(flash[regs.eerd.addr()]);
            regs.eerd.start(0);
            DPRINTF(EthernetEEPROM, "EEPROM: read addr: %#X data %#x\n",
                    regs.eerd.addr(), regs.eerd.data());
        }
        break;
      case REG_MDIC:
        regs.mdic = val;
        if (regs.mdic.i())
            panic("No support for interrupt on mdic complete\n");
        if (regs.mdic.phyadd() != 1)
            panic("No support for reading anything but phy\n");
        DPRINTF(Ethernet, "%s phy address %x\n",
                regs.mdic.op() == 1 ? "Writing" : "Reading",
                regs.mdic.regadd());
        switch (regs.mdic.regadd()) {
          case PHY_PSTATUS:
            regs.mdic.data(0x796D); // link up
            break;
          case PHY_PID:
            regs.mdic.data(params().phy_pid);
            break;
          case PHY_EPID:
            regs.mdic.data(params().phy_epid);
            break;
          case PHY_GSTATUS:
            regs.mdic.data(0x7C00);
            break;
          case PHY_EPSTATUS:
            regs.mdic.data(0x3000);
            break;
          case PHY_AGC:
            regs.mdic.data(0x180); // some random length
            break;
          default:
            regs.mdic.data(0);
        }
        regs.mdic.r(1);
        break;
      case REG_ICR:
        DPRINTF(Ethernet, "Writing ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                regs.icr(), regs.imr, regs.iam, regs.ctrl_ext.iame());
        if (regs.ctrl_ext.iame())
            regs.imr &= ~regs.iam;
        regs.icr = ~bits(val,30,0) & regs.icr();
        chkInterrupt();
        break;

      // TODO
      case REG_MQICR:
        DPRINTF(NepNicIntrMsi, "Writing MQICR. MQICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                regs.nep_ex_regs[qid].mqicr(), regs.nep_ex_regs[qid].imr, 
                regs.nep_ex_regs[qid].iam, regs.nep_ex_regs[qid].ctrl_ext.iame());
        if (regs.nep_ex_regs[qid].ctrl_ext.iame())
            regs.nep_ex_regs[qid].imr &= ~regs.nep_ex_regs[qid].iam;
        regs.nep_ex_regs[qid].mqicr = ~bits(val,30,0) & regs.nep_ex_regs[qid].mqicr();
        //rxManager.chkMultiMSIs();
        mqManager.getCompByIdx(qid).chkMultiMSI();
        break;
      case REG_ITR:
        if (qid > -1)
            regs.nep_ex_regs[qid].itr = val / 4;
        else
            regs.itr = val;
        break;
      case REG_ICS:
        if(qid < 0)
        {
            DPRINTF(EthernetIntr, "Posting interrupt because of ICS write\n");
            postInterrupt((IntTypes)val);
            for(int i = 0; i < mqManager.getNumTxQueues(); i++)
                postInterrupt_MSI((IntTypes)val, i);
        }
        else
        {
            DPRINTF(EthernetIntr, "Posting interrupt because of ICS write\n");
            postInterrupt_MSI((IntTypes)val, qid);
        }
        
        break;
      case REG_IMS:
        if (qid < 0){
            regs.imr |= val;
            chkInterrupt();
        } else {
            regs.nep_ex_regs[qid].imr |= val;
            mqManager.getCompByIdx(qid).chkMultiMSI();
        }
        break;
      case REG_IMC:
        if (qid > -1){
            regs.nep_ex_regs[qid].imr &= ~val;
            mqManager.getCompByIdx(qid).chkMultiMSI();}
        else{
            regs.imr &= ~val;
            chkInterrupt();}
        break;
      case REG_IAM:
        if (qid > -1)
            regs.nep_ex_regs[qid].iam = val;
        else
            regs.iam = val;
        break;
      case REG_RCTL:
        if (qid > -1) {
            DPRINTF(NepNicOthers, "write. REG_RCTL start\n");
            

            // TODO NEP
            regs.nep_ex_regs[qid].rctl = val;
            if (regs.nep_ex_regs[qid].rctl.rst()) {
                mqManager.rxDescCache_reset(qid);//rxDescCache.reset();
                //DPRINTF(EthernetSM, "RXS: Got RESET!\n");
                mqManager.getCompByIdx(qid).rxfifo.clear();
                //rxFifo.clear();
                regs.nep_ex_regs[qid].rctl.rst(0);
            }
            if (regs.nep_ex_regs[qid].rctl.en())
                mqManager.setIdxRxTick(qid,true);//rxTick = true;
            restartClock();
            DPRINTF(NepNicOthers, "write. REG_RCTL succ\n");
        }
        else{
            // oldrctl = regs.rctl;
            // regs.rctl = val;
            DPRINTF(NepNicOthers, "write. REG_RCTL(Not MQ)\n");
            // if (regs.rctl.rst()) {
            //     rxDescCache.reset();
                DPRINTF(EthernetSM, "RXS: Got RESET!\n");
                // rxFifo.clear();
            //     mqManager.getCompByIdx(0).rxfifo.clear();
            //     regs.rctl.rst(0);
            // }
            // if (regs.rctl.en()){
            //     //rxTick = true;
            //     mqManager.setIdxRxTick(0,true);//rxTick = true;
            // }
            // restartClock();
        }
        break;
      case REG_FCTTV:
        regs.fcttv = val;
        break;
      case REG_TCTL:
        if (qid < 0)
        {
            DPRINTF(NepNicOthers, "write. REG_TCTL(Not MQ)\n");

            // regs.tctl = val;
            // oldtctl = regs.tctl;
            // regs.tctl = val;
            // if (regs.tctl.en())
            //     txTick = true;
            // restartClock();
            // if (regs.tctl.en() && !oldtctl.en()) {
            //     txDescCache.reset();
            // }
        }
        else{
            // TODO
            DPRINTF(NepNicOthers, "write. REG_TCTL(MQ) qid %d\n", qid);
            regs.nep_ex_regs[qid].tctl = val;
            oldtctl = regs.nep_ex_regs[qid].tctl;
            regs.nep_ex_regs[qid].tctl = val;
            if (regs.nep_ex_regs[qid].tctl.en())
                mqManager.setIdxTxTick(qid, true);
                //txTick = true;
            restartClock();
            if (regs.nep_ex_regs[qid].tctl.en() && !oldtctl.en()) {
                //txDescCache.reset();
                mqManager.txDescCache_reset(qid);
            }
        }
        
        break;
      case REG_PBA:
        regs.pba.rxa(val);
        regs.pba.txa(64 - regs.pba.rxa());
        break;
      case REG_WUC:
      case REG_WUFC:
      case REG_WUS:
      case REG_LEDCTL:
      case REG_FCAL:
      case REG_FCAH:
      case REG_FCT:
      case REG_VET:
      case REG_AIFS:
      case REG_TIPG:
        ; // We don't care, so don't store anything
        break;
      case REG_IVAR0:
        warn("Writing to IVAR0, ignoring...\n");
        break;
      case REG_FCRTL:
        regs.fcrtl = val;
        break;
      case REG_FCRTH:
        regs.fcrth = val;
        break;
      case REG_RDBAL:
        if (qid < 0){
            regs.rdba.rdbal( val & ~mask(4));
            rxDescCache.areaChanged();
        }
        else{
            regs.nep_ex_regs[qid].rdba.rdbal( val & ~mask(4));
            mqManager.rxIdxAreaChanged(qid);//rxDescCache.areaChanged();
        }
        break;
      case REG_RDBAH:
        if (qid < 0){
            regs.rdba.rdbal( val & ~mask(4));
            rxDescCache.areaChanged();
        } else {
            regs.nep_ex_regs[qid].rdba.rdbah(val);
            mqManager.rxIdxAreaChanged(qid);//rxDescCache.areaChanged();
        }
        break;
      case REG_RDLEN:
        if (qid < 0){
            regs.rdlen = val & ~mask(7);
            rxDescCache.areaChanged();
        } else {
            regs.nep_ex_regs[qid].rdlen = val & ~mask(7);
            mqManager.rxIdxAreaChanged(qid);//rxDescCache.areaChanged();
        }
        break;
      case REG_SRRCTL:
        if (qid < 0){
            regs.srrctl = val;
        } else {
            regs.nep_ex_regs[qid].srrctl = val;
            DPRINTF(NepNicOthers, "write. REG_SRRCTL qid %d, val %d\n", qid, val);
        }
        break;
      case REG_RDH:
        if (qid < 0)
        {
            regs.rdh = val;
            rxDescCache.areaChanged();
        } else {
            regs.nep_ex_regs[qid].rdh = val;
            mqManager.rxIdxAreaChanged(qid);//rxDescCache.areaChanged();
        }
        break;
      case REG_RDT:
        if (qid < 0) {
            regs.rdt = val;
            DPRINTF(EthernetSM, "RXS: LEGACY RDT Updated.\n");
            if (drainState() == DrainState::Running) {
                DPRINTF(EthernetSM, "RXS: LEGACY RDT Fetching Descriptors!\n");
                rxDescCache.fetchDescriptors();
            } else {
                DPRINTF(EthernetSM, "RXS: LEGACY RDT NOT Fetching Desc b/c draining!\n");
            }
        } else {
            regs.nep_ex_regs[qid].rdt = val;
            DPRINTF(EthernetSM, "RXS: RDT Updated. For Q %d, val %#x\n", qid, val);
            if (drainState() == DrainState::Running) {
                DPRINTF(EthernetSM, "RXS: RDT Fetching Descriptors!\n");
                mqManager.rxDescCache_fetch(qid);//rxDescCache.fetchDescriptors();
            } else {
                DPRINTF(EthernetSM, "RXS: RDT NOT Fetching Desc b/c draining!\n");
            }
        }
        break;
      case REG_RDTR:
        if (qid < 0) {
            regs.rdtr = val;
        } else {
            regs.nep_ex_regs[qid].rdtr = val;
        }
        break;
      case REG_RADV:
        if (qid < 0) {
            regs.radv = val;
        } else {
            regs.nep_ex_regs[qid].radv = val;
        }
        break;
      case REG_RXDCTL:
        if (qid < 0) {
            regs.rxdctl = val;
        } else {
            regs.nep_ex_regs[qid].rxdctl = val;
        }
        break;
      case REG_TDBAL:
        if (qid < 0) {
            regs.tdba.tdbal( val & ~mask(4));
            txDescCache.areaChanged();
        } else {
            // TODO
            regs.nep_ex_regs[qid].tdba.tdbal(val & ~mask(4));
            mqManager.txIdxAreaChanged(qid);
        }
        break;
      case REG_TDBAH:
        if (qid < 0) {
            regs.tdba.tdbah(val);
            txDescCache.areaChanged();
        } else {
            regs.nep_ex_regs[qid].tdba.tdbah(val);
            mqManager.txIdxAreaChanged(qid);
        }
        break;
      case REG_TDLEN:
        if (qid < 0) {
            regs.tdlen = val & ~mask(7);
            txDescCache.areaChanged();
        } else {
            regs.nep_ex_regs[qid].tdlen = val & ~mask(7);
            mqManager.txIdxAreaChanged(qid);
        }
        break;
      case REG_TDH:
        if (qid < 0) {
            regs.tdh = val;
            txDescCache.areaChanged();
        } else {
            regs.nep_ex_regs[qid].tdh = val;
            mqManager.txIdxAreaChanged(qid);
        }
        break;
      case REG_TXDCA_CTL:
        if (qid < 0) {
            regs.txdca_ctl = val;
            if (regs.txdca_ctl.enabled())
                panic("No support for DCA\n");
        } else {
            regs.nep_ex_regs[qid].txdca_ctl = val;
            if (regs.nep_ex_regs[qid].txdca_ctl.enabled())
                panic("No support for DCA\n");
        }
        break;
      case REG_TDT:
        if (qid < 0) {
            regs.tdt = val;
            DPRINTF(EthernetSM, "TXS: TX LEGACY Tail pointer updated\n");
            if (drainState() == DrainState::Running) {
                DPRINTF(EthernetSM, "TXS: LEGACY TDT Fetching Descriptors!\n");
                txDescCache.fetchDescriptors();
            } else {
                DPRINTF(EthernetSM, "TXS: LEGACY TDT NOT Fetching Desc b/c draining!\n");
            }
        } else {
            regs.nep_ex_regs[qid].tdt = val;
            DPRINTF(EthernetSM, "TXS: TX Tail pointer updated\n");
            if (drainState() == DrainState::Running) {
                DPRINTF(EthernetSM, "TXS: TDT Fetching Descriptors!\n");
                //txDescCache.fetchDescriptors();
                mqManager.txDescCache_fetch(qid);
            } else {
                DPRINTF(EthernetSM, "TXS: TDT NOT Fetching Desc b/c draining!\n");
            }
        }
        break;
      case REG_TIDV:
        if (qid < 0) {
            regs.tidv = val;
        } else {
            regs.nep_ex_regs[qid].tidv = val;
        }
        break;
      case REG_TXDCTL:
        if (qid < 0) {
            regs.txdctl = val;
        } else {
            regs.nep_ex_regs[qid].txdctl = val;
        }
        break;
      case REG_TADV:
        if (qid < 0) {
            regs.tadv = val;
        } else {
            regs.nep_ex_regs[qid].tadv = val;
        }
        break;
      case REG_TDWBAL:
        if (qid < 0) {
            regs.tdwba &= ~mask(32);
            regs.tdwba |= val;
            txDescCache.completionWriteback(regs.tdwba & ~mask(1),
                                            regs.tdwba & mask(1));
        } else {
            regs.nep_ex_regs[qid].tdwba &= ~mask(32);
            regs.nep_ex_regs[qid].tdwba |= val;
            mqManager.txDescCache_completionWriteback(qid, regs.tdwba & ~mask(1),
                                                      regs.tdwba & mask(1));
        }
        break;
      case REG_TDWBAH:
        if (qid < 0) {
            regs.tdwba &= mask(32);
            regs.tdwba |= (uint64_t)val << 32;
            txDescCache.completionWriteback(regs.tdwba & ~mask(1),
                                            regs.tdwba & mask(1));
        } else {
            regs.nep_ex_regs[qid].tdwba &= mask(32);
            regs.nep_ex_regs[qid].tdwba |= (uint64_t)val << 32;
            mqManager.txDescCache_completionWriteback(qid, regs.tdwba & ~mask(1),
                                                      regs.tdwba & mask(1));
        }
        break;
      case REG_RXCSUM:
        if (qid < 0) {
            regs.rxcsum = val;
        } else {
            regs.nep_ex_regs[qid].rxcsum = val;
        }
        break;
      case REG_RLPML:
        if (qid < 0) {
            regs.rlpml = val;
        } else {
            regs.nep_ex_regs[qid].rlpml = val;
        }
        break;
      case REG_RFCTL:
        if (qid < 0) {
            regs.rfctl = val;
            if (regs.rfctl.exsten())  
                panic("Extended RX descriptors not implemented\n");
        } else {
            regs.nep_ex_regs[qid].rfctl = val;
            if (regs.nep_ex_regs[qid].rfctl.exsten())  
                panic("Extended RX descriptors not implemented\n");
        }
        break;
      case REG_MANC:
        regs.manc = val;
        break;
      case REG_SWSM:
        regs.swsm = val;
        if (regs.fwsm.eep_fw_semaphore())
            regs.swsm.swesmbi(0);
        break;
      case REG_SWFWSYNC:
        regs.sw_fw_sync = val;
        break;
      case REG_CTRL_MSI_TEST:       // NEPU RSS NIC
        DPRINTF(EthernetIntr, "CASE REG_CTRL_MSI_TEST\n");
        regs.ctrl_msi_test = val;
        // for just test...
        if (regs.ctrl_msi_test.interrupt_set())
        {
            sendingMSI(regs.ctrl_msi_test.irq());
        }
        else
        {
            clearMSI(regs.ctrl_msi_test.irq());
        }
        
        break;
      default:
        if (!IN_RANGE(daddr, REG_VFTA, VLAN_FILTER_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_RAL, RCV_ADDRESS_TABLE_SIZE*8) &&
            !IN_RANGE(daddr, REG_MTA, MULTICAST_TABLE_SIZE*4))
            panic("Write request to unknown register number: %#x\n", daddr);
    };

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
NepGbE::postInterrupt(IntTypes t, bool now)
{
    //postInterrupt_MSI(t, now);
    if(etherInt->get_int_mode() == INT_LEGACY){
        postInterrupt_LEGACY(t, now);
    }
    else if (etherInt->get_int_mode() == INT_MSI){
        postInterrupt_MSI(t, -1, now);
    }
    else{
        panic("Not implemented interrupt type!");
    }
    //sendingMSI(0);
}

void
NepGbE::postMqInterrupt(IntTypes t, int queue, bool now)
{
    // If queue < 0. Not multi-queue
    if(queue < 0){
        postInterrupt(t, now);
        return;
    }
    if(etherInt->get_int_mode() == INT_LEGACY){
        postInterrupt_LEGACY(t, now);
    }
    else if (etherInt->get_int_mode() == INT_MSI){
        postInterrupt_MSI(t, queue, now);
    }
    else{
        panic("Not implemented interrupt type!");
    }
    //sendingMSI(0);
}

void
NepGbE::postInterrupt_MSI(IntTypes t, int queue, bool now)
{
    // Send Interrupts via MSI
    
    // Check TYPE
    assert(t);

    // Interrupt is already pending [Current int_type is already pending]
    DPRINTF(NepNicIntrMsi,
            "EINT: postInterrupt_MSI() IntType %x\n", t);
    

    // Mq Handle
    if (queue < 0){
        //return;

        if (t & regs.icr() && !now)
            return;
        regs.icr = regs.icr() | t;
        
        // To do Cal interval
        Tick itr_interval = SimClock::Int::ns * 256 * regs.itr.interval();

        DPRINTF(NepNicIntrMsi,
                "EINT: postInterrupt_MSI() curTick(): %d itr: %d interval: %d, IntType %x\n",
                curTick(), regs.itr.interval(), itr_interval, t);


        // TODO.
        // interEvent Cannot handle MSI schedule! So we have to fix or create a new event for MSI intr.
        if (regs.itr.interval() == 0 || now ||
            lastInterrupt + itr_interval <= curTick()) {
            if (interEvent.scheduled()) {
                DPRINTF(NepNicIntrMsi, "NepGbE::postInterrupt_MSI interEvent is scheduled!!!\n");
                deschedule(interEvent);
            }
            cpuPostInt();
            DPRINTF(NepNicIntrMsi, "NepGbE::postInterrupt_MSI Post Int!!!\n");
        } else {
            Tick int_time = lastInterrupt + itr_interval;
            assert(int_time > 0);
            DPRINTF(NepNicIntrMsi, "EINT: Scheduling timer interrupt(MSI) for tick %d\n",
                    int_time);
            if (!interEvent.scheduled()) {
                schedule(interEvent, int_time);
            }
        }
    }
    else
    {
        if (t & regs.nep_ex_regs[queue].mqicr() && !now)
            return;
        regs.nep_ex_regs[queue].mqicr = regs.nep_ex_regs[queue].mqicr() | t;

        MultiQueueComponentSet& comp = mqManager.getCompByIdx(queue);

        // To do Cal interval
        Tick itr_interval = SimClock::Int::ns * 256 * regs.nep_ex_regs[queue].itr.interval();

        DPRINTF(NepNicIntrMsi,
                "EINT: postInterrupt_MSI() curTick(): %d itr: %d interval: %d, IntType %x\n",
                curTick(), regs.nep_ex_regs[queue].itr.interval(), itr_interval, t);


        // TODO.
        // interEvent Cannot handle MSI schedule! So we have to fix or create a new event for MSI intr.
        if (regs.nep_ex_regs[queue].itr.interval() == 0 || now ||
            comp.lastMultInterrupt + itr_interval <= curTick()) {
            if (comp.interMultEvent.scheduled()) {
                DPRINTF(NepNicIntrMsi, "NepGbE::postInterrupt_MSI interEvent is scheduled!!!\n");
                deschedule(comp.interMultEvent);
            }
            comp.cpuPostMultInt();
            DPRINTF(NepNicIntrMsi, "NepGbE::postInterrupt_MSI Post Int!!!\n");
        } else {
            Tick int_time = comp.lastMultInterrupt + itr_interval;
            assert(int_time > 0);
            DPRINTF(NepNicIntrMsi, "EINT: Scheduling timer interrupt(MSI) for tick %d\n",
                    int_time);
            if (!comp.interMultEvent.scheduled()) {
                schedule(comp.interMultEvent, int_time);
            }
        }
    }
}

void
NepGbE::MultiQueueComponentSet::postInterruptMultIntr(iGbReg::IntTypes t, bool now)
{
    // Send Interrupts via MSI
    
    // Check TYPE
    assert(t);

    // Interrupt is already pending [Current int_type is already pending]
    DPRINTF(NepNicIntrMsi,
            "EINT: postInterruptMultIntr() IntType %x\n", t);
    if (t & igbe->regs.nep_ex_regs[idxComponent].mqicr() && !now){
        DPRINTF(NepNicIntrMsi,
            "EINT: postInterruptMultIntr() IntType %x not now\n", t);
        return;
    }

    else
    {
        igbe->regs.nep_ex_regs[idxComponent].mqicr = igbe->regs.nep_ex_regs[idxComponent].mqicr() | t;
    }
    

    // To do Cal interval
    Tick itr_interval = SimClock::Int::ns * 256 * igbe->regs.nep_ex_regs[idxComponent].itr.interval();

    DPRINTF(NepNicIntrMsi,
            "EINT: postInterruptMultIntr() curTick(): %d itr: %d interval: %d, IntType %x\n",
            curTick(), igbe->regs.nep_ex_regs[idxComponent].itr.interval(), itr_interval, t);


    // TODO.
    // interEvent Cannot handle MSI schedule! So we have to fix or create a new event for MSI intr.
    if (igbe->regs.nep_ex_regs[idxComponent].itr.interval() == 0 || now ||
        lastMultInterrupt + itr_interval <= curTick()) {
        if (interMultEvent.scheduled()) {
            DPRINTF(NepNicIntrMsi, "NepGbE::postInterruptMultIntr interEvent is scheduled!!!\n");
            igbe->deschedule(interMultEvent);
        }
        cpuPostMultInt();
        DPRINTF(NepNicIntrMsi, "NepGbE::postInterruptMultIntr Post Int!!!\n");
    } else {
        Tick int_time = lastMultInterrupt + itr_interval;
        //if(igbe->global_int_time>=int_time) int_time = igbe->global_int_time+1;
        //igbe->global_int_time = int_time;
        assert(int_time > 0);
        DPRINTF(NepNicIntrMsi, "EINT: Scheduling timer interrupt(MSI) for tick %d\n",
                int_time);
        if (!interMultEvent.scheduled()) {
            igbe->schedule(interMultEvent, int_time);
        }
    }
}

void
NepGbE::postInterrupt_LEGACY(IntTypes t, bool now)
{
    assert(t);

    // Interrupt is already pending
    if (t & regs.icr() && !now)
        return;

    regs.icr = regs.icr() | t;

    Tick itr_interval = SimClock::Int::ns * 256 * regs.itr.interval();
    DPRINTF(EthernetIntr,
            "EINT: postInterrupt_LEGACY() curTick(): %d itr: %d interval: %d\n",
            curTick(), regs.itr.interval(), itr_interval);

    if (regs.itr.interval() == 0 || now ||
        lastInterrupt + itr_interval <= curTick()) {
        if (interEvent.scheduled()) {
            deschedule(interEvent);
        }
        cpuPostInt();
    } else {
        Tick int_time = lastInterrupt + itr_interval;
        assert(int_time > 0);
        DPRINTF(EthernetIntr, "EINT: Scheduling timer interrupt(legacy) for tick %d\n",
                int_time);
        if (!interEvent.scheduled()) {
            schedule(interEvent, int_time);
        }
    }
}

void
NepGbE::delayIntEvent()
{
    cpuPostInt();
}


void
NepGbE::cpuPostInt()
{

    etherDeviceStats.postedInterrupts++;

    if (!(regs.icr() & regs.imr)) {
        DPRINTF(Ethernet, "Interrupt Masked. Not Posting\n");
        return;
    }

    DPRINTF(Ethernet, "Posting Interrupt\n");


    if (interEvent.scheduled()) {
        deschedule(interEvent);
    }

    if (rdtrEvent.scheduled()) {//논의필요
        regs.icr.rxt0(1);
        deschedule(rdtrEvent);
    }
    if (radvEvent.scheduled()) {
        regs.icr.rxt0(1);
        deschedule(radvEvent);
    }
    if (tadvEvent.scheduled()) {
        regs.icr.txdw(1);
        deschedule(tadvEvent);
    }
    if (tidvEvent.scheduled()) {
        regs.icr.txdw(1);
        deschedule(tidvEvent);
    }

    regs.icr.int_assert(1);
    //DPRINTF(EthernetIntr, "EINT: Posting interrupt to CPU now. Vector %#x\n",
    //        regs.icr());

    if(etherInt->get_int_mode() == INT_LEGACY)
    {
        DPRINTF(EthernetIntr, "EINT: Posting interrupt to CPU now. LEGACY Vector %#x\n",
            regs.icr());
        intrPost();
    }
    else if(etherInt->get_int_mode() == INT_MSI)
    {
        // To do... send MSI to All Cores!
        DPRINTF(EthernetIntr, "EINT: Posting interrupt to CPU now. MSI Vector %#x\n",
            regs.icr());
        //sendingMSI(0);
    }
    else{
        DPRINTF(EthernetIntr, "EINT: Not Posting interrupt to CPU now. ERR. Vector %#x\n",
            regs.icr());
    }
    lastInterrupt = curTick();
}

void
NepGbE::MultiQueueComponentSet::cpuPostMultInt()
{

    igbe->etherDeviceStats.postedInterrupts++;

    if (!(igbe->regs.nep_ex_regs[idxComponent].mqicr() & igbe->regs.nep_ex_regs[idxComponent].imr)) {
        DPRINTF(NepNicIntrMsi, "Interrupt Masked. mqicr %#x, imr %#x Not Posting\n",
        igbe->regs.nep_ex_regs[idxComponent].mqicr(), igbe->regs.nep_ex_regs[idxComponent].imr);
        // TODO Find imr in driver
        return;
    }

    DPRINTF(NepNicIntrMsi, "Posting Interrupt\n");


    if (interMultEvent.scheduled()) {
        igbe->deschedule(interMultEvent);
    }

    if (rdtrMultEvent.scheduled()) {//논의필요
        igbe->regs.nep_ex_regs[idxComponent].mqicr.rxt0(1);
        igbe->deschedule(rdtrMultEvent);
    }
    if (radvMultEvent.scheduled()) {
        igbe->regs.nep_ex_regs[idxComponent].mqicr.rxt0(1);
        igbe->deschedule(radvMultEvent);
    }
    if (tadvMultEvent.scheduled()) {
        igbe->regs.nep_ex_regs[idxComponent].mqicr.txdw(1);
        igbe->deschedule(tadvMultEvent);
    }
    if (tidvMultEvent.scheduled()) {
        igbe->regs.nep_ex_regs[idxComponent].mqicr.txdw(1);
        igbe->deschedule(tidvMultEvent);
    }

    //regs.icr.int_assert(1);
    igbe->regs.nep_ex_regs[idxComponent].mqicr.int_assert(1);
    //DPRINTF(EthernetIntr, "EINT: Posting interrupt to CPU now. Vector %#x\n",
    //        regs.icr());



    // To do... send MSI to All Cores!
    DPRINTF(NepNicIntrMsi, "EINT: Posting interrupt Q %d to CPU now. MSI Vector %#x\n",
        idxComponent,
        igbe->regs.nep_ex_regs[idxComponent].mqicr());
    igbe->sendingMSI(idxComponent+1);

    lastMultInterrupt = curTick();
    if(igbe->global_int_time<lastMultInterrupt)igbe->global_int_time=lastMultInterrupt;
}


void
NepGbE::cpuClearInt()
{
    if (regs.icr.int_assert()) {
        regs.icr.int_assert(0);
        DPRINTF(EthernetIntr,
                "EINT: Clearing interrupt to CPU now. Vector %#x\n",
                regs.icr());
        if(etherInt->get_int_mode() == INT_LEGACY){
            intrClear();
        }
        else if (etherInt->get_int_mode() == INT_MSI){
            // To do... select core num.
            clearMSI(0);
        }
    }
}

void
NepGbE::chkInterrupt()
{
    DPRINTF(Ethernet, "Checking interrupts icr: %#x imr: %#x\n", regs.icr(),
            regs.imr);
    // Check if we need to clear the cpu interrupt
    if (!(regs.icr() & regs.imr)) {
        DPRINTF(Ethernet, "Mask cleaned all interrupts\n");
        if (interEvent.scheduled())
            deschedule(interEvent);
        if (regs.icr.int_assert())
            cpuClearInt();
    }
    DPRINTF(Ethernet, "ITR = %#X itr.interval = %#X\n",
            regs.itr(), regs.itr.interval());

    if (regs.icr() & regs.imr) {
        if (regs.itr.interval() == 0)  {
            cpuPostInt();
        } else {
            DPRINTF(Ethernet,
                    "Possibly scheduling interrupt because of imr write\n");
            if (!interEvent.scheduled()) {
                Tick t = curTick() + SimClock::Int::ns * 256 * regs.itr.interval();
                DPRINTF(Ethernet, "Scheduling for %d\n", t);
                schedule(interEvent, t);
            }
        }
    }
    mqManager.chkMultiMSIs();
}

void
NepGbE::MultiQueueComponentSet::cpuClearMultiInt()
{
    iGbReg::Regs::NEP_EX_REGS& regs = igbe->regs.nep_ex_regs[idxComponent];
    if (regs.mqicr.int_assert()) {
        regs.mqicr.int_assert(0);
        DPRINTF(NepNicIntrMsi,
                "EINT: Clearing interrupt to CPU now. Vector %#x\n",
                regs.mqicr());

        igbe->clearMSI(idxComponent+1);
    }
}

void
NepGbE::MultiQueueManager::chkMultiMSIs()
{
    DPRINTF(NepNicIntrMsi, "chkMultiMSIs check All mq-intr\n");
    for(int i = 0; i<num_rx_queues; i++)
    {
        mq_component_sets[i]->chkMultiMSI();
    }
}

void 
NepGbE::MultiQueueComponentSet::chkMultiMSI()
{
    DPRINTF(NepNicIntrMsi, "chkMultiMSI idxComponent %d\n", idxComponent);
    iGbReg::Regs::NEP_EX_REGS& regs = igbe->regs.nep_ex_regs[idxComponent];

    DPRINTF(NepNicIntrMsi, "Checking interrupts mqicr: %#x imr: %#x\n", regs.mqicr(),
            regs.imr);
    // Check if we need to clear the cpu interrupt
    if (!(regs.mqicr() & regs.imr)) {
        DPRINTF(NepNicIntrMsi, "Mask cleaned all mq interrupts\n");
        if (interMultEvent.scheduled())
            igbe->deschedule(interMultEvent);
        if (regs.mqicr.int_assert())
        {
            DPRINTF(NepNicIntrMsi, "Clean!! MQ %d\n", idxComponent);
            cpuClearMultiInt();
        }
    }
    DPRINTF(NepNicIntrMsi, "ITR = %#X itr.interval = %#X\n",
            regs.itr(), regs.itr.interval());

    if (regs.mqicr() & regs.imr) {
        if (regs.itr.interval() == 0)  {
            cpuPostMultInt(); //cpuPostInt();
        } else {
            DPRINTF(NepNicIntrMsi,
                    "Possibly scheduling interrupt because of imr write\n");
            if (!interMultEvent.scheduled()) {
                Tick t = curTick() + SimClock::Int::ns * 256 * regs.itr.interval();
                DPRINTF(NepNicIntrMsi, "Scheduling for %d\n", t);
                igbe->schedule(interMultEvent, t);
            }
        }
    }
}

void
NepGbE::chkMSI()
{

}


///////////////////////////// NepGbE::DescCache //////////////////////////////

template<class T>
NepGbE::DescCache<T>::DescCache(NepGbE *i, const std::string n, int s)
    : igbe(i), _name(n), cachePnt(0), size(s), curFetching(0),
      wbOut(0), moreToWb(false), wbAlignment(0), pktPtr(NULL),
      wbDelayEvent([this]{ writeback1(); }, n),
      fetchDelayEvent([this]{ fetchDescriptors1(); }, n),
      fetchEvent([this]{ fetchComplete(); }, n),
      wbEvent([this]{ wbComplete(); }, n)
{
    fetchBuf = new T[size];
    wbBuf = new T[size];
}

template<class T>
NepGbE::DescCache<T>::~DescCache()
{
    reset();
    delete[] fetchBuf;
    delete[] wbBuf;
}

template<class T>
void
NepGbE::DescCache<T>::areaChanged()
{
    if (usedCache.size() > 0 || curFetching || wbOut)
        panic("Descriptor Address, Length or Head changed. Bad\n");
    reset();

}

template<class T>
void
NepGbE::DescCache<T>::writeback(Addr aMask)
{
    int curHead = descHead();
    int max_to_wb = usedCache.size();

    // Check if this writeback is less restrictive that the previous
    // and if so setup another one immediately following it
    if (wbOut) {
        if (aMask < wbAlignment) {
            moreToWb = true;
            wbAlignment = aMask;
        }
        DPRINTF(EthernetDesc,
                "Writing back already in process, returning\n");
        return;
    }

    moreToWb = false;
    wbAlignment = aMask;


    DPRINTF(EthernetDesc, "Writing back descriptors head: %d tail: "
            "%d len: %d cachePnt: %d max_to_wb: %d descleft: %d\n",
            curHead, descTail(), descLen(), cachePnt, max_to_wb,
            descLeft());

    if (max_to_wb + curHead >= descLen()) {
        max_to_wb = descLen() - curHead;
        moreToWb = true;
        // this is by definition aligned correctly
    } else if (wbAlignment != 0) {
        // align the wb point to the mask
        max_to_wb = max_to_wb & ~wbAlignment;
    }

    DPRINTF(EthernetDesc, "Writing back %d descriptors\n", max_to_wb);

    if (max_to_wb <= 0) {

        return;
    }

    wbOut = max_to_wb;

    assert(!wbDelayEvent.scheduled());
    igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
    
}

template<class T>
void
NepGbE::DescCache<T>::writeback1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
        return;
    }

    DPRINTF(EthernetDesc, "Begining DMA of %d descriptors\n", wbOut);

    for (int x = 0; x < wbOut; x++) {
        assert(usedCache.size());
        memcpy(&wbBuf[x], usedCache[x], sizeof(T));
    }



    assert(wbOut);
    igbe->dmaWrite(pciToDma(descBase() + descHead() * sizeof(T)),
                   wbOut * sizeof(T), &wbEvent, (uint8_t*)wbBuf,
                   igbe->wbCompDelay);
}

template<class T>
void
NepGbE::DescCache<T>::fetchDescriptors()
{
    size_t max_to_fetch;

    if (curFetching) {
        DPRINTF(EthernetDesc,
                "Currently fetching %d descriptors, returning\n",
                curFetching);
        return;
    }

    if (descTail() >= cachePnt)
        max_to_fetch = descTail() - cachePnt;
    else
        max_to_fetch = descLen() - cachePnt;

    size_t free_cache = size - usedCache.size() - unusedCache.size();

    

    max_to_fetch = std::min(max_to_fetch, free_cache);


    DPRINTF(EthernetDesc, "Fetching descriptors head: %d tail: "
            "%d len: %d cachePnt: %d max_to_fetch: %d descleft: %d\n",
            descHead(), descTail(), descLen(), cachePnt,
            max_to_fetch, descLeft());

    // Nothing to do
    if (max_to_fetch == 0)
        return;

    // So we don't have two descriptor fetches going on at once
    curFetching = max_to_fetch;

    assert(!fetchDelayEvent.scheduled());
    igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
}

template<class T>
void
NepGbE::DescCache<T>::fetchDescriptors1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
        return;
    }

    

    DPRINTF(EthernetDesc, "Fetching descriptors at %#x (%#x) [%#x], size: %#x\n",
            descBase() + cachePnt * sizeof(T),
            pciToDma(descBase() + cachePnt * sizeof(T)),
            descBase(),
            curFetching * sizeof(T));
    assert(curFetching);
    igbe->dmaRead(pciToDma(descBase() + cachePnt * sizeof(T)),
                  curFetching * sizeof(T), &fetchEvent, (uint8_t*)fetchBuf,
                  igbe->fetchCompDelay);
    
}

template<class T>
void
NepGbE::DescCache<T>::fetchComplete()
{
    T *newDesc;
    
    DPRINTF(NepNicOthers, "NepGbE::DescCache<T>::fetchComplete() called\n");
    DPRINTF(EthernetDesc, "TestDescBufPrint %#llx\n", *(uint64_t*)fetchBuf);
    for (int x = 0; x < curFetching; x++) {
        newDesc = new T;
        memcpy(newDesc, &fetchBuf[x], sizeof(T));
        unusedCache.push_back(newDesc);

    }


#ifndef NDEBUG
    int oldCp = cachePnt;
#endif

    cachePnt += curFetching;
    assert(cachePnt <= descLen());
    if (cachePnt == descLen())
        cachePnt = 0;

    curFetching = 0;

    DPRINTF(EthernetDesc, "Fetching complete cachePnt %d -> %d\n",
            oldCp, cachePnt);

    
    enableSm();
    igbe->checkDrain();
}

template<class T>
void
NepGbE::DescCache<T>::wbComplete()
{

    

    long  curHead = descHead();
#ifndef NDEBUG
    long oldHead = curHead;
#endif

    for (int x = 0; x < wbOut; x++) {
        assert(usedCache.size());
        delete usedCache[0];
        usedCache.pop_front();


    }

    curHead += wbOut;
    wbOut = 0;

    if (curHead >= descLen())
        curHead -= descLen();

    // Update the head
    updateHead(curHead);

    DPRINTF(EthernetDesc, "Writeback complete curHead %d -> %d\n",
            oldHead, curHead);

    // If we still have more to wb, call wb now
    actionAfterWb();
    if (moreToWb) {
        moreToWb = false;
        DPRINTF(EthernetDesc, "Writeback has more todo\n");
        writeback(wbAlignment);
    }

    if (!wbOut) {
        igbe->checkDrain();

    }
    fetchAfterWb();
}

template<class T>
void
NepGbE::DescCache<T>::reset()
{
    DPRINTF(EthernetDesc, "Reseting descriptor cache\n");
    for (typename CacheType::size_type x = 0; x < usedCache.size(); x++)
        delete usedCache[x];
    for (typename CacheType::size_type x = 0; x < unusedCache.size(); x++)
        delete unusedCache[x];

    usedCache.clear();
    unusedCache.clear();

    cachePnt = 0;

}

template<class T>
void
NepGbE::DescCache<T>::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(cachePnt);
    SERIALIZE_SCALAR(curFetching);
    SERIALIZE_SCALAR(wbOut);
    SERIALIZE_SCALAR(moreToWb);
    SERIALIZE_SCALAR(wbAlignment);

    typename CacheType::size_type usedCacheSize = usedCache.size();
    SERIALIZE_SCALAR(usedCacheSize);
    for (typename CacheType::size_type x = 0; x < usedCacheSize; x++) {
        arrayParamOut(cp, csprintf("usedCache_%d", x),
                      (uint8_t*)usedCache[x],sizeof(T));
    }

    typename CacheType::size_type unusedCacheSize = unusedCache.size();
    SERIALIZE_SCALAR(unusedCacheSize);
    for (typename CacheType::size_type x = 0; x < unusedCacheSize; x++) {
        arrayParamOut(cp, csprintf("unusedCache_%d", x),
                      (uint8_t*)unusedCache[x],sizeof(T));
    }

    Tick fetch_delay = 0, wb_delay = 0;
    if (fetchDelayEvent.scheduled())
        fetch_delay = fetchDelayEvent.when();
    SERIALIZE_SCALAR(fetch_delay);
    if (wbDelayEvent.scheduled())
        wb_delay = wbDelayEvent.when();
    SERIALIZE_SCALAR(wb_delay);


}

template<class T>
void
NepGbE::DescCache<T>::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(cachePnt);
    UNSERIALIZE_SCALAR(curFetching);
    UNSERIALIZE_SCALAR(wbOut);
    UNSERIALIZE_SCALAR(moreToWb);
    UNSERIALIZE_SCALAR(wbAlignment);

    typename CacheType::size_type usedCacheSize;
    UNSERIALIZE_SCALAR(usedCacheSize);
    T *temp;
    for (typename CacheType::size_type x = 0; x < usedCacheSize; x++) {
        temp = new T;
        arrayParamIn(cp, csprintf("usedCache_%d", x),
                     (uint8_t*)temp,sizeof(T));
        usedCache.push_back(temp);
    }

    typename CacheType::size_type unusedCacheSize;
    UNSERIALIZE_SCALAR(unusedCacheSize);
    for (typename CacheType::size_type x = 0; x < unusedCacheSize; x++) {
        temp = new T;
        arrayParamIn(cp, csprintf("unusedCache_%d", x),
                     (uint8_t*)temp,sizeof(T));
        unusedCache.push_back(temp);
    }
    Tick fetch_delay = 0, wb_delay = 0;
    UNSERIALIZE_SCALAR(fetch_delay);
    UNSERIALIZE_SCALAR(wb_delay);
    if (fetch_delay)
        igbe->schedule(fetchDelayEvent, fetch_delay);
    if (wb_delay)
        igbe->schedule(wbDelayEvent, wb_delay);


}

///////////////////////////// NepGbE::RxDescCache //////////////////////////////

NepGbE::RxDescCache::RxDescCache(NepGbE *i, const std::string n, int s)
    : DescCache<RxDesc>(i, n, s), pktDone(false), splitCount(0),
    pktEvent([this]{ pktComplete(); }, n),
    pktHdrEvent([this]{ pktSplitDone(); }, n),
    pktDataEvent([this]{ pktSplitDone(); }, n)

{
    annSmFetch = "RX Desc Fetch";
    annSmWb = "RX Desc Writeback";
    annUnusedDescQ = "RX Unused Descriptors";
    annUnusedCacheQ = "RX Unused Descriptor Cache";
    annUsedCacheQ = "RX Used Descriptor Cache";
    annUsedDescQ = "RX Used Descriptors";
    annDescQ = "RX Descriptors";
}

void
NepGbE::RxDescCache::pktSplitDone()
{
    splitCount++;
    DPRINTF(EthernetDesc,
            "Part of split packet done: splitcount now %d\n", splitCount);
    assert(splitCount <= 2);
    if (splitCount != 2)
        return;
    splitCount = 0;
    DPRINTF(EthernetDesc,
            "Part of split packet done: calling pktComplete()\n");
    pktComplete();
}


/**
 * 이 함수의 실행을 유도하여 어느 주소로 패킷을 보내는지 알아본다.
 */
int
NepGbE::RxDescCache::writePacket(EthPacketPtr packet, int pkt_offset)
{
    assert(unusedCache.size());
    //if (!unusedCache.size())
    //    return false;

    pktPtr = packet;
    pktDone = false;
    unsigned buf_len, hdr_len;

    RxDesc *desc = unusedCache.front();
    switch (igbe->regs.nep_ex_regs[idx].srrctl.desctype()) {
      case RXDT_LEGACY:
        assert(pkt_offset == 0);
        bytesCopied = packet->length;
        DPRINTF(EthernetDesc, "Packet Length: %d Desc Size: %d\n",
                packet->length, igbe->regs.nep_ex_regs[idx].rctl.descSize());
        assert(packet->length < igbe->regs.nep_ex_regs[idx].rctl.descSize());
        igbe->dmaWrite(pciToDma(desc->legacy.buf),
                       packet->length, &pktEvent, packet->data,
                       igbe->rxWriteDelay);
        break;
      case RXDT_ADV_ONEBUF:
        assert(pkt_offset == 0);
        bytesCopied = packet->length;
        buf_len = igbe->regs.nep_ex_regs[idx].rctl.lpe() ? igbe->regs.nep_ex_regs[idx].srrctl.bufLen() :
            igbe->regs.nep_ex_regs[idx].rctl.descSize();
        DPRINTF(EthernetDesc, "Packet Length: %d srrctl: %#x Desc Size: %d\n",
                packet->length, igbe->regs.nep_ex_regs[idx].srrctl(), buf_len);
        assert(packet->length < buf_len);
        igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                       packet->length, &pktEvent, packet->data,
                       igbe->rxWriteDelay);
        desc->adv_wb.header_len = htole(0);
        desc->adv_wb.sph = htole(0);
        desc->adv_wb.pkt_len = htole((uint16_t)(pktPtr->length));
        break;
      case RXDT_ADV_SPLIT_A:
        int split_point;

        buf_len = igbe->regs.nep_ex_regs[idx].rctl.lpe() ? igbe->regs.nep_ex_regs[idx].srrctl.bufLen() :
            igbe->regs.nep_ex_regs[idx].rctl.descSize();
        hdr_len = igbe->regs.nep_ex_regs[idx].rctl.lpe() ? igbe->regs.nep_ex_regs[idx].srrctl.hdrLen() : 0;
        DPRINTF(EthernetDesc,
                "lpe: %d Packet Length: %d offset: %d srrctl: %#x "
                "hdr addr: %#x Hdr Size: %d desc addr: %#x Desc Size: %d\n",
                igbe->regs.nep_ex_regs[idx].rctl.lpe(), packet->length, pkt_offset,
                igbe->regs.nep_ex_regs[idx].srrctl(), desc->adv_read.hdr, hdr_len,
                desc->adv_read.pkt, buf_len);

        split_point = hsplit(pktPtr);

        if (packet->length <= hdr_len) {
            bytesCopied = packet->length;
            assert(pkt_offset == 0);
            DPRINTF(EthernetDesc, "Hdr split: Entire packet in header\n");
            igbe->dmaWrite(pciToDma(desc->adv_read.hdr),
                           packet->length, &pktEvent, packet->data,
                           igbe->rxWriteDelay);
            desc->adv_wb.header_len = htole((uint16_t)packet->length);
            desc->adv_wb.sph = htole(0);
            desc->adv_wb.pkt_len = htole(0);
        } else if (split_point) {
            if (pkt_offset) {
                // we are only copying some data, header/data has already been
                // copied
                int max_to_copy =
                    std::min(packet->length - pkt_offset, buf_len);
                bytesCopied += max_to_copy;
                DPRINTF(EthernetDesc,
                        "Hdr split: Continuing data buffer copy\n");
                igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                               max_to_copy, &pktEvent,
                               packet->data + pkt_offset, igbe->rxWriteDelay);//상의:rxWriteDelay를 공통적으로 설정하게할지?
                desc->adv_wb.header_len = htole(0);
                desc->adv_wb.pkt_len = htole((uint16_t)max_to_copy);
                desc->adv_wb.sph = htole(0);
            } else {
                int max_to_copy =
                    std::min(packet->length - split_point, buf_len);
                bytesCopied += max_to_copy + split_point;

                DPRINTF(EthernetDesc, "Hdr split: splitting at %d\n",
                        split_point);
                igbe->dmaWrite(pciToDma(desc->adv_read.hdr),
                               split_point, &pktHdrEvent,
                               packet->data, igbe->rxWriteDelay);
                igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                               max_to_copy, &pktDataEvent,
                               packet->data + split_point, igbe->rxWriteDelay);
                desc->adv_wb.header_len = htole(split_point);
                desc->adv_wb.sph = 1;
                desc->adv_wb.pkt_len = htole((uint16_t)(max_to_copy));
            }
        } else {
            panic("Header split not fitting within header buffer or "
                  "undecodable packet not fitting in header unsupported\n");
        }
        break;
      default:
        panic("Unimplemnted RX receive buffer type: %d\n",
              igbe->regs.nep_ex_regs[idx].srrctl.desctype());
    }
    return bytesCopied;

}

void
NepGbE::RxDescCache::pktComplete()
{
    assert(unusedCache.size());
    RxDesc *desc;
    desc = unusedCache.front();
    
    MultiQueueComponentSet& rxcomp = igbe->mqManager.getCompByIdx(idx);
    

    

    uint16_t crcfixup = igbe->regs.nep_ex_regs[idx].rctl.secrc() ? 0 : 4 ;//BST need to be implement
    DPRINTF(EthernetDesc, "pktPtr->length: %d bytesCopied: %d "
            "stripcrc offset: %d value written: %d %d\n",
            pktPtr->length, bytesCopied, crcfixup,
            htole((uint16_t)(pktPtr->length + crcfixup)),
            (uint16_t)(pktPtr->length + crcfixup));

    // no support for anything but starting at 0
    // SHIN. Temp disable
    //assert(igbe->regs.nep_rx_regs[idx].rxcsum.pcss() == 0);

    DPRINTF(EthernetDesc, "Packet written to memory updating Descriptor\n");

    uint16_t status = RXDS_DD;
    uint8_t err = 0;
    uint16_t ext_err = 0;
    uint16_t csum = 0;
    uint16_t ptype = 0;
    uint16_t ip_id = 0;

    assert(bytesCopied <= pktPtr->length);
    if (bytesCopied == pktPtr->length)
        status |= RXDS_EOP;

    IpPtr ip(pktPtr);
    Ip6Ptr ip6(pktPtr);

    //if (ip || ip6) {
    if (0) {
        if (ip) {
            DPRINTF(EthernetDesc, "Proccesing Ip packet with Id=%d\n",
                    ip->id());
            ptype |= RXDP_IPV4;
            ip_id = ip->id();
        }
        if (ip6)
            ptype |= RXDP_IPV6;

        if (ip && igbe->regs.nep_ex_regs[idx].rxcsum.ipofld()) {
            DPRINTF(EthernetDesc, "Checking IP checksum\n");
            status |= RXDS_IPCS;
            csum = htole(cksum(ip));
            igbe->etherDeviceStats.rxIpChecksums++;
            if (cksum(ip) != 0) {
                err |= RXDE_IPE;
                ext_err |= RXDEE_IPE;
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
            }
        }
        TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
        if (tcp && igbe->regs.nep_ex_regs[idx].rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking TCP checksum\n");
            status |= RXDS_TCPCS;
            ptype |= RXDP_TCP;
            csum = htole(cksum(tcp));
            igbe->etherDeviceStats.rxTcpChecksums++;
            if (cksum(tcp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                err |= RXDE_TCPE;
                ext_err |= RXDEE_TCPE;
            }
        }

        UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
        if (udp && igbe->regs.nep_ex_regs[idx].rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking UDP checksum\n");
            status |= RXDS_UDPCS;
            ptype |= RXDP_UDP;
            csum = htole(cksum(udp));
            igbe->etherDeviceStats.rxUdpChecksums++;
            if (cksum(udp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                ext_err |= RXDEE_TCPE;
                err |= RXDE_TCPE;
            }
        }
    } else { // if ip
        DPRINTF(EthernetSM, "Proccesing Non-Ip packet\n");
    }

    switch (igbe->regs.nep_ex_regs[idx].srrctl.desctype()) {
      case RXDT_LEGACY:
        desc->legacy.len = htole((uint16_t)(pktPtr->length + crcfixup));
        desc->legacy.status = htole(status);
        desc->legacy.errors = htole(err);
        // No vlan support at this point... just set it to 0
        desc->legacy.vlan = 0;
        break;
      case RXDT_ADV_SPLIT_A:
      case RXDT_ADV_ONEBUF:
        desc->adv_wb.rss_type = htole(0);
        desc->adv_wb.pkt_type = htole(ptype);
        if (igbe->regs.nep_ex_regs[idx].rxcsum.pcsd()) {
            // no rss support right now
            desc->adv_wb.rss_hash = htole(0);
        } else {
            desc->adv_wb.id = htole(ip_id);
            desc->adv_wb.csum = htole(csum);
        }
        desc->adv_wb.status = htole(status);
        desc->adv_wb.errors = htole(ext_err);
        // no vlan support
        desc->adv_wb.vlan_tag = htole(0);
        break;
      default:
        panic("NepGbE::RxDescCache::pktComplete() Unimplemnted RX receive buffer type %d\n",
              igbe->regs.nep_ex_regs[idx].srrctl.desctype());
    }

    DPRINTF(EthernetDesc, "Descriptor complete w0: %#x w1: %#x\n",
            desc->adv_read.pkt, desc->adv_read.hdr);

    if (bytesCopied == pktPtr->length) {
        DPRINTF(EthernetDesc,
                "Packet completely written to descriptor buffers\n");
        // Deal with the rx timer interrupts
        if (igbe->regs.nep_ex_regs[idx].rdtr.delay()) {
            Tick delay = igbe->regs.nep_ex_regs[idx].rdtr.delay() * igbe->intClock();
            // if(igbe->rdtrEvent.scheduled())
            //     DPRINTF(EthernetSM, "OouYa!! already scheduled\n");
            if(rxcomp.rdtrMultEvent.scheduled())
                DPRINTF(EthernetSM, "OouYa!! already scheduled\n");
            DPRINTF(EthernetSM, "RXS: Scheduling DTR for %d\n", delay);
            //igbe->reschedule(igbe->rdtrEvent, curTick() + delay);
            igbe->reschedule(rxcomp.rdtrMultEvent, curTick() + delay);
            DPRINTF(EthernetSM, "RXS: Scheduling DTR for %d\n", delay);
        }

        if (igbe->regs.nep_ex_regs[idx].radv.idv()) {
            Tick delay = igbe->regs.nep_ex_regs[idx].radv.idv() * igbe->intClock();
            DPRINTF(EthernetSM, "RXS: Scheduling ADV for %d\n", delay);
            // if (!igbe->radvEvent.scheduled()) {//상의 필요
            //     igbe->schedule(igbe->radvEvent, curTick() + delay);
            // }
            if (!rxcomp.radvMultEvent.scheduled()) {//상의 필요
                igbe->schedule(rxcomp.radvMultEvent, curTick() + delay);
            }
        }

        // if neither radv or rdtr, maybe itr is set...
        if (!igbe->regs.nep_ex_regs[idx].rdtr.delay() && !igbe->regs.nep_ex_regs[idx].radv.idv()) {
            DPRINTF(EthernetSM,
                    "RXS: Receive interrupt delay disabled, posting IT_RXT\n");
            
            //if(multiqueue)
                rxcomp.postInterruptMultIntr(IT_RXT);
            //else
                //igbe->postInterrupt(IT_RXT);//상의 필요
        }

        // If the packet is small enough, interrupt appropriately
        // I wonder if this is delayed or not?!
        if (pktPtr->length <= igbe->regs.nep_ex_regs[idx].rsrpd.idv()) {
            DPRINTF(EthernetSM,
                    "RXS: Posting IT_SRPD beacuse small packet received\n");
            
            //if(multiqueue)
                rxcomp.postInterruptMultIntr(IT_SRPD);
            //else
                //igbe->postInterrupt(IT_SRPD);
        }
        bytesCopied = 0;
    }

    pktPtr = NULL;
    igbe->checkDrain();
    enableSm();
    pktDone = true;

    
    DPRINTF(EthernetDesc, "Processing of this descriptor complete\n");
    
    unusedCache.pop_front();
    
    usedCache.push_back(desc);
}

void
NepGbE::RxDescCache::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->mqManager.allRXTickOn();//igbe->rxTick = true;
        igbe->restartClock();
    }
}

bool
NepGbE::RxDescCache::packetDone()
{
    if (pktDone) {
        pktDone = false;
        return true;
    }
    return false;
}

bool
NepGbE::RxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled() || wbEvent.scheduled() ||
        fetchEvent.scheduled() || pktHdrEvent.scheduled() ||
        pktDataEvent.scheduled();

}

void
NepGbE::RxDescCache::serialize(CheckpointOut &cp) const
{
    DescCache<RxDesc>::serialize(cp);
    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(splitCount);
    SERIALIZE_SCALAR(bytesCopied);
}

void
NepGbE::RxDescCache::unserialize(CheckpointIn &cp)
{
    DescCache<RxDesc>::unserialize(cp);
    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(splitCount);
    UNSERIALIZE_SCALAR(bytesCopied);
}


///////////////////////////// NepGbE::TxDescCache //////////////////////////////

NepGbE::TxDescCache::TxDescCache(NepGbE *i, const std::string n, int s)
    : DescCache<TxDesc>(i,n, s), pktDone(false), isTcp(false),
      pktWaiting(false), pktMultiDesc(false),
      completionAddress(0), completionEnabled(false),
      useTso(false), tsoHeaderLen(0), tsoMss(0), tsoTotalLen(0), tsoUsedLen(0),
      tsoPrevSeq(0), tsoPktPayloadBytes(0), tsoLoadedHeader(false),
      tsoPktHasHeader(false), tsoDescBytesUsed(0), tsoCopyBytes(0), tsoPkts(0),
    pktEvent([this]{ pktComplete(); }, n),
    headerEvent([this]{ headerComplete(); }, n),
    nullEvent([this]{ nullCallback(); }, n)
{
    annSmFetch = "TX Desc Fetch";
    annSmWb = "TX Desc Writeback";
    annUnusedDescQ = "TX Unused Descriptors";
    annUnusedCacheQ = "TX Unused Descriptor Cache";
    annUsedCacheQ = "TX Used Descriptor Cache";
    annUsedDescQ = "TX Used Descriptors";
    annDescQ = "TX Descriptors";
}

void
NepGbE::TxDescCache::processContextDesc()
{
    assert(unusedCache.size());
    TxDesc *desc;

    DPRINTF(EthernetDesc, "Checking and  processing context descriptors\n");

    while (!useTso && unusedCache.size() &&
           TxdOp::isContext(unusedCache.front())) {
        DPRINTF(EthernetDesc, "Got context descriptor type...\n");

        desc = unusedCache.front();
        DPRINTF(EthernetDesc, "Descriptor upper: %#x lower: %#X\n",
                desc->d1, desc->d2);


        // is this going to be a tcp or udp packet?
        isTcp = TxdOp::tcp(desc) ? true : false;

        // setup all the TSO variables, they'll be ignored if we don't use
        // tso for this connection
        tsoHeaderLen = TxdOp::hdrlen(desc);
        tsoMss  = TxdOp::mss(desc);

        if (TxdOp::isType(desc, TxdOp::TXD_CNXT) && TxdOp::tse(desc)) {
            DPRINTF(EthernetDesc, "TCP offload enabled for packet hdrlen: "
                    "%d mss: %d paylen %d\n", TxdOp::hdrlen(desc),
                    TxdOp::mss(desc), TxdOp::getLen(desc));
            useTso = true;
            tsoTotalLen = TxdOp::getLen(desc);
            tsoLoadedHeader = false;
            tsoDescBytesUsed = 0;
            tsoUsedLen = 0;
            tsoPrevSeq = 0;
            tsoPktHasHeader = false;
            tsoPkts = 0;
            tsoCopyBytes = 0;
        }

        TxdOp::setDd(desc);
        unusedCache.pop_front();
        
        usedCache.push_back(desc);
    }

    if (!unusedCache.size())
        return;

    desc = unusedCache.front();
    if (!useTso && TxdOp::isType(desc, TxdOp::TXD_ADVDATA) &&
        TxdOp::tse(desc)) {
        DPRINTF(EthernetDesc, "TCP offload(adv) enabled for packet "
                "hdrlen: %d mss: %d paylen %d\n",
                tsoHeaderLen, tsoMss, TxdOp::getTsoLen(desc));
        useTso = true;
        tsoTotalLen = TxdOp::getTsoLen(desc);
        tsoLoadedHeader = false;
        tsoDescBytesUsed = 0;
        tsoUsedLen = 0;
        tsoPrevSeq = 0;
        tsoPktHasHeader = false;
        tsoPkts = 0;
    }

    if (useTso && !tsoLoadedHeader) {
        // we need to fetch a header
        DPRINTF(EthernetDesc, "Starting DMA of TSO header\n");
        assert(TxdOp::isData(desc) && TxdOp::getLen(desc) >= tsoHeaderLen);
        pktWaiting = true;
        assert(tsoHeaderLen <= 256);
        igbe->dmaRead(pciToDma(TxdOp::getBuf(desc)),
                      tsoHeaderLen, &headerEvent, tsoHeader, 0);
    }
}

void
NepGbE::TxDescCache::headerComplete()
{
    DPRINTF(EthernetDesc, "TSO: Fetching TSO header complete\n");
    pktWaiting = false;

    assert(unusedCache.size());
    TxDesc *desc = unusedCache.front();
    DPRINTF(EthernetDesc, "TSO: len: %d tsoHeaderLen: %d\n",
            TxdOp::getLen(desc), tsoHeaderLen);

    if (TxdOp::getLen(desc) == tsoHeaderLen) {
        tsoDescBytesUsed = 0;
        tsoLoadedHeader = true;
        unusedCache.pop_front();
        usedCache.push_back(desc);
    } else {
        DPRINTF(EthernetDesc, "TSO: header part of larger payload\n");
        tsoDescBytesUsed = tsoHeaderLen;
        tsoLoadedHeader = true;
    }
    enableSm();
    igbe->checkDrain();
}

unsigned
NepGbE::TxDescCache::getPacketSize(EthPacketPtr p)
{
    if (!unusedCache.size())
        return 0;

    DPRINTF(EthernetDesc, "Starting processing of descriptor\n");

    assert(!useTso || tsoLoadedHeader);
    TxDesc *desc = unusedCache.front();

    if (useTso) {
        DPRINTF(EthernetDesc, "getPacket(): TxDescriptor data "
                "d1: %#llx d2: %#llx\n", desc->d1, desc->d2);
        DPRINTF(EthernetDesc, "TSO: use: %d hdrlen: %d mss: %d total: %d "
                "used: %d loaded hdr: %d\n", useTso, tsoHeaderLen, tsoMss,
                tsoTotalLen, tsoUsedLen, tsoLoadedHeader);

        if (tsoPktHasHeader)
            tsoCopyBytes =  std::min((tsoMss + tsoHeaderLen) - p->length,
                                     TxdOp::getLen(desc) - tsoDescBytesUsed);
        else
            tsoCopyBytes =  std::min(tsoMss,
                                     TxdOp::getLen(desc) - tsoDescBytesUsed);
        unsigned pkt_size =
            tsoCopyBytes + (tsoPktHasHeader ? 0 : tsoHeaderLen);

        DPRINTF(EthernetDesc, "TSO: descBytesUsed: %d copyBytes: %d "
                "this descLen: %d\n",
                tsoDescBytesUsed, tsoCopyBytes, TxdOp::getLen(desc));
        DPRINTF(EthernetDesc, "TSO: pktHasHeader: %d\n", tsoPktHasHeader);
        DPRINTF(EthernetDesc, "TSO: Next packet is %d bytes\n", pkt_size);
        return pkt_size;
    }

    DPRINTF(EthernetDesc, "Next TX packet is %d bytes\n",
            TxdOp::getLen(unusedCache.front()));
    return TxdOp::getLen(desc);
}

void
NepGbE::TxDescCache::getPacketData(EthPacketPtr p)
{
    assert(unusedCache.size());

    TxDesc *desc;
    desc = unusedCache.front();

    DPRINTF(EthernetDesc, "getPacketData(): TxDescriptor data "
            "d1: %#llx d2: %#llx\n", desc->d1, desc->d2);
    assert((TxdOp::isLegacy(desc) || TxdOp::isData(desc)) &&
           TxdOp::getLen(desc));

    pktPtr = p;

    pktWaiting = true;

    DPRINTF(EthernetDesc, "Starting DMA of packet at offset %d\n", p->length);

    if (useTso) {
        assert(tsoLoadedHeader);
        if (!tsoPktHasHeader) {
            DPRINTF(EthernetDesc,
                    "Loading TSO header (%d bytes) into start of packet\n",
                    tsoHeaderLen);
            memcpy(p->data, &tsoHeader,tsoHeaderLen);
            p->length +=tsoHeaderLen;
            tsoPktHasHeader = true;
        }
    }

    if (useTso) {
        DPRINTF(EthernetDesc,
                "Starting DMA of packet at offset %d length: %d\n",
                p->length, tsoCopyBytes);
        igbe->dmaRead(pciToDma(TxdOp::getBuf(desc))
                      + tsoDescBytesUsed,
                      tsoCopyBytes, &pktEvent, p->data + p->length,
                      igbe->txReadDelay);
        tsoDescBytesUsed += tsoCopyBytes;
        assert(tsoDescBytesUsed <= TxdOp::getLen(desc));
    } else {
        igbe->dmaRead(pciToDma(TxdOp::getBuf(desc)),
                      TxdOp::getLen(desc), &pktEvent, p->data + p->length,
                      igbe->txReadDelay);
    }
}

void
NepGbE::TxDescCache::pktComplete()
{

    TxDesc *desc;
    assert(unusedCache.size());
    assert(pktPtr);

    MultiQueueComponentSet& mqcomp = igbe->mqManager.getCompByIdx(idx);

    

    DPRINTF(EthernetDesc, "DMA of packet complete\n");


    desc = unusedCache.front();
    assert((TxdOp::isLegacy(desc) || TxdOp::isData(desc)) &&
           TxdOp::getLen(desc));

    DPRINTF(EthernetDesc, "TxDescriptor data d1: %#llx d2: %#llx\n",
            desc->d1, desc->d2);

    // Set the length of the data in the EtherPacket
    if (useTso) {
        DPRINTF(EthernetDesc, "TSO: use: %d hdrlen: %d mss: %d total: %d "
            "used: %d loaded hdr: %d\n", useTso, tsoHeaderLen, tsoMss,
            tsoTotalLen, tsoUsedLen, tsoLoadedHeader);
        pktPtr->simLength += tsoCopyBytes;
        pktPtr->length += tsoCopyBytes;
        tsoUsedLen += tsoCopyBytes;
        DPRINTF(EthernetDesc, "TSO: descBytesUsed: %d copyBytes: %d\n",
            tsoDescBytesUsed, tsoCopyBytes);
    } else {
        pktPtr->simLength += TxdOp::getLen(desc);
        pktPtr->length += TxdOp::getLen(desc);
    }



    if ((!TxdOp::eop(desc) && !useTso) ||
        (pktPtr->length < ( tsoMss + tsoHeaderLen) &&
         tsoTotalLen != tsoUsedLen && useTso)) {
        assert(!useTso || (tsoDescBytesUsed == TxdOp::getLen(desc)));
        unusedCache.pop_front();
        usedCache.push_back(desc);

        tsoDescBytesUsed = 0;
        pktDone = true;
        pktWaiting = false;
        pktMultiDesc = true;

        DPRINTF(EthernetDesc, "Partial Packet Descriptor of %d bytes Done\n",
                pktPtr->length);
        pktPtr = NULL;

        enableSm();
        igbe->checkDrain();
        return;
    }


    pktMultiDesc = false;
    // no support for vlans
    assert(!TxdOp::vle(desc));

    // we only support single packet descriptors at this point
    if (!useTso)
        assert(TxdOp::eop(desc));

    // set that this packet is done
    if (TxdOp::rs(desc))
        TxdOp::setDd(desc);

    DPRINTF(EthernetDesc, "TxDescriptor data d1: %#llx d2: %#llx\n",
            desc->d1, desc->d2);

    if (useTso) {
        IpPtr ip(pktPtr);
        Ip6Ptr ip6(pktPtr);
        if (ip) {
            DPRINTF(EthernetDesc, "TSO: Modifying IP header. Id + %d\n",
                    tsoPkts);
            ip->id(ip->id() + tsoPkts++);
            ip->len(pktPtr->length - EthPtr(pktPtr)->size());
        }
        if (ip6)
            ip6->plen(pktPtr->length - EthPtr(pktPtr)->size());
        TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
        if (tcp) {
            DPRINTF(EthernetDesc,
                    "TSO: Modifying TCP header. old seq %d + %d\n",
                    tcp->seq(), tsoPrevSeq);
            tcp->seq(tcp->seq() + tsoPrevSeq);
            if (tsoUsedLen != tsoTotalLen)
                tcp->flags(tcp->flags() & ~9); // clear fin & psh
        }
        UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
        if (udp) {
            DPRINTF(EthernetDesc, "TSO: Modifying UDP header.\n");
            udp->len(pktPtr->length - EthPtr(pktPtr)->size());
        }
        tsoPrevSeq = tsoUsedLen;
    }

    if (DTRACE(EthernetDesc)) {
        IpPtr ip(pktPtr);
        if (ip)
            DPRINTF(EthernetDesc, "Proccesing Ip packet with Id=%d\n",
                    ip->id());
        else
            DPRINTF(EthernetSM, "Proccesing Non-Ip packet\n");
    }

    // Checksums are only ofloaded for new descriptor types
    if (TxdOp::isData(desc) && (TxdOp::ixsm(desc) || TxdOp::txsm(desc))) {
        DPRINTF(EthernetDesc, "Calculating checksums for packet\n");
        IpPtr ip(pktPtr);
        Ip6Ptr ip6(pktPtr);
        assert(ip || ip6);
        if (ip && TxdOp::ixsm(desc)) {
            ip->sum(0);
            ip->sum(cksum(ip));
            igbe->etherDeviceStats.txIpChecksums++;
            DPRINTF(EthernetDesc, "Calculated IP checksum\n");
        }
        if (TxdOp::txsm(desc)) {
            TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
            UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
            if (tcp) {
                tcp->sum(0);
                tcp->sum(cksum(tcp));
                igbe->etherDeviceStats.txTcpChecksums++;
                DPRINTF(EthernetDesc, "Calculated TCP checksum\n");
            } else if (udp) {
                assert(udp);
                udp->sum(0);
                udp->sum(cksum(udp));
                igbe->etherDeviceStats.txUdpChecksums++;
                DPRINTF(EthernetDesc, "Calculated UDP checksum\n");
            } else {
                panic("Told to checksum, but don't know how\n");
            }
        }
    }

    if (TxdOp::ide(desc)) {
        // Deal with the rx timer interrupts
        DPRINTF(EthernetDesc, "Descriptor had IDE set\n");
        if (igbe->regs.nep_ex_regs[idx].tidv.idv()) {
            Tick delay = igbe->regs.nep_ex_regs[idx].tidv.idv() * igbe->intClock();
            DPRINTF(EthernetDesc, "setting tidv\n");
            igbe->reschedule(mqcomp.tidvMultEvent, curTick() + delay, true);
        }

        if (igbe->regs.nep_ex_regs[idx].tadv.idv() && igbe->regs.nep_ex_regs[idx].tidv.idv()) {
            Tick delay = igbe->regs.nep_ex_regs[idx].tadv.idv() * igbe->intClock();
            DPRINTF(EthernetDesc, "setting tadv\n");
            if (!mqcomp.tadvMultEvent.scheduled()) {
                igbe->schedule(mqcomp.tadvMultEvent, curTick() + delay);
            }
        }
    }


    if (!useTso ||  TxdOp::getLen(desc) == tsoDescBytesUsed) {
        DPRINTF(EthernetDesc, "Descriptor Done\n");
        unusedCache.pop_front();
        usedCache.push_back(desc);
        tsoDescBytesUsed = 0;
    }

    if (useTso && tsoUsedLen == tsoTotalLen)
        useTso = false;


    DPRINTF(EthernetDesc,
            "------Packet of %d bytes ready for transmission-------\n",
            pktPtr->length);
    pktDone = true;
    pktWaiting = false;
    pktPtr = NULL;
    tsoPktHasHeader = false;

    if (igbe->regs.nep_ex_regs[idx].txdctl.wthresh() == 0) {
        
        DPRINTF(EthernetDesc, "WTHRESH == 0, writing back descriptor\n");
        writeback(0);
    } else if (!igbe->regs.nep_ex_regs[idx].txdctl.gran() && igbe->regs.nep_ex_regs[idx].txdctl.wthresh() <=
               descInBlock(usedCache.size())) {
        DPRINTF(EthernetDesc, "used > WTHRESH, writing back descriptor\n");
        
        writeback((igbe->cacheBlockSize()-1)>>4);
    } else if (igbe->regs.nep_ex_regs[idx].txdctl.wthresh() <= usedCache.size()) {
        DPRINTF(EthernetDesc, "used > WTHRESH, writing back descriptor\n");
        
        writeback((igbe->cacheBlockSize()-1)>>4);
    }

    enableSm();
    igbe->checkDrain();
}

void
NepGbE::TxDescCache::actionAfterWb()
{
    DPRINTF(EthernetDesc, "actionAfterWb() completionEnabled: %d\n",
            completionEnabled);
    igbe->postInterrupt(iGbReg::IT_TXDW);
    if (completionEnabled) {
        descEnd = igbe->regs.tdh();
        DPRINTF(EthernetDesc,
                "Completion writing back value: %d to addr: %#x\n", descEnd,
                completionAddress);
        igbe->dmaWrite(pciToDma(mbits(completionAddress, 63, 2)),
                       sizeof(descEnd), &nullEvent, (uint8_t*)&descEnd, 0);
    }
}

void
NepGbE::TxDescCache::serialize(CheckpointOut &cp) const
{
    DescCache<TxDesc>::serialize(cp);

    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(isTcp);
    SERIALIZE_SCALAR(pktWaiting);
    SERIALIZE_SCALAR(pktMultiDesc);

    SERIALIZE_SCALAR(useTso);
    SERIALIZE_SCALAR(tsoHeaderLen);
    SERIALIZE_SCALAR(tsoMss);
    SERIALIZE_SCALAR(tsoTotalLen);
    SERIALIZE_SCALAR(tsoUsedLen);
    SERIALIZE_SCALAR(tsoPrevSeq);;
    SERIALIZE_SCALAR(tsoPktPayloadBytes);
    SERIALIZE_SCALAR(tsoLoadedHeader);
    SERIALIZE_SCALAR(tsoPktHasHeader);
    SERIALIZE_ARRAY(tsoHeader, 256);
    SERIALIZE_SCALAR(tsoDescBytesUsed);
    SERIALIZE_SCALAR(tsoCopyBytes);
    SERIALIZE_SCALAR(tsoPkts);

    SERIALIZE_SCALAR(completionAddress);
    SERIALIZE_SCALAR(completionEnabled);
    SERIALIZE_SCALAR(descEnd);
}

void
NepGbE::TxDescCache::unserialize(CheckpointIn &cp)
{
    DescCache<TxDesc>::unserialize(cp);

    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(isTcp);
    UNSERIALIZE_SCALAR(pktWaiting);
    UNSERIALIZE_SCALAR(pktMultiDesc);

    UNSERIALIZE_SCALAR(useTso);
    UNSERIALIZE_SCALAR(tsoHeaderLen);
    UNSERIALIZE_SCALAR(tsoMss);
    UNSERIALIZE_SCALAR(tsoTotalLen);
    UNSERIALIZE_SCALAR(tsoUsedLen);
    UNSERIALIZE_SCALAR(tsoPrevSeq);;
    UNSERIALIZE_SCALAR(tsoPktPayloadBytes);
    UNSERIALIZE_SCALAR(tsoLoadedHeader);
    UNSERIALIZE_SCALAR(tsoPktHasHeader);
    UNSERIALIZE_ARRAY(tsoHeader, 256);
    UNSERIALIZE_SCALAR(tsoDescBytesUsed);
    UNSERIALIZE_SCALAR(tsoCopyBytes);
    UNSERIALIZE_SCALAR(tsoPkts);

    UNSERIALIZE_SCALAR(completionAddress);
    UNSERIALIZE_SCALAR(completionEnabled);
    UNSERIALIZE_SCALAR(descEnd);
}

bool
NepGbE::TxDescCache::packetAvailable()
{
    if (pktDone) {
        pktDone = false;
        return true;
    }
    return false;
}

void
NepGbE::TxDescCache::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        //igbe->txTick = true;
        igbe->mqManager.allTXTickOn();
        igbe->restartClock();
    }
}

bool
NepGbE::TxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled() || wbEvent.scheduled() ||
        fetchEvent.scheduled();
}


///////////////////////////////////// NepGbE /////////////////////////////////

void
NepGbE::restartClock()
{
    if (!tickEvent.scheduled() && (mqManager.checkRxTick() || mqManager.checkTxTick() || mqManager.checkTxFifoTick()) &&
        drainState() == DrainState::Running)
        schedule(tickEvent, clockEdge(Cycles(1)));
}

DrainState
NepGbE::drain()
{
    unsigned int count(0);
    /*if (rxDescCache.hasOutstandingEvents() ||
        txDescCache.hasOutstandingEvents()) {
        count++;
    }temp disabled*/

    //txFifoTick = false;
    //txTick = false;
    mqManager.allTXTickOff();
    mqManager.allTxFifoTickOff();
    mqManager.allRXTickOff();//rxTick = false

    if (tickEvent.scheduled())
        deschedule(tickEvent);

    if (count) {
        DPRINTF(Drain, "NepGbE not drained\n");
        return DrainState::Draining;
    } else
        return DrainState::Drained;
}

void
NepGbE::drainResume()
{
    Drainable::drainResume();

    // txFifoTick = true;
    // txTick = true;
    mqManager.allTXTickOn();
    mqManager.allTxFifoTickOn();
    mqManager.allRXTickOn();//rxTick = true;
    restartClock();
    DPRINTF(EthernetSM, "resuming from drain");
}

void
NepGbE::checkDrain()
{
    if (drainState() != DrainState::Draining)
        return;

    mqManager.allTXTickOff();
    mqManager.allTxFifoTickOff();
    mqManager.allRXTickOff();//rxTick = false;


    bool has = mqManager.hasOutstandingEvents(); //rxDescCache.hasOutstandingEvents()
    if (!has &&
        !txDescCache.hasOutstandingEvents()) {
        DPRINTF(Drain, "NepGbE done draining, processing drain event\n");
        signalDrainDone();
    }
}

void
NepGbE::txStateMachine()
{
    if (!regs.tctl.en()) {
        txTick = false;
        DPRINTF(EthernetSM, "TXS: TX disabled, stopping ticking\n");
        return;
    }

    // If we have a packet available and it's length is not 0 (meaning it's not
    // a multidescriptor packet) put it in the fifo, otherwise an the next
    // iteration we'll get the rest of the data
    if (txPacket && txDescCache.packetAvailable()
        && !txDescCache.packetMultiDesc() && txPacket->length) {
        DPRINTF(EthernetSM, "TXS: packet placed in TX FIFO\n");
#ifndef NDEBUG
        bool success =
#endif
            txFifo.push(txPacket);
        txFifoTick = true && drainState() != DrainState::Draining;
        assert(success);
        txPacket = NULL;
        txDescCache.writeback((cacheBlockSize()-1)>>4);
        return;
    }

    // Only support descriptor granularity
    if (regs.txdctl.lwthresh() &&
        txDescCache.descLeft() < (regs.txdctl.lwthresh() * 8)) {
        DPRINTF(EthernetSM, "TXS: LWTHRESH caused posting of TXDLOW\n");
        postInterrupt(IT_TXDLOW);
    }

    if (!txPacket) {
        txPacket = std::make_shared<EthPacketData>(16384);
    }

    if (!txDescCache.packetWaiting()) {
        if (txDescCache.descLeft() == 0) {
            postInterrupt(IT_TXQE);
            
            txDescCache.writeback(0);
            txDescCache.fetchDescriptors();
            DPRINTF(EthernetSM, "TXS: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n");
            txTick = false;
            return;
        }


        if (!(txDescCache.descUnused())) {
            
            txDescCache.fetchDescriptors();
            
            DPRINTF(EthernetSM, "TXS: No descriptors available in cache, "
                    "fetching and stopping ticking\n");
            txTick = false;
            return;
        }
        


        txDescCache.processContextDesc();
        if (txDescCache.packetWaiting()) {
            DPRINTF(EthernetSM,
                    "TXS: Fetching TSO header, stopping ticking\n");
            txTick = false;
            return;
        }

        unsigned size = txDescCache.getPacketSize(txPacket);
        if (size > 0 && txFifo.avail() > size) {
            
            DPRINTF(EthernetSM, "TXS: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", size);
            txFifo.reserve(size);
            txDescCache.getPacketData(txPacket);
        } else if (size == 0) {
            DPRINTF(EthernetSM, "TXS: getPacketSize returned: %d\n", size);
            DPRINTF(EthernetSM,
                    "TXS: No packets to get, writing back used descriptors\n");
            
            txDescCache.writeback(0);
        } else {
            
            DPRINTF(EthernetSM, "TXS: FIFO full, stopping ticking until space "
                    "available in FIFO\n");
            txTick = false;
        }


        return;
    }
    DPRINTF(EthernetSM, "TXS: Nothing to do, stopping ticking\n");
    txTick = false;
}

bool
NepGbE::ethRxPkt(EthPacketPtr pkt)
{
    DPRINTF(NepNicOthers, "ethRxPkt called %#x\n", pkt);
    return mqManager.InsertEthPktToRxQ(pkt);
}

PacketFifo&
NepGbE::MultiQueueManager::getTxFifo(){
    int idx_curr = CurrentTxWire();
    DPRINTF(NepTx,"getTxFifo idx_curr %d\n", idx_curr);
    //int first = idx_curr;
    
    MultiQueueComponentSet& curr_entry = *mq_component_sets[idx_curr];
    idx_last_deQ_try=idx_curr;
    //bool& rxTick = curr_entry.available;

    PacketFifo& txFifo = curr_entry.txfifo;

    setIdxTxFifoTick(idx_curr, false);
    return txFifo;
}

int
NepGbE::txWire()
{
    PacketFifo& txFifo = mqManager.getTxFifo();

    int idx = txFifo.getIdx();

    DPRINTF(NepTx,"TxFifo idx %d\n", idx);
    
    //txFifoTick = false;
    DPRINTF(EthernetSM, "NepGbE::txWire() is called\n");

    if (txFifo.empty()) {
        
        DPRINTF(EthernetSM, "TxFIFO: FIFO EMPTY\n");
        return -1;
    }

    DPRINTF(EthernetSM, "NepGbE::txWire() FIFO is not empty\n");

    
    if (etherInt->sendPacket(txFifo.front())) {
        
        if (DTRACE(EthernetSM)) {
            IpPtr ip(txFifo.front());
            if (ip)
                DPRINTF(EthernetSM, "Transmitting Ip packet with Id=%d\n",
                        ip->id());
            else
                DPRINTF(EthernetSM, "Transmitting Non-Ip packet\n");
        }
        DPRINTF(EthernetSM,
                "TxFIFO: Successful transmit, bytes available in fifo: %d\n",
                txFifo.avail());

        etherDeviceStats.txBytes += txFifo.front()->length;
        etherDeviceStats.txPackets++;

        txFifo.pop();
    }
    DPRINTF(EthernetSM, "NepGbE::txWire() return idx %d\n", idx);
    return idx;
}

void
NepGbE::SimpleLoad()
{
    // BST. Test Simple Load
    test_count = std::rand();
    DPRINTF(NepNicOthers, "nep_test\n");
    if(test_packet==NULL){
        test_packet=std::make_shared<EthPacketData>(16384);
        struct eth_hdr test_eth_hdr;
        test_eth_hdr.eth_type=htons(ETH_TYPE_IP);
        //test_eth_hdr.eth_type=8;
        memcpy(test_packet->data, &test_eth_hdr, ETH_HDR_LEN);
        //DPRINTF(NepNicOthers,"nep_test: eth dst%d\n",);
        struct ip_hdr test_ip_hdr;
        
        /*
        int ip_pool[4];
        ip_pool[0]=3000000000;
        ip_pool[1]=970000000;
        ip_pool[2]=3600000000;
        ip_pool[3]=1000000000;
        */
        //test_ip_hdr.ip_dst=htonl(ip_pool[test_count/5000%4]);
        //test_ip_hdr.ip_dst=htonl(3232235521);//192.168.0.1
        test_ip_hdr.ip_dst=htonl(rand());
        test_ip_hdr.ip_src=htonl(rand());//10.150.4.25
        test_ip_hdr.ip_hl=5;//ramdom size
        test_ip_hdr.ip_p=IP_PROTO_TCP;
        struct tcp_hdr test_tcp_hdr;
        test_tcp_hdr.th_sport=htonl(rand());
        test_tcp_hdr.th_dport=htonl(rand());
        memcpy(test_packet->data+ETH_HDR_LEN, &test_ip_hdr, test_ip_hdr.ip_hl*4);
        IpPtr ip(test_packet);      //ip4정보
        DPRINTF(NepNicOthers,"nep_test: ip dst %d\n",ip->dst());
        DPRINTF(NepNicOthers,"nep_test: ip src %d\n",ip->src());
        memcpy(test_packet->data+ETH_HDR_LEN+test_ip_hdr.ip_hl*4, &test_tcp_hdr, TCP_HDR_LEN);
        char test_text[4][14] = {"Neptunia", "PurpleHeart", "NoireChan", "KSBTKSBT"};
        DPRINTF(NepNicOthers, "nep_test. str %s\n",test_text[test_count % 4]);
        //char test_data[14] ="Neptunia";
        memcpy(test_packet->data+ETH_HDR_LEN+test_ip_hdr.ip_hl*4+TCP_HDR_LEN, test_text[test_count % 4], 14);
        test_packet->length=260;
        test_packet->simLength=260;
    }
    DPRINTF(EthernetSM, "nep_make_packet\n");
    ethRxPkt(test_packet);
    DPRINTF(EthernetSM, "packet_done\n");
    test_packet=NULL;
    
    schedule(loadGenEvent, curTick() + 50*SimClock::Int::ms);
}

void
NepGbE::tick()
{
    DPRINTF(EthernetSM, "NepGbE: -------------- Cycle --------------\n");

    

    inTick = true;

    /*if (rxTick)
        rxStateMachine();*/
    // TODO Change to while
    uint32_t rx_q_mask = 0;
    while(mqManager.checkRxTick()){
        int curr_q = mqManager.RunRxTick();
        DPRINTF(EthernetSM, "RxTick qid %d\n", curr_q);
        
        //if(curr_q < 0) break;
        int curr_mask = 1 << curr_q;

        //if(rx_q_mask & curr_mask) break;
        rx_q_mask |= curr_mask;
    }
    // if(mqManager.checkRxTick())
    //     mqManager.RunRxTick();

    // if (txTick)
    //     txStateMachine();
    // TODO Change to while
    // while(mqManager.checkTxTick())
    //     mqManager.RunTxTick();
    uint32_t tx_q_mask = 0;
    while(mqManager.checkTxTick()){
        int curr_q = mqManager.RunTxTick();
        DPRINTF(EthernetSM, "TxTick qid %d\n", curr_q);
        
        //if(curr_q < 0) break;
        int curr_mask = 1 << curr_q;

        //if(tx_q_mask & curr_mask) break;
        tx_q_mask |= curr_mask;
    }
    // if(mqManager.checkTxTick())
    //     mqManager.RunTxTick();

    // If txWire returns and txFifoTick is still set, that means the data we
    // sent to the other end was already accepted and we can send another
    // frame right away. This is consistent with the previous behavior which
    // would send another frame if one was ready in ethTxDone. This version
    // avoids growing the stack with each frame sent which can cause stack
    // overflow.
    while (mqManager.checkTxFifoTick()){
        int curr_q = txWire();
        if(curr_q > -1)
            DPRINTF(EthernetSM, "checkTxFifoTick qid %d\n", curr_q);
    }

    if (mqManager.checkRxTick() || mqManager.checkTxTick() || mqManager.checkTxFifoTick())
        schedule(tickEvent, curTick() + clockPeriod());

    inTick = false;
}

void
NepGbE::ethTxDone()
{
    // restart the tx state machines if they are stopped
    // fifo to send another packet
    // tx sm to put more data into the fifo
    for(int i = 0; i < mqManager.getNumTxQueues(); i++){
        mqManager.setIdxTxFifoTick(i, true && drainState() != DrainState::Draining);
        if(mqManager.getCompByIdx(i).txDescCache.descLeft() != 0 && drainState() != DrainState::Draining)
            mqManager.setIdxTxTick(i, true);
    }

    if (!inTick)
        restartClock();
    DPRINTF(EthernetSM, "TxFIFO: Transmission complete\n");
}

int
NepGbE::addrToQid(Addr daddr) {
    return (daddr / (uint64_t)REG_END) - 1;
}

void
NepGbE::serialize(CheckpointOut &cp) const
{
    PciDevice::serialize(cp);

    DPRINTF(EthernetSM, "NepGbE::serialize\n");

    regs.serialize(cp);
    SERIALIZE_SCALAR(eeOpBits);
    SERIALIZE_SCALAR(eeAddrBits);
    SERIALIZE_SCALAR(eeDataBits);
    SERIALIZE_SCALAR(eeOpcode);
    SERIALIZE_SCALAR(eeAddr);
    SERIALIZE_SCALAR(lastInterrupt);
    SERIALIZE_ARRAY(flash,iGbReg::EEPROM_SIZE);

    //rxFifo.serialize("rxfifo", cp);
    txFifo.serialize("txfifo", cp);

    bool txPktExists = txPacket != nullptr;
    SERIALIZE_SCALAR(txPktExists);
    if (txPktExists)
        txPacket->serialize("txpacket", cp);

    Tick rdtr_time = 0, radv_time = 0, tidv_time = 0, tadv_time = 0,
        inter_time = 0;

    if (rdtrEvent.scheduled())
        rdtr_time = rdtrEvent.when();
    SERIALIZE_SCALAR(rdtr_time);

    if (radvEvent.scheduled())
        radv_time = radvEvent.when();
    SERIALIZE_SCALAR(radv_time);

    if (tidvEvent.scheduled())
        tidv_time = tidvEvent.when();
    SERIALIZE_SCALAR(tidv_time);

    if (tadvEvent.scheduled())
        tadv_time = tadvEvent.when();
    SERIALIZE_SCALAR(tadv_time);

    if (interEvent.scheduled())
        inter_time = interEvent.when();
    SERIALIZE_SCALAR(inter_time);

    SERIALIZE_SCALAR(pktOffset);

    txDescCache.serializeSection(cp, "TxDescCache");
    //rxDescCache.serializeSection(cp, "RxDescCache");
}

void
NepGbE::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);

    DPRINTF(EthernetSM, "NepGbE::unserialize\n");

    regs.unserialize(cp);
    UNSERIALIZE_SCALAR(eeOpBits);
    UNSERIALIZE_SCALAR(eeAddrBits);
    UNSERIALIZE_SCALAR(eeDataBits);
    UNSERIALIZE_SCALAR(eeOpcode);
    UNSERIALIZE_SCALAR(eeAddr);
    UNSERIALIZE_SCALAR(lastInterrupt);
    UNSERIALIZE_ARRAY(flash,iGbReg::EEPROM_SIZE);

    //rxFifo.unserialize("rxfifo", cp);
    txFifo.unserialize("txfifo", cp);

    bool txPktExists;
    UNSERIALIZE_SCALAR(txPktExists);
    if (txPktExists) {
        txPacket = std::make_shared<EthPacketData>(16384);
        txPacket->unserialize("txpacket", cp);
    }

    mqManager.allRXTickOn();//rxTick = true;
    txTick = true;
    txFifoTick = true;

    Tick rdtr_time, radv_time, tidv_time, tadv_time, inter_time;
    UNSERIALIZE_SCALAR(rdtr_time);
    UNSERIALIZE_SCALAR(radv_time);
    UNSERIALIZE_SCALAR(tidv_time);
    UNSERIALIZE_SCALAR(tadv_time);
    UNSERIALIZE_SCALAR(inter_time);

    if (rdtr_time)
        schedule(rdtrEvent, rdtr_time);

    if (radv_time)
        schedule(radvEvent, radv_time);

    if (tidv_time)
        schedule(tidvEvent, tidv_time);

    if (tadv_time)
        schedule(tadvEvent, tadv_time);

    if (inter_time)
        schedule(interEvent, inter_time);

    UNSERIALIZE_SCALAR(pktOffset);

    txDescCache.unserializeSection(cp, "TxDescCache");
    //rxDescCache.unserializeSection(cp, "RxDescCache");
}


// SHIN. Get ref of component set for enQ
NepGbE::MultiQueueComponentSet&
NepGbE::MultiQueueManager::getTargetRxQ(EthPacketPtr epkt)
{
    int q = (uint32_t)(rx_policy->getTargetRxQ(epkt));//%num_rx_queues;
    DPRINTF(NepNicRxManager, "NepGbE::RxManager::getTargetRxQ. Q is %d\r", q);
    //q=0;
    return *mq_component_sets[q];
} // For This Project, we will use hashing

// SHIN. Get ref of component set for deQ
int
NepGbE::MultiQueueManager::CurrentRx()
{
    return rx_policy->CurrentSender(map_active_queues, num_rx_queues);
} // For this project, we will use RR

int
NepGbE::MultiQueueManager::CurrentTx()
{
    return tx_policy->CurrentTx((char*)&num_tx_queues);
} // For this project, we will use RR

int
NepGbE::MultiQueueManager::CurrentTxWire()
{
    return tx_policy->CurrentTxWire((char*)&num_tx_queues);
} // For this project, we will use RR

// SHIN. BST
// 와이어에서 받아서 넣는건데 일단 이렇게 빼봄.
bool 
NepGbE::MultiQueueManager::InsertEthPktToRxQ(EthPacketPtr pkt){
    // SHIN. For debug
    DPRINTF(NepNicOthers, "InsertEthPktToRxQ igbe addr %#x\n", igbe);

    // SHIN. Set Refs
    iGbReg::Regs& regs = igbe->regs;
    MultiQueueComponentSet& curr_entry = getTargetRxQ(pkt);
    idx_last_enQ_try = curr_entry.idxComponent;
    bool& rxTick = curr_entry.rx_available;    
    PacketFifo& rxFifo = curr_entry.rxfifo;
    bool& txTick = curr_entry.tx_available; // It was created for later use in tx's policy research.
    EventFunctionWrapper& tickEvent = igbe->tickEvent;
    
    System* sys = igbe->sys;
    uint64_t& macAddr = igbe->macAddr;
    //map_active_queues=(1<<idx_last_enQ_try)|map_active_queues;//BST change active queue map

    // Stats...
    igbe->etherDeviceStats.rxBytes += pkt->length;//may be need to be changed
    igbe->etherDeviceStats.rxPackets++;


    DPRINTF(Ethernet, "RxFIFO: Receiving pcakte from wire\n");
    
    if (!regs.nep_ex_regs[idx_last_enQ_try].rctl.en()) {//changed mul_reg
        DPRINTF(Ethernet, "RxFIFO: RX not enabled, dropping\n");
        
        return true;
    }

    // restart the state machines if they are stopped
    rxTick = true && igbe->drainState() != DrainState::Draining;
    setIdxRxTick(idx_last_enQ_try, rxTick);
    if ((rxTick || txTick) && !tickEvent.scheduled()) {
        DPRINTF(EthernetSM,
                "RXS: received packet into fifo, starting ticking\n");
        
        // SHIN. Temp Disable!!
        igbe->restartClock();
    }

    if (!rxFifo.push(pkt)) {
        DPRINTF(Ethernet, "RxFIFO: Packet won't fit in fifo... dropped\n");
        
        //if(multiqueue)
            curr_entry.postInterruptMultIntr(IT_RXO, true);
        //else
            //igbe->postInterrupt(IT_RXO, true);
        
        return false;
    }


    return true;
}

// SHIN.
// DMA 영역이 바뀌었을 때 처리
bool
NepGbE::MultiQueueManager::areaChanged(){
    int i;
    for(i=0; i<num_rx_queues; i++){
        mq_component_sets[i]->rxDescCache.areaChanged();
        
        // set Offsets
        mq_component_sets[i]->rxDescCache.setOffset(0);      // BST. 이거 어쩔지 같이 생각해보자.
    }
    return true;
}

// SHIN. BST
// 호스트로 보내는건데 일단 이렇게 빼봄
int 
NepGbE::MultiQueueManager::RunRxTick(){   // Bring NepGbE::rxStateMachine() from NepGbE
    iGbReg::Regs& regs = igbe->regs;
    DPRINTF(NepNicOthers, "RunRxTick\n");
    int idx_curr=CurrentRx();
    DPRINTF(NepNicOthers, "get current sender\n");
    if(idx_curr==-1)//no current sender
    {
        DPRINTF(NepNicOthers, "No sender\n");
        return idx_curr;
    }
    idx_last_deQ_try=idx_curr;
    MultiQueueComponentSet& curr_entry = *mq_component_sets[idx_curr];
    idx_last_deQ_try=idx_curr;
    //bool& rxTick = curr_entry.available;
    RxDescCache& rxDescCache = curr_entry.rxDescCache;
    bool& rxDmaPacket = curr_entry.rxDmaPacket;
    PacketFifo& rxFifo = curr_entry.rxfifo;
    int& pktOffset = curr_entry.pktOffset;
    if (!regs.nep_ex_regs[idx_curr].rctl.en()) {//changed multi reg
        setIdxRxTick(idx_curr,false);
        DPRINTF(EthernetSM, "RXS: RX disabled, stopping ticking\n");
        return idx_curr;
    }

    // If the packet is done check for interrupts/descriptors/etc
    if (rxDescCache.packetDone()) {
        rxDmaPacket = false;
        DPRINTF(EthernetSM, "RXS: Packet completed DMA to memory\n");
        int descLeft = rxDescCache.descLeft();
        DPRINTF(EthernetSM, "RXS: descLeft: %d rdmts: %d rdlen: %d\n",
                descLeft, regs.nep_ex_regs[idx_curr].rctl.rdmts(), regs.nep_ex_regs[idx_curr].rdlen());

        // rdmts 2->1/8, 1->1/4, 0->1/2
        int ratio = (1ULL << (regs.nep_ex_regs[idx_curr].rctl.rdmts() + 1));
        if (descLeft * ratio <= regs.nep_ex_regs[idx_curr].rdlen()) {
            DPRINTF(Ethernet, "RXS: Interrupting (RXDMT) "
                    "because of descriptors left\n");
            //if(multiqueue)
                curr_entry.postInterruptMultIntr(IT_RXDMT);
            //else
                //igbe->postInterrupt(IT_RXDMT);
        }

        if (rxFifo.empty())
            rxDescCache.writeback(0);

        if (descLeft == 0) {
            
            rxDescCache.writeback(0);
            DPRINTF(EthernetSM, "RXS: No descriptors left in ring, forcing"
                    " writeback and stopping ticking\n");
            setIdxRxTick(idx_curr,false);
        }

        // only support descriptor granulaties
        assert(regs.nep_ex_regs[idx_curr].rxdctl.gran());

        if (regs.nep_ex_regs[idx_curr].rxdctl.wthresh() >= rxDescCache.descUsed()) {
            DPRINTF(EthernetSM,
                    "RXS: Writing back because WTHRESH >= descUsed\n");
            
            if (regs.nep_ex_regs[idx_curr].rxdctl.wthresh() < (igbe->cacheBlockSize()>>4))
                rxDescCache.writeback(regs.nep_ex_regs[idx_curr].rxdctl.wthresh()-1);
            else
                rxDescCache.writeback((igbe->cacheBlockSize()-1)>>4);
        }

        if ((rxDescCache.descUnused() < regs.nep_ex_regs[idx_curr].rxdctl.pthresh()) &&
            ((rxDescCache.descLeft() - rxDescCache.descUnused()) >
             regs.nep_ex_regs[idx_curr].rxdctl.hthresh())) {
            DPRINTF(EthernetSM, "RXS: Fetching descriptors because "
                    "descUnused < PTHRESH\n");
            
            rxDescCache.fetchDescriptors();
        }

        if (rxDescCache.descUnused() == 0) {
            
            rxDescCache.fetchDescriptors();
            
            DPRINTF(EthernetSM, "RXS: No descriptors available in cache, "
                    "fetching descriptors and stopping ticking\n");
            setIdxRxTick(idx_curr,false);
        }
        return idx_curr;
    }

    if (rxDmaPacket) {
        DPRINTF(EthernetSM,
                "RXS: stopping ticking until packet DMA completes\n");
        setIdxRxTick(idx_curr,false);
        return idx_curr;
    }

    if (!rxDescCache.descUnused()) {
        
        rxDescCache.fetchDescriptors();
        
        DPRINTF(EthernetSM, "RXS: No descriptors available in cache, "
                "stopping ticking\n");
        setIdxRxTick(idx_curr,false);
        DPRINTF(EthernetSM, "RXS: No descriptors available, fetching\n");
        return idx_curr;
    }
    

    if (rxFifo.empty()) {
    
        DPRINTF(EthernetSM, "RXS: RxFIFO empty, stopping ticking\n");
        setIdxRxTick(idx_curr,false);
        return idx_curr;
    }
    
    EthPacketPtr pkt;
    pkt = rxFifo.front();


    pktOffset = rxDescCache.writePacket(pkt, pktOffset);
    DPRINTF(EthernetSM, "RXS: Writing packet into memory\n");
    if (pktOffset == pkt->length) {
    
        DPRINTF(EthernetSM, "RXS: Removing packet from FIFO\n");
        pktOffset = 0;
    
        rxFifo.pop();
    }

    DPRINTF(EthernetSM, "RXS: stopping ticking until packet DMA completes\n");
    setIdxRxTick(idx_curr,false);
    rxDmaPacket = true;
    
    return idx_curr;
}

int
NepGbE::MultiQueueManager::RunTxTick(){
    iGbReg::Regs& regs = igbe->regs;
    DPRINTF(NepNicOthers, "RunTxTick\n");
    int idx_curr=CurrentTx();
    //DPRINTF(NepNicOthers, "get current sender\n");
    DPRINTF(NepNicOthers, "RunTxTick idx_curr is %d\n", idx_curr);
    if(idx_curr==-1)//no current sender
    {
        DPRINTF(NepNicOthers, "RunTxTick idx_curr is %d\n", idx_curr);
        return idx_curr;
    }
    idx_last_deQ_try=idx_curr;
    MultiQueueComponentSet& curr_entry = *mq_component_sets[idx_curr];
    idx_last_deQ_try=idx_curr;
    //bool& rxTick = curr_entry.available;
    TxDescCache& txDescCache = curr_entry.txDescCache;
    EthPacketPtr& txPacket = curr_entry.txPacket;
    PacketFifo& txFifo = curr_entry.txfifo;
    
    //curr_entry.rdtrProcessMultIntr();
    if (!regs.nep_ex_regs[idx_curr].tctl.en()) {
        setIdxTxTick(idx_curr, false);
        DPRINTF(EthernetSM, "TXS: TX%d disabled, stopping ticking\n", idx_curr);
        return idx_curr;
    }

    // If we have a packet available and it's length is not 0 (meaning it's not
    // a multidescriptor packet) put it in the fifo, otherwise an the next
    // iteration we'll get the rest of the data
    if (txPacket && txDescCache.packetAvailable()
        && !txDescCache.packetMultiDesc() && txPacket->length) {
    
        DPRINTF(EthernetSM, "TXS: packet placed in TX FIFO\n");
#ifndef NDEBUG
        bool success =
#endif
            txFifo.push(txPacket);
        
        curr_entry.txfifo_available = true && igbe->drainState() != DrainState::Draining;
        assert(success);
        txPacket = NULL;
    
        txDescCache.writeback((igbe->cacheBlockSize()-1)>>4);
        return idx_curr;
    }

    // Only support descriptor granularity
    if (regs.txdctl.lwthresh() &&
        txDescCache.descLeft() < (regs.txdctl.lwthresh() * 8)) {
        DPRINTF(EthernetSM, "TXS: LWTHRESH caused posting of TXDLOW\n");
        curr_entry.postInterruptMultIntr(iGbReg::IT_TXDLOW);
    }

    if (!txPacket) {
        txPacket = std::make_shared<EthPacketData>(16384);
    }

    if (!txDescCache.packetWaiting()) {
        if (txDescCache.descLeft() == 0) {
            curr_entry.postInterruptMultIntr(iGbReg::IT_TXQE);
    
            txDescCache.writeback(0);
            txDescCache.fetchDescriptors();
            DPRINTF(EthernetSM, "TXS: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n");
            setIdxTxTick(idx_curr, false);
            return idx_curr;
        }


        if (!(txDescCache.descUnused())) {
            txDescCache.fetchDescriptors();
            DPRINTF(EthernetSM, "TXS: No descriptors available in cache, "
                    "fetching and stopping ticking\n");
            setIdxTxTick(idx_curr, false);
            return idx_curr;
        }
    

        txDescCache.processContextDesc();
        if (txDescCache.packetWaiting()) {
            DPRINTF(EthernetSM,
                    "TXS: Fetching TSO header, stopping ticking\n");
            setIdxTxTick(idx_curr, false);
            return idx_curr;
        }

        unsigned size = txDescCache.getPacketSize(txPacket);
        if (size > 0 && txFifo.avail() > size) {
            DPRINTF(EthernetSM, "TXS: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", size);
            txFifo.reserve(size);
            txDescCache.getPacketData(txPacket);
        } else if (size == 0) {
            DPRINTF(EthernetSM, "TXS: getPacketSize returned: %d\n", size);
            DPRINTF(EthernetSM,
                    "TXS: No packets to get, writing back used descriptors\n");
            txDescCache.writeback(0);
        } else {
            DPRINTF(EthernetSM, "TXS: FIFO full, stopping ticking until space "
                    "available in FIFO\n");
            setIdxTxTick(idx_curr, false);
        }


        return idx_curr;
    }
    DPRINTF(EthernetSM, "TXS: Nothing to do, stopping ticking\n");
    setIdxTxTick(idx_curr, false);
    return idx_curr;
}

// SHIN. TO DO
void
NepGbE::MultiQueueManager::setRxDecsAddr(){}
void
NepGbE::MultiQueueManager::setTxDecsAddr(){}

void
NepGbE::MultiQueueManager::setIdxRxTick(int idx, bool val){//Set RxTick on or off
    mq_component_sets[idx]->rx_available=val;
    //if(map_active_queues&(1<<idx)&&!val){
    //    map_active_queues-=(1<<idx);
    //}
}

void
NepGbE::MultiQueueManager::setIdxTxTick(int idx, bool val){
    mq_component_sets[idx]->tx_available=val;
}

void
NepGbE::MultiQueueManager::setIdxTxFifoTick(int idx, bool val){
    mq_component_sets[idx]->txfifo_available=val;
}

void
NepGbE::MultiQueueManager::allRXTickOff(){
    for(int i=0; i<num_rx_queues; i++)
    {
        setIdxRxTick(i,false);   
    }
}

void
NepGbE::MultiQueueManager::allRXTickOn(){
    for(int i=0; i<num_rx_queues; i++)
    {
        setIdxRxTick(i,true);   
    }
}

void
NepGbE::MultiQueueManager::allTXTickOff(){
    for(int i=0; i<num_tx_queues; i++)
    {
        setIdxTxTick(i,false);   
    }
}

void
NepGbE::MultiQueueManager::allTXTickOn(){
    for(int i=0; i<num_tx_queues; i++)
    {
        setIdxTxTick(i,true);   
    }
}

void
NepGbE::MultiQueueManager::allTxFifoTickOff(){
    for(int i=0; i<num_tx_queues; i++)
    {
        setIdxTxFifoTick(i,false);   
    }
}

void
NepGbE::MultiQueueManager::allTxFifoTickOn(){
    for(int i=0; i<num_tx_queues; i++)
    {
        setIdxTxFifoTick(i,true);   
    }
}


// SHIN. RxManager의 생성자
// 그냥 내부 구성요소들을 생성하는게 하는 일의 전부임
// 크기 정책은 일단 걍 나눠서 씀
NepGbE::MultiQueueManager::MultiQueueManager(NepGbE* __igbe, 
                        int num_rx_q, int rx_queue_max_size, int rx_desc_max_size,
                        int num_tx_q, int tx_queue_max_size, int tx_desc_max_size)
    :igbe(__igbe), num_rx_queues(num_rx_q), num_tx_queues(num_tx_q)
{
    DPRINTF(NepNicOthers, "Rx Manager Init Start! Max Q %d\n", num_rx_q);
    _name = __igbe->name()+".rx_manager";
    int i =0;
    
    for (; i < num_rx_q; i++){
        // MultiQueueComponentSet* elem = new MultiQueueComponentSet(__igbe, 
        //         rx_queue_max_size / num_rx_q, rx_desc_max_size / num_rx_q,
        //         tx_queue_max_size / num_tx_q, tx_desc_max_size / num_tx_q, _name, i);
        MultiQueueComponentSet* elem = new MultiQueueComponentSet(__igbe, 
                rx_queue_max_size, rx_desc_max_size,
                tx_queue_max_size, tx_desc_max_size, _name, i);
        mq_component_sets.push_back(elem);
        DPRINTF(NepNicRxManager, "Rx Manager push Queue %d\n", i);
    }
    rx_policy = new RxSidePolicyRSS(__igbe,num_rx_q);
    tx_policy = new TxSidePolicyRSS(__igbe);
    DPRINTF(NepNicOthers, "Rx Manager Init End!\n");

}


// SHIN.
// 같이 쓰이는거 묶음의 생성자일 뿐.
NepGbE::MultiQueueComponentSet::MultiQueueComponentSet(NepGbE* igbe, int rxfifo_size, int rx_desc_size, 
            int txfifo_size, int tx_desc_size, std::string _manager_name, int idx)
    :rxDescCache(igbe,_manager_name+".RxDesc["+std::to_string(idx)+"]", rx_desc_size), rxfifo(rxfifo_size), 
    txDescCache(igbe,_manager_name+".TxDesc["+std::to_string(idx)+"]", tx_desc_size), txfifo(txfifo_size), 
    idxComponent(idx), igbe(igbe), _name(_manager_name + ".CompSet[" + std::to_string(idx) + "]"),
    interMultEvent([this]{ delayIntEventMultIntr(); }, name() + ".InterEvent"),
    rdtrMultEvent([this]{ rdtrProcessMultIntr(); },    name() + ".rdtrEvent"),
    radvMultEvent([this]{ radvProcessMultIntr(); },    name() + ".radvEvent"),
    tadvMultEvent([this]{ tadvProcessMultIntr(); },    name() + ".tadvEvent"),
    tidvMultEvent([this]{ tidvProcessMultIntr(); },    name() + ".tidvEvent")
{
    rxDescCache.setIdx(idx);
    txDescCache.setIdx(idx);
    rxfifo.setIdx(idx);
    txfifo.setIdx(idx);
}

// NepGbE::TxComponentSet::TxComponentSet(NepGbE* igbe, int Qsize, int desc_size, std::string _manager_name, int idx)
//     :txDescCache(igbe,_manager_name+".TxDesc["+std::to_string(idx)+"]", desc_size), txfifo(Qsize), idxComponent(idx)
// {
//     //txDescCache.setIdx(idx);
// }

RxSidePolicy::RxSidePolicy(NepGbE* __igbe){
    igbe = __igbe;
}
TxSidePolicy::TxSidePolicy(NepGbE* __igbe){
    igbe = __igbe;
}
NepGbE::MultiQueueManager::~MultiQueueManager(){
    delete rx_policy;
    while(mq_component_sets.size()){
        MultiQueueComponentSet* elem = mq_component_sets.front();
        mq_component_sets.pop_back();
        delete elem;
    }
}


void
NepGbE::MultiQueueComponentSet::delayIntEventMultIntr()
{
    cpuPostMultInt();
}

void
NepGbE::MultiQueueComponentSet::rdtrProcessMultIntr()
{
    rxDescCache.writeback(0);
    DPRINTF(NepNicIntrMsi,
            "NepNic! Posting RXT interrupt because RDTR timer expired\n");
    //cpuPostMultInt
    //postInterrupt(iGbReg::IT_RXT);
    postInterruptMultIntr(iGbReg::IT_RXT);
}

void
NepGbE::MultiQueueComponentSet::radvProcessMultIntr()
{
    rxDescCache.writeback(0);
    DPRINTF(NepNicIntrMsi,
            "NepNic! Posting RXT interrupt because RADV timer expired\n");
    //postInterrupt(iGbReg::IT_RXT);
    postInterruptMultIntr(iGbReg::IT_RXT);
}

void
NepGbE::MultiQueueComponentSet::tadvProcessMultIntr()
{
    txDescCache.writeback(0);
    DPRINTF(NepNicIntrMsi,
            "NepNic! Posting TXDW interrupt because TADV timer expired\n");
    //postInterrupt(iGbReg::IT_TXDW);
    postInterruptMultIntr(iGbReg::IT_TXDW);
}

void
NepGbE::MultiQueueComponentSet::tidvProcessMultIntr()
{
    txDescCache.writeback(0);
    DPRINTF(NepNicIntrMsi,
            "NepNic! Posting TXDW interrupt because TIDV timer expired\n");
    //postInterrupt(iGbReg::IT_TXDW);
    postInterruptMultIntr(iGbReg::IT_TXDW);
}