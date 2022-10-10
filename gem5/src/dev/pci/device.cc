/*
 * Copyright (c) 2013, 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 */

/* @file
 * A single PCI device configuration space entry.
 */

#include "dev/pci/device.hh"

#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "debug/PciDevice.hh"
#include "debug/NepMsi.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"

#include "dev/dma_engine.hh"

PciDevice::PciDevice(const PciDeviceParams &p)
    : DmaDevice(p),
      _busAddr(p.pci_bus, p.pci_dev, p.pci_func),
      PMCAP_BASE(p.PMCAPBaseOffset),
      PMCAP_ID_OFFSET(p.PMCAPBaseOffset+PMCAP_ID),
      PMCAP_PC_OFFSET(p.PMCAPBaseOffset+PMCAP_PC),
      PMCAP_PMCS_OFFSET(p.PMCAPBaseOffset+PMCAP_PMCS),
      MSICAP_BASE(p.MSICAPBaseOffset),
      MSIXCAP_BASE(p.MSIXCAPBaseOffset),
      MSIXCAP_ID_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_ID),
      MSIXCAP_MXC_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_MXC),
      MSIXCAP_MTAB_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_MTAB),
      MSIXCAP_MPBA_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_MPBA),
      PXCAP_BASE(p.PXCAPBaseOffset),

      hostInterface(p.host->registerDevice(this, _busAddr,
                                            (PciIntPin)p.InterruptPin)),
      pioDelay(p.pio_latency),
      configDelay(p.config_latency)
{
    fatal_if(p.InterruptPin >= 5,
             "Invalid PCI interrupt '%i' specified.", p.InterruptPin);

    BARs[0] = p.BAR0;
    BARs[1] = p.BAR1;
    BARs[2] = p.BAR2;
    BARs[3] = p.BAR3;
    BARs[4] = p.BAR4;
    BARs[5] = p.BAR5;

    int idx = 0;
    for (auto *bar: BARs) {
        auto *mu = dynamic_cast<PciMemUpperBar *>(bar);
        // If this is the upper 32 bits of a memory BAR, try to connect it to
        // the lower 32 bits.
        if (mu) {
            fatal_if(idx == 0,
                    "First BAR in %s is upper 32 bits of a memory BAR.", idx);
            auto *ml = dynamic_cast<PciMemBar *>(BARs[idx - 1]);
            fatal_if(!ml, "Upper 32 bits of memory BAR in %s doesn't come "
                    "after the lower 32.");
            mu->lower(ml);
        }
        idx++;
    }

    config.vendor = htole(p.VendorID);
    config.device = htole(p.DeviceID);
    config.command = htole(p.Command);
    config.status = htole(p.Status);
    config.revision = htole(p.Revision);
    config.progIF = htole(p.ProgIF);
    config.subClassCode = htole(p.SubClassCode);
    config.classCode = htole(p.ClassCode);
    config.cacheLineSize = htole(p.CacheLineSize);
    config.latencyTimer = htole(p.LatencyTimer);
    config.headerType = htole(p.HeaderType);
    config.bist = htole(p.BIST);

    idx = 0;
    for (auto *bar: BARs)
        config.baseAddr[idx++] = bar->write(hostInterface, 0);

    config.cardbusCIS = htole(p.CardbusCIS);
    config.subsystemVendorID = htole(p.SubsystemVendorID);
    config.subsystemID = htole(p.SubsystemID);
    config.expansionROM = htole(p.ExpansionROM);
    config.capabilityPtr = htole(p.CapabilityPtr);
    // Zero out the 7 bytes of reserved space in the PCI Config space register.
    bzero(config.reserved, 7*sizeof(uint8_t));
    config.interruptLine = htole(p.InterruptLine);
    config.interruptPin = htole(p.InterruptPin);
    config.minimumGrant = htole(p.MinimumGrant);
    config.maximumLatency = htole(p.MaximumLatency);

    // Initialize the capability lists
    // These structs are bitunions, meaning the data is stored in host
    // endianess and must be converted to Little Endian when accessed
    // by the guest
    // PMCAP
    pmcap.pid = (uint16_t)p.PMCAPCapId; // pid.cid
    pmcap.pid |= (uint16_t)p.PMCAPNextCapability << 8; //pid.next
    pmcap.pc = p.PMCAPCapabilities;
    pmcap.pmcs = p.PMCAPCtrlStatus;

    // MSICAP
    msicap.mid = (uint16_t)p.MSICAPCapId; //mid.cid
    msicap.mid |= (uint16_t)p.MSICAPNextCapability << 8; //mid.next
    msicap.mc = p.MSICAPMsgCtrl;
    msicap.ma = p.MSICAPMsgAddr;
    msicap.mua = p.MSICAPMsgUpperAddr;
    msicap.md = p.MSICAPMsgData;
    msicap.mmask = p.MSICAPMaskBits;
    msicap.mpend = p.MSICAPPendingBits;

    // MSIXCAP
    msixcap.mxid = (uint16_t)p.MSIXCAPCapId; //mxid.cid
    msixcap.mxid |= (uint16_t)p.MSIXCAPNextCapability << 8; //mxid.next
    msixcap.mxc = p.MSIXMsgCtrl;
    msixcap.mtab = p.MSIXTableOffset;
    msixcap.mpba = p.MSIXPbaOffset;

    // allocate MSIX structures if MSIXCAP_BASE
    // indicates the MSIXCAP is being used by having a
    // non-zero base address.
    // The MSIX tables are stored by the guest in
    // little endian byte-order as according the
    // PCIe specification.  Make sure to take the proper
    // actions when manipulating these tables on the host
    uint16_t msixcap_mxc_ts = msixcap.mxc & 0x07ff;
    if (MSIXCAP_BASE != 0x0) {
        int msix_vecs = msixcap_mxc_ts + 1;
        MSIXTable tmp1 = {{0UL,0UL,0UL,0UL}};
        msix_table.resize(msix_vecs, tmp1);

        MSIXPbaEntry tmp2 = {0};
        int pba_size = msix_vecs / MSIXVECS_PER_PBA;
        if ((msix_vecs % MSIXVECS_PER_PBA) > 0) {
            pba_size++;
        }
        msix_pba.resize(pba_size, tmp2);
    }
    MSIX_TABLE_OFFSET = msixcap.mtab & 0xfffffffc;
    MSIX_TABLE_END = MSIX_TABLE_OFFSET +
                     (msixcap_mxc_ts + 1) * sizeof(MSIXTable);
    MSIX_PBA_OFFSET = msixcap.mpba & 0xfffffffc;
    MSIX_PBA_END = MSIX_PBA_OFFSET +
                   ((msixcap_mxc_ts + 1) / MSIXVECS_PER_PBA)
                   * sizeof(MSIXPbaEntry);
    if (((msixcap_mxc_ts + 1) % MSIXVECS_PER_PBA) > 0) {
        MSIX_PBA_END += sizeof(MSIXPbaEntry);
    }

    // PXCAP
    pxcap.pxid = (uint16_t)p.PXCAPCapId; //pxid.cid
    pxcap.pxid |= (uint16_t)p.PXCAPNextCapability << 8; //pxid.next
    pxcap.pxcap = p.PXCAPCapabilities;
    pxcap.pxdcap = p.PXCAPDevCapabilities;
    pxcap.pxdc = p.PXCAPDevCtrl;
    pxcap.pxds = p.PXCAPDevStatus;
    pxcap.pxlcap = p.PXCAPLinkCap;
    pxcap.pxlc = p.PXCAPLinkCtrl;
    pxcap.pxls = p.PXCAPLinkStatus;
    pxcap.pxdcap2 = p.PXCAPDevCap2;
    pxcap.pxdc2 = p.PXCAPDevCtrl2;

    for (int i = 0; i <32 ; i++){
        msi_sended.push_back(new MsiSended(i));
        msi_sended[i]->host=this;
        msi_sended[i]->cleaned=true;
    }

    // for(int i = 0; i < p.num_msi_engine; i++){
    //     msi_engines_ports.push_back(new MultiDmaEngineSlavePort(p.name + "msiport_devside", *this));
    // }
}

Tick
PciDevice::readConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    /* Return 0 for accesses to unimplemented PCI configspace areas */
    if (offset >= PCI_DEVICE_SPECIFIC &&
        offset < PCI_CONFIG_SIZE) {
        warn_once("Device specific PCI config space "
                  "(Capability) for %s!\n", this->name());
        readCapability(pkt);
    } else if (offset > PCI_CONFIG_SIZE) {
        panic("Out-of-range access to PCI config space!\n");
    } else {

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        pkt->setLE<uint8_t>(config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        pkt->setLE<uint16_t>(*(uint16_t*)&config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint16_t>());
        break;
      case sizeof(uint32_t):
        pkt->setLE<uint32_t>(*(uint32_t*)&config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    }
    pkt->makeAtomicResponse();
    return configDelay;

}

AddrRangeList
PciDevice::getAddrRanges() const
{
    AddrRangeList ranges;
    PciCommandRegister command = letoh(config.command);
    for (auto *bar: BARs) {
        if (command.ioSpace && bar->isIo())
            ranges.push_back(bar->range());
        if (command.memorySpace && bar->isMem())
            ranges.push_back(bar->range());
    }
    return ranges;
}

Tick
PciDevice::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    /* No effect if we write to config space that is not implemented*/
    if (offset >= PCI_DEVICE_SPECIFIC &&
        offset < PCI_CONFIG_SIZE) {
        warn_once("Device specific PCI config space "
                  "not implemented for %s!\n", this->name());
        writeCapability(pkt);
    } else if (offset > PCI_CONFIG_SIZE) {
        panic("Out-of-range access to PCI config space!\n");
    } else {

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        switch (offset) {
          case PCI0_INTERRUPT_LINE:
            config.interruptLine = pkt->getLE<uint8_t>();
            break;
          case PCI_CACHE_LINE_SIZE:
            config.cacheLineSize = pkt->getLE<uint8_t>();
            break;
          case PCI_LATENCY_TIMER:
            config.latencyTimer = pkt->getLE<uint8_t>();
            break;
          /* Do nothing for these read-only registers */
          case PCI0_INTERRUPT_PIN:
          case PCI0_MINIMUM_GRANT:
          case PCI0_MAXIMUM_LATENCY:
          case PCI_CLASS_CODE:
          case PCI_REVISION_ID:
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        switch (offset) {
          case PCI_COMMAND:
            config.command = pkt->getLE<uint8_t>();
            // IO or memory space may have been enabled/disabled.
            pioPort.sendRangeChange();
            break;
          case PCI_STATUS:
            config.status = pkt->getLE<uint8_t>();
            break;
          case PCI_CACHE_LINE_SIZE:
            config.cacheLineSize = pkt->getLE<uint8_t>();
            break;
          default:
            //writeCapability(pkt);
            break;
            //panic("writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint16_t>());
        break;
      case sizeof(uint32_t):
        switch (offset) {
          case PCI0_BASE_ADDR0:
          case PCI0_BASE_ADDR1:
          case PCI0_BASE_ADDR2:
          case PCI0_BASE_ADDR3:
          case PCI0_BASE_ADDR4:
          case PCI0_BASE_ADDR5:
            {
                int num = BAR_NUMBER(offset);
                auto *bar = BARs[num];
                config.baseAddr[num] =
                    htole(bar->write(hostInterface, pkt->getLE<uint32_t>()));
                pioPort.sendRangeChange();
            }
            break;

          case PCI0_ROM_BASE_ADDR:
            if (letoh(pkt->getLE<uint32_t>()) == 0xfffffffe)
                config.expansionROM = htole((uint32_t)0xffffffff);
            else
                config.expansionROM = pkt->getLE<uint32_t>();
            break;

          case PCI_COMMAND:
            // This could also clear some of the error bits in the Status
            // register. However they should never get set, so lets ignore
            // it for now
            config.command = pkt->getLE<uint32_t>();
            // IO or memory space may have been enabled/disabled.
            pioPort.sendRangeChange();
            break;

          default:
            //writeCapability(pkt);
            DPRINTF(PciDevice, "Writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    }
    pkt->makeAtomicResponse();
    return configDelay;
}

// NEPU CAPABILITY
/**
 * 결국에는 이것도 일반적인 write에 통합이 되어야하지만
 * 우선은 MSI만 간단하게 만들 것이니까 여기서 대충 해결하자.
 */
int
PciDevice::findCapability(int offset)
{
    // 1. Find CAPA type 일단 PCIe 확장 영역은 무시하고 진행
    int type = CAPA_ID_NULL;
    if (PMCAP_BASE && offset >= PMCAP_BASE && offset < PMCAP_BASE + sizeof(PMCAP))    type = CAPA_ID_PM;
    if (MSICAP_BASE && offset >= MSICAP_BASE && offset < MSICAP_BASE + sizeof(MSICAP))    type = CAPA_ID_MSI;
    if (MSIXCAP_BASE && offset >= MSIXCAP_BASE && offset < MSIXCAP_BASE + sizeof(MSIXCAP))    type = CAPA_ID_MSIX;
    if (PXCAP_BASE && offset >= PXCAP_BASE && offset < PXCAP_BASE + sizeof(PXCAP))    type = CAPA_ID_PX;
    
    DPRINTF(PciDevice,
        "Find CAPA dev %#x func %#x reg %x type = %s\n",
        _busAddr.dev, _busAddr.func, offset, getCapabilityName(type));
    DPRINTF(PciDevice, "BASE %x size %x\n", MSICAP_BASE, sizeof(MSICAP));
        
    return type;
}

std::string
PciDevice::getCapabilityName(int type)
{
    switch(type){
        case CAPA_ID_NULL:
            return "ERR";
        case CAPA_ID_PM:
            return "PM";
        case CAPA_ID_MSI:
            return "MSI";
        case CAPA_ID_MSIX:
            return "MSIX";
        case CAPA_ID_PX:
            return "PX";
        default:
            return "UNKNOWN";
    }
}

/**
 * PCI 의 Capability 를 고려해서 쓰기요청을 수행
 * PCIe의 확장 영역은 아직 미구현 상태
 */
void
PciDevice::writeCapability(PacketPtr pkt)
{
    // 1. Find CAPA type 일단 PCIe 확장 영역은 무시하고 진행
    // 나중에 PCIe 영역 지원을 추가한다면 
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;//(PCI_CONFIG_EXTENDED_SIZE-1);
    int type = findCapability(offset);
    int pkt_size = pkt->getSize();
    DPRINTF(PciDevice, "writeCapability\n");

    uint8_t* data = nullptr;
    int inter_offset = offset;
    /* Return 0 for accesses to unimplemented PCI configspace areas */
    /* 이 부분에서 처리 */
    switch(type){
        case CAPA_ID_NULL:
            panic("This is not a capability field!! Maybe RO area!");
            break;
        case CAPA_ID_PM:
            //base = PMCAP_BASE;
            inter_offset = offset - PMCAP_BASE;
            data = (pmcap.data) + inter_offset;
            break;
        case CAPA_ID_MSI:
            //base = MSICAP_BASE;
            inter_offset = offset - MSICAP_BASE;
            data = (msicap.data) + inter_offset;
            //*(uint8_t *)(msicap.data + inter_pos) = pkt->getLE<uint8_t>();
            break;
        case CAPA_ID_MSIX:
            //base = MSIXCAP_BASE;
            inter_offset = offset - MSIXCAP_BASE;
            data = (msixcap.data) + inter_offset;
            break;
        case CAPA_ID_PX:
            //base = PXCAP_BASE;
            inter_offset = offset - PXCAP_BASE;
            data = (pxcap.data) + inter_offset;
            break;
        default:
            panic("This capability is not implemanted!");
    }
    if(!data) return;
    // 2. 유형에 따른 처리를 하자

    // 2-1 적당한 기록 위치를 찾는다. 정확한 구현을 위해서는 각 지점마다 RW 여부를 알아야한다.
    // 우선은 gem5의 동작이 완전하다고 가정하고 쓰기를 항상 허용한다.
    switch(pkt_size){
        case sizeof(uint8_t):
            *(uint8_t *)(data) = pkt->getLE<uint8_t>();
            DPRINTF(PciDevice,
            "writeCapability: dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint8_t>());
            break;
        case sizeof(uint16_t):
            *(uint16_t *)(data) = pkt->getLE<uint16_t>();
            DPRINTF(PciDevice,
            "writeCapability: dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint16_t>());
            break;
        case sizeof(uint32_t):
            *(uint32_t *)(data) = pkt->getLE<uint32_t>();
            DPRINTF(PciDevice,
            "writeCapability: dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint32_t>());
            break;
    }
}

/**
 * PCI 의 Capability 를 고려해서 읽기요청을 수행
 * 기존의 주소 방식은 구조체에서 순서대로 아무 거나 읽어온 것에 가까움
 * 즉, config의 data를 초과한 것임.
 * 따라서 기본 영역을 벗어난 경우에 대해서는 아래와 같은 함수를 써서 처리하도록 함
 * PCIe 는 일단 신경 쓰지 말자.
 */
void
PciDevice::readCapability(PacketPtr pkt)
{
    int offset = pkt->getAddr() & (PCI_CONFIG_EXTENDED_SIZE-1);
    
    // 1. 유형 찾기
    int type = findCapability(offset);
    uint8_t* data = nullptr;
    int inter_offset = offset;
    /* Return 0 for accesses to unimplemented PCI configspace areas */
    /* 이 부분에서 처리 */
    switch(type){
        case CAPA_ID_NULL:
            //panic("This is not a capability field!! Maybe RO area!");
            break;
        case CAPA_ID_PM:
            //base = PMCAP_BASE;
            inter_offset = offset - PMCAP_BASE;
            data = (pmcap.data) + inter_offset;
            break;
        case CAPA_ID_MSI:
            //base = MSICAP_BASE;
            inter_offset = offset - MSICAP_BASE;
            data = (msicap.data) + inter_offset;
            //*(uint8_t *)(msicap.data + inter_pos) = pkt->getLE<uint8_t>();
            break;
        case CAPA_ID_MSIX:
            //base = MSIXCAP_BASE;
            inter_offset = offset - MSIXCAP_BASE;
            data = (msixcap.data) + inter_offset;
            break;
        case CAPA_ID_PX:
            //base = PXCAP_BASE;
            inter_offset = offset - PXCAP_BASE;
            data = (pxcap.data) + inter_offset;
            break;
        default:
            panic("This capability is not implemanted!");
    }
    if(!data) return;

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        pkt->setLE<uint8_t>(*data);
        DPRINTF(PciDevice,
            "readCapability:  dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        pkt->setLE<uint16_t>(*(uint16_t*)data);
        DPRINTF(PciDevice,
            "readCapability:  dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint16_t>());
        break;
      case sizeof(uint32_t):
        pkt->setLE<uint32_t>(*(uint32_t*)data);
        DPRINTF(PciDevice,
            "readCapability:  dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    //pkt->makeAtomicResponse();
}

Tick
PciDevice::sendingMSI(int int_local_num=0)
{
    uint64_t addr = 0;
    int packet_size;
    uint32_t data_val;
    Tick latency = 0;

    //if(!send) return 0;
    send = false;

    // 주소는 configuration 에서 들고오기
    //addr=msicap.mua;
    addr <<= 32;
    addr |= msicap.ma;

    packet_size = 4;

    //data = (uint8_t*)(&(msicap->md));
    data_val = msicap.md;
    data_val += int_local_num;
    data_val <<= 16;
    data_val |= MSI_SET;
    //data_val = msicap->md;
    //data_val |= MSI_SET;
    //data_val <<= 16;
    //data_val += int_local_num;

    if(!msi_sended[int_local_num]->sended){
        DPRINTF(NepMsi, "Send msg %d is ready already\n", int_local_num);
        return 0;
    }

    //msi_sended[int_local_num]->cleaned=false;
    
    uint8_t *data = (uint8_t*)malloc(4);
    
    memcpy(data, &data_val, packet_size);
    
    DPRINTF(NepMsi, "Write DMA MSI SEND! ADDR %#x, DATA %#x, data_val %#x\n", addr, *(uint32_t*)data, data_val);
    //dmaWrite(addr, packet_size, &msisend, data, 0);
    //dmaWrite(addr, packet_size, nullptr, data, 0);
    //msi_sended[int_local_num]->sended = false;
    //msi_sended[int_local_num]->cleaned = false;
    //msi_sended[int_local_num]->cleaned=false;
    msi_sended[int_local_num]->sended=false;
    // (((MultiDmaEngineMasterPort&)(msi_engines_ports[0]->getPort())).getDmaEngine())
    //     .msiWrite(addr, packet_size, &msi_sended[int_local_num]->msisend, data, 0);
    msiWrite(addr, packet_size, &msi_sended[int_local_num]->msisend, data, 0);
    //msiWrite(addr, packet_size, nullptr, data, 0);
    // if(msi_engines.size())
    //     msi_engines[0]->msiWrite(addr, packet_size, &msi_sended[int_local_num]->msisend, data, 0);
    // else
    //     msiWrite(addr, packet_size, &msi_sended[int_local_num]->msisend, data, 0);
    //msiWrite(addr, packet_size, nullptr, data, 0);
    //void dmaWrite(Addr addr, int size, Event *event, uint8_t *data,
    //        Tick delay = 0)
    //Request::UNCACHEABLE

    return latency;
}

Tick
PciDevice::clearMSI(int int_local_num=0)
{

    if(msi_sended[int_local_num]->cleaned){
        DPRINTF(NepMsi, "Clean msg %d is ready already\n", int_local_num);
        //return 0;
    }
    


    uint64_t addr = 0;
    int packet_size;
    uint8_t *data = (uint8_t*)malloc(4);
    uint32_t data_val;

    Tick latency = 0;

    // 주소는 configuration 에서 들고오기
    //addr=msicap.mua;
    addr <<= 32;
    addr |= msicap.ma;

    packet_size = 4;

    //data = (uint8_t*)(&(msicap->md));
    data_val = msicap.md;
    data_val += int_local_num;
    data_val <<= 16;
    data_val |= MSI_CLEAR;
    //data_val = msicap->md;
    //data_val |= MSI_CLEAR;
    //data_val <<= 16;
    //data_val += int_local_num;

    memcpy(data, &data_val, packet_size);
    
    DPRINTF(NepMsi, "Write DMA MSI CLEAR! ADDR %#x, DATA %#x\n", addr, *(uint32_t*)data);
    // msi_sended[int_local_num]->cleaned=true;
    // msi_sended[int_local_num]->sended=false;
    msiWrite(addr, packet_size, &msi_sended[int_local_num]->msiclean, data, 0);
    //msiWrite(addr, packet_size, nullptr, data, 0);
    //void dmaWrite(Addr addr, int size, Event *event, uint8_t *data,
    //        Tick delay = 0)

    return latency;
}

void
MsiSended::msiComplete()
{
    // Interrupt Clear!
    //uint32_t int_num = msicap->md;
    DPRINTF(NepMsi,"MSI %d Sended\n", localnum);
    //host->clearMSI(localnum);
    sended=true;
    //cleaned=false;
}

void
MsiSended::msiCleanComplete()
{
    // Interrupt Clear!
    //uint32_t int_num = msicap->md;
    DPRINTF(NepMsi,"MSI %d Clean Sended\n", localnum);
    //sended=false;
    cleaned=true;
}

void
PciDevice::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(config.data, sizeof(config.data) / sizeof(config.data[0]));

    // serialize the capability list registers
    paramOut(cp, csprintf("pmcap.pid"), uint16_t(pmcap.pid));
    paramOut(cp, csprintf("pmcap.pc"), uint16_t(pmcap.pc));
    paramOut(cp, csprintf("pmcap.pmcs"), uint16_t(pmcap.pmcs));

    paramOut(cp, csprintf("msicap.mid"), uint16_t(msicap.mid));
    paramOut(cp, csprintf("msicap.mc"), uint16_t(msicap.mc));
    paramOut(cp, csprintf("msicap.ma"), uint32_t(msicap.ma));
    SERIALIZE_SCALAR(msicap.mua);
    paramOut(cp, csprintf("msicap.md"), uint16_t(msicap.md));
    SERIALIZE_SCALAR(msicap.mmask);
    SERIALIZE_SCALAR(msicap.mpend);

    paramOut(cp, csprintf("msixcap.mxid"), uint16_t(msixcap.mxid));
    paramOut(cp, csprintf("msixcap.mxc"), uint16_t(msixcap.mxc));
    paramOut(cp, csprintf("msixcap.mtab"), uint32_t(msixcap.mtab));
    paramOut(cp, csprintf("msixcap.mpba"), uint32_t(msixcap.mpba));

    // Only serialize if we have a non-zero base address
    if (MSIXCAP_BASE != 0x0) {
        uint16_t msixcap_mxc_ts = msixcap.mxc & 0x07ff;
        int msix_array_size = msixcap_mxc_ts + 1;
        int pba_array_size = msix_array_size/MSIXVECS_PER_PBA;
        if ((msix_array_size % MSIXVECS_PER_PBA) > 0) {
            pba_array_size++;
        }

        SERIALIZE_SCALAR(msix_array_size);
        SERIALIZE_SCALAR(pba_array_size);

        for (int i = 0; i < msix_array_size; i++) {
            paramOut(cp, csprintf("msix_table[%d].addr_lo", i),
                     msix_table[i].fields.addr_lo);
            paramOut(cp, csprintf("msix_table[%d].addr_hi", i),
                     msix_table[i].fields.addr_hi);
            paramOut(cp, csprintf("msix_table[%d].msg_data", i),
                     msix_table[i].fields.msg_data);
            paramOut(cp, csprintf("msix_table[%d].vec_ctrl", i),
                     msix_table[i].fields.vec_ctrl);
        }
        for (int i = 0; i < pba_array_size; i++) {
            paramOut(cp, csprintf("msix_pba[%d].bits", i),
                     msix_pba[i].bits);
        }
    }

    paramOut(cp, csprintf("pxcap.pxid"), uint16_t(pxcap.pxid));
    paramOut(cp, csprintf("pxcap.pxcap"), uint16_t(pxcap.pxcap));
    paramOut(cp, csprintf("pxcap.pxdcap"), uint32_t(pxcap.pxdcap));
    paramOut(cp, csprintf("pxcap.pxdc"), uint16_t(pxcap.pxdc));
    paramOut(cp, csprintf("pxcap.pxds"), uint16_t(pxcap.pxds));
    paramOut(cp, csprintf("pxcap.pxlcap"), uint32_t(pxcap.pxlcap));
    paramOut(cp, csprintf("pxcap.pxlc"), uint16_t(pxcap.pxlc));
    paramOut(cp, csprintf("pxcap.pxls"), uint16_t(pxcap.pxls));
    paramOut(cp, csprintf("pxcap.pxdcap2"), uint32_t(pxcap.pxdcap2));
    paramOut(cp, csprintf("pxcap.pxdc2"), uint32_t(pxcap.pxdc2));

    for(int i = 0; i < msi_sended.size(); i++)
    {
        msi_sended[i]->serialize(csprintf("msi_sended[%d]", i), cp);
    }
}

void
PciDevice::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(config.data,
                      sizeof(config.data) / sizeof(config.data[0]));

    for (int idx = 0; idx < BARs.size(); idx++)
        BARs[idx]->write(hostInterface, config.baseAddr[idx]);

    // unserialize the capability list registers
    uint16_t tmp16;
    uint32_t tmp32;
    paramIn(cp, csprintf("pmcap.pid"), tmp16);
    pmcap.pid = tmp16;
    paramIn(cp, csprintf("pmcap.pc"), tmp16);
    pmcap.pc = tmp16;
    paramIn(cp, csprintf("pmcap.pmcs"), tmp16);
    pmcap.pmcs = tmp16;

    paramIn(cp, csprintf("msicap.mid"), tmp16);
    msicap.mid = tmp16;
    paramIn(cp, csprintf("msicap.mc"), tmp16);
    msicap.mc = tmp16;
    paramIn(cp, csprintf("msicap.ma"), tmp32);
    msicap.ma = tmp32;
    UNSERIALIZE_SCALAR(msicap.mua);
    paramIn(cp, csprintf("msicap.md"), tmp16);;
    msicap.md = tmp16;
    UNSERIALIZE_SCALAR(msicap.mmask);
    UNSERIALIZE_SCALAR(msicap.mpend);

    paramIn(cp, csprintf("msixcap.mxid"), tmp16);
    msixcap.mxid = tmp16;
    paramIn(cp, csprintf("msixcap.mxc"), tmp16);
    msixcap.mxc = tmp16;
    paramIn(cp, csprintf("msixcap.mtab"), tmp32);
    msixcap.mtab = tmp32;
    paramIn(cp, csprintf("msixcap.mpba"), tmp32);
    msixcap.mpba = tmp32;

    // Only allocate if MSIXCAP_BASE is not 0x0
    if (MSIXCAP_BASE != 0x0) {
        int msix_array_size;
        int pba_array_size;

        UNSERIALIZE_SCALAR(msix_array_size);
        UNSERIALIZE_SCALAR(pba_array_size);

        MSIXTable tmp1 = {{0UL, 0UL, 0UL, 0UL}};
        msix_table.resize(msix_array_size, tmp1);

        MSIXPbaEntry tmp2 = {0};
        msix_pba.resize(pba_array_size, tmp2);

        for (int i = 0; i < msix_array_size; i++) {
            paramIn(cp, csprintf("msix_table[%d].addr_lo", i),
                    msix_table[i].fields.addr_lo);
            paramIn(cp, csprintf("msix_table[%d].addr_hi", i),
                    msix_table[i].fields.addr_hi);
            paramIn(cp, csprintf("msix_table[%d].msg_data", i),
                    msix_table[i].fields.msg_data);
            paramIn(cp, csprintf("msix_table[%d].vec_ctrl", i),
                    msix_table[i].fields.vec_ctrl);
        }
        for (int i = 0; i < pba_array_size; i++) {
            paramIn(cp, csprintf("msix_pba[%d].bits", i),
                    msix_pba[i].bits);
        }
    }

    paramIn(cp, csprintf("pxcap.pxid"), tmp16);
    pxcap.pxid = tmp16;
    paramIn(cp, csprintf("pxcap.pxcap"), tmp16);
    pxcap.pxcap = tmp16;
    paramIn(cp, csprintf("pxcap.pxdcap"), tmp32);
    pxcap.pxdcap = tmp32;
    paramIn(cp, csprintf("pxcap.pxdc"), tmp16);
    pxcap.pxdc = tmp16;
    paramIn(cp, csprintf("pxcap.pxds"), tmp16);
    pxcap.pxds = tmp16;
    paramIn(cp, csprintf("pxcap.pxlcap"), tmp32);
    pxcap.pxlcap = tmp32;
    paramIn(cp, csprintf("pxcap.pxlc"), tmp16);
    pxcap.pxlc = tmp16;
    paramIn(cp, csprintf("pxcap.pxls"), tmp16);
    pxcap.pxls = tmp16;
    paramIn(cp, csprintf("pxcap.pxdcap2"), tmp32);
    pxcap.pxdcap2 = tmp32;
    paramIn(cp, csprintf("pxcap.pxdc2"), tmp32);
    pxcap.pxdc2 = tmp32;
    pioPort.sendRangeChange();

    for(int i = 0; i < msi_sended.size(); i++)
    {
        msi_sended[i]->unserialize(csprintf("msi_sended[%d]", i), cp);
    }
}

void 
MsiSended::serialize(const std::string &base, CheckpointOut &cp) const
{
    paramOut(cp, base + ".localnum", localnum);
    paramOut(cp, base + ".sended", sended);
    paramOut(cp, base + ".cleaned", cleaned);
}

void 
MsiSended::unserialize(const std::string &base, CheckpointIn &cp)
{
    paramIn(cp, base + ".localnum", localnum);
    paramIn(cp, base + ".sended", sended);
    paramIn(cp, base + ".cleaned", cleaned);
}