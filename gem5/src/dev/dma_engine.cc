#include "sim/system.hh"

#include "dev/dma_device.hh"
#include "dev/dma_engine.hh"
#include "dev/net/nep_GBe.hh"
#include "params/MultiDmaEngine.hh"


MultiDmaEngine::MultiDmaEngine(const Params &p)
    :DmaDevice(p), name_(p.name), masterPort(p.name + ".msiport_engineside",  *this)
{

}

AddrRangeList 
MultiDmaEngine::getAddrRanges() const 
{
    return masterPort.getAddrRanges();    
}


MultiDmaEngineMasterPort::MultiDmaEngineMasterPort(const std::string& _name, MultiDmaEngine& _bridge)
: MasterPort(_name, &_bridge), bridge(_bridge)
{
    
}

MultiDmaEngineSlavePort::MultiDmaEngineSlavePort(const std::string& _name, PciDevice& _bridge)
: SlavePort(_name, &_bridge), bridge(_bridge)
{
    
}

Port &
MultiDmaEngine::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "msiport_engineside") {
        return masterPort;
    } else {
        return DmaDevice::getPort(if_name, idx);
    }
}


// Port &
// MultiDmaEngineLinker::getPort(const std::string &if_name, PortID idx)
// {
//     if (if_name == "default") {
//         return masterPort;
//     } else if (if_name == "master") {
//         return masterPort;
//     } else if (if_name == "slave") {
//         return slavePort;
//     } else {
//         return ClockedObject::getPort(if_name, idx);
//     }
// }



// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::MultiDmaEngineLinkerSlavePort(const std::string& _name, MultiDmaEngineLinker& _bridge)
// :SlavePort(_name, &_bridge), bridge(_bridge)
// {
    
// }


// bool
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::recvTimingReq(PacketPtr pkt){
    
//     return bridge.masterPort.sendTimingReq(pkt);
// }



// Tick
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::recvAtomic(PacketPtr pkt){
    
//     return bridge.masterPort.sendAtomic(pkt);
// }

// void
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::recvFunctional(PacketPtr pkt){
    
//     return bridge.masterPort.sendFunctional(pkt);
// }

// AddrRangeList
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::getAddrRanges() const
// {
    
//     return ranges;
// }

// bool 
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::recvTimingSnoopResp(PacketPtr pkt){
    
//     return bridge.masterPort.sendTimingSnoopResp(pkt);
// }

// void 
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::recvRespRetry(){
    
//     panic("DdioBridge::DdioBridgeSlavePort::recvRespRetry()");
//     //bridge.getDestinationMasterPort(nullptr).sendRetryResp();
// }


// void
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::sendRetryReq(){
    
//     panic("DdioBridge::DdioBridgeSlavePort::sendRetryReq()");
// }

// void
// MultiDmaEngineLinker::MultiDmaEngineLinkerSlavePort::sendRetrySnoopResp(){
    
//     panic("DdioBridge::DdioBridgeSlavePort::sendRetrySnoopResp()");
// }

// MultiDmaEngineLinker::MultiDmaEngineLinkerMasterPort::MultiDmaEngineLinkerMasterPort(const std::string& _name, MultiDmaEngineLinker& _bridge)
// :MasterPort(_name, &_bridge), bridge(_bridge)
// {

// }



// Tick
// MultiDmaEngineLinker::MultiDmaEngineLinkerMasterPort::recvAtomicSnoop(PacketPtr pkt){
//     if(isSnooping()){
        
//         return bridge.slavePort.sendAtomicSnoop(pkt);
//     }
//     //panic("recvAtomicSnoop! port typ %s, pkt %s\n", getPortType()==PORT_TYPE_FOR_MLC?"MLC":"MEM", pkt->print());
//     return 0;
// }

// void
// MultiDmaEngineLinker::MultiDmaEngineLinkerMasterPort::recvFunctionalSnoop(PacketPtr pkt){
//     if(1){
        
//         bridge.slavePort.sendFunctionalSnoop(pkt);
//         return;
//     }
// }

// bool
// MultiDmaEngineLinker::MultiDmaEngineLinkerMasterPort::recvTimingResp(PacketPtr pkt){
//     if(isSnooping()){
//         //DPRINTF(AdaptiveDdioBridge, "recvTimingResp, pkt %s\n", pkt->print());
//         return bridge.slavePort.sendTimingResp(pkt);
//     }
//     //panic("recvTimingResp! port typ %s, pkt %s\n", getPortType()==PORT_TYPE_FOR_MLC?"MLC":"MEM", pkt->print());
//     return true;
// }

// void 
// MultiDmaEngineLinker::MultiDmaEngineLinkerMasterPort::recvTimingSnoopReq(PacketPtr pkt){
//     if( (!(pkt->hasData())) && pkt->cmd==MemCmd::UpgradeReq)
//     {
//         //return;
//     }
//     return bridge.slavePort.sendTimingSnoopReq(pkt);
// }

// void
// MultiDmaEngineLinker::MultiDmaEngineLinkerMasterPort::recvReqRetry(){
//     //DPRINTF(AdaptiveDdioBridge, "recvReqRetry\n");
//     //bridge.slavePort.sendRetryReq();
//     return;
// }

// void
// MultiDmaEngineLinker::MultiDmaEngineLinkerMasterPort::recvRetrySnoopResp(){

//     //DPRINTF(AdaptiveDdioBridge, "recvRetrySnoopResp\n");
//     //bridge.slavePort.sendRetrySnoopResp();
// }


// MultiDmaEngineLinker::MultiDmaEngineLinker(MultiDmaEngineLinkerParams *p)
//     :ClockedObject(p)
// {
    
// }

// MultiDmaEngineLinker::~MultiDmaEngineLinker(){

// }



// MultiDmaEngineLinker *
// MultiDmaEngineLinkerParams::create()
// {
//     return new MultiDmaEngineLinker(this);
// }

