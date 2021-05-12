#ifndef __DEV_DMA_ENGINE_HH__
#define __DEV_DMA_ENGINE_HH__

#include <deque>
#include <string>

#include "base/inet.hh"
#include "debug/EthernetDesc.hh"
#include "debug/EthernetIntr.hh"
#include "debug/NepNicRxManager.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/nep_GBe_defs.hh"
#include "dev/net/pktfifo.hh"
#include "dev/pci/device.hh"
#include "mem/mem_object.hh"
#include "params/MultiDmaEngine.hh"
//#include "params/MultiDmaEngineLinker.hh"

class NepGbE;

class MultiDmaEngine;

class MultiDmaEngineMasterPort : public MasterPort
{
  private:
    /** The bridge to which this port belongs. */
    MultiDmaEngine& bridge;       
    
  public:
    MultiDmaEngineMasterPort(const std::string& _name, MultiDmaEngine& _bridge);
    MultiDmaEngine& getDmaEngine(){return bridge;}
    
  protected:
    Tick recvAtomicSnoop(PacketPtr pkt) {return 0;}
    void recvFunctionalSnoop(PacketPtr pkt) {}
    bool recvTimingResp(PacketPtr pkt) {return true;}
    void recvTimingSnoopReq(PacketPtr pkt) {}
    void recvReqRetry() {}
    virtual void recvRetrySnoopResp() {}
    bool isSnooping() const {return false;}
};

class MultiDmaEngineSlavePort : public SlavePort
{
  private:
    /** The bridge to which this port belongs. */
    PciDevice& bridge;
    const AddrRangeList ranges;

  public:
    MultiDmaEngineSlavePort(const std::string& _name, PciDevice& bridge);

  protected:
    virtual bool recvTimingReq(PacketPtr pkt){return true;}
    virtual Tick recvAtomic(PacketPtr pkt) {return 0;}
    virtual void recvFunctional(PacketPtr pkt) {}
    virtual AddrRangeList getAddrRanges() const {return ranges;}
    
    virtual bool recvTimingSnoopResp(PacketPtr pkt) {return true;}
    virtual void recvRespRetry() {}

    void sendRetryReq() {}
    void sendRetrySnoopResp() {}

    bool isSnooping() const { return false; }
};

class MultiDmaEngine : public DmaDevice{
  public:
    //DmaDevice* host;
    friend DmaDevice;
    std::string name_;
    virtual Tick writeConfig(PacketPtr pkt){return 0;}
    virtual Tick readConfig(PacketPtr pkt){return 0;}
    virtual Tick read(PacketPtr pkt){return 0;}
    virtual Tick write(PacketPtr pkt){return 0;}
    AddrRangeList getAddrRanges() const ;
    //Stats::Scalar test;
    std::string name(){return name_;}
    void regStats() override {
        DmaDevice::regStats();
    } 
    void resetStats() override {
        DmaDevice::resetStats();
    }
    
    PARAMS(MultiDmaEngine);
    
    MultiDmaEngineMasterPort masterPort;
    Port& getPort(const std::string &if_name, PortID idx) override;
    MultiDmaEngine(const MultiDmaEngineParams &p);
    //MultiDmaEngine(const MultiDmaEngineParams *p, DmaDevice* host, std::string _name);
    ~MultiDmaEngine(){}

  public:
    void serialize(CheckpointOut &cp) const override {}
    void unserialize(CheckpointIn &cp) override {}
};


// class MultiDmaEngineLinker : public ClockedObject
// {
//   private:
//     /** Port that handles requests that don't match any of the interfaces.*/
//     PortID defaultPortID;

//   protected:
//     class MultiDmaEngineLinkerSlavePort : public SlavePort
//     {
//       private:
//         /** The bridge to which this port belongs. */
//         MultiDmaEngineLinker& bridge;
//         const AddrRangeList ranges;

//       public:
//         MultiDmaEngineLinkerSlavePort(const std::string& _name, MultiDmaEngineLinker& _bridge);

//       protected:
//         virtual bool recvTimingReq(PacketPtr pkt);
//         virtual Tick recvAtomic(PacketPtr pkt);
//         virtual void recvFunctional(PacketPtr pkt);
//         virtual AddrRangeList getAddrRanges() const;
        
//         virtual bool recvTimingSnoopResp(PacketPtr pkt);
//         virtual void recvRespRetry();

//         void sendRetryReq();
//         void sendRetrySnoopResp();

//         bool isSnooping() const { return true; }
//     };

//     class MultiDmaEngineLinkerMasterPort : public MasterPort
//     {
//       private:
//         /** The bridge to which this port belongs. */
//         MultiDmaEngineLinker& bridge;       
        
//       public:
//         MultiDmaEngineLinkerMasterPort(const std::string& _name, MultiDmaEngineLinker& _bridge);
        
//       protected:
//         Tick recvAtomicSnoop(PacketPtr pkt);
//         void recvFunctionalSnoop(PacketPtr pkt);
//         bool recvTimingResp(PacketPtr pkt);
//         void recvTimingSnoopReq(PacketPtr pkt);
//         void recvReqRetry();
//         virtual void recvRetrySnoopResp();
//         bool isSnooping() const;
        
//         /** The master and slave ports of the crossbar */
//     };

//   public:
//     MultiDmaEngineLinkerSlavePort slavePort;
//     MultiDmaEngineLinkerMasterPort masterPort;
//     MultiDmaEngineLinker(MultiDmaEngineLinkerParams *p);
//     ~MultiDmaEngineLinker();
//     Port &getPort(const std::string &if_name, PortID idx=InvalidPortID) override;
// };

#endif